// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "original_usb_queue_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <iostream>

namespace blofeld::test::usb_queue {
using namespace usb_host;

namespace {
constexpr Message note{.data = {0x90, 60, 100}, .header = 9};
constexpr unsigned x_mask = 0x40;
constexpr unsigned condition_mask = 0x0f;

Bytes stream(const Fixture &fixture) {
  auto bytes = fixture.old_events();
  const auto record = fixture.record();
  bytes.insert(bytes.end(), record.begin(), record.end());
  return bytes;
}

unsigned compare_flags(const unsigned left, const unsigned right) {
  const auto difference = (left + 256 - right) & byte_mask;
  const unsigned negative = (difference & 0x80U) != 0 ? 8 : 0;
  const unsigned zero = difference == 0 ? 4 : 0;
  const unsigned overflow = ((left ^ right) & (left ^ difference) & 0x80U) != 0 ? 2 : 0;
  const unsigned carry = left < right ? 1 : 0;
  return x_mask | negative | zero | overflow | carry;
}

void pulc_case(const Firmware &firmware,
               const unsigned cursor,
               const bool full,
               const bool after_irq) {
  const auto read = full ? (cursor + event_bytes) & cursor_mask : cursor;
  Fixture fixture(firmware,
                  {.message = note, .flags = 4, .read = read, .write = cursor, .busy = true});
  while (fixture.board().cpu().disassemble() != "PULC") { fixture.step(); }
  const auto pc = fixture.board().cpu().state().pc;
  const auto saved_ccr = compare_flags((cursor + event_bytes) & cursor_mask, read);
  require(fixture.board().cpu().state().sp == stack - 8 &&
              fixture.board().bus().read(stack - 8) == saved_ccr,
          "The actual original PSHC stores independently expected comparison flags before STAA");
  if (after_irq) {
    fixture.acknowledge();
    fixture.service(usb_vector);
  }
  std::cout << "PREFIX PULC " << cursor << ' ' << full << ' ' << after_irq << ' '
            << fixture.counters().instructions << ' ' << fixture.counters().usb_irqs << ' '
            << fixture.counters().timer_irqs << '\n';
  replay(fixture, [&](const unsigned attempt) {
    if (!after_irq) {
      fixture.acknowledge();
      fixture.service(usb_vector);
    }
    require(
        fixture.board().cpu().state().pc == pc &&
            (fixture.board().cpu().state().ccr & x_mask) == 0 &&
            fixture.board().bus().read(stack - 8) == saved_ccr,
        "The original nested USB IRQ clears X but leaves the producer's older PSHC byte intact");
    fixture.step();
    require(fixture.board().cpu().state().ccr == (saved_ccr & condition_mask) &&
                fixture.board().cpu().state().sp == stack - 7,
            "Actual PULC restores comparison flags while refusing to re-enable the saved X mask");
    fixture.run();
    const auto next = (cursor + event_bytes) & cursor_mask;
    fixture.check_return({.result = note.header,
                          .read = full ? (read + endpoint_bytes) & cursor_mask : read,
                          .write = next,
                          .masks = 0});
    if (full) {
      fixture.drain(stream(fixture));
    } else {
      require(fixture.board().bus().request_usb_in(UsbBuffer::endpoint2).response ==
                  UsbResponse::nak,
              "A deferred event remains queued after an empty-ring IRQ instead of manufacturing a "
              "packet");
    }
    std::cout << "PULC " << cursor << ' ' << full << ' ' << after_irq << ' ' << attempt << ' '
              << fixture.counters().instructions << ' ' << fixture.counters().usb_irqs << '\n';
  });
}

void busy_case(const Firmware &firmware) {
  Fixture fixture(firmware, {.message = note, .flags = 4, .read = 4, .busy = true});
  fixture.until(wait_pc);
  for (unsigned cycle = 0; cycle < 3; ++cycle) { fixture.until(wait_pc, true); }
  require(fixture.counters().ccr_calls == 3 && fixture.board().bus().read(tick) == initial_tick &&
              fixture.board().bus().read(reader) == 4 && fixture.board().bus().read(writer) == 0,
          "Real full/busy waiting neither fabricates elapsed ticks nor returns a successful queue "
          "result");
  std::cout << "PREFIX BUSY 0 0 0 " << fixture.counters().instructions << ' '
            << fixture.counters().usb_irqs << ' ' << fixture.counters().timer_irqs << '\n';
  replay(fixture, [&](const unsigned attempt) {
    fixture.acknowledge();
    fixture.service(usb_vector);
    fixture.run();
    constexpr unsigned after_batch = 68;
    fixture.check_return({.result = note.header, .read = after_batch, .write = 4, .masks = 0});
    fixture.drain(stream(fixture));
    require(fixture.board().bus().read(tick) == initial_tick,
            "Actual endpoint ACK/IRQ unblocks the producer without host counter or CCR writes");
    std::cout << "BUSY " << attempt << ' ' << fixture.counters().instructions << ' '
              << fixture.counters().usb_irqs << '\n';
  });
}

void deadline_case(const Firmware &firmware) {
  Fixture fixture(firmware, {.message = note, .flags = 4, .read = 4, .busy = true, .timer = true});
  fixture.until(wait_pc);
  std::cout << "PREFIX DEADLINE 0 0 0 " << fixture.counters().instructions << ' '
            << fixture.counters().usb_irqs << ' ' << fixture.counters().timer_irqs << '\n';
  replay(fixture, [&](const unsigned attempt) {
    for (unsigned elapsed = 1; elapsed <= 16; ++elapsed) {
      require(fixture.board().bus().oscillator_cycles_until_rti() == timer_period,
              "Each actual oscillator timeout begins with its full independently expected divider "
              "period");
      fixture.board().bus().advance_oscillator(timer_period);
      const auto before = fixture.counters().instructions;
      fixture.service(timer_vector);
      require(fixture.counters().instructions == before + 4 &&
                  fixture.board().bus().read(tick) == ((initial_tick + elapsed) & byte_mask) &&
                  (fixture.board().bus().read(hc12::crg_flags) & 0x80U) == 0,
              "Original scan-inhibited RTI executes four native instructions, increments the byte "
              "and acknowledges RTIF");
      if (elapsed < 16) { fixture.until(wait_pc, true); }
    }
    fixture.run();
    fixture.check_return({.result = 0, .read = 4, .write = 0, .masks = x_mask});
    require(fixture.counters().timer_irqs == 16 && fixture.counters().usb_irqs == 0 &&
                fixture.board().bus().read(tick) == 14 &&
                fixture.board().bus().oscillator_cycles_until_rti() == timer_period,
            "The original exact byte deadline wraps FE->0E and fails without publishing or USB "
            "completion");
    std::cout << "DEADLINE " << attempt << ' ' << fixture.counters().instructions << ' '
              << fixture.counters().timer_irqs << '\n';
  });
}

void skipped_deadline_case(const Firmware &firmware) {
  constexpr unsigned skipped_ticks = 17;
  constexpr unsigned extra_ticks = 255;
  constexpr unsigned after_skip = 15;
  constexpr unsigned deadline = 14;
  Fixture fixture(firmware, {.message = note, .flags = 4, .read = 4, .busy = true, .timer = true});
  const auto next_tick = [&fixture]() {
    require(fixture.board().bus().oscillator_cycles_until_rti() == timer_period,
            "Every supplied period starts at the actual documented oscillator divider");
    fixture.board().bus().advance_oscillator(timer_period);
    const auto before = fixture.counters().instructions;
    fixture.service(timer_vector);
    require(fixture.counters().instructions == before + 4 &&
                (fixture.board().bus().read(hc12::crg_flags) & 0x80U) == 0,
            "Only the original four-instruction timer ISR updates the tick and acknowledges RTIF");
  };
  fixture.until(wait_pc);
  // Supply an explicit interrupt schedule while the producer is paused. This
  // distinguishes exact byte equality from elapsed-time tests, not board timing.
  for (unsigned elapsed = 0; elapsed < skipped_ticks; ++elapsed) { next_tick(); }
  require(
      fixture.board().bus().read(tick) == after_skip,
      "Seventeen original timer entries pass the saved FE+16 deadline without a host tick write");
  fixture.until(wait_pc, true);
  require(fixture.counters().ccr_calls == 1 && fixture.board().bus().read(writer) == 0 &&
              fixture.board().bus().read(reader) == 4,
          "A skipped deadline neither times out nor publishes the queued event");
  std::cout << "PREFIX SKIP 0 0 0 " << fixture.counters().instructions << ' '
            << fixture.counters().usb_irqs << ' ' << fixture.counters().timer_irqs << '\n';
  replay(fixture, [&](const unsigned attempt) {
    for (unsigned elapsed = 1; elapsed <= extra_ticks; ++elapsed) {
      next_tick();
      require(fixture.board().bus().read(tick) == ((after_skip + elapsed) & byte_mask),
              "Actual original timer increments retain the independent byte-wrap expectation");
      if (elapsed < extra_ticks) { fixture.until(wait_pc, true); }
    }
    fixture.run();
    fixture.check_return({.result = 0, .read = 4, .write = 0, .masks = x_mask});
    require(fixture.board().bus().read(tick) == deadline &&
                fixture.counters().timer_irqs == skipped_ticks + extra_ticks &&
                fixture.counters().usb_irqs == 0,
            "Exact equality after another255 native ticks fails without publishing or a USB ACK");
    std::cout << "SKIP " << attempt << ' ' << fixture.counters().instructions << ' '
              << fixture.counters().timer_irqs << '\n';
  });
}
}

void test_interrupts(const Firmware &firmware) {
  for (const auto cursor : {0U, cursor_mask}) {
    for (const bool full : {false, true}) {
      pulc_case(firmware, cursor, full, false);
      pulc_case(firmware, cursor, full, true);
    }
  }
  busy_case(firmware);
  deadline_case(firmware);
  skipped_deadline_case(firmware);
}
}
