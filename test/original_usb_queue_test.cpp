// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/firmware.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "original_usb_queue_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace blofeld::test::usb_queue {
using namespace usb_host;

namespace {
constexpr std::array<Message, 16> messages{{
    {.data = {0xf0, 1, 2}, .header = 4},
    {.data = {0xf7, 3, 4}, .header = 5},
    {.data = {1, 0xf7, 2}, .header = 6},
    {.data = {1, 2, 0xf7}, .header = 7},
    {.data = {1, 0x80, 2}, .header = 6},
    {.data = {1, 2, 0x80}, .header = 7},
    {.data = {1, 2, 3}, .header = 4},
    {.data = {0x80, 48, 100}, .header = 8},
    {.data = {0x90, 48, 100}, .header = 9},
    {.data = {0xa1, 48, 2}, .header = 10},
    {.data = {0xb1, 7, 32}, .header = 11},
    {.data = {0xc1, 7, 0x55}, .header = 12},
    {.data = {0xd1, 7, 0x55}, .header = 13},
    {.data = {0xe1, 1, 2}, .header = 14},
    {.data = {0xf8, 0x23, 0x45}, .header = 15},
    {.data = {0, 0, 0}, .header = 4},
}};
constexpr std::array<unsigned, 3> flag_patterns{0, 4, 0xff};
constexpr Message note{.data = {0x90, 60, 100}, .header = 9};
constexpr unsigned idle_control = 0x1000;
constexpr unsigned busy_control = 0x5000;
constexpr unsigned masked_irq = 0x50;
constexpr unsigned unmasked_irq = 0x40;
constexpr unsigned raw_cursor = 128;

void header_case(const Firmware &firmware, const unsigned id, const Input input) {
  Fixture fixture(firmware, input);
  const auto next = (input.write + event_bytes) & cursor_mask;
  const bool forced = input.message.header >= 5 && input.message.header <= 7;
  const bool flush = !input.busy && (forced || (input.flags & 4U) == 0);
  replay(fixture, [&](const unsigned attempt) {
    fixture.run();
    fixture.check_return({.result = input.message.header,
                          .read = flush ? next : input.read,
                          .write = next,
                          .masks = flush ? 0 : unmasked_irq});
    if (input.busy) {
      require(word(fixture.board().bus(), input_control) == busy_control,
              "A queued message cannot overwrite the earlier unacknowledged IN packet");
      fixture.acknowledge();
      fixture.service(usb_vector);
      fixture.drain(fixture.record());
    } else if (flush) {
      fixture.drain(fixture.record());
    } else {
      require(word(fixture.board().bus(), input_control) == idle_control &&
                  fixture.board().bus().request_usb_in(UsbBuffer::endpoint2).response ==
                      UsbResponse::nak,
              "A deferred message remains in the real ring without an invented packet");
    }
    std::cout << "HEADER " << id << ' ' << input.flags << ' ' << input.write << ' ' << input.busy
              << ' ' << attempt << ' ' << fixture.counters().instructions << ' '
              << fixture.counters().usb_irqs << ' ' << fixture.counters().timer_irqs << '\n';
  });
}

void full_case(const Firmware &firmware,
               const unsigned cursor,
               const bool busy,
               const bool masked) {
  const auto read = (cursor + event_bytes) & cursor_mask;
  Fixture fixture(firmware,
                  {.message = note,
                   .flags = 4,
                   .read = read,
                   .write = cursor,
                   .ccr = masked ? masked_irq : unmasked_irq,
                   .busy = busy});
  replay(fixture, [&](const unsigned attempt) {
    fixture.run();
    if (masked) {
      fixture.check_return({.result = 0, .read = read, .write = cursor, .masks = masked_irq});
      require(
          fixture.counters().ccr_calls == 1 && fixture.counters().usb_irqs == 0 &&
              word(fixture.board().bus(), input_control) == (busy ? busy_control : idle_control),
          "The full ring uses actual CCR.I to fail after storing, without publishing or flushing");
    } else {
      const auto advanced = (read + endpoint_bytes) & cursor_mask;
      fixture.check_return({.result = note.header, .read = advanced, .write = read, .masks = 0});
      auto bytes = fixture.old_events();
      const auto record = fixture.record();
      bytes.insert(bytes.end(), record.begin(), record.end());
      fixture.drain(bytes);
      require(
          fixture.counters().ccr_calls == 1,
          "The full producer calls its genuine CCR reader and batch sender despite deferred flags");
    }
    std::cout << "FULL " << cursor << ' ' << busy << ' ' << masked << ' ' << attempt << ' '
              << fixture.counters().instructions << ' ' << fixture.counters().usb_irqs << '\n';
  });
}

void raw_alias_case(const Firmware &firmware) {
  Fixture fixture(firmware, {.message = note, .flags = 4, .write = raw_cursor, .raw_alias = true});
  replay(fixture, [&](const unsigned attempt) {
    fixture.run();
    // Raw128 aliases reader/writer. The status byte replaces writer before its
    // actual reload: (0x90+4)&0x7c =0x14. No host-side index normalization.
    constexpr unsigned published = 0x14;
    fixture.check_return(
        {.result = note.header, .read = note.header, .write = published, .masks = unmasked_irq});
    require(fixture.board().bus().read(writer + 1) == note.data[1] &&
                fixture.board().bus().read(writer + 2) == note.data[2] &&
                fixture.counters().usb_irqs == 0,
            "Raw insertion alias and live write-cursor reload retain the original native memory "
            "effects");
    std::cout << "ALIAS " << attempt << ' ' << fixture.counters().instructions << '\n';
  });
}

void test_headers(const Firmware &firmware) {
  for (unsigned id = 0; id < messages.size(); ++id) {
    for (const auto flags : flag_patterns) {
      for (const auto cursor : {0U, cursor_mask}) {
        for (const bool busy : {false, true}) {
          header_case(firmware,
                      id,
                      {.message = messages[id],
                       .flags = flags,
                       .read = cursor,
                       .write = cursor,
                       .busy = busy});
        }
      }
    }
  }
  for (const auto cursor : {0U, cursor_mask}) {
    full_case(firmware, cursor, false, true);
    full_case(firmware, cursor, true, true);
    full_case(firmware, cursor, false, false);
  }
  raw_alias_case(firmware);
}
}
}

int main(const int count, char **arguments) {
  try {
    blofeld::test::usb_host::require(count == 2, "Usage: original-usb-queue original.zip");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    blofeld::test::usb_queue::test_headers(firmware);
    blofeld::test::usb_queue::test_interrupts(firmware);
    std::cout << "SUMMARY headers=192 full=6 alias=1 pulc=8 busy=1 deadline=1 skipped-deadline=1 "
                 "restores-per-case=2 "
                 "own-peripheral-inputs=1 original-boot=0\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
