// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard_chain_fixture.hpp"

#include "blofeld/dsp_gpio.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/gpio_pads.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/hcs08_bdc.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "keyboard_handoff_fixture.hpp"
#include "keyboard_reset_fixture.hpp"
#include "lcd_fixture.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "original_update_forward_fixture.hpp"
#include "original_update_transport_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <ios>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

namespace blofeld::test::keyboard_chain {
namespace fw = update_forward;
using hcs08::require;

namespace {
constexpr unsigned ram_first = 0x80, ram_end = 0x480, algorithm = 0xa0, algorithm_offset = 838,
                   upload_end = 0x124;
constexpr unsigned trim_address = 0xffbe, buffer = 0x200, stack_low = 0x7c;
constexpr unsigned word_bytes = 3, byte_mask = 255, helper_prefix = 2, helper_extent = 1018;
constexpr unsigned tscr2 = 0x4d, active_timer = 0x90;
constexpr std::array<std::uint8_t, 3> own_clear_call{0x16, 0x8d, 0x70};
constexpr unsigned caller_ddre_operand = 16, lcd_enable = 0x10, unknown_ram_canary = 0xcc;

std::vector<std::uint8_t> extract(const std::vector<transport::Block> &blocks) {
  std::vector<std::uint8_t> result;
  for (unsigned n = 0; n < helper_extent; ++n) {
    const auto offset = n + helper_prefix;
    result.push_back(static_cast<std::uint8_t>(
        (blocks.at(2).expected.at(offset / word_bytes) >> (16U - 8U * (offset % word_bytes))) &
        byte_mask));
  }
  return result;
}
}

Chain::Chain(const Firmware &image,
             const std::vector<transport::Block> &blocks,
             const transport::Templates &templates,
             std::ostream &output,
             const Scenario inputs,
             const hcs08_programmer::Memory *snapshot)
    : firmware(image),
      trace(output),
      scenario(inputs),
      filter(output),
      detail(&filter),
      helper(extract(blocks)),
      sender(image, blocks, templates, detail, scenario.phase, kh::EntryCcr::cleared_lcd) {
  detail.exceptions(std::ios::badbit | std::ios::failbit);
  auto &pair = sender.pair;
  display = std::make_unique<LcdFixture>(pair.mcu.bus());
  auto caller = fw::caller_program(true);
  // Our lab caller establishes output data before enabling its GPIO drivers.
  std::rotate(caller.begin(), caller.begin() + 5, caller.begin() + 10);
  caller.at(caller_ddre_operand) = lcd_enable;
  // The standalone stage2 fixture invokes the real resident LCD clear first.
  // Reset/controller setup alone does not initialize the display RAM.
  constexpr unsigned setup_bytes = 23;
  caller.insert(caller.begin() + setup_bytes, own_clear_call.begin(), own_clear_call.end());
  for (unsigned n = 0; n < caller.size(); ++n) {
    pair.mcu.bus().write(static_cast<std::uint16_t>(kh::caller + n), caller[n]);
  }
  pair.mcu.bus().write(fw::bus_flags, 1);
  // Explicit application-timer fixture input, not inferred application startup.
  pair.mcu.bus().write(tscr2, 2);
  pair.mcu.bus().write(hc12::tscr1, active_timer);
  peer = std::make_unique<kr::Fixture>(
      pair.mcu, image, helper, scenario.target_phase, trace, snapshot);
  auto &memory = peer->target.memory;
  if (snapshot == nullptr) {
    for (unsigned a = ram_first; a < ram_end; ++a) {
      memory.known.reset(a);
      memory.bytes[a] = unknown_ram_canary;
    }
    ++memory.generation;
    ++memory.code_generation;
    memory.accesses.clear();
  }
  expected = {blofeld::hcs08_bdc::background,
              blofeld::hcs08_bdc::read_byte,
              trim_address >> 8U,
              trim_address & byte_mask};
  for (unsigned a = ram_first; a < upload_end; ++a) {
    expected.insert(expected.end(),
                    {blofeld::hcs08_bdc::write_byte,
                     a >> 8U,
                     a & byte_mask,
                     a < algorithm ? 0U : helper.at(a - algorithm + algorithm_offset)});
  }
  expected.push_back(blofeld::hcs08_bdc::background);
  for (unsigned n = 0; n < page_bytes; ++n) {
    const auto a = buffer + n;
    expected.insert(
        expected.end(),
        {blofeld::hcs08_bdc::write_byte, a >> 8U, a & byte_mask, source_byte(page_offset + 2 + n)});
  }
  constexpr unsigned programmer = 0xa0;
  expected.insert(expected.end(),
                  {blofeld::hcs08_bdc::write_byte,
                   0,
                   ram_first,
                   source_byte(page_offset),
                   blofeld::hcs08_bdc::write_byte,
                   0,
                   ram_first + 1,
                   source_byte(page_offset + 1),
                   blofeld::hcs08_bdc::write_pc,
                   0,
                   programmer,
                   blofeld::hcs08_bdc::write_sp,
                   4,
                   stack_low,
                   8});
  require(expected.size() == expected_rx, "Independent original BDC upload/control frame extent");
}

unsigned Chain::source_byte(const unsigned offset) const {
  return (sender.blocks.at(2).expected.at(offset / word_bytes) >>
          (16U - 8U * (offset % word_bytes))) &
         byte_mask;
}

GpioPadSample Chain::irq_pads(const std::uint32_t mask) {
  require((sender.pair.mcu.bus().gpio_outputs(McuPort::e).mask & mask & 3U) == 0,
          "The declared DSP IRQ routes require actual MCU inputs");
  auto sample = peer->pads(McuPort::e, mask & ~1U);
  if ((mask & 1U) != 0) {
    const auto output = sender.pair.dsp.bus().gpio(DspPort::e).outputs();
    constexpr unsigned irq = 0x200;
    const bool known = (output.mask & output.known & irq) != 0;
    if (known) {
      sample.known |= 1U;
      if ((output.levels & irq) != 0) { sample.levels |= 1U; }
    }
    ++sender.irq_queries;
    sender.irq_unknown += static_cast<unsigned>(!known);
  }
  return sample;
}

void Chain::connect() {
  auto &pair = sender.pair;
  pair.mcu.bus().attach_gpio_pad_source(
      McuPort::e, [this](const std::uint32_t mask) { return irq_pads(mask); });
  pair.mcu.bus().attach_gpio_pad_source(
      McuPort::b, [this](const std::uint32_t mask) { return peer->pads(McuPort::b, mask); });
  pair.additional_bus_write = [this](const BusEvent &event) {
    display->observe(event);
    peer->observe(event);
  };
  pair.mcu.set_cycle_observer([this](const Hc12BusCycle &cycle) {
    peer->advance(cycle);
    sender.pair.on_cycle(cycle);
  });
  peer->received_byte = [this](const std::uint8_t value) { received(value); };
}

void Chain::received(const std::uint8_t value) {
  require(cursor < expected.size() && value == expected.at(cursor),
          "Native controller GPIO byte differs at original frame index=" + std::to_string(cursor));
  ++cursor;
}

void Chain::prefix() {
  const auto order = scenario.order;
  const auto phase = scenario.phase;
  auto &pair = sender.pair;
  // Checkpoint copies retain the obligation but drop callbacks into the old board.
  // Rebind the same unchanged native MCU state with a composed IRQ/reset peer.
  const auto registers = fw::registers(pair.mcu.cpu().state());
  const auto ticks = pair.mcu.run_clock().elapsed_ticks();
  const auto saved = pair.mcu.checkpoint();
  const auto lcd = *display;
  pair.mcu.restore(saved);
  display = std::make_unique<LcdFixture>(pair.mcu.bus(), lcd);
  pair.connect_mcu_pads();
  require(fw::registers(pair.mcu.cpu().state()) == registers &&
              pair.mcu.run_clock().elapsed_ticks() == ticks,
          "Board observer composition preserves every native MCU register and clock");
  pair.start(order == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp,
             phase,
             kh::prefix_end);
  connect();
  while (pair.mcu.cpu().state().pc != fw::target) {
    require(pair.link.msteps < kh::instruction_limit,
            "Native original controller-copy/LCD prefix bound");
    sender.step();
  }
  const auto entry = pair.mcu.cpu().state();
  trace << "COPY_BOUNDARY pc=" << entry.pc << " d=" << entry.d << " x=" << entry.x
        << " y=" << entry.y << " sp=" << entry.sp << " ccr=" << static_cast<unsigned>(entry.ccr)
        << " irq-entries=" << sender.irq_entries << " irq-returns=" << sender.irq_returns
        << " sent=" << sender.sent << " received=" << sender.received << " loaded=" << sender.loaded
        << '\n';
  sender.entry();
  const auto copied = pair.link.msteps;
  while (pair.mcu.cpu().state().pc != kh::prefix_end) {
    require(pair.link.msteps - copied < kh::helper_steps,
            "Original helper checksum receiver bound");
    sender.step();
  }
  require(pair.link.msteps - copied == kh::helper_steps && sender.returned == 3 &&
              sender.initial_reads == 4 && sender.sent == kh::complete_samples &&
              sender.loaded == kh::checksum_word + 2,
          "Unchanged original loader/checksum prefix retains the real helper frame");
  trace << "PREFIX mcu=" << pair.mcu.cpu().state().pc << " sp=" << pair.mcu.cpu().state().sp
        << " sent=" << sender.sent << " loaded=" << sender.loaded << " steps=" << pair.link.msteps
        << '\n';
}
}
