// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/gpio_pads.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "original_update_forward_fixture.hpp"
#include "original_update_transport_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <ostream>
#include <string_view>
#include <vector>

namespace blofeld::test::keyboard_handoff {
namespace gp = blofeld::test::gpio_pair;
namespace fw = blofeld::test::update_forward;
namespace transport = blofeld::test::update_transport;
using blofeld::test::require;

constexpr unsigned caller = 0x2000, source = 0xef00, helper_bytes = 1018;
constexpr unsigned resident_samples = 1360, complete_samples = 1364;
constexpr unsigned source_load = 0x2e18, next_chunk = 0x2e1b, prefix_end = 0x121a;
constexpr unsigned checksum_word = 340, entry_sp = 0x21f4, cache = entry_sp - 8;
constexpr unsigned instruction_limit = 200000, alignment_limit = 1000000;
constexpr unsigned data_mask = 63, byte_mask = 255, upper_ccr = 0xf0;
constexpr unsigned entry_ccr = 0x90, helper_steps = 95;
// Resident LEAY -1,X precedes TBNE of the old zero X at the final count test.
constexpr unsigned terminal_counter = 0xffff;
constexpr unsigned second_call = 0x1213, third_call = 0x1217;
constexpr std::array<unsigned, 4> chunk_pcs{0x2e1d, 0x2e22, 0x2e27, 0x2e2c};
constexpr std::array<unsigned, 4> sample_pcs{0x14fb, 0x1507, 0x1520, 0x1537};
constexpr std::array<unsigned, 3> returns{0x1211, 0x1216, prefix_end};
constexpr std::array layouts{
    transport::Layout{.mode = 3, .address = source, .declared = 42, .stored = 42, .packets = 6},
    transport::Layout{.mode = 0, .address = 0x2e00, .declared = 63, .stored = 63, .packets = 9},
    transport::Layout{
        .mode = 3, .address = source, .declared = 1542, .stored = 1542, .packets = 221},
    transport::Layout{.mode = 0, .address = 0x2e00, .declared = 57, .stored = 58, .packets = 9}};

enum class EntryCcr : std::uint8_t { setup = 0x90, cleared_lcd = 0xb0 };

struct Fixture {
  gp::OriginalGpioPair pair;
  const std::vector<transport::Block> &blocks;
  const EntryCcr required_ccr;
  std::optional<CpuState> interrupted;
  unsigned loaded = 0, sent = 0, received = 0, initial_reads = 0, returned = 0;
  unsigned irq_entries = 0, irq_returns = 0, irq_queries = 0, irq_unknown = 0;

  Fixture(const Firmware &firmware,
          const std::vector<transport::Block> &input,
          const transport::Templates &templates,
          std::ostream &trace,
          const unsigned phase,
          const EntryCcr expected_upper_ccr = EntryCcr::setup)
      : pair(firmware, gp::cases.back(), trace), blocks(input), required_ccr(expected_upper_ccr) {
    const auto epoch = pair.dsp.run_clock().native_epoch();
    const auto before = pair.dsp.cpu().debug_state().native_instructions;
    trace << "PRELOAD block=2\n";
    transport::Transfer data(firmware,
                             blocks.at(2),
                             layouts.at(2),
                             templates,
                             trace,
                             {.owner = &pair.dsp, .from = pair.foreground});
    data.run();
    trace << "PRELOAD block=3\n";
    transport::Transfer program(firmware,
                                blocks.at(3),
                                layouts.at(3),
                                templates,
                                trace,
                                {.owner = &pair.dsp, .reuse = true});
    program.run();
    require(pair.dsp.run_clock().native_epoch() == epoch &&
                pair.dsp.cpu().state().pc == fw::update_entry && pair.dsp.cpu().state().sp == 1 &&
                pair.dsp.cpu().debug_state().native_instructions - before ==
                    data.steps + program.steps,
            "Both original downloads retain their real launch frame and one native DSP epoch");
    trace << "PRELOAD_END pc=" << pair.dsp.cpu().state().pc << " epoch=" << epoch
          << " steps=" << data.steps + program.steps << '\n';
    align(phase);
    connect();
    pair.additional_completion.observe = [this](const DspScheduledStep &step) { observe(step); };
  }

  void align(const unsigned phase) {
    const auto original = pair.mcu.cpu().state();
    // Use the own BRA loop installed by OriginalGpioPair's setup, before copying.
    pair.mcu.cpu().set_state(
        {.pc = gp::alignment_caller, .sp = gp::stack, .ccr = gp::alignment_ccr});
    const auto ratio = gp::common_hz / pair.mcu.run_clock().timebase_hz();
    require(gp::common_hz % pair.mcu.run_clock().timebase_hz() == 0,
            "Explicit prefix clock conversion must be integral");
    unsigned count = 0;
    while (pair.mcu.run_clock().elapsed_ticks() * ratio <
           pair.dsp.run_clock().elapsed_ticks() + phase) {
      require(++count < alignment_limit, "Own native prefix must reach the declared phase horizon");
      pair.mcu.cpu().step();
    }
    pair.mcu.cpu().set_state(original);
    pair.sync();
  }

  void connect() {
    // Declared pre-existing IRQ output state, not a guessed board reset default.
    pair.dsp.bus()
        .gpio(DspPort::e)
        .write({.reg = DspGpioRegister::direction,
                .value = fw::port_e_strobe,
                .mask = fw::port_e_strobe});
    pair.dsp.bus()
        .gpio(DspPort::e)
        .write(
            {.reg = DspGpioRegister::data, .value = fw::port_e_strobe, .mask = fw::port_e_strobe});
    // Own request loop is outside the complete original helper's 1200..15f9 copy.
    for (unsigned n = 0; n < fw::request_caller.size(); ++n) {
      pair.mcu.bus().write(static_cast<std::uint16_t>(caller + n), fw::request_caller[n]);
    }
    pair.mcu.bus().write(hc12::ddrq, fw::handshake_strobe);
    pair.mcu.bus().write(hc12::ptq, fw::handshake_strobe);
    auto state = pair.mcu.cpu().state();
    state.pc = caller;
    state.ccr = fw::foreground_ccr;
    pair.mcu.cpu().set_state(state);
    pair.mcu.bus().set_pin_drive(McuPort::e, {});
    pair.mcu.bus().attach_gpio_pad_source(McuPort::e, [this](const std::uint32_t bits) {
      const auto remote = pair.dsp.bus().gpio(DspPort::e).outputs();
      const auto local = pair.mcu.bus().gpio_outputs(McuPort::e);
      require((local.mask & bits & 3U) == 0, "Declared IRQ route requires actual MCU inputs");
      const bool known = (remote.mask & remote.known & fw::port_e_strobe) != 0;
      const auto irq_known = 2U | (known ? 1U : 0U);
      const auto irq_levels = 2U | ((remote.levels & fw::port_e_strobe) != 0 ? 1U : 0U);
      const auto known_bits = (irq_known | (local.known & ~3U)) & bits;
      if ((bits & 3U) != 0) {
        ++irq_queries;
        irq_unknown += static_cast<unsigned>((irq_known & bits) != (bits & 3U));
      }
      return GpioPadSample{.known = known_bits,
                           .levels = (irq_levels | (local.levels & ~3U)) & known_bits};
    });
    pair.sync();
  }

  [[nodiscard]] unsigned chunk(const unsigned index) const {
    return (blocks.at(2).expected.at(index / 4) >> (fw::first_chunk_shift - (6U * (index % 4)))) &
           data_mask;
  }

  void observe(const DspScheduledStep &step) {
    pair.mcu.bus().observe_interrupt_pads();
    const auto before = step.plan.before.cpu;
    const auto after = pair.dsp.cpu().state();
    if (before.pc == source_load) {
      const auto value = (after.a >> gp::word_bits) & gp::word_mask;
      require(before.r[1] == source + loaded && after.r[1] == source + loaded + 1 &&
                  value == blocks.at(2).expected.at(loaded) && after.pc == source_load + 1,
              "Actual sender source load, register and cursor differ from complete original words");
      pair.trace << "SOURCE index=" << loaded << " address=" << source + loaded
                 << " value=" << value << '\n';
      ++loaded;
    }
    if (std::ranges::find(chunk_pcs, before.pc) == chunk_pcs.end()) { return; }
    const auto value = (before.a >> fw::accumulator_extension_shift) & byte_mask;
    const auto output = pair.dsp.bus().gpio(DspPort::c).outputs();
    require(value == chunk(sent) &&
                (output.mask & output.known & fw::data_outputs) == fw::data_outputs &&
                (output.levels & fw::data_outputs) == value && after.pc == before.pc + 1,
            "Original MOVEP result must equal the independent next six-bit group");
    pair.trace << "CHUNK index=" << sent << " value=" << value << " pc=" << before.pc << '\n';
    ++sent;
  }

  void step() {
    const auto before = pair.mcu.cpu().state();
    if (before.pc == second_call || before.pc == third_call) {
      require(before.d == cache, "Original argument reload discards the prior checksum byte");
    }
    pair.trace << "M " << pair.mcu.run_clock().elapsed_ticks() << ' ' << hex(before.pc) << '\n';
    const auto result = pair.mcu.cpu().step();
    ++pair.link.msteps;
    pair.schedule->end_instruction();
    const auto after = pair.mcu.cpu().state();
    if (result.interrupt_vector) {
      require(result.interrupt_vector == fw::xirq_vector && !result.instruction_executed &&
                  !result.stack_reused && after.pc == fw::request_irq &&
                  after.sp == before.sp - fw::interrupt_frame_bytes,
              "Native XIRQ must use the original vector and actual nine-byte stack frame");
      interrupted = before;
      ++irq_entries;
    }
    if (before.pc == fw::request_rti) {
      if (!interrupted.has_value()) { throw Fault("Original XIRQ return lacks its entry frame"); }
      require(fw::registers(after) == fw::registers(interrupted.value()),
              "Original RTI restores every foreground register before native copy entry");
      ++irq_returns;
    }
    if (before.pc == fw::receiver_sample) {
      require((after.d & data_mask) == chunk(received),
              "Resident native receiver samples the actual next DSP-emitted chunk");
      pair.trace << "CONSUMER index=" << received << " value=" << (after.d & data_mask) << '\n';
      ++received;
    }
    if (std::ranges::find(sample_pcs, before.pc) != sample_pcs.end()) {
      require(initial_reads < sample_pcs.size() && before.pc == sample_pcs.at(initial_reads),
              "Original helper reads its four chunks in their original instruction order");
      const auto value =
          initial_reads == 0 || initial_reads == 3 ? after.d >> 8U : after.d & byte_mask;
      const auto expected =
          chunk(resident_samples + initial_reads) | (initial_reads % 2 == 0 ? 128U : 0U);
      require(value == expected, "Actual helper LDAA/LDAB must sample authentic checksum pins");
      pair.trace << "HELPER_SAMPLE index=" << initial_reads << " value=" << value << '\n';
      ++initial_reads;
    }
    if (returned < returns.size() && after.pc == returns.at(returned)) { returned_byte(); }
  }

  void returned_byte() {
    const auto state = pair.mcu.cpu().state();
    const auto expected =
        (blocks.at(2).expected.at(checksum_word) >> (16U - 8U * returned)) & byte_mask;
    require((state.d & byte_mask) == expected && state.y == terminal_counter &&
                state.sp == entry_sp - (returned < 2 ? 15U : 13U) &&
                state.x == cache + (returned == 2 ? 1U : 0U) &&
                (state.ccr & upper_ccr) == static_cast<unsigned>(required_ccr) &&
                pair.mcu.bus().read(cache + 2) == returned &&
                pair.mcu.bus().read(cache) ==
                    ((blocks.at(2).expected.at(checksum_word) >> 8U) & byte_mask) &&
                pair.mcu.bus().read(cache + 1) ==
                    (blocks.at(2).expected.at(checksum_word) & byte_mask),
            "Original checksum byte, cache cursor, stack and upper CCR differ");
    pair.trace << "BYTE index=" << returned << " value=" << expected << " sp=" << state.sp
               << " x=" << state.x << '\n';
    ++returned;
  }

  void entry() {
    const auto state = pair.mcu.cpu().state();
    require(state.pc == fw::target && state.d == fw::service_table && state.sp == entry_sp &&
                state.y == terminal_counter &&
                (state.ccr & upper_ccr) == static_cast<unsigned>(required_ccr) &&
                irq_entries == 1 && irq_returns == 1 && sent == resident_samples &&
                received == resident_samples && loaded == checksum_word + 1 &&
                pair.mcu.bus().read_memory({.address = entry_sp, .size = 2}) == fw::helper_return,
            "Actual original copy/JSR must supply the complete helper and callback frame");
    verify_copy();
    pair.trace << "ENTRY pc=" << state.pc << " d=" << state.d << " sp=" << state.sp
               << " chunks=" << sent << " samples=" << received << " loaded=" << loaded << '\n';
  }

  void verify_copy() {
    const auto &words = blocks.at(2).expected;
    for (unsigned n = 0; n < helper_bytes + 32; ++n) {
      const auto offset = n + 2;
      const auto expected = n < helper_bytes
                                ? (words.at(offset / 3) >> (16U - 8U * (offset % 3))) & byte_mask
                                : gp::ram_canary;
      require(pair.mcu.bus().read(static_cast<std::uint16_t>(fw::target + n)) == expected,
              "Native copy delivers exactly 1018 original bytes without overwriting its RAM tail");
    }
  }

  void execute(const unsigned order, const unsigned phase) {
    pair.start(order == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp,
               phase,
               prefix_end);
    while (pair.mcu.cpu().state().pc != fw::target) {
      require(pair.link.msteps < instruction_limit, "Original coupled helper copy bound");
      step();
    }
    entry();
    const auto copied = pair.link.msteps;
    while (pair.mcu.cpu().state().pc != prefix_end) {
      require(pair.link.msteps - copied < helper_steps, "Original helper initial receiver bound");
      step();
    }
    require(pair.link.msteps - copied == helper_steps && sent == complete_samples &&
                received == resident_samples && initial_reads == 4 && returned == 3 &&
                loaded == checksum_word + 2 && pair.dsp.cpu().state().pc == next_chunk,
            "Real copy handoff must leave the authentic checksum word for the helper receiver");
  }
};
} // namespace blofeld::test::keyboard_handoff
