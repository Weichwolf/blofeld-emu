// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_midi.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu/mcu_sci.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned oscillator = 12000000, common_hz = 540000000;
constexpr unsigned entry = 0x9000, byte_mask = 0xff, protection = 0x7f0d, security = 0x7f0f;
constexpr unsigned unsecured = 0xfe, nop = 0xa7, wai = 0x3e;
constexpr unsigned lds = 0xcf, immediate = 0x86, store = 0x7a, load = 0xb6;
constexpr unsigned bit_test = 0x85, equal_branch = 0x27, branch = 0x20;
constexpr unsigned baud_divisor = 12, receive_enable = 4, ready_mask = 0x20;
constexpr unsigned reset_edges = 195, native_budget = 400000;
constexpr unsigned replay_suffix = 4000, initial_suffix = 80, reset_prefix = 100;
constexpr unsigned slow_suffix = 400, fixed_bus_hz = 6000000;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

McuFlashImage image(const std::vector<std::uint8_t> &code) {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = entry & byte_mask;
  // Entry 9000 maps to internal flash offset 1000 in the unpaged window.
  constexpr unsigned flash_base = 0x8000;
  for (unsigned i = 0; i < code.size(); ++i) { bytes.at(entry - flash_base + i) = code.at(i); }
  return McuFlashImage::from_known(bytes);
}

reference::Config config(const bool small = false, const unsigned rate = oscillator) {
  reference::Config c;
  c.mcu = {.clock = {.oscillator_hz = rate,
                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
           .memory_timing = McuRunMemory::unstretched_internal};
  c.pipeline = reference::PipelineTiming{
      .power_on = {3},
      .reset_pad = McuResetPad::high,
      .schedule = {.dsp = {.extal_hz = oscillator,
                           .timebase_hz = common_hz,
                           .timing = DspFetchTiming::reference_cycle_order,
                           .execution = DspReferenceExecution::staged_register_boundaries,
                           .pipeline = DspFetchPipeline::automatic_memory,
                           .resets = DspFetchResetTiming::reference_cycle_boundary},
                   .shared_write_order = McuDspTieOrder::dsp_before_mcu,
                   .maximum_dsp_cycles_per_period = 16,
                   .external_write_order = McuExternalWriteOrder::external_before_mcu,
                   .wait_order = McuDspWaitOrder::dsp_before_mcu,
                   .reset_order = McuDspResetOrder::dsp_before_mcu,
                   .initial_reset = McuDspFetchInitialReset{
                       .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}}};
  c.operations = native_budget;
  c.release_ticks = std::numeric_limits<std::uint64_t>::max();
  c.midi_sci_timed =
      reference::SciMidiRoute{.pin = small ? SciReceivePin::ps4 : SciReceivePin::pr2};
  c.initial = {{.port = McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  c.dsp_input_order = DspExternalOrder::external_before_dsp;
  c.maximum_dsp_inputs_per_advance = 4;
  return c;
}

struct Fixture {
  explicit Fixture(const reference::Config &c,
                   const std::vector<std::uint8_t> &code = {lds, 0x20, 0, wai})
      : flash(image(code)),
        execution{.image = flash,
                  .rom = rom,
                  .specification = BLOFELD_DEFAULT_SLA,
                  .flash_out = {},
                  .trace = &trace,
                  .instructions = native_budget},
        board(c, execution) {}

  McuFlashImage flash;
  std::vector<std::uint32_t> rom;
  std::ostringstream trace;
  reference::Execution execution;
  reference::Board board;
};

void advance(reference::Board &board, const unsigned count) {
  for (unsigned i = 0; i < count; ++i) { board.advance(); }
}

void idle_por_and_exact_copy() {
  Fixture f(config());
  require(f.board.status().mcu_cycles == 0 && f.board.status().midi_sci_events == 1,
          "Time-zero idle wire event never invents an IPBus cycle during held POR");
  f.board.initialize();
  constexpr std::array<std::uint8_t, 2> bytes{0x90, 0x40};
  f.board.enqueue_midi(bytes);
  advance(f.board, reset_edges + initial_suffix);
  const auto prefix = f.board.status();
  const auto saved = f.board.checkpoint();
  f.trace.str({});
  f.trace.clear();
  advance(f.board, replay_suffix);
  const auto expected = f.board.status();
  const auto trace = f.trace.str();
  require(expected.midi_sci_frames == 2 && expected.midi_queued == 0 && expected.waiting,
          "Original WAI keeps common wire time and finishes complete frames without a receiver");
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.board.restore(saved);
    require(f.board.status() == prefix, "Coordinated restore retains partial timed SCI source");
    f.trace.str({});
    f.trace.clear();
    advance(f.board, replay_suffix);
    require(f.board.status() == expected && f.trace.str() == trace,
            "Restored MCU/queue/partial bit/event cursor reproduce the exact suffix");
  }
}

void wire_during_held_reset() {
  constexpr unsigned slow_oscillator = 1000000;
  Fixture f(config(false, slow_oscillator));
  f.board.initialize();
  constexpr std::array<std::uint8_t, 1> bytes{0xa5};
  f.board.enqueue_midi(bytes);
  advance(f.board, reset_prefix);
  const auto state = f.board.status();
  require(!state.mcu_state_known && state.mcu_cycles == 0 && state.midi_sci_events == 4 &&
              state.midi_sci_frames == 0 && state.midi_queued == 1,
          "Three actual serial bits elapse inside held POR without normal MCU clocks");
  constexpr std::array<const char *, 3> expected{
      "SCI-MIDI-BIT requested=17280 actual=17280 level=0 bit=0 held-reset=1",
      "SCI-MIDI-BIT requested=34560 actual=34560 level=1 bit=1 held-reset=1",
      "SCI-MIDI-BIT requested=51840 actual=51840 level=0 bit=2 held-reset=1"};
  for (const auto *line : expected) {
    require(f.trace.str().contains(line), "Independent literal A5 wire deadlines inside reset");
  }
  const auto saved = f.board.checkpoint();
  advance(f.board, slow_suffix);
  const auto end = f.board.status();
  require(end.midi_sci_frames == 1 && end.midi_queued == 0 && end.waiting,
          "Reset release preserves in-flight serial time rather than restarting the byte");
  f.board.restore(saved);
  advance(f.board, slow_suffix);
  require(f.board.status() == end,
          "Held-reset restore preserves wire phase through native release");
}

struct Program {
  std::vector<std::uint8_t> code{lds, 0x20, 0};
  unsigned received_pc = 0;

  void write(const unsigned address, const unsigned value) {
    code.insert(code.end(),
                {immediate,
                 static_cast<std::uint8_t>(value),
                 store,
                 static_cast<std::uint8_t>(address >> 8U),
                 static_cast<std::uint8_t>(address & byte_mask)});
  }

  explicit Program(const bool small, const unsigned divisor = baud_divisor) {
    write(hc12::modrr, small ? 1 : 0);
    write(hc12::refdv, divisor == baud_divisor ? 0 : 1);
    write(hc12::sci_baud_low, divisor);
    write(hc12::sci_control2, receive_enable);
    const auto poll = code.size();
    constexpr unsigned retry_displacement = 0xf9;
    code.insert(code.end(),
                {load,
                 0,
                 static_cast<std::uint8_t>(hc12::sci_status1),
                 bit_test,
                 ready_mask,
                 equal_branch,
                 retry_displacement,
                 load,
                 0,
                 static_cast<std::uint8_t>(hc12::sci_data_low)});
    received_pc = entry + static_cast<unsigned>(code.size());
    const auto back = 256U - static_cast<unsigned>(code.size() + 2 - poll);
    code.insert(code.end(), {branch, static_cast<std::uint8_t>(back)});
  }
};

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw std::runtime_error("Expected timed SCI MIDI rejection");
}

void rejected_contracts() {
  constexpr unsigned cases = 8;
  for (unsigned kind = 0; kind < cases; ++kind) {
    auto c = config();
    switch (kind) {
      case 0: c.pipeline.reset(); break;
      case 1: c.midi = reference::MidiRoute{.bus_hz = fixed_bus_hz}; break;
      case 2: c.midi_dsp = reference::DspMidiRoute{}; break;
      case 3: c.dsp_input_order = DspExternalOrder::unqualified; break;
      case 4: c.maximum_dsp_inputs_per_advance = 0; break;
      case 5:
        c.midi_sci_timed = reference::SciMidiRoute{.pin = static_cast<SciReceivePin>(2)};
        break;
      case 6: c.events = {{.port = McuPort::r, .drive = {.mask = 4, .levels = 4}}}; break;
      case 7: c.dsp_events = {{.ticks = 0, .level = true}}; break;
      default: throw std::runtime_error("Unknown directed rejection case");
    }
    const auto message = rejects([&] { const Fixture f(c); });
    require(!message.empty(), "Unsupported ownership/routes and simultaneous sources fail visibly");
  }
}

void event_budget_and_trusted_restore() {
  constexpr unsigned slow_oscillator = 500000;
  auto c = config(false, slow_oscillator);
  c.maximum_dsp_inputs_per_advance = 1;
  Program program(false);
  program.code = {lds, 0x20, 0};
  constexpr unsigned largest_divider = 15;
  program.write(hc12::refdv, largest_divider);
  program.code.push_back(wai);
  Fixture f(c, program.code);
  f.board.initialize();
  constexpr unsigned queued_frames = 64;
  const std::array<std::uint8_t, queued_frames> bytes{};
  f.board.enqueue_midi(bytes);
  const auto saved = f.board.checkpoint();
  const auto failure = rejects([&] {
    for (unsigned i = 0; i < slow_suffix; ++i) { f.board.advance(); }
  });
  require(f.board.status().faulted && failure.contains("budget"),
          "A real long native period cannot silently drop wire events beyond the declared budget");
  static_cast<void>(rejects([&] { f.board.advance(); }));
  f.board.restore(saved);
  require(
      !f.board.status().faulted && f.board.status().midi_queued == queued_frames,
      "Trusted coordinated restore recovers source and native state after event-budget failure");
}

void unresolved_transfer_still_fails() {
  const Program program(false);
  Fixture f(config(), program.code);
  f.board.initialize();
  constexpr std::array<std::uint8_t, 1> byte{0x90};
  f.board.enqueue_midi(byte);
  const auto failure = rejects([&] {
    for (unsigned i = 0; i < replay_suffix; ++i) { f.board.advance(); }
  });
  require(f.board.status().faulted && failure.contains("Unverified SCI stop-to-data") &&
              failure.contains("0x00cc"),
          "Without an explicit hypothesis the original unresolved hardware phase stays visible");
}

void all_bytes_with_explicit_transfer_reference() {
  constexpr unsigned all_bytes = 256;
  for (const bool small : {false, true}) {
    const unsigned divisor = small ? baud_divisor / 2 : baud_divisor;
    const Program program(small, divisor);
    auto c = config(small);
    c.midi_reference = SciReceiveReference{.baud_divisor = divisor};
    Fixture f(c, program.code);
    f.board.initialize();
    std::array<std::uint8_t, all_bytes> bytes{};
    for (unsigned i = 0; i < all_bytes; ++i) { bytes.at(i) = static_cast<std::uint8_t>(i); }
    f.board.enqueue_midi(bytes);
    unsigned received = 0;
    while (received < all_bytes && !f.board.finished()) {
      f.board.advance();
      const auto state = f.board.status();
      if (state.mcu_state_known && state.mcu.pc == program.received_pc) {
        require(state.mcu.d >> 8U == received,
                "Native polling reads each literal byte under an explicitly declared transfer "
                "hypothesis");
        ++received;
      }
    }
    require(received == all_bytes && !f.board.status().faulted,
            "Both routes receive all 256 bytes under declared transfer timing and native divider "
            "setup");
    std::cout << "Native route " << small << " received " << received << " bytes\n";
  }
}
}

int main() {
  try {
    idle_por_and_exact_copy();
    wire_during_held_reset();
    rejected_contracts();
    event_budget_and_trusted_restore();
    unresolved_transfer_still_fails();
    all_bytes_with_explicit_transfer_reference();
    std::cout << "Timed SCI MIDI POR/WAI/restore, native receive and clock-change checks pass\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
