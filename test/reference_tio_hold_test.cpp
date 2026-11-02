// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/reference_board.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "esai_fixture.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"
#include "storage/nand.hpp"
#include "storage/nand_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
constexpr unsigned oscillator = 12000000, timebase = 1080000000;
constexpr unsigned entry = 0x9000, nop = 0xa7, byte_mask = 0xff;
constexpr unsigned protection = 0x7f0d, security = 0x7f0f, unsecured = 0xfe;
constexpr unsigned budget = 480, suffix = 240, release_tick = 900, edge_tick = 24000;
constexpr unsigned control0 = 0xffff8f, control1 = 0xffff8b;
constexpr unsigned load0 = 0xffff8e, load1 = 0xffff8a;
constexpr unsigned capture_enable = 0x61, inverse = 0x100, data_input = 0x1000;
constexpr unsigned captured = 0x200000, word_bits = 24;

McuFlashImage image() {
  // Original-style reset control: real MCU stores assert/release U0 twice.
  constexpr unsigned load_a = 0x86;
  constexpr unsigned store_a = 0x7a;
  constexpr unsigned direction_u = hc12::ptu + 2;
  std::vector<std::uint8_t> code;
  const auto store = [&](const unsigned address, const std::uint8_t value) {
    code.insert(code.end(),
                {load_a,
                 value,
                 store_a,
                 static_cast<std::uint8_t>(address >> 8U),
                 static_cast<std::uint8_t>(address & byte_mask)});
  };
  store(hc12::ptu, 0);
  store(direction_u, 1);
  code.insert(code.end(), 8, nop);
  store(hc12::ptu, 1);
  constexpr unsigned running_delay = 32;
  code.insert(code.end(), running_delay, nop);
  store(hc12::ptu, 0);
  code.insert(code.end(), 8, nop);
  store(hc12::ptu, 1);
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  constexpr unsigned flash_base = 0x8000;
  for (std::size_t index = 0; index < code.size(); ++index) {
    bytes.at(entry - flash_base + index) = code[index];
  }
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

std::vector<std::uint32_t> program(const bool initial) {
  // FM MOVEP #immediate,X:qq and MOVE X:ea,X0/X1 after each real pin reset.
  constexpr unsigned movep = 0x07f400;
  constexpr unsigned low_x = 0xffff80;
  constexpr unsigned read_x0 = 0x44f000;
  constexpr unsigned read_x1 = 0x45f000;
  std::vector<std::uint32_t> words;
  const auto write = [&](const unsigned address, const unsigned value) {
    words.insert(words.end(), {movep | (address - low_x), value});
  };
  const auto setup = [&] {
    write(load0, 0);
    write(load1, 0);
    write(control0, capture_enable | (initial ? inverse : 0U));
    write(control1, capture_enable);
  };
  setup();
  constexpr unsigned sample_delay = 64;
  constexpr unsigned rom_words = 2048;
  words.insert(words.end(), sample_delay, 0);
  words.insert(words.end(), {read_x0, control0, read_x1, control1});
  words.resize(rom_words);
  return words;
}

reference::Config config(const bool initial) {
  reference::Config c;
  c.mcu = {.clock = {.oscillator_hz = oscillator,
                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
           .memory_timing = McuRunMemory::unstretched_internal};
  c.dsp_clock = {.extal_hz = oscillator,
                 .timebase_hz = timebase,
                 .switch_timing = DspClockSwitch::reference_cycle_boundary};
  c.dsp = {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
           .maximum_cycles_per_period = 16,
           .writes = McuWriteOrder::before_mcu,
           .external_writes = McuWriteOrder::before_mcu,
           .initial_reset = OwnDspResetInputs{
               .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  c.dma_grant = DspDmaGrant::available;
  c.mcu_u0_release = true;
  c.operations = budget;
  c.select_n = 2;
  c.clock = 4;
  c.mosi = 16;
  constexpr unsigned serial = 0x16;
  c.initial = {{.port = McuPort::b, .drive = {.mask = serial, .levels = 2}},
               {.port = McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  c.dsp_input_order = DspPeerOrder::peer_first;
  c.maximum_dsp_inputs_per_advance = 4;
  c.dsp_initial = {{.index = 0, .level = initial, .synchronized = true},
                   {.index = 1, .level = !initial, .synchronized = true}};
  c.dsp_events = {{.ticks = edge_tick, .index = 0, .level = !initial, .synchronized = true}};
  return c;
}

struct Fixture {
  Fixture(const reference::Config &c, const bool initial)
      : flash(image()),
        rom(program(initial)),
        execution{.image = flash,
                  .rom = rom,
                  .specification = BLOFELD_DEFAULT_SLA,
                  .flash_out = {},
                  .trace = &trace,
                  .instructions = budget},
        board(c, execution) {}

  McuFlashImage flash;
  std::vector<std::uint32_t> rom;
  std::ostringstream trace;
  reference::Execution execution;
  reference::Board board;
};

void steps(reference::Board &board, const unsigned count) {
  for (unsigned index = 0; index < count; ++index) { board.advance(); }
}

std::uint64_t expected_registers(const bool initial, const bool edge) {
  // INV applies to DI readback as well as capture polarity. Both selected
  // physical edge directions therefore leave logical DI high after capture.
  const auto x0 = capture_enable | (initial ? inverse : 0U) | (edge ? (data_input | captured) : 0U);
  const auto x1 = capture_enable | (!initial ? data_input : 0U);
  return x0 | (std::uint64_t{x1} << word_bits);
}

void reset_and_forks(const bool initial) {
  constexpr std::array cuts{0U, 12U, 14U, 48U, 52U, 58U, 80U, 160U};
  for (const auto cut : cuts) {
    Fixture f(config(initial), initial);
    f.board.initialize();
    steps(f.board, cut);
    const auto saved = f.board.checkpoint();
    const auto prefix = f.board.status();
    f.trace.str({});
    f.trace.clear();
    steps(f.board, suffix);
    const auto expected = f.board.status();
    const auto trace = f.trace.str();
    require(expected.dsp.x == expected_registers(initial, true) && !expected.faulted,
            "MCU pin reset preserves external source history; only the actual later edge captures");
    for (unsigned replay = 0; replay < 2; ++replay) {
      f.board.restore(saved);
      require(f.board.status() == prefix, "Restored native owners retain the exact prefix");
      f.trace.str({});
      f.trace.clear();
      steps(f.board, suffix);
      require(f.board.status() == expected && f.trace.str() == trace,
              "Source level, qualification and event cursor restore together across MCU-driven DSP "
              "reset");
    }
  }
}

void midi_idle() {
  auto c = config(true);
  c.dsp_initial.erase(c.dsp_initial.begin());
  c.dsp_events.clear();
  c.midi_dsp =
      reference::DspMidiRoute{.after_ticks = release_tick, .index = 0, .synchronized = true};
  Fixture f(c, true);
  steps(f.board, suffix);
  const auto actual = f.board.status();
  // The source samples idle on the free-running 32us MIDI bit grid. A reset
  // must preserve that grid as well as the held level; it creates no frame.
  constexpr unsigned midi_bit_ticks = timebase / 31250;
  const auto idle_observations = 1 + ((actual.common_ticks - release_tick) / midi_bit_ticks);
  require(actual.dsp.x == expected_registers(true, false) &&
              actual.midi_dsp_events == idle_observations && actual.midi_dsp_frames == 0,
          "A real idle MIDI source survives MCU-driven DSP reset without inventing a frame or "
          "capture edge");
}

void nand_ready_ownership() {
  auto c = config(true);
  c.dsp_initial.clear();
  c.dsp_events.clear();
  c.midi_dsp =
      reference::DspMidiRoute{.after_ticks = release_tick, .index = 1, .synchronized = true};
  c.nand = reference::NandConnection{.image = NandImage{},
                                     .continuation = NandReadContinuation::k9f1208_j};
  require(rejects([&] { const Fixture f(c, true); }).contains("NAND Ready owns DSP timer1"),
          "An implicit MIDI source cannot replace the NAND Ready owner on TIO1");
}

void raw_or_unknown_inputs() {
  for (const auto level : {std::optional<bool>{true}, std::optional<bool>{}}) {
    auto c = config(true);
    c.dsp_initial.front().level = level;
    c.dsp_initial.front().synchronized = false;
    c.dsp_events.clear();
    Fixture f(c, true);
    const auto error = rejects([&] { steps(f.board, suffix); });
    require(error.contains("Unknown DSP timer capture input") && f.board.status().faulted,
            "Unknown and raw-only sources retain the existing synchronizer qualification guard");
  }
}
}

int main() {
  try {
    reset_and_forks(false);
    reset_and_forks(true);
    midi_idle();
    raw_or_unknown_inputs();
    nand_ready_ownership();
    std::cout
        << "Held TIO/MIDI source, MCU-driven DSP reset and coordinated restore checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
