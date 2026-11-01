// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/reference_board.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "esai_fixture.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
constexpr unsigned oscillator = 12000000, entry = 0x9000, byte_mask = 0xff;
constexpr unsigned nop = 0xa7, protection = 0x7f0d, security = 0x7f0f, unsecured = 0xfe;
constexpr unsigned budget = 256, rom_words = 1024, suffix = 24;
constexpr unsigned u_data = 0x278, u_direction = 0x27a, u_enable = 0x27c, u_select = 0x27d;
constexpr unsigned load_a = 0x86, store_a = 0x7a, read_a = 0xb6;
constexpr unsigned reset_cycles = 5, assert_cycle = 12, release_base = 16;
constexpr unsigned other_u = 0x3e, other_levels = 0x2a, expected_a = 0x2b;
constexpr unsigned serial_pins = 0x16, scheduled_release = 450;
constexpr unsigned power_edges = 195, power_ticks = 9225, reference_bus_ticks = 90;

enum class Mode : std::uint8_t {
  release_input,
  release_high,
  never_assert,
  reset_latch,
  reassert,
  bias
};

void store(std::vector<std::uint8_t> &code, const unsigned address, const std::uint8_t value) {
  code.insert(code.end(),
              {load_a,
               value,
               store_a,
               static_cast<std::uint8_t>(address >> 8U),
               static_cast<std::uint8_t>(address & byte_mask)});
}

McuFlashImage image(const Mode mode, const unsigned delay) {
  std::vector<std::uint8_t> code;
  if (mode == Mode::reset_latch) {
    store(code, u_direction, 1);
  } else if (mode == Mode::bias) {
    store(code, u_select, 1);
    store(code, u_enable, 1);
  } else {
    store(code, u_data, 0);
    if (mode != Mode::never_assert) {
      store(code, u_direction, 1);
      code.insert(code.end(), delay, nop);
      store(code,
            mode == Mode::release_input ? u_direction : u_data,
            mode == Mode::release_input ? 0 : 1);
    }
    if (mode == Mode::reassert) {
      code.insert(code.end(), suffix, nop);
      store(code, u_data, 0);
    }
    code.insert(code.end(),
                {read_a,
                 static_cast<std::uint8_t>(u_data >> 8U),
                 static_cast<std::uint8_t>(u_data & byte_mask)});
  }
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

reference::Config config(const unsigned hz) {
  reference::Config c;
  c.mcu = {.clock = {.oscillator_hz = oscillator,
                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
           .memory_timing = McuRunMemory::unstretched_internal};
  c.dsp_clock = {.extal_hz = oscillator,
                 .timebase_hz = hz,
                 .switch_timing = DspClockSwitch::reference_cycle_boundary};
  c.dsp = {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
           .maximum_cycles_per_period = 16,
           .writes = McuWriteOrder::before_mcu,
           .external_writes = McuWriteOrder::before_mcu,
           .reset_dsp_first = true,
           .initial_reset = OwnDspResetInputs{
               .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  c.dma_grant = DspDmaGrant::available;
  c.mcu_u0_release = true;
  c.operations = budget;
  c.select_n = 2;
  c.clock = 4;
  c.mosi = 16;
  c.initial = {{.port = McuPort::b, .drive = {.mask = serial_pins, .levels = 2}},
               {.port = McuPort::u, .drive = {.mask = other_u, .levels = other_levels}}};
  return c;
}

struct Fixture {
  Fixture(const reference::Config &c, const Mode mode, const unsigned delay)
      : flash(image(mode, delay)),
        rom(rom_words, 0),
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

void cold_release(const unsigned hz, const Mode mode, const unsigned delay) {
  Fixture f(config(hz), mode, delay);
  f.board.initialize();
  const auto period = std::uint64_t{hz} / (oscillator / 2);
  require(f.board.status().held_reset && f.board.status().dsp_cycles == 0 &&
              f.board.status().common_ticks == reset_cycles * period,
          "The normal Board starts held; a passive initial high is not an MCU assertion/release");
  steps(f.board, 2);
  require(f.board.status().held_reset && f.board.status().dsp_steps == 0,
          "Writing zero to an input latch cannot assert or release the DSP");
  steps(f.board, 2);
  const auto asserted = f.board.checkpoint();
  require(f.trace.str().contains("DSP-U0-ASSERT at=0x0278") &&
              f.trace.str().contains("tick=" + std::to_string(assert_cycle * period)),
          "Independent CPU12 instruction periods locate the actual output-low write");
  steps(f.board, delay + 2);
  const auto released = f.board.status();
  const auto at = (release_base + delay) * period;
  require(!released.held_reset && released.dsp_steps == 0 && released.dsp_cycles == 0 &&
              released.common_ticks == at + period &&
              f.trace.str().contains("DSP-RELEASE requested=0 actual=" + std::to_string(at)),
          "The actual MCU write releases reset and completes two EXTAL recovery periods");
  const auto saved = f.board.checkpoint();
  f.trace.str({});
  steps(f.board, suffix);
  const auto expected = f.board.status();
  const auto trace = f.trace.str();
  require(expected.dsp_steps > 0 && (expected.mcu.d >> 8U) == expected_a,
          "Native DSP runs after release; MCU reads the pull-up and unchanged other U inputs");
  f.board.restore(saved);
  f.trace.str({});
  steps(f.board, suffix);
  require(f.board.status() == expected && f.trace.str() == trace,
          "A fresh owner reproduces the released native suffix and functional input projection");
  f.board.restore(asserted);
  steps(f.board, delay + 2);
  require(f.board.status() == released,
          "A held joint checkpoint retains the observed assertion before subsequent release");
}

void power_on_release(const unsigned hz) {
  auto c = config(hz);
  c.power_on = reference::PowerOnTiming{.inputs = {3}, .reset_pad = McuResetPad::high};
  Fixture f(c, Mode::release_input, 0);
  require(f.board.status().common_ticks == 0 && f.board.status().held_reset,
          "Digital power-on starts at tick zero with DSP reset held");
  steps(f.board, power_edges);
  const auto scale = std::uint64_t{hz} / 540000000U;
  const auto before = f.board.status();
  require(before.common_ticks == power_ticks * scale && before.mcu.pc == entry &&
              before.instructions == 0 && before.held_reset && before.dsp_cycles == 0,
          "195 reset/vector periods complete before any original MCU instruction or DSP fetch");
  const auto saved = f.board.checkpoint();
  steps(f.board, 6);
  const auto released = f.board.status();
  const auto at = (power_ticks + (release_base - reset_cycles) * reference_bus_ticks) * scale;
  require(!released.held_reset && released.dsp_steps == 0 &&
              released.common_ticks == at + reference_bus_ticks * scale &&
              f.trace.str().contains("DSP-RELEASE requested=0 actual=" + std::to_string(at)),
          "The real MCU code, rather than the POR timer, releases DSP reset");
  f.board.restore(saved);
  steps(f.board, 6);
  require(f.board.status() == released,
          "The power-on checkpoint reproduces the same MCU-driven reset release");
}

void guards() {
  constexpr unsigned hz = 540000000;
  Fixture held(config(hz), Mode::never_assert, 0);
  steps(held.board, suffix);
  require(held.board.status().held_reset && held.board.status().dsp_cycles == 0 &&
              (held.board.status().mcu.d >> 8U) == expected_a,
          "A continually high input remains readable without inventing an assertion");
  Fixture reset_latch(config(hz), Mode::reset_latch, 0);
  steps(reset_latch.board, 2);
  require(reset_latch.board.status().held_reset &&
              reset_latch.trace.str().contains("DSP-U0-ASSERT at=0x0278"),
          "The defined zero PTU reset latch asserts reset when its direction becomes output");
  for (const auto mode : {Mode::bias, Mode::reassert}) {
    Fixture f(config(hz), mode, 0);
    const auto fault = rejects([&] { steps(f.board, budget); });
    require(f.board.status().faulted && fault.contains("at=0x0278") && fault.contains("mcu-pc=") &&
                fault.contains("dsp-pc=") &&
                fault.contains(mode == Mode::reassert ? "operation=assert" : "operation=read"),
            "Opposing pulls and unsupported reassertion remain explicit");
  }
  for (const unsigned kind : {0U, 1U, 2U, 3U}) {
    auto c = config(hz);
    if (kind == 0) { c.release_ticks = 1; }
    if (kind == 1) { c.initial.push_back({.port = McuPort::u, .drive = {.mask = 1}}); }
    if (kind == 2) { c.hreq = {.destination = McuPort::u, .mask = 1}; }
    if (kind == 3) {
      c.loopback = {
          .source = McuPort::e, .source_mask = 4, .destination = McuPort::u, .destination_mask = 1};
    }
    require(!rejects([&] { const Fixture f(c, Mode::release_input, 0); }).empty(),
            "A reset source cannot silently share its input or a scheduled release");
  }
}

void configuration(const std::filesystem::path &directory) {
  std::filesystem::create_directories(directory);
  const auto path = directory / "reset.board";
  const std::string prefix =
      "blofeld-reference-board 2\nclock 12000000 0 12000000 540000000 reference-boundaries\n"
      "dsp-execution reference-effects\nordering dsp-before-mcu dsp-before-mcu 32\n"
      "spi-b 02 04 10 reject\noperations 4096\ndsp-release ";
  for (const std::string source : {"mcu-u0", "450", "-1", "unknown"}) {
    {
      std::ofstream output(path);
      output.exceptions(std::ios::failbit | std::ios::badbit);
      output << prefix << source << " low\n";
    }
    if (source == "mcu-u0" || source == "450") {
      const auto c = reference::load_config(path);
      require(c.mcu_u0_release == (source == "mcu-u0") &&
                  c.release_ticks == (c.mcu_u0_release ? 0U : scheduled_release),
              "The reset source is explicit; numeric scheduled releases retain their ticks");
    } else {
      require(rejects([&] {
                static_cast<void>(reference::load_config(path));
              }).contains("Invalid reference-board number"),
              "Unknown or negative reset sources fail at configuration loading");
    }
  }
}
}

int main(const int count, char **args) {
  try {
    require(count == 2, "A private test artifact directory is required");
    configuration(args[1]);
    for (const unsigned hz : {540000000U, 1080000000U}) {
      power_on_release(hz);
      for (const auto mode : {Mode::release_input, Mode::release_high}) {
        for (const unsigned delay : {0U, 3U, 8U}) { cold_release(hz, mode, delay); }
      }
    }
    guards();
    std::cout
        << "Native MCU U0 release, independent times, pull-up and fresh-owner replay passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
