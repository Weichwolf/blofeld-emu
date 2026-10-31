// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/mcu_dsp_runtime.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
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
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace blofeld::test::session {
inline constexpr unsigned oscillator = 12000000, common_hz = 540000000;
inline constexpr unsigned entry = 0x9000, byte_mask = 0xff, native_budget = 50000;
inline constexpr unsigned lds = 0xcf, immediate = 0x86, store = 0x7a, load = 0xb6, wai = 0x3e;
inline constexpr unsigned bit_test = 0x85, equal_branch = 0x27, branch = 0x20;
inline constexpr unsigned divisor = 12, ready_mask = 0x20;

inline void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

inline std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw std::runtime_error("Expected host MIDI session refusal");
}

inline reference::Config config(const bool small = false) {
  reference::Config c;
  c.mcu = {.clock = {.oscillator_hz = oscillator,
                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
           .memory_timing = McuRunMemory::unstretched_internal};
  c.dsp_clock = {.extal_hz = oscillator,
                 .timebase_hz = common_hz,
                 .switch_timing = blofeld::DspClockSwitch::reference_cycle_boundary};
  c.dsp = {
      .timing =
          blofeld::OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
      .maximum_cycles_per_period = 16,
      .writes = blofeld::McuWriteOrder::before_mcu,
      .external_writes = blofeld::McuWriteOrder::before_mcu,
      .wait_dsp_first = true,
      .reset_dsp_first = true,
      .initial_reset = blofeld::OwnDspResetInputs{
          .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  c.dma_grant = blofeld::DspDmaGrant::unknown;
  c.power_on = reference::PowerOnTiming{.inputs = {3}, .reset_pad = blofeld::McuResetPad::high};
  c.release_ticks = std::numeric_limits<std::uint64_t>::max();
  c.operations = native_budget;
  c.midi_sci_timed =
      reference::SciMidiRoute{.pin = small ? SciReceivePin::ps4 : SciReceivePin::pr2};
  c.midi_reference = SciReceiveReference{.baud_divisor = divisor};
  c.initial = {{.port = McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  c.dsp_input_order = DspPeerOrder::peer_first;
  c.maximum_dsp_inputs_per_advance = 4;
  return c;
}

struct Program {
  explicit Program(const bool small = false) {
    write(hc12::modrr, small ? 1 : 0);
    write(hc12::sci_baud_low, divisor);
    write(hc12::sci_control2, 4);
    constexpr unsigned retry_displacement = 0xf9;
    constexpr unsigned loop_displacement = 0xf4;
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
    code.insert(code.end(), {branch, loop_displacement});
  }

  void write(const unsigned address, const unsigned value) {
    code.insert(code.end(),
                {immediate,
                 static_cast<std::uint8_t>(value),
                 store,
                 static_cast<std::uint8_t>(address >> 8U),
                 static_cast<std::uint8_t>(address & byte_mask)});
  }

  std::vector<std::uint8_t> code{lds, 0x20, 0};
  unsigned received_pc = 0;
};

inline McuFlashImage image(const std::vector<std::uint8_t> &code) {
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  constexpr unsigned flash_base = 0x8000;
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = entry & byte_mask;
  for (unsigned i = 0; i < code.size(); ++i) { bytes.at(entry - flash_base + i) = code.at(i); }
  return McuFlashImage::from_known(bytes);
}

struct Fixture {
  explicit Fixture(const reference::Config &c, const std::vector<std::uint8_t> &code)
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
}
