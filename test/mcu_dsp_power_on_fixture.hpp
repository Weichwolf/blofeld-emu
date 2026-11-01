// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "mcu/mcu_machine.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::power_on {
inline constexpr unsigned oscillator = 12000000, timebase = 540000000, boot = 0xff0000;
inline constexpr unsigned drive = 128, settle = 64, reset_period = 45, bus_period = 90;
inline constexpr unsigned vector_cycles = 5, nop_words = 512, early_release = 90;
inline constexpr unsigned early_recovery_end = 180;
inline constexpr DspRunClockInputs clocks{.extal_hz = oscillator,
                                          .timebase_hz = timebase,
                                          .switch_timing =
                                              DspClockSwitch::reference_cycle_boundary};

void require(bool condition, const char *message);
McuRunContract contract();
McuDspRuntimeInputs inputs();
DspDmaInputs pins();
dsp56300::Registers nop_registers(std::uint64_t cycles);
dsp56300::Progress nop_progress(std::uint64_t cycles);

struct Observation {
  dsp56300::Registers registers;
  dsp56300::Progress progress;
  std::optional<dsp56300::InstructionEffect> pending;
  std::optional<std::uint32_t> issued;
  std::uint64_t start;
  std::uint64_t completion;
  std::uint64_t clk2;
  bool captured;
  bool retired;
  bool operator==(const Observation &) const = default;
};

struct Fixture {
  Fixture(unsigned synchronization, bool install_rom);
  Fixture(const Fixture &) = delete;
  Fixture &operator=(const Fixture &) = delete;
  ~Fixture();
  DspRuntimeEvents events();
  void attach();
  void record(const DspRuntimeCycle &cycle);
  void restore(const McuDspRuntimeCheckpoint &snapshot, unsigned source_cursor);
  void advance_reset(unsigned edges, McuResetPad pad);
  void step();
  McuMachine mcu;
  OwnDspMachine dsp{clocks};
  std::shared_ptr<const bool> identity = std::make_shared<const bool>(true);
  std::array<std::uint64_t, 2> event_ticks{0, reset_period};
  unsigned cursor = 0;
  std::uint64_t recovery_end = early_recovery_end;
  std::vector<std::uint64_t> observed;
  std::vector<Observation> trace;
  std::string slots;
  std::unique_ptr<McuDspRuntime> runtime;
};

void missing_rom();
void guards();
}
