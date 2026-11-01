// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::mcu_dsp {
inline constexpr unsigned dsp_hz = 12000000, common_hz = 540000000;
inline constexpr unsigned entry = 0x8000, boot = 0xff0000;
inline constexpr unsigned byte_mask = 0xff, nop = 0xa7;
inline constexpr std::uint64_t dsp_period = 45, recovery_ticks = 90;

void require(bool condition, const char *message);
OwnDspResetInputs reset_inputs();
McuDspRuntimeInputs runtime_inputs();
DspDmaInputs cycle_inputs();

inline std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const std::exception &error) { return error.what(); }
  throw Fault("Expected native MCU/DSP control refusal");
}

struct Options {
  std::span<const std::uint8_t> mcu_program;
  std::span<const std::uint32_t> dsp_program;
  unsigned mcu_hz = dsp_hz;
  unsigned mcu_phase = 0;
  std::optional<std::uint16_t> shared_address = std::nullopt;
  McuDspRuntimeInputs runtime = runtime_inputs();
  bool release = false;
};

// Authored MCU/DSP programs, real reset vectors and actual native period hooks.
// No injected CPU registers, fabricated MCU offers or legacy DSP assembler.
// Attach observers after fixture pad/latch setup; restore reattaches them.
class Fixture {
public:
  explicit Fixture(Options options = {});
  void attach();
  CpuStep step();
  void run(unsigned count);
  void restore(const McuDspRuntimeCheckpoint &snapshot);
  McuDspRuntime::Observer observer();
  McuMachine mcu;
  OwnDspMachine dsp;
  std::unique_ptr<McuDspRuntime> runtime;
  std::vector<DspRuntimeCycle> cycles;
  std::vector<BusEvent> writes;
  McuDspRuntime::CycleInputs provide_inputs = cycle_inputs;
  McuDspRuntime::Observer on_cycle;
  std::function<void(const BusEvent &)> on_write;
  std::function<void(Hc12BusCycle)> before_cycle;
  bool deliver_write = true;

  std::optional<std::uint16_t> shared_address = std::nullopt;
};
}
