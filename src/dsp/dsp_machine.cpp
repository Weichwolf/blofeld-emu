// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_machine.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <cstdint>
#include <string>

namespace blofeld {
DspMachine::DspMachine(const DspRunClockInputs clock)
    : io_(bus_),
      cpu_(dsp56300::Memory::external(), &io_),
      interrupts_(bus_, cpu_),
      clock_(clock),
      clock_inputs_(clock) {
  io_.program_observer_ = {.context = &interrupts_,
                           .completed = [](void *owner, const unsigned address) {
                             static_cast<DspInterruptAdmission *>(owner)->fetch_completed(address);
                           }};
}

void DspMachine::require_unclocked(const char *const operation) const {
  if (active_ || !clocked_owner_.expired()) {
    throw Fault("DSP execution owner is busy operation=" + std::string(operation) +
                " pc=" + hex(cpu_.registers().pc, 6));
  }
}

void DspMachine::require_running() const {
  const auto reset = clock_.hardware_reset_state();
  if (!reset || reset->phase != DspHardwareResetPhase::complete || clock_.faulted()) {
    throw Fault("DSP execution requires completed hardware reset pc=" +
                hex(cpu_.registers().pc, 6));
  }
}

void DspMachine::check_hardware_reset(const DspResetInputs inputs) const {
  if (active_) { throw Fault("DSP reset requires an inactive native effect"); }
  constexpr unsigned mode_mask = 15;
  constexpr unsigned spi_mode = 5;
  if (inputs.mode_known != mode_mask || inputs.mode_levels != spi_mode ||
      !inputs.pinit.has_value() || inputs.recovery_cycles < 2 || clock_.hardware_reset_active()) {
    throw Fault("DSP reset requires stable mode5/known PINIT and "
                "at least two recovery periods pc=" +
                hex(cpu_.registers().pc, 6));
  }
}

void DspMachine::begin_hardware_reset(const DspResetInputs inputs) {
  require_unclocked("reset-assert");
  perform_hardware_reset(inputs);
}

void DspMachine::perform_hardware_reset(const DspResetInputs inputs) {
  check_hardware_reset(inputs);
  auto regs = cpu_.registers();
  try {
    clock_.restart_native_epoch();
    bus_.observe_irqa((inputs.mode_levels & 1U) != 0);
    bus_.pll().observe_pinit(inputs.pinit);
    bus_.reset();
    cpu_.reset();
    interrupts_.reset_after_cpu_reset();
    // FM figures 5-3/5-4, sections 4.3.4, 5.4.3/5.4.4 and mode5 boot.
    // Retain registers without reset initialization, including EP and SZ.
    constexpr dsp56300::Word reset_status = 0xc00300;
    constexpr dsp56300::Word reset_omr = 0x305;
    constexpr dsp56300::Word boot_entry = 0xff0000;
    regs.pc = boot_entry;
    regs.sr = reset_status;
    regs.omr = reset_omr;
    regs.sp = 0;
    regs.sc = 0;
    regs.vba = 0;
    regs.m.fill(dsp56300::word_mask);
    cpu_.set_registers(regs);
    io_.clear_failure();
    io_.peripheral_observer_ = {};
    clock_.begin_hardware_reset(inputs.recovery_cycles);
  } catch (...) {
    clock_.fail();
    throw;
  }
}

std::uint64_t DspMachine::advance_hardware_reset_ticks(const std::uint64_t maximum_ticks) {
  require_unclocked("reset-time");
  return clock_.advance_hardware_reset_ticks(maximum_ticks);
}

void DspMachine::release_hardware_reset() {
  require_unclocked("reset-release");
  check_reset_release();
  clock_.release_hardware_reset();
}

void DspMachine::check_reset_release() const {
  // PEN captures either observed PINIT level on assertion. PLL-enabled reset
  // recovery/lock is not yet modeled; it cannot use the bypass-EXTAL contract.
  constexpr unsigned pll_enabled = 0x2000;
  const auto control = bus_.pll().read();
  if ((control & pll_enabled) != 0) {
    throw Fault("DSP reset-release requires qualified PLL recovery at X:0xfffffd value=" +
                hex(control, 6) + " pc=" + hex(cpu_.registers().pc, 6));
  }
}

bool DspMachine::admit_interrupt(const dsp56300::InstructionWords foreground) {
  require_unclocked("interrupt-admission");
  require_running();
  return interrupts_.admit(foreground);
}
}
