// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/own_dsp_machine.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>

namespace blofeld {
OwnDspMachine::OwnDspMachine(const DspRunClockInputs clock)
    : io_(bus_), cpu_(io_), interrupts_(bus_, cpu_), clock_(clock) {}

void OwnDspMachine::require_running() const {
  const auto reset = clock_.hardware_reset_state();
  if (!reset || reset->phase != DspHardwareResetPhase::complete || clock_.faulted()) {
    throw Fault("Own DSP execution requires completed hardware reset pc=" +
                hex(cpu_.registers().pc, 6));
  }
}

void OwnDspMachine::begin_hardware_reset(const OwnDspResetInputs inputs) {
  constexpr unsigned mode_mask = 15;
  constexpr unsigned spi_mode = 5;
  if (inputs.mode_known != mode_mask || inputs.mode_levels != spi_mode ||
      !inputs.pinit.has_value() || inputs.pinit.value() || inputs.recovery_cycles < 2 ||
      clock_.hardware_reset_active() || cpu_.pending_instruction() || interrupts_.pending()) {
    throw Fault("Own DSP reset requires a completed boundary, stable mode5/PINIT-low and "
                "at least two recovery periods pc=" +
                hex(cpu_.registers().pc, 6));
  }
  auto regs = cpu_.registers();
  try {
    clock_.restart_native_epoch();
    bus_.observe_irqa((inputs.mode_levels & 1U) != 0);
    bus_.pll().observe_pinit(inputs.pinit);
    bus_.reset();
    cpu_.reset();
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
    clock_.begin_hardware_reset(inputs.recovery_cycles);
  } catch (...) {
    clock_.fail();
    throw;
  }
}

std::uint64_t OwnDspMachine::advance_hardware_reset_ticks(const std::uint64_t maximum_ticks) {
  return clock_.advance_hardware_reset_ticks(maximum_ticks);
}

void OwnDspMachine::release_hardware_reset() {
  clock_.release_hardware_reset();
}

dsp56300::InstructionProgress
OwnDspMachine::begin_fetch(const dsp56300::InstructionPublication publication) {
  require_running();
  return cpu_.begin_fetch(publication);
}

dsp56300::InstructionProgress OwnDspMachine::advance(const dsp56300::EffectTicket &ticket) {
  require_running();
  auto result = interrupts_.advance(ticket);
  if (result.fault.kind == dsp56300::FaultKind::none) { bus_.observe_omr(cpu_.registers().omr); }
  return result;
}

bool OwnDspMachine::admit_interrupt(const dsp56300::InstructionWords foreground) {
  require_running();
  return interrupts_.admit(foreground);
}
}
