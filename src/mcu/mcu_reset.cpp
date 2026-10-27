// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/mcu_reset.hpp"

#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"

#include <cstdint>
#include <limits>

namespace blofeld {
namespace {
constexpr unsigned drive_cycles = 128;
constexpr unsigned settling_cycles = 64;
constexpr unsigned minimum_sync = 3, maximum_sync = 6;
}

McuResetSequence::McuResetSequence(const McuResetSource source,
                                   const unsigned synchronization_cycles)
    : state_{.requested = source,
             .selected = source,
             .synchronization_cycles = synchronization_cycles} {
  if ((source != McuResetSource::external && source != McuResetSource::cop &&
       source != McuResetSource::power_on) ||
      synchronization_cycles < minimum_sync || synchronization_cycles > maximum_sync) {
    throw Fault("MCU reset requires a known source and explicit3..6 SYSCLK synchronization cycles");
  }
}

bool McuResetSequence::active() const {
  return state_.phase != McuResetPhase::complete;
}

bool McuResetSequence::drives_low() const {
  return state_.phase == McuResetPhase::driving;
}

void McuResetSequence::edge(const McuResetPad pad) {
  if (state_.phase == McuResetPhase::ready || !active()) {
    throw Fault("MCU reset sequence cannot clock beyond its vector boundary");
  }
  if (pad != McuResetPad::unknown && pad != McuResetPad::low && pad != McuResetPad::high) {
    throw Fault("Invalid MCU reset pad observation");
  }
  const auto release = drive_cycles + state_.synchronization_cycles;
  const auto sample = release + settling_cycles;
  const bool needs_pad = state_.phase == McuResetPhase::external_hold || state_.edges == sample - 1;
  if (needs_pad && pad == McuResetPad::unknown) {
    throw Fault("Unknown MCU RESET pad at reset-source sampling boundary");
  }
  if (state_.edges == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("MCU reset edge counter exhausted");
  }
  ++state_.edges;
  if (state_.phase == McuResetPhase::external_hold) {
    if (pad == McuResetPad::high) { state_.phase = McuResetPhase::ready; }
  } else if (state_.edges == release) {
    state_.phase = McuResetPhase::settling;
  } else if (state_.edges == sample) {
    if (pad == McuResetPad::low) {
      state_.selected = McuResetSource::external;
      state_.phase = McuResetPhase::external_hold;
    } else {
      state_.phase = McuResetPhase::ready;
    }
  }
}

std::uint16_t McuResetSequence::vector() const {
  if (state_.phase != McuResetPhase::ready && state_.phase != McuResetPhase::complete) {
    throw Fault("MCU reset vector is unknown before RESET pad arbitration");
  }
  return state_.selected == McuResetSource::cop ? hc12::cop_reset_vector : hc12::reset_vector;
}

void McuResetSequence::complete() {
  if (state_.phase != McuResetPhase::ready) {
    throw Fault("MCU reset vector boundary is not ready");
  }
  state_.phase = McuResetPhase::complete;
}
}
