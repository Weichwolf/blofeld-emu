// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_interlocks.hpp"

#include "common/bus.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace blofeld {
namespace {
void age(unsigned &remaining, const unsigned elapsed) {
  remaining = remaining > elapsed ? remaining - elapsed : 0;
}
}

unsigned DspInterlocks::delay(const DspInterlockInstruction &instruction) const {
  unsigned result = 0;
  for (std::size_t index = 0; index < agu_.size(); ++index) {
    if ((instruction.agu_reads & (1U << index)) != 0) { result = std::max(result, agu_.at(index)); }
  }
  for (std::size_t index = 0; index < arithmetic_.size(); ++index) {
    if ((instruction.move_reads & (1U << index)) != 0) {
      const auto remaining = std::max(arithmetic_.at(index), transfer_.at(index));
      const auto advance = instruction.move_read_advance;
      result = std::max(result, remaining > advance ? remaining - advance : 0);
    }
  }
  if (instruction.status_read) {
    if (uncertain_status_ > std::max(result, status_)) {
      throw Fault("Unqualified DSP interlock status timing after partial accumulator read");
    }
    result = std::max(result, status_);
  }
  return result;
}

unsigned DspInterlocks::preview(const DspInterlockInstruction &instruction) const {
  if (prepared_) { throw Fault("DSP interlock preview requires a completed instruction"); }
  constexpr unsigned maximum_units = 1048576;
  constexpr unsigned agu_mask = 255;
  constexpr unsigned accumulator_mask = 3;
  if (instruction.native_units == 0 || instruction.native_units > maximum_units ||
      ((instruction.agu_reads | instruction.agu_writes) & ~agu_mask) != 0 ||
      ((instruction.move_reads | instruction.move_writes | instruction.arithmetic_writes) &
       ~accumulator_mask) != 0 ||
      instruction.agu_write_delay > 3 || instruction.transfer_write_delay > 1 ||
      instruction.move_read_advance > 1) {
    throw Fault("Invalid explicit DSP dependency description");
  }
  return delay(instruction);
}

void DspInterlocks::prepare(const DspInterlockInstruction &instruction) {
  const auto stalls = preview(instruction);
  prepared_ = instruction;
  prepared_stalls_ = stalls;
  fetched_ = false;
}

void DspInterlocks::observe_opcode(const std::uint32_t opcode) {
  if (!prepared_ || fetched_ || prepared_->opcode != opcode) {
    throw Fault("DSP interlock opcode fetch changed after its timing preview");
  }
  fetched_ = true;
}

void DspInterlocks::commit(const DspInterlockInstruction &instruction, const unsigned stalls) {
  const auto elapsed =
      instruction.native_units +
      stalls; // Native units are bounded; at most three dependency clocks are added.
  for (auto &remaining : agu_) { age(remaining, elapsed); }
  for (auto &remaining : arithmetic_) { age(remaining, elapsed); }
  for (auto &remaining : transfer_) { age(remaining, elapsed); }
  age(status_, elapsed);
  age(uncertain_status_, elapsed);
  for (std::size_t index = 0; index < agu_.size(); ++index) {
    if ((instruction.agu_writes & (1U << index)) != 0) {
      agu_.at(index) = instruction.agu_write_delay;
    }
  }
  for (std::size_t index = 0; index < arithmetic_.size(); ++index) {
    if ((instruction.move_writes & (1U << index)) != 0) {
      transfer_.at(index) = instruction.transfer_write_delay;
    }
    if ((instruction.arithmetic_writes & (1U << index)) != 0) { arithmetic_.at(index) = 1; }
  }
  if (instruction.status_write) { status_ = 2; }
  if (instruction.partial_status_source) { uncertain_status_ = 2; }
}

unsigned DspInterlocks::complete(const unsigned native_units) {
  if (!prepared_ || !fetched_ || prepared_->native_units != native_units) {
    throw Fault("DSP interlock completion differs from its fetched instruction preview");
  }
  const auto stalls = prepared_stalls_;
  commit(*prepared_, stalls);
  prepared_.reset();
  prepared_stalls_ = 0;
  fetched_ = false;
  return stalls;
}

void DspInterlocks::reset() noexcept {
  *this = DspInterlocks{};
}

void DspInterlocks::elapse_idle(const unsigned units) {
  if (prepared_ || units == 0) {
    throw Fault("DSP dependency idle time needs an inactive history");
  }
  for (auto &remaining : agu_) { age(remaining, units); }
  for (auto &remaining : arithmetic_) { age(remaining, units); }
  for (auto &remaining : transfer_) { age(remaining, units); }
  age(status_, units);
  age(uncertain_status_, units);
}
}
