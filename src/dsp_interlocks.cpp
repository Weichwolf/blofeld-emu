// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_interlocks.hpp"

#include "blofeld/bus.hpp"

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
    if (unknown_status_ != 0) {
      throw Fault("Unqualified DSP interlock timing for status read after accumulator transfer");
    }
    result = std::max(result, status_);
  }
  return result;
}

unsigned DspInterlocks::preview(const std::uint32_t opcode, const unsigned native_units) const {
  if (prepared_) { throw Fault("DSP interlock preview requires a completed instruction"); }
  return delay(decode_interlock_instruction(opcode, native_units));
}

void DspInterlocks::prepare(const std::uint32_t opcode, const unsigned native_units) {
  if (prepared_) { throw Fault("DSP interlock instruction is already prepared"); }
  const auto instruction = decode_interlock_instruction(opcode, native_units);
  const auto stalls = delay(instruction);
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
  const auto elapsed = instruction.native_units + stalls; // Decoder bounds each input to <=3.
  for (auto &remaining : agu_) { age(remaining, elapsed); }
  for (auto &remaining : arithmetic_) { age(remaining, elapsed); }
  for (auto &remaining : transfer_) { age(remaining, elapsed); }
  age(status_, elapsed);
  age(unknown_status_, elapsed);
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
  if (instruction.arithmetic_writes != 0) { status_ = 2; }
  if (instruction.move_reads != 0) { unknown_status_ = 2; }
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
}
