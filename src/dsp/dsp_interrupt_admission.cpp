// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_interrupt_admission.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <stdexcept>

namespace blofeld {
void DspInterruptAdmission::reset_after_cpu_reset() noexcept {
  request_.reset();
  vector_ = 0;
  first_ = false;
}

bool DspInterruptAdmission::admit(const dsp56300::InstructionWords foreground) {
  if (request_) { throw std::logic_error("DSP interrupt already owns vector acknowledgement"); }
  constexpr unsigned mask_shift = 8;
  constexpr unsigned mask_bits = 3;
  constexpr dsp56300::Word vector_base_mask = 0xffff00;
  const auto regs = cpu_.registers();
  const auto candidate = bus_.interrupt_request((regs.sr >> mask_shift) & mask_bits);
  if (!candidate || !cpu_.interrupt_ready(foreground)) { return false; }
  const auto address = (regs.vba & vector_base_mask) | candidate->vector;
  bus_.validate_admitted_interrupt(*candidate);
  cpu_.request_interrupt(address, candidate->level, foreground);
  request_ = *candidate;
  vector_ = address;
  first_ = false;
  return true;
}

bool DspInterruptAdmission::admit() {
  if (request_) { throw std::logic_error("DSP interrupt already owns vector acknowledgement"); }
  constexpr unsigned mask_shift = 8;
  constexpr unsigned mask_bits = 3;
  constexpr dsp56300::Word vector_base_mask = 0xffff00;
  const auto regs = cpu_.registers();
  const auto candidate = bus_.interrupt_request((regs.sr >> mask_shift) & mask_bits);
  if (!candidate || !cpu_.interrupt_ready()) { return false; }
  const auto address = (regs.vba & vector_base_mask) | candidate->vector;
  bus_.validate_admitted_interrupt(*candidate);
  cpu_.request_interrupt(address, candidate->level);
  request_ = *candidate;
  vector_ = address;
  first_ = false;
  return true;
}

void DspInterruptAdmission::fetch_completed(const unsigned address) {
  if (!request_) { return; }
  if (address != vector_ + (first_ ? 1U : 0U)) {
    throw std::logic_error("DSP interrupt encountered an unexpected program read");
  }
  if (!first_) {
    first_ = true;
    return;
  }
  bus_.acknowledge_admitted_interrupt(*request_);
  request_.reset();
  first_ = false;
}
}
