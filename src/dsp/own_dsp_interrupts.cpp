// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/own_dsp_interrupts.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp56300/core.hpp"

#include <stdexcept>

namespace blofeld {
void OwnDspInterrupts::reset_after_cpu_reset() noexcept {
  request_.reset();
  vector_ = 0;
  first_ = false;
}

bool OwnDspInterrupts::admit(const dsp56300::InstructionWords foreground) {
  if (request_) { throw std::logic_error("DSP interrupt already owns vector acknowledgement"); }
  constexpr unsigned mask_shift = 8;
  constexpr unsigned mask_bits = 3;
  constexpr dsp56300::Word vector_base_mask = 0xffff00;
  const auto regs = cpu_.registers();
  const auto candidate = bus_.interrupt_request((regs.sr >> mask_shift) & mask_bits);
  if (!candidate || !cpu_.interruptible(foreground)) { return false; }
  const auto address = (regs.vba & vector_base_mask) | candidate->vector;
  bus_.validate_admitted_interrupt(*candidate);
  cpu_.request_interrupt(address, candidate->level, foreground);
  request_ = *candidate;
  vector_ = address;
  first_ = false;
  return true;
}

dsp56300::InstructionProgress OwnDspInterrupts::advance(const dsp56300::EffectTicket &ticket) {
  const auto before = cpu_.pending_instruction();
  if (!before) { throw std::logic_error("DSP interrupt owner has no staged effect to advance"); }
  const auto effect = before->effect;
  const bool vector_fetch = request_ && (effect.kind == dsp56300::EffectKind::fetch_opcode ||
                                         effect.kind == dsp56300::EffectKind::fetch_extension);
  if (vector_fetch && (effect.space != dsp56300::Space::program ||
                       effect.address != vector_ + (first_ ? 1U : 0U))) {
    throw std::logic_error("DSP interrupt owner encountered an unexpected vector fetch");
  }
  auto result = cpu_.advance_instruction(ticket);
  // Only a successfully advanced native fetch can consume vector ownership.
  // Invalid tickets and refused bus reads cannot acknowledge a peripheral.
  if (vector_fetch && result.fault.kind == dsp56300::FaultKind::none) {
    if (first_) {
      bus_.acknowledge_admitted_interrupt(request_.value());
      request_.reset();
      first_ = false;
    } else {
      first_ = true;
    }
  }
  return result;
}
}
