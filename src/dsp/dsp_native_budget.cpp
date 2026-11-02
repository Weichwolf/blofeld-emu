// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <limits>

namespace blofeld {
namespace {
bool internal(const dsp56300::EffectKind kind) {
  using enum dsp56300::EffectKind;
  return kind == address_update || kind == move_update || kind == alu_update ||
         kind == unit_update || kind == pcu_update || kind == retire;
}
}

void OwnDspFetch::check_reference_timing() const {
  if (budget_active_) {
    throw Fault("DSP native instruction budget owns its pending effects pc=" +
                hex(machine_.cpu_.registers().pc, 6));
  }
}

void OwnDspFetch::advance_native(OwnDspFetchCycleResult &result,
                                 const dsp56300::PendingInstruction &pending,
                                 const bool native_budget) {
  if (native_budget) {
    advance_native_budget(result, pending);
    return;
  }
  result.native_pc = pending.effect.pc;
  result.native = machine_.interrupts_.advance(pending.ticket);
}

void OwnDspFetch::advance_native_budget(OwnDspFetchCycleResult &result,
                                        dsp56300::PendingInstruction pending) {
  constexpr unsigned maximum_internal_effects = 128;
  if (!budget_active_ || native_cycles_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("DSP native cycle budget is unavailable pc=" + hex(pending.effect.pc, 6));
  }
  ++native_cycles_;
  for (unsigned index = 0; index < maximum_internal_effects; ++index) {
    if (pending.effect.kind == dsp56300::EffectKind::retire) {
      const auto delta = machine_.cpu_.retirement_delta(pending.ticket);
      if (delta.instructions != 1 || delta.estimated_cycles == 0) {
        throw Fault("DSP native retirement has no instruction budget pc=" +
                    hex(pending.effect.pc, 6));
      }
      if (native_cycles_ < delta.estimated_cycles) { return; }
    }
    result.native_pc = pending.effect.pc;
    result.native = machine_.interrupts_.advance(pending.ticket);
    if (result.native->fault.kind != dsp56300::FaultKind::none || result.native->retired) {
      return;
    }
    const auto &next = result.native->pending;
    if (!next || !internal(next->effect.kind)) { return; }
    pending = *next;
  }
  throw Fault("DSP native CPU publication bound exceeded pc=" + hex(pending.effect.pc, 6));
}

void OwnDspFetch::issue_after_retirement(OwnDspFetchCycleResult &result) {
  constexpr unsigned cache_enable = 1U << 19U;
  const auto state = fetch_.view();
  const auto regs = machine_.cpu_.registers();
  if (!state.pending_address && state.used < state.words.size() && !native_fetch_ &&
      !native_latch_ && !repeated_ && !interrupt_vector_ && state.omr == regs.omr &&
      (regs.sr & cache_enable) == 0) {
    fetch_.issue();
    result.issued_address = fetch_.view().pending_address;
  }
}
}
