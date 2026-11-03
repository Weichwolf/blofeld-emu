// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

namespace blofeld {
namespace {
class ObservedEffectGuard {
public:
  ObservedEffectGuard(bool &active,
                      DspIoReadObserver &reads,
                      DspIoWriteObserver &writes,
                      const DspIoReadObserver read_observer,
                      const DspIoWriteObserver write_observer,
                      dsp56300::Core *const rebind = nullptr)
      : active_(active), reads_(reads), writes_(writes), rebind_(rebind) {
    if (rebind_ != nullptr) { rebind_->clear_code_cache(); }
    active_ = true;
    reads_ = read_observer;
    writes_ = write_observer;
  }

  ~ObservedEffectGuard() {
    reads_ = {};
    writes_ = {};
    active_ = false;
    if (rebind_ != nullptr) { rebind_->clear_code_cache(); }
  }

  ObservedEffectGuard(const ObservedEffectGuard &) = delete;
  ObservedEffectGuard &operator=(const ObservedEffectGuard &) = delete;

private:
  bool &active_;
  DspIoReadObserver &reads_;
  DspIoWriteObserver &writes_;
  dsp56300::Core *rebind_;
};
}

dsp56300::RunResult OwnDspMachine::run_unclocked(const DspIoWriteObserver observer) {
  return run_observed_unclocked({}, observer);
}

dsp56300::RunResult OwnDspMachine::run_observed_unclocked(const DspIoReadObserver reads,
                                                          const DspIoWriteObserver writes) {
  require_unclocked("synchronous-instruction");
  require_running();
  if (advancing_effect_ || !captured_fetch_owner_.expired() || interrupts_.pending() ||
      cpu_.pending_instruction()) {
    const auto state = cpu_.registers();
    throw Fault("Own DSP synchronous instruction requires an inactive unclocked owner pc=" +
                hex(state.pc, 6) + " omr=" + hex(state.omr, 6));
  }
  const bool observes_ram = reads.completed != nullptr || writes.completed != nullptr;
  const ObservedEffectGuard guard(advancing_effect_,
                                  io_.read_observer_,
                                  io_.write_observer_,
                                  reads,
                                  writes,
                                  observes_ram ? &cpu_ : nullptr);
  auto result = cpu_.run(1);
  if (result.fault.kind == dsp56300::FaultKind::none && result.dispatches == 1) {
    bus_.observe_omr(cpu_.registers().omr);
  }
  return result;
}

dsp56300::InstructionProgress OwnDspMachine::advance_observed(const dsp56300::EffectTicket &ticket,
                                                              const DspIoReadObserver reads,
                                                              const DspIoWriteObserver writes) {
  require_unclocked("observed-native-effect");
  require_running();
  if (advancing_effect_ || !captured_fetch_owner_.expired()) {
    const auto state = cpu_.registers();
    throw Fault("Own DSP observed effect requires an inactive unclocked owner pc=" +
                hex(state.pc, 6) + " omr=" + hex(state.omr, 6));
  }
  const ObservedEffectGuard guard(
      advancing_effect_, io_.read_observer_, io_.write_observer_, reads, writes);
  auto result = interrupts_.advance(ticket);
  if (result.fault.kind == dsp56300::FaultKind::none) { bus_.observe_omr(cpu_.registers().omr); }
  return result;
}
}
