// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

namespace blofeld {
namespace {
class InstructionGuard {
public:
  InstructionGuard(bool &active, DspIoWriteObserver &slot, const DspIoWriteObserver observer)
      : active_(active), slot_(slot) {
    active_ = true;
    slot_ = observer;
  }

  ~InstructionGuard() {
    slot_ = {};
    active_ = false;
  }

  InstructionGuard(const InstructionGuard &) = delete;
  InstructionGuard &operator=(const InstructionGuard &) = delete;

private:
  bool &active_;
  DspIoWriteObserver &slot_;
};
}

dsp56300::RunResult OwnDspMachine::run_unclocked(const DspIoWriteObserver observer) {
  require_unclocked("synchronous-instruction");
  require_running();
  if (advancing_effect_ || !captured_fetch_owner_.expired() || interrupts_.pending() ||
      cpu_.pending_instruction()) {
    const auto state = cpu_.registers();
    throw Fault("Own DSP synchronous instruction requires an inactive unclocked owner pc=" +
                hex(state.pc, 6) + " omr=" + hex(state.omr, 6));
  }
  const InstructionGuard guard(advancing_effect_, io_.write_observer_, observer);
  auto result = cpu_.run(1);
  if (result.fault.kind == dsp56300::FaultKind::none && result.dispatches == 1) {
    bus_.observe_omr(cpu_.registers().omr);
  }
  return result;
}
}
