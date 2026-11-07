// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>

namespace blofeld {
namespace {
class ExecutionGuard {
public:
  ExecutionGuard(bool &active, DspPeripheralObserver &slot, const DspPeripheralObserver observer)
      : active_(active), slot_(slot), previous_(slot) {
    if (active_) { throw Fault("Recursive DSP execution"); }
    active_ = true;
    slot_ = observer;
  }

  ~ExecutionGuard() {
    slot_ = previous_;
    active_ = false;
  }

  ExecutionGuard(const ExecutionGuard &) = delete;
  ExecutionGuard &operator=(const ExecutionGuard &) = delete;

private:
  bool &active_;
  DspPeripheralObserver &slot_;
  DspPeripheralObserver previous_;
};
}

dsp56300::RunResult DspMachine::execute(const std::uint32_t instructions,
                                        const DspPeripheralObserver observer) {
  require_running();
  const ExecutionGuard guard(active_, io_.peripheral_observer_, observer);
  const auto result = cpu_.run(instructions);
  if (result.fault.kind == dsp56300::FaultKind::none) { bus_.observe_omr(cpu_.registers().omr); }
  return result;
}

dsp56300::RunResult DspMachine::execute_until(const std::uint64_t base_cycle_deadline,
                                              const DspPeripheralObserver observer) {
  require_running();
  const ExecutionGuard guard(active_, io_.peripheral_observer_, observer);
  // The timeline admits mapping effects after paying the complete instruction time.
  return cpu_.run_until(base_cycle_deadline);
}

dsp56300::RunResult DspMachine::run(const std::uint32_t instructions) {
  require_unclocked("run");
  return execute(instructions);
}

dsp56300::RunResult DspMachine::run_unclocked() {
  return run(1);
}
}
