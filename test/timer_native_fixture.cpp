// SPDX-License-Identifier: GPL-3.0-only
#include "timer_native_fixture.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56300/core.hpp"

namespace blofeld::test::timer_native {
void install_vector(DspMachine &machine, const unsigned vector, const unsigned opcode) {
  machine.bus().write({.address = vector, .value = opcode});
  machine.bus().write({.address = vector + 1, .value = 0});
}

void service(DspMachine &machine,
             const DspInterruptRequest request,
             const dsp56300::InstructionWords foreground) {
  const auto before = machine.cpu().state().progress;
  const auto ticks = machine.clock().elapsed_ticks();
  require(machine.bus().interrupt_request(0) == request && machine.admit_interrupt(foreground),
          "The actual eligible timer source admits the installed foreground instruction");
  require(machine.io().inspect_program(request.vector).has_value() &&
              machine.io().inspect_program(request.vector + 1).has_value() &&
              machine.interrupt_pending() && machine.bus().interrupt_request(0) == request,
          "Side-effect-free vector inspection cannot acknowledge an admitted source");
  for (unsigned word = 0; word < 2; ++word) {
    const auto result = machine.run_unclocked();
    require(
        result.retired == 1 && result.fault.kind == dsp56300::FaultKind::none &&
            !machine.interrupt_pending(),
        "The native vector prefetch reads both words before acknowledgement and retires each word");
  }
  require(!machine.interrupt_pending() &&
              machine.cpu().state().progress.instructions == before.instructions + 2 &&
              machine.cpu().state().progress.estimated_cycles == before.estimated_cycles + 2 &&
              machine.clock().elapsed_ticks() == ticks,
          "The two real native vector instructions execute without inventing board clock periods");
}
}
