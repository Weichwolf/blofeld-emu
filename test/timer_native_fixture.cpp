// SPDX-License-Identifier: GPL-3.0-only
#include "timer_native_fixture.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

namespace blofeld::test::timer_native {
void install_vector(OwnDspMachine &machine, const unsigned vector, const unsigned opcode) {
  machine.bus().write({.address = vector, .value = opcode});
  machine.bus().write({.address = vector + 1, .value = 0});
}

void service(OwnDspMachine &machine,
             const DspInterruptRequest request,
             const dsp56300::InstructionWords foreground) {
  const auto before = machine.cpu().progress();
  const auto ticks = machine.clock().elapsed_ticks();
  require(machine.bus().interrupt_request(0) == request && machine.admit_interrupt(foreground),
          "Actual eligible timer source admits the installed foreground instruction");
  unsigned reads = 0;
  for (unsigned word = 0; word < 2; ++word) {
    auto progress = machine.begin_fetch();
    for (unsigned effect = 0; effect < effect_bound && progress.pending && !progress.retired;
         ++effect) {
      const auto pending = progress.pending.value();
      const bool fetch = pending.effect.kind == dsp56300::EffectKind::fetch_opcode ||
                         pending.effect.kind == dsp56300::EffectKind::fetch_extension;
      if (fetch) {
        require(word == 0 &&
                    pending.effect.kind == (reads == 0 ? dsp56300::EffectKind::fetch_opcode
                                                       : dsp56300::EffectKind::fetch_extension) &&
                    pending.effect.space == dsp56300::Space::program &&
                    pending.effect.address == request.vector + reads && reads < 2 &&
                    machine.bus().interrupt_request(0) == request && machine.interrupt_pending(),
                "Both real vector reads precede acknowledgement of the selected timer source");
      }
      progress = machine.advance(pending.ticket);
      require(progress.fault.kind == dsp56300::FaultKind::none,
              "Actual fast timer vector remains healthy");
      if (fetch) {
        ++reads;
        require(machine.interrupt_pending() == (reads == 1),
                "Timer acknowledgement ownership ends only after the second actual vector read");
      }
    }
    require(progress.retired, "Each one-word timer vector instruction genuinely retires");
  }
  require(reads == 2 && !machine.interrupt_pending() &&
              machine.cpu().progress().instructions == before.instructions + 2 &&
              machine.cpu().progress().estimated_cycles == before.estimated_cycles + 2 &&
              machine.clock().elapsed_ticks() == ticks,
          "Two one-cycle native instructions retire without inventing board clock periods");
}
}
