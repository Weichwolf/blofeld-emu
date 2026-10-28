// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <string>

namespace blofeld::test::memory_pipeline {
namespace {
using native_access::fault;

void unqualified_forms() {
  for (const auto *text : {"clr a x:(r0+n0),a0",
                           "clr a x:-(r0),a0",
                           "move x:>$7,x0",
                           "move x:<$7,x0",
                           "move r0,x:(r0)+",
                           "clr a x:(r0+n0),a",
                           "add x1,a x:(r0)+,a",
                           "move x:(r0)+,a y:(r4)+,b"}) {
    DspMachine machine;
    const std::array code{std::string(text)};
    prepare(machine, code);
    DspFetchRun run(contract(), machine);
    clocks(run, machine, 2);
    const auto before = machine.cpu().debug_state();
    const auto error =
        fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("automatic DSP decode") && error.contains("pc=") &&
                error.contains("omr=") && run.completed_cycles() == 2 &&
                machine.cpu().debug_state() == before && machine.bus().last_dma_cycle() == 2,
            std::string(text) + ": unsupported native formats fail before address or bus effects");
  }
}

void fifo_is_not_a_ram_resource() {
  DspMachine machine;
  const std::array<std::string, 3> code{"add x1,a", "move x:(r0)+,x0", "move x:(r0)+,x1"};
  prepare(machine, code, {.pointer = reference::fifo, .modifier = linear_modifier});
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = shi_control, .value = spi_configuration});
  reference::send(machine.bus().shi(), first);
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  const auto before = machine.cpu().debug_state();
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(error.contains("pc=0x00011d") && error.contains("omr=") && error.contains("0xffff94") &&
              run.completed_cycles() == 6 && machine.cpu().debug_state() == before &&
              machine.bus().last_dma_cycle() == 6 &&
              machine.bus().read({.space = DspSpace::x, .address = reference::fifo}) == first,
          "Unqualified MMIO preflight precedes DMA/F2/E2/AG2 and preserves the real receive FIFO");
}

void invalid_address_tickets() {
  for (const bool memory : {false, true}) {
    DspMachine machine;
    const std::array<std::string, 1> code{"nop"};
    prepare(machine, code);
    DspFetchRun run(contract(), machine);
    DspReferenceCycleInputs input;
    input.order = DspReferenceBusOrder::dma_then_fetch_then_native;
    input.dma.cycle = 1;
    input.dma.known_requests = 15;
    if (memory) {
      input.memory = DspAddressStageTicket{};
    } else {
      input.address_update = DspAddressStageTicket{};
    }
    const auto before = machine.cpu().debug_state();
    const auto error =
        fault([&] { static_cast<void>(machine.advance_reference_bus_cycle(input)); });
    require(error.contains("address-stage owner") && machine.cpu().debug_state() == before &&
                machine.bus().last_dma_cycle() == 0,
            "Unavailable/default address capabilities fail before admission or bus effects");
    clocks(run, machine, 7);
    require(machine.cpu().state().pc == body + 1,
            "Pure ticket refusal leaves automatic ownership usable");
  }
}
}

void guard_checks() {
  unqualified_forms();
  fifo_is_not_a_ram_resource();
  invalid_address_tickets();
}
}
