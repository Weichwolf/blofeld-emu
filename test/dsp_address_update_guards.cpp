// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp_address_update_fixture.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <iostream>
#include <string>

namespace blofeld::test::address_update {
namespace {
void reserved_modifier(const unsigned modifier) {
  const Case input{.r = 7, .n = 3, .m = modifier, .mode = 3};
  constexpr unsigned reg = 5;
  DspMachine machine;
  const auto before = prepare(machine, input, reg);
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 3);
  const auto saved = run.checkpoint(machine);
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    run.restore(saved, machine);
    const auto message = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "A reserved modifier must fail at its first actual AG1");
    require(message.find("reserved modifier AG1 update (R5)") != std::string::npos &&
                message.find("pc=" + hex(entry, 6)) != std::string::npos && run.faulted() &&
                machine.cpu().debug_state() == before,
            "Reserved U arithmetic retains full CPU state and fails visibly before publication: " +
                message);
  }
}

void operand_aliases() {
  const Case input{.r = 7, .n = 3, .m = word_mask, .expected = 8, .mode = 3};
  DspMachine machine;
  const auto before = prepare(machine, input, 0);
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 3);
  const auto saved = run.checkpoint(machine);
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    run.restore(saved, machine);
    DspInstructionPipeline component{.profile = DspFetchPipeline::automatic_memory, .view = {}};
    const auto fetch = machine.reference_fetch();
    require(fetch.used != 0 && fetch.words.front().address == entry,
            "The component captures the real queued F2 occurrence");
    component.view.cycle.ag1 = fetch.words.front();
    component.capture_address(machine);
    const auto captured = component.addresses[0];
    if (!captured) { throw Fault("The actual U component lost its AG1 ticket"); }
    const auto &ticket = *captured;
    const auto epoch = machine.bus().last_dma_cycle();
    if (!epoch) { throw Fault("The actual three-clock prefix has no DMA cycle boundary"); }
    DspReferenceCycleInputs early{.dma = {.cycle = epoch.value() + 1,
                                          .known_requests = 15,
                                          .timing = DspDmaTiming::reference_sampled_cycles,
                                          .grant = DspDmaGrant::available},
                                  .address_only = ticket,
                                  .order = DspReferenceBusOrder::dma_then_fetch_then_native};
    const auto unpublished =
        checkpoint::rejects([&] { static_cast<void>(machine.advance_reference_bus_cycle(early)); },
                            "An unpublished AG1 record cannot enter E1");
    require(unpublished.find("phase mismatch") != std::string::npos &&
                machine.cpu().debug_state() == before,
            "Early E1 rejection does not publish the pointer or native result");
    early.address_only.reset();
    early.address_update = ticket;
    static_cast<void>(machine.advance_reference_bus_cycle(early));
    const auto updated = machine.cpu().debug_state();
    const auto published_epoch = machine.bus().last_dma_cycle();
    DspReferenceCycleInputs inputs{.dma = {.cycle = epoch.value() + 2,
                                           .known_requests = 15,
                                           .timing = DspDmaTiming::reference_sampled_cycles,
                                           .grant = DspDmaGrant::available},
                                   .memory = ticket,
                                   .order = DspReferenceBusOrder::dma_then_fetch_then_native};
    const auto wrong =
        checkpoint::rejects([&] { static_cast<void>(machine.advance_reference_bus_cycle(inputs)); },
                            "Bus preflight must refuse a U record in its data slot");
    require(wrong.find("actual data operand") != std::string::npos,
            "Bus admission does not invent a U memory resource");
    inputs.address_only = ticket;
    const auto duplicate =
        checkpoint::rejects([&] { static_cast<void>(machine.advance_reference_bus_cycle(inputs)); },
                            "The same U record cannot occupy both instruction and data slots");
    require(duplicate.find("cannot acquire a data operand") != std::string::npos &&
                machine.cpu().debug_state() == updated &&
                machine.bus().last_dma_cycle() == published_epoch,
            "Wrong/duplicate slots fail before DMA/time/native effects");
  }
}

void captured_opcode() {
  const Case input{.r = word_mask, .n = 3, .m = word_mask, .expected = 0, .mode = 3};
  DspMachine machine;
  auto expected = prepare(machine, input, 5);
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 3);
  const auto saved = run.checkpoint(machine);
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    run.restore(saved, machine);
    machine.bus().write({.address = entry, .value = 0});
    pipeline::clocks(run, machine, 4);
    auto final = expected;
    final.cpu.r[5] = 0;
    ++final.cpu.pc;
    ++final.native_instructions;
    ++final.native_cycles;
    require(machine.cpu().debug_state() == final && machine.bus().read({.address = entry}) == 0,
            "U E1 consumes its real captured F2 word while backing P memory retains the new NOP");
  }
}

struct Clear {
  unsigned reg = 0, mode = 0;
};

void parallel_clear(const Clear sample) {
  const auto reg = sample.reg;
  const auto mode = sample.mode;
  const Case input{.r = 7, .n = 3, .m = word_mask, .mode = mode};
  DspMachine machine;
  auto expected = prepare(machine, input, reg);
  const auto clear = checkpoint::assemble("clr a");
  constexpr unsigned alu_mask = 0xff;
  require(clear.size() == 1, "The own parallel CLR has one native word");
  machine.bus().write({.address = entry, .value = opcode(input, reg) | (clear[0] & alu_mask)});
  constexpr std::array targets{4U, 10U, 6U, 8U};
  expected.cpu.r.at(reg) = targets.at(mode);
  expected.cpu.a = 0;
  // FM CLR: E,N,V clear; U,Z set; S,L,C unchanged.
  constexpr unsigned changed = extension | unnormalized | negative | zero | overflow;
  expected.cpu.sr = (expected.cpu.sr & ~changed) | unnormalized | zero;
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 4);
  const auto saved = run.checkpoint(machine);
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    run.restore(saved, machine);
    pipeline::clocks(run, machine, 3);
    require(machine.cpu().debug_state() == expected,
            "The original parallel ALU and U update both execute through native E1/E2");
  }
}
}

void guard_checks() {
  constexpr std::array reserved{0x8000U, 0xc000U};
  for (const auto modifier : reserved) { reserved_modifier(modifier); }
  operand_aliases();
  captured_opcode();
  for (unsigned reg = 0; reg < 8; ++reg) {
    for (unsigned mode = 0; mode < 4; ++mode) { parallel_clear({.reg = reg, .mode = mode}); }
  }
  std::cout << "Reserved modifiers,operand aliases,F2 ownership and32 native parallel CLR windows "
               "passed\n";
}
}
