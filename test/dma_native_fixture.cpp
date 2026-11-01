// SPDX-License-Identifier: GPL-3.0-only
#include "dma_native_fixture.hpp"

#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <exception>
#include <optional>
#include <string>

namespace blofeld::test::dma_test {
void NativeProgram::word(const std::uint32_t value) {
  constexpr unsigned word_mask = 0xffffff;
  require(value <= word_mask, "Authored native diagnostic words remain24-bit");
  words_.push_back(value);
}

void NativeProgram::write(const DspStore store) {
  // FM MOVEP immediate: low-X/low-Y qq and high-X pp groups.
  constexpr unsigned low = 0xffff80;
  constexpr unsigned high = 0xffffc0;
  constexpr unsigned end = 0xffffff;
  constexpr unsigned low_x = 0x07f400;
  constexpr unsigned low_y = 0x07b480;
  constexpr unsigned high_x = 0x08f480;
  require(store.address >= low && store.address <= end &&
              (store.space == DspSpace::x || (store.space == DspSpace::y && store.address < high)),
          "Literal DMA fixture supports only the documented MOVEP groups it uses");
  auto prefix = store.address < high ? low_x : high_x;
  if (store.space == DspSpace::y) { prefix = low_y; }
  word(prefix | (store.address - (store.address < high ? low : high)));
  word(store.value);
}

unsigned NativeProgram::pc() const {
  return boot + static_cast<unsigned>(words_.size());
}

void NativeProgram::install(DspBus &bus) const {
  bus.install_rom({.start = boot, .words = words_});
}

void reset_native(OwnDspMachine &machine, const NativeProgram &program) {
  program.install(machine.bus());
  constexpr std::uint64_t hold = 90;
  constexpr std::uint64_t recovery = 90;
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  require(machine.advance_hardware_reset_ticks(hold) == hold, "Actual DMA fixture reset hold");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(recovery) == recovery,
          "Actual DMA fixture reset recovery");
}

void step_native(OwnDspMachine &machine) {
  const auto before = machine.cpu().progress().instructions;
  const auto ticks = machine.clock().elapsed_ticks();
  auto step = machine.begin_fetch();
  constexpr unsigned limit = 32;
  for (unsigned effect = 0; effect < limit && step.pending && !step.retired &&
                            step.fault.kind == dsp56300::FaultKind::none;
       ++effect) {
    step = machine.advance(step.pending->ticket);
  }
  if (step.fault.kind != dsp56300::FaultKind::none) {
    std::string cause = "native instruction fault";
    const auto failure = machine.io().failure();
    if (failure && failure->cause) {
      try {
        std::rethrow_exception(failure->cause);
      } catch (const std::exception &error) { cause = error.what(); }
    }
    throw Fault("Own DMA fixture native pc=" + hex(step.fault.pc, 6) +
                " omr=" + hex(machine.cpu().registers().omr, 6) + ": " + cause);
  }
  require(step.retired && machine.cpu().progress().instructions == before + 1 &&
              machine.clock().elapsed_ticks() == ticks,
          "Exactly one untimed native instruction retires without inventing clock periods");
}

DspDmaBusResult dma(OwnDspMachine &machine, const DspDmaInputs inputs, const DspCoreBusUse core) {
  try {
    return machine.bus().advance_dma(inputs, core);
  } catch (const Fault &error) {
    const auto cpu = machine.cpu().registers();
    throw Fault("Own DMA fixture standalone phase pc=" + hex(cpu.pc, 6) +
                " omr=" + hex(cpu.omr, 6) + ": " + error.what());
  }
}

std::optional<DspDmaWord> dma(OwnDspMachine &machine, const DspDmaInputs inputs) {
  return dma(machine,
             inputs,
             {.accesses = {}, .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy})
      .word;
}
}
