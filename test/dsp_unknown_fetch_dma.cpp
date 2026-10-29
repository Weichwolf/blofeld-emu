// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
namespace test = blofeld::test;
namespace banks = test::banks;
namespace dma = test::dma_test;
constexpr unsigned base = 0x100;
constexpr unsigned source = 0x800;
constexpr unsigned ordinary_omr = 0x300;
constexpr unsigned status = 0xc00300;
constexpr unsigned program_copy = 0x88024a; // DE,DTM1,IRQ A,P→P,fixed/fixed.
constexpr unsigned jump = 0x0af080;
constexpr unsigned all_ones = 0xffffff;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

void refuses(const auto &operation, const std::string &expected) {
  try {
    operation();
  } catch (const Fault &error) {
    require(std::string(error.what()).contains(expected), "Refusal lost context: " + expected);
    return;
  }
  throw Fault("Unknown captured opcode was resolved by a later DMA write");
}

DspReferenceCycleInputs cycle(const banks::Cycle input) {
  return {.dma = banks::inputs(input), .order = DspReferenceBusOrder::dma_then_fetch_then_native};
}

DspDebugState prepare(DspMachine &machine, const unsigned value) {
  test::DspProgram setup;
  setup.immediate("omr", ordinary_omr);
  setup.immediate("sr", status);
  for (const auto store : {DspStore{.address = dma::source0, .value = source},
                           DspStore{.address = dma::destination0, .value = base},
                           DspStore{.address = dma::counter0},
                           DspStore{.address = dma::control0, .value = program_copy}}) {
    setup.emit("movep #>" + test::number(store.value) + ",x:<<" + test::number(store.address));
  }
  setup.word(jump);
  setup.word(base);
  setup.install(machine.bus());
  machine.bus().write({.address = source, .value = value});
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  static_cast<void>(machine.advance_dma(banks::inputs({.index = 0}), banks::use()));
  test::checkpoint::run(machine, 7);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == base && before.cpu.omr == ordinary_omr && before.cpu.sr == status &&
              !machine.bus().sample_program(base),
          "Genuine native MOVEP setup enters the unknown destination and arms actual P→P DMA");
  const auto pending =
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries);
  if (!pending.pending) { throw Fault("Own native opcode is not genuinely suspended"); }
  static_cast<void>(machine.start_reference_fetch(pending.pending->ticket));
  return before;
}

void captured_unknown_survives_dma(const unsigned value) {
  DspMachine machine;
  const auto before = prepare(machine, value);
  auto address = cycle({.index = 1});
  address.fetch = DspProgramFetchStage::address;
  static_cast<void>(machine.advance_reference_bus_cycle(address));
  auto read = cycle({.index = 2, .requests = 1});
  read.fetch = DspProgramFetchStage::read;
  const auto captured = machine.advance_reference_bus_cycle(read);
  const auto raw = machine.reference_fetch();
  require(captured.dma.phase && captured.dma.phase->kind == DspDmaPhaseKind::source_read &&
              !captured.dma.core_stall && !captured.dma.word && captured.core_access_count == 1 &&
              captured.core_accesses.front().target.address == base && raw.used == 1 &&
              raw.words.front().address == base && !raw.words.front().data &&
              machine.cpu().debug_state() == before,
          "Actual unknown F2 and different protected-bank DMA source both complete without CPU "
          "effects");
  const auto saved = machine.checkpoint();
  const auto original = machine.reference_pending();
  if (!original) { throw Fault("Own pending opcode lost its admission ticket"); }
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    require(machine.reference_fetch() == raw,
            "Joint restore retains the unknown F2 queue and pending DMA destination");
    refuses([&] { static_cast<void>(machine.advance_reference_access(original->ticket)); },
            "Stale, foreign or unavailable native reference access ticket");
    unsigned write_cycle = 3;
    if (replay != 0) {
      const auto held = machine.advance_reference_bus_cycle(
          cycle({.index = write_cycle, .requests = 1, .grant = DspDmaGrant::wait}));
      require(!held.dma.word && !machine.bus().sample_program(base) &&
                  machine.reference_fetch() == raw && machine.cpu().debug_state() == before,
              "A held actual DMA destination cannot initialize the captured unknown cell");
      ++write_cycle;
    }
    const auto written =
        machine.advance_reference_bus_cycle(cycle({.index = write_cycle, .requests = 1}));
    require(written.dma.word && written.dma.word->value == value &&
                machine.bus().read({.address = base}) == value &&
                machine.reference_fetch() == raw && machine.cpu().debug_state() == before,
            "Actual later DMA initializes RAM while the original F2 sample remains unknown");
    const auto current = machine.reference_pending();
    if (!current) { throw Fault("Restored pending opcode lost its fresh capability"); }
    refuses([&] { static_cast<void>(machine.advance_reference_access(current->ticket)); },
            "Unknown DSP program sample read P:" + hex(base, 6) + " pc=" + hex(base, 6) +
                " omr=" + hex(ordinary_omr, 6));
    require(machine.cpu().debug_state() == before,
            "Unknown opcode admission has no native effects");
  }
}
}

int main() {
  try {
    for (const unsigned value : {0U, all_ones}) { captured_unknown_survives_dma(value); }
    std::cout
        << "PASS genuine P→P DMA,unknown actual F2,wait,two joint suffixes and stale grants\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
