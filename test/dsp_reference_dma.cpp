// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dma_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>

namespace blofeld::test::reference {
namespace {
namespace transfer = dma_test;

DspReferenceProgress until(DspMachine &machine, const DspNativeAccessKind kind) {
  auto progress = machine.begin_reference_instruction(boundaries);
  constexpr unsigned maximum_accesses = 16;
  for (unsigned budget = 0; !progress.retired && budget < maximum_accesses; ++budget) {
    if (request(progress).kind == kind) { return progress; }
    progress = machine.advance_reference_access(ticket(progress));
  }
  throw Fault("Own reference instruction did not reach its expected access");
}

struct Positions {
  unsigned source_store;
  unsigned overwrite;
  unsigned load;
};

Positions prepare_dma(DspMachine &machine) {
  DspProgram code;
  code.immediate("x0", second);
  for (const auto store : {DspStore{.address = transfer::source0, .value = source},
                           DspStore{.address = transfer::destination0, .value = destination},
                           DspStore{.address = transfer::counter0},
                           DspStore{.address = transfer::control0, .value = transfer::word_copy}}) {
    code.emit("movep #>" + number(store.value) + ",x:<<" + number(store.address));
  }
  const auto source_store = code.pc();
  code.emit("move x0,x:>$660");
  code.immediate("x0", third);
  const auto overwrite = code.pc();
  code.emit("move x0,x:>$660");
  const auto load = code.pc();
  code.emit("move y:>$900,x0");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = source, .value = first});
  machine.bus().write({.space = DspSpace::y, .address = destination, .value = first});
  static_cast<void>(machine.advance_dma(banks::inputs({.index = 0}), banks::use()));
  for (unsigned index = 0; index < 5; ++index) { machine.cpu().step(); }
  return {.source_store = source_store, .overwrite = overwrite, .load = load};
}

void held_dma_and_native_load() {
  DspMachine machine;
  const auto positions = prepare_dma(machine);
  auto progress = until(machine, DspNativeAccessKind::write);
  expect(progress,
         {.kind = DspNativeAccessKind::write,
          .target = {.space = DspSpace::x, .address = source},
          .value = second,
          .ordinal = 2,
          .pc = positions.source_store});
  const std::array source_use{DspMemoryAccess{.target = {.space = DspSpace::x, .address = source},
                                              .operation = DspMemoryOperation::write}};
  const auto denied =
      machine.advance_dma(banks::inputs({.index = 1, .requests = 1}), banks::use(source_use));
  require(denied.phase && denied.phase->kind == DspDmaPhaseKind::source_read && !denied.word &&
              !denied.core_stall,
          "A genuinely pending native source-bank store wins the declared fresh DMA phase");
  require(machine.advance_reference_access(ticket(progress)).retired,
          "The actual native source store is admitted before the delayed DMA read");
  static_cast<void>(machine.advance_dma(banks::inputs({.index = 2, .requests = 1}), banks::use()));
  execute(machine); // Real native immediate, not a fabricated register assignment.
  progress = until(machine, DspNativeAccessKind::write);
  require(request(progress).pc == positions.overwrite && request(progress).value == third,
          "The next native store has a distinct real source value");
  const auto held =
      machine.advance_dma(banks::inputs({.index = 3, .requests = 1, .grant = DspDmaGrant::wait}),
                          banks::use(source_use));
  require(held.phase && held.phase->value == second && !held.core_stall && !held.word,
          "The held destination owns a different bank; its captured source does not change");
  require(machine.advance_reference_access(ticket(progress)).retired,
          "The actual native source overwrite proceeds while the unrelated DMA write waits");
  progress = until(machine, DspNativeAccessKind::read);
  expect(progress,
         {.kind = DspNativeAccessKind::read,
          .target = {.space = DspSpace::y, .address = destination},
          .ordinal = 2,
          .pc = positions.load});
  const auto before_load = machine.cpu().debug_state();
  const std::array load_use{
      DspMemoryAccess{.target = {.space = DspSpace::y, .address = destination}}};
  const auto stalled = machine.advance_dma(
      banks::inputs({.index = 4, .requests = 1, .grant = DspDmaGrant::wait}), banks::use(load_use));
  require(stalled.core_stall && !stalled.word && machine.cpu().debug_state() == before_load,
          "The declared held-word conflict leaves the actual native load suspended");
  const auto pending = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(pending);
    const auto copied =
        machine.advance_dma(banks::inputs({.index = 5, .requests = 1}), banks::use(load_use));
    require(copied.word && copied.word->value == second && copied.core_stall &&
                machine.cpu().debug_state() == before_load,
            "Restored DMA completion precedes core admission without hidden instruction execution");
    progress = machine.advance_reference_access(ticket(machine));
    require(progress.retired && completed(progress).value == second &&
                machine.cpu().state().x == second &&
                machine.cpu().debug_state().native_instructions ==
                    before_load.native_instructions + 1 &&
                machine.bus().read({.space = DspSpace::x, .address = source}) == third &&
                machine.bus().read({.space = DspSpace::y, .address = destination}) == second &&
                machine.bus().read({.space = DspSpace::x, .address = transfer::status}) ==
                    transfer::all_done,
            "Both native pending-load replays receive the old captured DMA word after a real "
            "overwrite");
  }
}

void parallel_effect_prefix() {
  constexpr unsigned x_address = 0x100;
  constexpr unsigned y_address = 0x500;
  constexpr unsigned parallel_store = 0xbb1800; // FM X/Y parallel A/B stores with postincrement.
  DspMachine machine;
  DspProgram code;
  code.immediate("a1", first);
  code.immediate("b1", second);
  code.immediate("r0", x_address);
  code.immediate("r4", y_address);
  code.word(parallel_store);
  code.install(machine.bus());
  for (unsigned index = 0; index < 4; ++index) { machine.cpu().step(); }
  machine.bus().write({.space = DspSpace::x, .address = x_address});
  machine.bus().write({.space = DspSpace::y, .address = y_address});
  const auto before = machine.cpu().debug_state();
  auto progress = until(machine, DspNativeAccessKind::write);
  require(request(progress).target.space == DspSpace::x && request(progress).value == first,
          "The real parallel instruction first presents its X-store in semantic interpreter order");
  progress = machine.advance_reference_access(ticket(progress));
  require(request(progress).target.space == DspSpace::y && request(progress).value == second &&
              machine.cpu().debug_state() == before,
          "The second effect is still pending and register retirement remains deferred");
  const auto prefix = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(prefix);
    // Deliberate semantic-boundary mutation; serial callbacks are not two silicon clocks.
    machine.bus().write({.space = DspSpace::x, .address = x_address, .value = third});
    progress = machine.advance_reference_access(ticket(machine));
    require(progress.retired && machine.cpu().state().r[0] == x_address + 1 &&
                machine.cpu().state().r[4] == y_address + 1 &&
                machine.cpu().debug_state().native_instructions == before.native_instructions + 1 &&
                machine.bus().read({.space = DspSpace::x, .address = x_address}) == third &&
                machine.bus().read({.space = DspSpace::y, .address = y_address}) == second,
            "Recomputation never repeats an admitted store or applies either postincrement twice");
  }
}
}

void dma_checks() {
  held_dma_and_native_load();
  parallel_effect_prefix();
}
}
