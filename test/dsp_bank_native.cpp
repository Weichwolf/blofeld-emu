// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <variant>
#include <vector>

namespace blofeld::test::banks {
namespace {
using esai::rejects;
using esai::require;

DspReferencePending pending_read(const DspMachine &machine) {
  const auto pending = machine.reference_pending();
  if (!pending) { throw Fault("Native controller read lost its admission ticket"); }
  return *pending;
}

DspReferencePending source_read(DspMachine &machine) {
  auto progress = machine.begin_reference_instruction(DspReferenceExecution::access_boundaries);
  for (unsigned effect = 0; effect < 8 && progress.pending; ++effect) {
    const auto *access = std::get_if<DspNativeAccess>(&progress.pending->effect);
    require(access != nullptr, "The native source-register query has only memory effects");
    if (access->kind == DspNativeAccessKind::read && access->target.space == DspSpace::x &&
        access->target.address == transfer::source0) {
      return *progress.pending;
    }
    progress = machine.advance_reference_access(progress.pending->ticket);
  }
  throw Fault("Native MOVEP did not yield its actual DMA source-register read");
}

void native_controller_query() {
  // The original foreground compares DSR0 bit5. Qualify both explicit access
  // orders, without choosing their simultaneous silicon sampling order.
  constexpr unsigned last_word = 0x67f;
  constexpr unsigned next_half = 0x680;
  constexpr unsigned phase = 0x20;
  DspMachine machine;
  DspProgram code;
  code.immediate("x0", phase);
  code.emit("movep x:<<$ffffef,a");
  code.emit("and #>$20,a");
  code.emit("cmp x0,a");
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  static_cast<void>(machine.advance_dma(inputs({.index = 0}), use()));
  machine.bus().write({.space = DspSpace::x, .address = last_word, .value = first});
  configure(machine.bus(), {.source = last_word, .control = transfer::software_copy});
  machine.cpu().step();
  const auto pending = source_read(machine);
  const std::array core{
      DspMemoryAccess{.target = std::get<DspNativeAccess>(pending.effect).target}};
  const auto read = machine.advance_dma(inputs({.index = 1}), use(core));
  require(read.phase && read.phase->kind == read_phase && !read.word && !read.core_stall,
          "An actually suspended native controller read leaves the DMA source path available");
  machine.bus().write({.space = DspSpace::x, .address = last_word, .value = second});
  const auto frozen = machine.checkpoint();
  for (const bool before_write : {true, false}) {
    for (unsigned replay = 0; replay < 2; ++replay) {
      machine.restore(frozen);
      const auto before = machine.cpu().debug_state();
      const auto restored = pending_read(machine);
      if (before_write) {
        require(machine.advance_reference_access(restored.ticket).retired,
                "The before-write branch retires the real source-register MOVEP");
      }
      const auto completed = machine.advance_dma(inputs({.index = 2}), use(core));
      require(completed.word && completed.word->value == first && !completed.core_stall &&
                  machine.bus().read({.space = DspSpace::x, .address = transfer::source0}) ==
                      next_half &&
                  machine.bus().read({.space = DspSpace::y, .address = transfer::destination}) ==
                      first,
              "DMA alone publishes the next half and stores its captured, unmodified word");
      if (!before_write) {
        require(machine.cpu().debug_state() == before &&
                    pending_read(machine).ticket == restored.ticket,
                "DMA advances neither the pending instruction nor its access ticket");
        require(machine.advance_reference_access(restored.ticket).retired,
                "The after-write branch retires the real source-register MOVEP");
      }
      constexpr unsigned accumulator_shift = 24;
      constexpr unsigned word_mask = 0xffffff;
      const auto observed = (machine.cpu().state().a >> accumulator_shift) & word_mask;
      require(observed == (before_write ? last_word : next_half) &&
                  machine.cpu().debug_state().native_instructions == before.native_instructions + 1,
              "Each native MOVEP samples its explicitly ordered actual controller value once");
      machine.cpu().step();
      machine.cpu().step();
      constexpr unsigned zero = 1U << 2U;
      require(((machine.cpu().state().sr & zero) != 0) == before_write,
              "Native AND/CMP sees equal old phase or the DMA-generated changed half");
    }
  }
}

void native_held_word() {
  constexpr unsigned prefix_units = 17; // Five setup*2, store2, NOP1, immediate2, overwrite2.
  constexpr unsigned total_units = 19;  // The final long Y-memory load adds2 nominal units.
  DspMachine machine;
  DspProgram code;
  code.immediate("x0", first);
  for (const auto store :
       {DspStore{.address = transfer::source0, .value = transfer::ring_start},
        DspStore{.address = transfer::destination0, .value = transfer::destination},
        DspStore{.address = transfer::counter0},
        DspStore{.address = transfer::control0, .value = transfer::word_copy}}) {
    code.emit("movep #>" + number(store.value) + ",x:<<" + number(store.address));
  }
  code.emit("move x0,x:>$660");
  const auto nop = code.pc();
  code.emit("nop");
  code.immediate("x0", 0);
  code.emit("move x0,x:>$660");
  const auto load = code.pc();
  code.emit("move y:>$900,y0");
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  machine.bus().write({.space = DspSpace::x, .address = transfer::ring_start, .value = second});
  machine.bus().write({.space = DspSpace::y, .address = transfer::destination, .value = second});
  static_cast<void>(machine.advance_dma(inputs({.index = 0}), use()));
  for (unsigned index = 0; index < 5; ++index) { machine.cpu().step(); }
  std::vector<DspNativeAccess> stores;
  const DspNativeAccessObserver observer{.observe = [&](const DspNativeAccess &access) {
    if (access.kind == DspNativeAccessKind::write && access.stage == DspNativeAccessStage::after) {
      stores.push_back(access);
    }
  }};
  machine.cpu().step(observer);
  const std::array source_use{
      DspMemoryAccess{.target = {.space = DspSpace::x, .address = transfer::ring_start},
                      .operation = DspMemoryOperation::write}};
  const auto denied = machine.advance_dma(inputs({.index = 1, .requests = 1}), use(source_use));
  require(denied.phase && denied.phase->kind == read_phase && !denied.phase->value &&
              !denied.word && !denied.core_stall && stores.size() == 1 &&
              stores.front().value == first,
          "The actual native source store wins the explicitly declared contended read phase");
  machine.cpu().step();
  const std::array opcode_use{DspMemoryAccess{.target = {.address = nop}}};
  const auto captured = machine.advance_dma(inputs({.index = 2, .requests = 1}), use(opcode_use));
  require(captured.phase && captured.phase->kind == read_phase && !captured.word &&
              !captured.core_stall,
          "Known P-ROM opcode occupancy does not contend with the independent X-RAM source");
  machine.cpu().step();
  machine.cpu().step(observer);
  require(stores.size() == 2 && stores.back().value == 0 &&
              machine.cpu().debug_state().native_cycles == prefix_units &&
              machine.cpu().state().pc == load,
          "Actual native overwrite happens after capture; the pending load has not executed");
  const std::array destination_use{
      DspMemoryAccess{.target = {.space = DspSpace::y, .address = transfer::destination}}};
  const auto waited = machine.advance_dma(
      inputs({.index = 3, .requests = 1, .grant = DspDmaGrant::wait}), use(destination_use));
  require(waited.phase && waited.phase->kind == write_phase && waited.phase->value == first &&
              waited.core_stall && !waited.word &&
              machine.bus().read({.space = DspSpace::y, .address = transfer::destination}) ==
                  second,
          "A globally waiting held word still blocks the known contending core load");
  const auto frozen = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(frozen);
    const auto before = machine.cpu().debug_state();
    const auto completed =
        machine.advance_dma(inputs({.index = 4, .requests = 1}), use(destination_use));
    require(completed.core_stall && completed.word && completed.word->value == first &&
                completed.word->source == DspDmaAddress{.address = transfer::ring_start} &&
                completed.word->destination ==
                    DspDmaAddress{.space = DspDmaSpace::y, .address = transfer::destination} &&
                machine.cpu().debug_state() == before &&
                machine.bus().read({.space = DspSpace::x, .address = transfer::ring_start}) == 0,
            "The held DMA write completes before core admission, without hidden native execution");
    machine.cpu().step();
    require(machine.cpu().state().y == first &&
                machine.cpu().debug_state().native_cycles == total_units &&
                machine.bus().read({.space = DspSpace::x, .address = transfer::status}) ==
                    transfer::all_done,
            "Both replays execute the real pending load after the original captured word arrives");
  }
}

void unqualified_conflict() {
  DspMachine machine;
  require(rejects([&] {
            static_cast<void>(machine.advance_dma(inputs({.index = 0}), {}));
          }).contains("pc=0xff0000"),
          "Missing occupancy qualification fails with native execution context");
  static_cast<void>(machine.advance_dma(inputs({.index = 0}), use()));
  constexpr unsigned upper = 0xc000;
  constexpr unsigned other_section = 0xc400;
  machine.bus().write({.space = DspSpace::x, .address = upper, .value = first});
  machine.bus().write({.space = DspSpace::x, .address = other_section, .value = second});
  machine.bus().write({.space = DspSpace::y, .address = transfer::destination, .value = second});
  configure(machine.bus(), {.source = upper});
  const std::array unsafe{
      DspMemoryAccess{.target = {.space = DspSpace::x, .address = other_section}}};
  const auto error = rejects([&] {
    static_cast<void>(machine.advance_dma(inputs({.index = 1, .requests = 1}), use(unsafe)));
  });
  require(
      error.contains("source read X:0x00c000 core=X:0x00c400") && error.contains("pc=0xff0000") &&
          error.contains("cycle=1 channel=0") &&
          machine.bus().read({.space = DspSpace::y, .address = transfer::destination}) == second,
      "Unsafe8K-subsection contention has no invented safe wait, lost request or destination I/O");
  const std::array safe{DspMemoryAccess{.target = {.space = DspSpace::x, .address = upper + 1}}};
  const auto denied = machine.advance_dma(inputs({.index = 1, .requests = 1}), use(safe));
  require(denied.phase && !denied.word && !denied.core_stall,
          "The same uncommitted cycle can instead qualify protected same1K contention");
  static_cast<void>(machine.advance_dma(inputs({.index = 2, .requests = 1}), use()));
  const auto completed = machine.advance_dma(inputs({.index = 3, .requests = 1}), use());
  require(completed.word && completed.word->value == first,
          "The actual captured request survives denial and completes without another rising edge");
}

void caller_qualification() {
  DspBus bus;
  constexpr unsigned peripheral = 0xffff94;
  const std::array fifo{DspMemoryAccess{.target = {.space = DspSpace::x, .address = peripheral}}};
  require(rejects([&] {
            static_cast<void>(bus.advance_dma(inputs({.index = 0}), use(fifo)));
          }).contains("Unqualified DSP peripheral memory resource read"),
          "Unqualified core peripheral-bus sharing is rejected before request sampling");
  static_cast<void>(bus.advance_dma(inputs({.index = 0}), use()));
  constexpr std::array accesses{
      DspMemoryAccess{.target = {.space = DspSpace::x, .address = transfer::ring_start}},
      DspMemoryAccess{.target = {.space = DspSpace::y, .address = transfer::ring_start}}};
  configure(bus);
  bus.write({.space = DspSpace::x, .address = transfer::ring_start, .value = first});
  const auto waited = bus.advance_dma(inputs({.index = 1, .requests = 1}), use(accesses));
  require(waited.phase && !waited.word && !waited.core_stall,
          "One contending access in a declared simultaneous X/Y group delays the fresh DMA read");
  static_cast<void>(bus.advance_dma(inputs({.index = 2, .requests = 1}), use()));
  const auto complete = bus.advance_dma(inputs({.index = 3, .requests = 1}), use());
  require(complete.word && complete.word->value == first,
          "Grouped occupancy never becomes two invented DMA clocks or repeated request samples");
}
}

void native_checks() {
  native_controller_query();
  native_held_word();
  unqualified_conflict();
  caller_qualification();
}
}
