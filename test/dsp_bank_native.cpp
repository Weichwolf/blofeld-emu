// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <string>
#include <vector>

namespace blofeld::test::banks {
namespace {
using esai::rejects;
using esai::require;
constexpr unsigned immediate_x0 = 0x44f400;
constexpr unsigned source_to_a = 0x084e2f;
constexpr unsigned and_immediate = 0x0140c6;
constexpr unsigned compare_x0_a = 0x200045;
constexpr unsigned store_x0 = 0x447000;
constexpr unsigned load_y0 = 0x4ef000;
constexpr unsigned effect_limit = 16;

void healthy(const OwnDspMachine &machine, const dsp56300::InstructionProgress &progress) {
  if (progress.fault.kind == dsp56300::FaultKind::none) { return; }
  constexpr std::array spaces{"P:", "X:", "Y:"};
  const auto fault = progress.fault;
  std::string operation = "execute";
  if (fault.kind == dsp56300::FaultKind::memory_read ||
      fault.kind == dsp56300::FaultKind::io_read) {
    operation = "read";
  } else if (fault.kind == dsp56300::FaultKind::memory_write ||
             fault.kind == dsp56300::FaultKind::io_write) {
    operation = "write";
  } else if (fault.kind == dsp56300::FaultKind::io_bit) {
    operation = "bit";
  }
  throw Fault("Bank native " + operation + " " +
              std::string(spaces.at(static_cast<unsigned>(fault.space))) + hex(fault.address, 6) +
              " pc=" + hex(fault.pc, 6) + " omr=" + hex(machine.cpu().registers().omr, 6) +
              " value=" + hex(fault.value, 6) +
              " tick=" + std::to_string(machine.clock().elapsed_ticks()));
}

dsp56300::PendingInstruction pending_read(const OwnDspMachine &machine) {
  const auto pending = machine.cpu().pending_instruction();
  if (!pending || pending->effect.kind != dsp56300::EffectKind::read ||
      pending->effect.space != dsp56300::Space::x || pending->effect.address != transfer::source0) {
    throw Fault("Native controller read lost its actual source-register ticket");
  }
  return *pending;
}

dsp56300::PendingInstruction seek_access(OwnDspMachine &machine,
                                         const dsp56300::EffectKind kind,
                                         const dsp56300::Space space,
                                         const unsigned address) {
  auto progress = machine.begin_fetch();
  for (unsigned count = 0; count < effect_limit; ++count) {
    healthy(machine, progress);
    if (!progress.pending || progress.retired) { break; }
    const auto pending = progress.pending.value();
    if (pending.effect.kind == kind && pending.effect.space == space &&
        pending.effect.address == address) {
      return pending;
    }
    progress = machine.advance(pending.ticket);
  }
  throw Fault("Native diagnostic did not yield its expected access at " + hex(address, 6) +
              " pc=" + hex(machine.cpu().registers().pc, 6));
}

void finish_pending(OwnDspMachine &machine) {
  const auto before = machine.cpu().progress().instructions;
  const auto ticks = machine.clock().elapsed_ticks();
  for (unsigned count = 0; count < effect_limit; ++count) {
    const auto pending = machine.cpu().pending_instruction();
    if (!pending) { throw Fault("Native bank instruction lost its pending effect"); }
    const auto result = machine.advance(pending->ticket);
    healthy(machine, result);
    if (result.retired) {
      require(machine.cpu().progress().instructions == before + 1 &&
                  machine.clock().elapsed_ticks() == ticks,
              "Exactly one genuine pending instruction retires without inventing clock periods");
      return;
    }
  }
  throw Fault("Native bank instruction exceeded its bounded effect count");
}

void finish_access(OwnDspMachine &machine, const dsp56300::PendingInstruction &pending) {
  const auto before = machine.cpu().progress().instructions;
  const auto ticks = machine.clock().elapsed_ticks();
  const auto result = machine.advance(pending.ticket);
  healthy(machine, result);
  if (!result.retired) { finish_pending(machine); }
  require(machine.cpu().progress().instructions == before + 1 &&
              machine.clock().elapsed_ticks() == ticks,
          "The retained access ticket executes once and retires without charging a clock period");
}

dsp56300::InstructionEffect source_store(OwnDspMachine &machine) {
  const auto pending =
      seek_access(machine, dsp56300::EffectKind::write, dsp56300::Space::x, transfer::ring_start);
  finish_access(machine, pending);
  require(machine.bus().read({.space = DspSpace::x, .address = transfer::ring_start}) ==
              pending.effect.value,
          "The genuine native source store reaches actual RAM before its observation");
  return pending.effect;
}

void native_controller_query() {
  // Authored FM MOVE/MOVEP/AND/CMP forms qualify both register-update orders.
  constexpr unsigned last_word = 0x67f;
  constexpr unsigned next_half = 0x680;
  constexpr unsigned phase = 0x20;
  OwnDspMachine machine(transfer::native_clocks);
  transfer::NativeProgram code;
  for (const auto word :
       std::array{immediate_x0, phase, source_to_a, and_immediate, phase, compare_x0_a}) {
    code.word(word);
  }
  transfer::reset_native(machine, code);
  static_cast<void>(transfer::dma(machine, inputs({.index = 0}), use()));
  machine.bus().write({.space = DspSpace::x, .address = last_word, .value = first});
  configure(machine.bus(), {.source = last_word, .control = transfer::software_copy});
  transfer::step_native(machine);
  const auto pending =
      seek_access(machine, dsp56300::EffectKind::read, dsp56300::Space::x, transfer::source0);
  const std::array core{
      DspMemoryAccess{.target = {.space = DspSpace::x, .address = pending.effect.address}}};
  const auto read = transfer::dma(machine, inputs({.index = 1}), use(core));
  require(read.phase && read.phase->kind == read_phase && !read.word && !read.core_stall,
          "A suspended native controller read leaves the actual DMA source path available");
  machine.bus().write({.space = DspSpace::x, .address = last_word, .value = second});
  const auto frozen = machine.checkpoint();
  for (const bool before_write : {true, false}) {
    for (unsigned replay = 0; replay < 2; ++replay) {
      machine.restore(frozen);
      const auto before = machine.cpu().registers();
      const auto progress = machine.cpu().progress();
      const auto restored = pending_read(machine);
      if (before_write) { finish_access(machine, restored); }
      const auto completed = transfer::dma(machine, inputs({.index = 2}), use(core));
      require(completed.word && completed.word->value == first && !completed.core_stall &&
                  machine.bus().read({.space = DspSpace::x, .address = transfer::source0}) ==
                      next_half &&
                  machine.bus().read({.space = DspSpace::y, .address = transfer::destination}) ==
                      first,
              "DMA publishes the next half and stores its original captured word");
      if (!before_write) {
        require(machine.cpu().registers() == before && machine.cpu().progress() == progress &&
                    pending_read(machine).effect == restored.effect,
                "DMA advances neither the native instruction nor its real pending ticket");
        finish_access(machine, restored);
      }
      constexpr unsigned accumulator_shift = 24;
      constexpr unsigned word_mask = 0xffffff;
      const auto observed = (machine.cpu().registers().a >> accumulator_shift) & word_mask;
      require(observed == (before_write ? last_word : next_half) &&
                  machine.cpu().progress().instructions == progress.instructions + 1,
              "Each native MOVEP samples its explicitly ordered actual controller value once");
      transfer::step_native(machine);
      transfer::step_native(machine);
      constexpr unsigned zero = 4;
      constexpr unsigned negative = 8;
      require((machine.cpu().registers().sr & (zero | negative)) ==
                      (before_write ? zero : negative) &&
                  ((machine.cpu().registers().a >> accumulator_shift) & word_mask) ==
                      (before_write ? phase : 0U) &&
                  machine.cpu().registers().x == phase &&
                  machine.cpu().progress().instructions == progress.instructions + 3,
              "Native AND/CMP retains independent operands, exact retired count and old/new N/Z");
    }
  }
}

void native_held_word() {
  constexpr unsigned prefix_units = 17;
  constexpr unsigned total_units = 19;
  OwnDspMachine machine(transfer::native_clocks);
  transfer::NativeProgram code;
  code.word(immediate_x0);
  code.word(first);
  for (const auto store :
       {DspStore{.space = DspSpace::x, .address = transfer::source0, .value = transfer::ring_start},
        DspStore{.space = DspSpace::x,
                 .address = transfer::destination0,
                 .value = transfer::destination},
        DspStore{.space = DspSpace::x, .address = transfer::counter0},
        DspStore{
            .space = DspSpace::x, .address = transfer::control0, .value = transfer::word_copy}}) {
    code.write(store);
  }
  code.word(store_x0);
  code.word(transfer::ring_start);
  const auto nop = code.pc();
  code.word(0);
  code.word(immediate_x0);
  code.word(0);
  code.word(store_x0);
  code.word(transfer::ring_start);
  const auto load = code.pc();
  code.word(load_y0);
  code.word(transfer::destination);
  transfer::reset_native(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = transfer::ring_start, .value = second});
  machine.bus().write({.space = DspSpace::y, .address = transfer::destination, .value = second});
  static_cast<void>(transfer::dma(machine, inputs({.index = 0}), use()));
  for (unsigned index = 0; index < 5; ++index) { transfer::step_native(machine); }
  std::vector<dsp56300::InstructionEffect> stores;
  stores.push_back(source_store(machine));
  const std::array source_use{
      DspMemoryAccess{.target = {.space = DspSpace::x, .address = transfer::ring_start},
                      .operation = DspMemoryOperation::write}};
  const auto denied = transfer::dma(machine, inputs({.index = 1, .requests = 1}), use(source_use));
  require(denied.phase && denied.phase->kind == read_phase && !denied.phase->value &&
              !denied.word && !denied.core_stall && stores.size() == 1 &&
              stores.front().value == first,
          "The actual native source store wins the declared contended DMA read phase");
  transfer::step_native(machine);
  const std::array opcode_use{DspMemoryAccess{.target = {.address = nop}}};
  const auto captured =
      transfer::dma(machine, inputs({.index = 2, .requests = 1}), use(opcode_use));
  require(captured.phase && captured.phase->kind == read_phase && !captured.word &&
              !captured.core_stall,
          "Known P-ROM occupancy does not contend with the independent X-RAM source");
  transfer::step_native(machine);
  stores.push_back(source_store(machine));
  require(
      stores.size() == 2 && stores.back().value == 0 &&
          machine.cpu().progress().estimated_cycles == prefix_units &&
          machine.cpu().progress().instructions == 9 && machine.cpu().registers().pc == load,
      "Native overwrite occurs after DMA capture and before admitting the actual destination load");
  const auto pending =
      seek_access(machine, dsp56300::EffectKind::read, dsp56300::Space::y, transfer::destination);
  const std::array destination_use{
      DspMemoryAccess{.target = {.space = DspSpace::y, .address = pending.effect.address}}};
  const auto waited = transfer::dma(machine,
                                    inputs({.index = 3, .requests = 1, .grant = DspDmaGrant::wait}),
                                    use(destination_use));
  require(waited.phase && waited.phase->kind == write_phase && waited.phase->value == first &&
              waited.core_stall && !waited.word &&
              machine.bus().read({.space = DspSpace::y, .address = transfer::destination}) ==
                  second,
          "A globally waiting held word blocks the actual suspended native destination read");
  const auto frozen = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(frozen);
    const auto before = machine.cpu().registers();
    const auto progress = machine.cpu().progress();
    const auto saved_read = machine.cpu().pending_instruction();
    if (!saved_read) {
      throw Fault("The native destination read lost its actual admission ticket");
    }
    const auto completed =
        transfer::dma(machine, inputs({.index = 4, .requests = 1}), use(destination_use));
    const auto held = machine.cpu().pending_instruction();
    require(completed.core_stall && completed.word && completed.word->value == first &&
                completed.word->source == DspDmaAddress{.address = transfer::ring_start} &&
                completed.word->destination ==
                    DspDmaAddress{.space = DspDmaSpace::y, .address = transfer::destination} &&
                machine.cpu().registers() == before && machine.cpu().progress() == progress &&
                held && saved_read->effect == held->effect &&
                machine.bus().read({.space = DspSpace::x, .address = transfer::ring_start}) == 0,
            "DMA writes its original held word without consuming the actual native load ticket");
    finish_access(machine, *saved_read);
    require(machine.cpu().registers().y == first &&
                machine.cpu().progress().estimated_cycles == total_units &&
                machine.cpu().progress().instructions == 10 &&
                machine.bus().read({.space = DspSpace::x, .address = transfer::status}) ==
                    transfer::all_done,
            "Both genuine pending loads read the captured DMA word after its destination commit");
  }
}

void unqualified_conflict() {
  OwnDspMachine machine(transfer::native_clocks);
  transfer::NativeProgram code;
  code.word(0);
  transfer::reset_native(machine, code);
  require(rejects([&] {
            static_cast<void>(transfer::dma(machine, inputs({.index = 0}), {}));
          }).contains("pc=0xff0000"),
          "Missing occupancy qualification fails with native execution context");
  static_cast<void>(transfer::dma(machine, inputs({.index = 0}), use()));
  constexpr unsigned upper = 0xc000;
  constexpr unsigned other_section = 0xc400;
  machine.bus().write({.space = DspSpace::x, .address = upper, .value = first});
  machine.bus().write({.space = DspSpace::x, .address = other_section, .value = second});
  machine.bus().write({.space = DspSpace::y, .address = transfer::destination, .value = second});
  configure(machine.bus(), {.source = upper});
  const std::array unsafe{
      DspMemoryAccess{.target = {.space = DspSpace::x, .address = other_section}}};
  const auto error = rejects([&] {
    static_cast<void>(transfer::dma(machine, inputs({.index = 1, .requests = 1}), use(unsafe)));
  });
  require(error.contains("source read X:0x00c000 core=X:0x00c400") &&
              error.contains("pc=0xff0000") && error.contains("cycle=1 channel=0") &&
              machine.bus().read({.space = DspSpace::y, .address = transfer::destination}) ==
                  second,
          "Unsafe8K-subsection contention cannot invent a safe wait or commit destination I/O");
  const std::array safe{DspMemoryAccess{.target = {.space = DspSpace::x, .address = upper + 1}}};
  const auto denied = transfer::dma(machine, inputs({.index = 1, .requests = 1}), use(safe));
  require(denied.phase && !denied.word && !denied.core_stall,
          "The same uncommitted phase can instead qualify protected same1K contention");
  static_cast<void>(transfer::dma(machine, inputs({.index = 2, .requests = 1}), use()));
  const auto completed = transfer::dma(machine, inputs({.index = 3, .requests = 1}), use());
  require(completed.word && completed.word->value == first,
          "The captured request survives denial and completes without another rising edge");
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
