// SPDX-License-Identifier: GPL-3.0-only
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_io_map.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
namespace native = test::dma_test;
constexpr auto functional = DspWriteContract::functional_decode;
constexpr unsigned first_io = 0xffff80, row_size = 16, word_mask = 0xffffff;
constexpr unsigned pattern = 0x13579b, canary = 0xa55a33, source = 0x660, destination = 0x900;
constexpr unsigned source0 = 0xffffef, destination0 = 0xffffee;
constexpr unsigned counter0 = 0xffffed, control0 = 0xffffec, enable = 0x800000;
// FM table10-2/6: word request A, fixed source/destination, X->Y or X->X.
constexpr unsigned copy_xy = 0x880244, copy_xx = 0x880240;
constexpr unsigned unimplemented_readonly = 0xffff94;
constexpr unsigned maximum_cycles = 100, prefix_nops = 8;
constexpr unsigned slots = 128, x_assigned = 86, y_assigned = 52;

// Independent row transcription of UM tables3-7/3-8, not a query of the decoder.
constexpr std::array<std::string_view, 8> x_rows{"RRMMMMMMMMMMMMMM",
                                                 "MMMMMRRRRRRRRRRR",
                                                 "MMMMMMMRMMMMRRRR",
                                                 "RRRMMMMMMMMMMMMM",
                                                 "RRRRRRRRRRRRRRRR",
                                                 "MMMMMMMMMMMMMMMM",
                                                 "MMMMMMMMMMMMMMMM",
                                                 "MMMMMMRRRRRMMMMM"};
constexpr std::array<std::string_view, 8> y_rows{"MMMMMMMRMMMMRRRR",
                                                 "RRRMMMMMMMMMMMMM",
                                                 "MMMMMMMMMMMMMMMM",
                                                 "MMMMMMMMMRRRRRRR",
                                                 "RRRRRRRRRRRRRRRR",
                                                 "RRRRRRRRRRRRRRRR",
                                                 "RRRRRRRRRRRRRRRR",
                                                 "RRRRRMMMRRRRRRRR"};

void all_reserved_slots() {
  DspBus bus;
  bus.set_write_contract(functional);
  std::uint64_t writes = 0;
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    const auto &rows = space == DspSpace::x ? x_rows : y_rows;
    for (unsigned offset = 0; offset < rows.size() * row_size; ++offset) {
      if (rows.at(offset / row_size)[offset % row_size] != 'R') { continue; }
      const DspAddress target{.space = space, .address = first_io + offset};
      const DspMemoryAccess access{.target = target, .operation = DspMemoryOperation::write};
      const auto resource = bus.memory_resource(access);
      require(!resource.selected && !resource.rom && resource.bank == space &&
                  resource.word == target.address && resource.block_words == 0 &&
                  bus.unselected_writes() == writes,
              "Each independently reserved slot selects no register or memory resource");
      for (const unsigned value : {0U, pattern, word_mask}) {
        bus.write({.space = space, .address = target.address, .value = value});
        ++writes;
        require(bus.unselected_writes() == writes &&
                    !rejects([&] { static_cast<void>(bus.read(target)); }).empty() &&
                    !rejects([&] {
                       static_cast<void>(bus.memory_resource({.target = target}));
                     }).empty(),
                "Actual discarded writes create neither readable cells nor register responses");
      }
      require(bus.memory_conflict(
                  {.core = access, .dma = {.target = {.space = DspSpace::x, .address = source}}}) ==
                  DspMemoryConflict::independent,
              "An unselected register cannot contend with a genuine RAM source");
    }
  }
  require(writes == std::uint64_t{3} * (slots - x_assigned + slots - y_assigned),
          "All reserved X/Y slots are counted independently of assigned-register implementation");
  const auto copy = bus;
  require(copy.write_contract() == functional && copy.unselected_writes() == writes,
          "Trusted copies retain the selected decoder and actual committed-write count");
  bus.reset();
  require(bus.write_contract() == functional && bus.unselected_writes() == 0,
          "Reset preserves the functional decoder and begins a new observation epoch");
}

void guards() {
  for (const auto contract :
       {DspWriteContract::strict, DspWriteContract::internal_writable_storage}) {
    DspBus bus;
    bus.set_write_contract(contract);
    require(rejects([&] {
              bus.write({.space = DspSpace::x, .address = first_io});
            }).contains("Unimplemented DSP peripheral write") &&
                bus.unselected_writes() == 0,
            "Both existing diagnostic contracts retain their reserved-I/O refusal");
  }
  DspBus bus;
  bus.set_write_contract(functional);
  for (const auto access : {DspStore{.space = DspSpace::x, .address = unimplemented_readonly},
                            DspStore{.space = DspSpace::x, .address = first_io, .value = 0x1000000},
                            DspStore{.space = DspSpace::x, .address = 0x1000000},
                            DspStore{.space = DspSpace::p, .address = first_io},
                            DspStore{.space = DspSpace::x, .address = 0x20000}}) {
    require(!rejects([&] { bus.write(access); }).empty() && bus.unselected_writes() == 0,
            "Assigned, invalid, P-space and external unmapped destinations still fail");
  }
  OwnDspIo io(bus);
  bool previous = true;
  const bool accepted =
      io.bit(dsp56300::Space::x, first_io, native::boot, 0, dsp56300::BitAction::set, previous);
  const auto &failure = io.failure();
  require(!accepted && previous && failure && failure->operation == OwnDspIo::Operation::bit &&
              failure->address == first_io && failure->pc == native::boot &&
              bus.unselected_writes() == 0,
          "A native RMW callback cannot invent an old reserved value or complete a store");
  require(!rejects([&] {
             static_cast<void>(
                 bus.memory_resource({.target = {.space = DspSpace::y, .address = first_io},
                                      .operation = DspMemoryOperation::write}));
           }).empty(),
          "The same Y address remains the actual assigned secondary ESAI register");
}

DspDmaInputs inputs(const unsigned cycle, const unsigned request) {
  return {.cycle = cycle,
          .known_requests = 15,
          .requests = request,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

struct Transfer {
  unsigned destination;
  unsigned control;
};

void configure(DspBus &bus, const Transfer transfer) {
  bus.write({.space = DspSpace::x, .address = source, .value = pattern});
  bus.write({.space = DspSpace::y, .address = destination, .value = canary});
  bus.write({.space = DspSpace::x, .address = source0, .value = source});
  bus.write({.space = DspSpace::x, .address = destination0, .value = transfer.destination});
  bus.write({.space = DspSpace::x, .address = counter0});
  bus.write({.space = DspSpace::x, .address = control0, .value = transfer.control});
}

void dma_destination() {
  DspBus bus;
  bus.set_write_contract(functional);
  configure(bus, {.destination = first_io, .control = copy_xx});
  const DspCoreBusUse idle{.accesses = {},
                           .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy};
  static_cast<void>(bus.advance_dma(inputs(0, 0), idle));
  const auto read = bus.advance_dma(inputs(1, 1), idle);
  const auto write = bus.advance_dma(inputs(2, 0), idle);
  require(
      read.phase && read.phase->kind == DspDmaPhaseKind::source_read && !read.word && write.phase &&
          write.phase->kind == DspDmaPhaseKind::destination_write && write.word &&
          write.word->value == pattern && bus.unselected_writes() == 1 && !bus.has_active_dma() &&
          bus.read({.space = DspSpace::x, .address = source}) == pattern,
      "Real DMA reads its source and completes an unselected destination without a register ack");
  require(!rejects([&] {
             static_cast<void>(bus.read({.space = DspSpace::x, .address = first_io}));
           }).empty(),
          "DMA completion cannot initialize a reserved read value");
}

native::NativeProgram program(const bool padding) {
  native::NativeProgram p;
  if (padding) {
    for (unsigned i = 0; i < prefix_nops; ++i) { p.word(0); }
  }
  p.write({.space = DspSpace::x, .address = first_io, .value = pattern});
  for (unsigned i = 0; i < maximum_cycles; ++i) { p.word(0); }
  return p;
}

void synchronous() {
  OwnDspMachine machine(native::native_clocks);
  machine.bus().set_write_contract(functional);
  native::reset_native(machine, program(false));
  const auto snapshot = machine.checkpoint();
  auto expected = machine.cpu().registers();
  expected.pc += 2;
  const auto result = machine.run_unclocked();
  require(result.dispatches == 1 && result.fault.kind == dsp56300::FaultKind::none &&
              machine.cpu().registers() == expected && machine.bus().unselected_writes() == 1,
          "A real MOVEP retires with the independent two-word PC effect and unchanged flags");
  OwnDspMachine fresh(native::native_clocks);
  fresh.restore(snapshot);
  native::step_native(fresh);
  require(fresh.cpu().registers() == expected && fresh.bus().unselected_writes() == 1 &&
              fresh.bus().write_contract() == functional,
          "A fresh native owner reproduces the genuine write without a peripheral response");
}

void overlap() {
  unsigned overlaps = 0;
  // Directed phase offsets cover both actual DMA phases around the native MOVEP.
  for (unsigned request = 1; request < 32; ++request) {
    OwnDspMachine machine(native::native_clocks);
    machine.bus().set_write_contract(functional);
    native::reset_native(machine, program(true));
    configure(machine.bus(), {.destination = destination, .control = copy_xy});
    OwnDspTimeline timeline(
        machine, OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts, 0);
    bool copied = false;
    bool retired = false;
    const auto store_pc = native::boot + prefix_nops;
    for (unsigned cycle = 0; cycle < maximum_cycles && (!copied || !retired); ++cycle) {
      const auto before = machine.cpu().registers();
      const auto writes = machine.bus().unselected_writes();
      const auto event = timeline.advance_one(timeline.next_deadline(),
                                              OwnDspBoundary::through,
                                              inputs(cycle, cycle == request ? 1U : 0U));
      if (!event) { throw std::runtime_error("Expected an actual native reference period"); }
      require(!event->dma.core_stall &&
                  (!event->native || event->native->fault.kind == dsp56300::FaultKind::none),
              "F1/F2 and an unselected I/O write remain independent of actual DMA phases");
      if (event->dma.word) {
        require(!copied && event->dma.word->value == pattern,
                "Exactly one real request transfers the independently supplied source value");
        copied = true;
      }
      overlaps += static_cast<unsigned>(machine.bus().unselected_writes() != writes &&
                                        event->dma.phase.has_value());
      if (event->native && event->native->retired && event->native_pc == store_pc) {
        auto expected = before;
        expected.pc = store_pc + 2;
        require(machine.cpu().registers() == expected,
                "Timed MOVEP preserves every non-PC register and condition flag");
        retired = true;
      }
    }
    require(copied && retired && !timeline.faulted() && machine.bus().unselected_writes() == 1 &&
                machine.bus().read({.space = DspSpace::y, .address = destination}) == pattern &&
                (machine.bus().read({.space = DspSpace::x, .address = control0}) & enable) == 0,
            "All authored offsets complete native execution and actual RAM transfer");
  }
  require(overlaps == 2, "Both source-read and destination-write overlap the real discarded write");
}
}

int main() {
  try {
    all_reserved_slots();
    guards();
    dma_destination();
    synchronous();
    overlap();
    std::cout << "Decoded reserved writes, strict guards, native execution and real DMA passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
