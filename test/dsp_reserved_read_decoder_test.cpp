// SPDX-License-Identifier: GPL-3.0-only
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
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
#include <optional>
#include <stdexcept>
#include <string_view>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
namespace native = test::dma_test;
constexpr auto functional = DspWriteContract::functional_decode;
constexpr unsigned first_io = 0xffff80, row_size = 16, word_mask = 0xffffff;
constexpr unsigned payload = 0x123456, canary = 0xa55aa5, invalid_word = 0x1000000;
constexpr unsigned reserved_x = 0xffff80, gpio_f_data = 0xfffff5, vacant_y = 0x16000;
constexpr unsigned source = 0x660, destination = 0x900, destination_canary = 0x987654;
constexpr unsigned source0 = 0xffffef, destination0 = 0xffffee;
constexpr unsigned counter0 = 0xffffed, control0 = 0xffffec, copy_xy = 0x880244;
constexpr unsigned immediate_r1 = 0x61f400, indirect_y0 = 0x4ed900;
constexpr unsigned periods = 40, offsets = 25, cuts = 20;
constexpr std::array contracts{
    DspWriteContract::strict, DspWriteContract::internal_writable_storage, functional};
// Independent transcription of DSP56371UM table3-8; M includes unimplemented registers.
constexpr std::array<std::string_view, 8> y_rows{"MMMMMMMRMMMMRRRR",
                                                 "RRRMMMMMMMMMMMMM",
                                                 "MMMMMMMMMMMMMMMM",
                                                 "MMMMMMMMMRRRRRRR",
                                                 "RRRRRRRRRRRRRRRR",
                                                 "RRRRRRRRRRRRRRRR",
                                                 "RRRRRRRRRRRRRRRR",
                                                 "RRRRRMMMRRRRRRRR"};
constexpr unsigned reserved_slots = 76;

void decoder(const unsigned word) {
  DspBus bus;
  bus.set_write_contract(functional);
  bus.set_unselected_io_read_word(word);
  std::uint64_t count = 0;
  for (unsigned offset = 0; offset < y_rows.size() * row_size; ++offset) {
    if (y_rows.at(offset / row_size)[offset % row_size] != 'R') { continue; }
    const DspAddress target{.space = DspSpace::y, .address = first_io + offset};
    const auto resource = bus.memory_resource({.target = target});
    require(!resource.selected && !resource.rom && resource.bank == DspSpace::y &&
                resource.word == target.address && resource.block_words == 0 &&
                bus.unselected_reads() == count,
            "Resource lookahead neither selects storage nor counts a completed read");
    require(bus.read(target) == word && bus.unselected_reads() == ++count,
            "Every independently reserved Y slot returns the explicit virtual-instance word");
    bus.write({.space = target.space, .address = target.address, .value = payload});
    require(bus.read(target) == word && bus.unselected_reads() == ++count,
            "Unselected writes cannot create a readable register or RAM cell");
    require(bus.memory_conflict({.core = {.target = target}, .dma = {.target = {}}}) ==
                    DspMemoryConflict::independent &&
                bus.unselected_reads() == count,
            "Absent storage has no conflict with a real instruction fetch");
  }
  require(count == std::uint64_t{2} * reserved_slots && bus.unselected_writes() == reserved_slots,
          "All reserved Y slots were checked independently of the implementation decoder");
  const auto saved = bus;
  require(saved.unselected_io_read_word() == word && saved.unselected_reads() == count,
          "Trusted bus copies retain both configured word and completed-read observations");
  bus.reset();
  require(bus.unselected_io_read_word() == word && bus.unselected_reads() == 0 &&
              bus.read({.space = DspSpace::y, .address = word_mask}) == word,
          "Reset keeps the virtual instance and starts a new observation epoch");
}

void guards() {
  for (const auto contract : contracts) {
    DspBus bus;
    bus.set_write_contract(contract);
    const DspAddress target{.space = DspSpace::y, .address = word_mask};
    require(!bus.unselected_io_read_word() && !rejects([&] { (void)bus.read(target); }).empty(),
            "All contracts retain read refusal unless an explicit word is configured");
    bus.set_unselected_io_read_word(canary);
    if (contract != functional) {
      require(!rejects([&] { (void)bus.read(target); }).empty() &&
                  !rejects([&] { (void)bus.memory_resource({.target = target}); }).empty() &&
                  bus.unselected_reads() == 0,
              "An explicit word cannot bypass either strict diagnostic contract");
    }
    bus.set_unselected_io_read_word(std::nullopt);
    require(!rejects([&] { (void)bus.read(target); }).empty() && bus.unselected_reads() == 0,
            "Disabling the word restores refusal without fabricating a read");
  }
  DspBus bus;
  bus.set_write_contract(functional);
  bus.set_unselected_io_read_word(canary);
  require(!rejects([&] { bus.set_unselected_io_read_word(invalid_word); }).empty() &&
              bus.unselected_io_read_word() == canary,
          "An invalid 25-bit word fails before mutating the selected virtual instance");
  for (const auto target : {DspAddress{.space = DspSpace::x, .address = reserved_x},
                            DspAddress{.space = DspSpace::y, .address = gpio_f_data},
                            DspAddress{.space = DspSpace::y, .address = vacant_y},
                            DspAddress{.space = DspSpace::y},
                            DspAddress{.space = DspSpace::p, .address = word_mask},
                            DspAddress{.space = DspSpace::y, .address = invalid_word}}) {
    require(!rejects([&] { (void)bus.read(target); }).empty() && bus.unselected_reads() == 0,
            "Reserved X, assigned unknown Y, RAM, vacant storage and invalid addresses fail");
  }
  OwnDspIo io(bus);
  bool previous = true;
  const bool accepted =
      io.bit(dsp56300::Space::y, word_mask, native::boot, 0, dsp56300::BitAction::test, previous);
  const auto &failure = io.failure();
  require(!accepted && previous && failure && failure->pc == native::boot &&
              failure->address == word_mask && bus.unselected_reads() == 0,
          "The native bit interface cannot manufacture an old value from a word-read model");
  require(!rejects([&] {
             (void)bus.memory_resource({.target = {.space = DspSpace::y, .address = word_mask},
                                        .phase = DspMemoryPhase::peripheral_bit});
           }).empty(),
          "Bit resource admission preserves the unsupported peripheral guard");
}

native::NativeProgram program() {
  native::NativeProgram p;
  p.word(immediate_r1);
  p.word(word_mask);
  p.word(indirect_y0);
  p.word(indirect_y0);
  for (unsigned i = 0; i < periods; ++i) { p.word(0); }
  return p;
}

void seed(OwnDspMachine &machine, const unsigned word) {
  machine.bus().set_write_contract(functional);
  machine.bus().set_unselected_io_read_word(word);
  native::reset_native(machine, program());
  auto &bus = machine.bus();
  bus.write({.space = DspSpace::y, .value = payload});
  bus.write({.space = DspSpace::x, .address = source, .value = payload});
  bus.write({.space = DspSpace::y, .address = destination, .value = destination_canary});
  bus.write({.space = DspSpace::x, .address = source0, .value = source});
  bus.write({.space = DspSpace::x, .address = destination0, .value = destination});
  bus.write({.space = DspSpace::x, .address = counter0});
  bus.write({.space = DspSpace::x, .address = control0, .value = copy_xy});
}

struct Observation {
  unsigned copies = 0;
  unsigned overlaps = 0;
  bool loaded = false;
};

void advance(OwnDspMachine &machine,
             OwnDspTimeline &timeline,
             const unsigned cycle,
             const unsigned request,
             Observation &observation) {
  const auto count = machine.bus().unselected_reads();
  const auto before_sr = machine.cpu().registers().sr;
  const auto event = timeline.advance_one(timeline.next_deadline(),
                                          OwnDspBoundary::through,
                                          {.cycle = cycle,
                                           .known_requests = 15,
                                           .requests = cycle == request ? 1U : 0U,
                                           .timing = DspDmaTiming::reference_sampled_cycles,
                                           .grant = DspDmaGrant::available});
  if (!event) { throw std::runtime_error("Expected an actual native reference period"); }
  require(!event->dma.core_stall &&
              (!event->native || event->native->fault.kind == dsp56300::FaultKind::none),
          "Actual instruction and DMA periods retain their guards and make progress");
  require(machine.cpu().registers().sr == before_sr,
          "Native MOVE and NOP execution preserves every condition flag");
  observation.overlaps +=
      static_cast<unsigned>(event->dma.phase && machine.bus().unselected_reads() != count);
  if (event->dma.word) {
    require(event->dma.word->value == payload, "DMA transfers the real independent source word");
    ++observation.copies;
  }
  if (event->native && event->native->retired && event->native_pc == native::boot + 2) {
    const auto state = machine.cpu().registers();
    require(state.r[1] == 0 && (state.y & word_mask) == machine.bus().unselected_io_read_word() &&
                machine.bus().unselected_reads() == 1,
            "The real Y:(R1)+ load returns the declared word and wraps the 24-bit cursor");
    observation.loaded = true;
  }
}

void finish(OwnDspMachine &machine,
            OwnDspTimeline &timeline,
            const unsigned first,
            const unsigned request,
            Observation &observation) {
  for (unsigned cycle = first; cycle < periods; ++cycle) {
    advance(machine, timeline, cycle, request, observation);
  }
  const auto state = machine.cpu().registers();
  require(!timeline.faulted() && state.r[1] == 1 && (state.y & word_mask) == payload &&
              machine.bus().unselected_reads() == 1 && observation.loaded &&
              observation.copies == 1 &&
              machine.bus().read({.space = DspSpace::y, .address = destination}) == payload,
          "Both native loads execute, the canary is overwritten and real DMA completes once");
}

void native_overlap(const unsigned word) {
  unsigned overlaps = 0;
  for (unsigned request = 1; request <= offsets; ++request) {
    OwnDspMachine machine(native::native_clocks);
    seed(machine, word);
    OwnDspTimeline timeline(machine, OwnDspAutomaticTiming::one_effect_per_cycle, 0);
    Observation observation;
    finish(machine, timeline, 0, request, observation);
    overlaps += observation.overlaps;
  }
  require(overlaps == 2, "The actual reserved read overlaps both real DMA phase kinds");
}

void checkpoints() {
  constexpr unsigned request = 7;
  for (unsigned cut = 0; cut <= cuts; ++cut) {
    OwnDspMachine machine(native::native_clocks);
    seed(machine, canary);
    OwnDspTimeline timeline(machine, OwnDspAutomaticTiming::one_effect_per_cycle, 0);
    Observation prefix;
    for (unsigned cycle = 0; cycle < cut; ++cycle) {
      advance(machine, timeline, cycle, request, prefix);
    }
    const auto saved = timeline.checkpoint();
    auto original = prefix;
    finish(machine, timeline, cut, request, original);
    OwnDspMachine fresh(native::native_clocks);
    OwnDspTimeline restored(fresh, saved);
    auto replay = prefix;
    finish(fresh, restored, cut, request, replay);
    require(fresh.cpu().registers() == machine.cpu().registers() &&
                fresh.bus().unselected_io_read_word() == canary &&
                replay.overlaps == original.overlaps,
            "Fresh owners preserve in-flight read, DMA, decoder word and actual observations");
  }
}
}

int main() {
  try {
    guards();
    for (const unsigned word : {0U, word_mask, canary}) {
      decoder(word);
      native_overlap(word);
    }
    checkpoints();
    std::cout << "Reserved Y reads, native loads, DMA phases and fresh checkpoints passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
