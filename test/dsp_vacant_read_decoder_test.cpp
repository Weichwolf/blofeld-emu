// SPDX-License-Identifier: GPL-3.0-only
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
namespace native = test::dma_test;
constexpr auto functional = DspWriteContract::functional_decode;
constexpr unsigned internal_end = 0x20000;
constexpr unsigned rom_first = 0x4000;
constexpr unsigned rom_end = 0xc000;
constexpr unsigned x_low_end = 0x3000;
constexpr unsigned y_low_end = 0x2000;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned pattern = 0x13579b;
constexpr unsigned other = 0x2468ac;
constexpr unsigned vacant = 0x18000;
constexpr unsigned moved = 0x10000;
constexpr unsigned switched_p = 0x1000;
constexpr unsigned reserved_io = 0xffff80;
constexpr unsigned load_x1 = 0x45f000;
constexpr unsigned word_bits = 24;

struct Layout {
  unsigned omr, x_end, y_end;
};

// Independent intervals transcribed from DSP56371UM tables3-1..3-6.
constexpr std::array layouts{Layout{.omr = 5, .x_end = 0x12000, .y_end = 0x16000},
                             Layout{.omr = 0x600085, .x_end = 0x10000, .y_end = 0x16000},
                             Layout{.omr = 0x400085, .x_end = 0x10000, .y_end = 0x14000},
                             Layout{.omr = 0x200085, .x_end = 0x10000, .y_end = 0x10000},
                             Layout{.omr = 0x000085, .x_end = 0x10000, .y_end = 0xe000}};

void sweep_layout(const Layout layout, const unsigned word) {
  DspBus bus;
  bus.set_write_contract(functional);
  bus.set_unselected_data_read_word(word);
  bus.observe_omr(layout.omr);
  bus.advance_instruction_cycles(3);
  std::uint64_t count = 0;
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    const auto lower = space == DspSpace::x ? x_low_end : y_low_end;
    const auto upper = space == DspSpace::x ? layout.x_end : layout.y_end;
    unsigned absent = 0;
    for (unsigned address = 0; address < internal_end; ++address) {
      const bool ram = address < lower || (address >= rom_end && address < upper);
      const bool rom = address >= rom_first && address < rom_end;
      const DspAddress target{.space = space, .address = address};
      const auto resource = bus.memory_resource({.target = target});
      require(resource.selected == (ram || rom) && bus.unselected_reads() == count,
              "Resource queries preserve the independent map and do not consume reads");
      if (ram || rom) { continue; }
      ++absent;
      require(resource.bank == space && resource.word == address && !resource.rom &&
                  resource.block_words == 0 && bus.read(target) == word &&
                  bus.unselected_reads() == ++count,
              "Only a genuinely vacant logical address returns the explicit data word");
      bus.write({.space = space, .address = address, .value = other});
      require(bus.read(target) == word && bus.unselected_reads() == ++count,
              "A discarded vacant write cannot create RAM or change the configured read word");
    }
    require(absent == internal_end - lower - (rom_end - rom_first) - (upper - rom_end),
            "The full census covers every vacant address in the independent memory map");
  }
  const auto copy = bus;
  require(copy.unselected_data_read_word() == word && copy.unselected_reads() == count,
          "Trusted bus copies retain the instance word and actual committed-read count");
  bus.reset();
  require(bus.unselected_data_read_word() == word && bus.unselected_reads() == 0,
          "Reset preserves the explicit virtual instance and starts a new observation epoch");
}

void sweep(const unsigned word) {
  for (const auto layout : layouts) { sweep_layout(layout, word); }
}

void guards() {
  for (const auto contract :
       {DspWriteContract::strict, DspWriteContract::internal_writable_storage, functional}) {
    DspBus bus;
    bus.set_write_contract(contract);
    const DspAddress target{.space = DspSpace::x, .address = vacant};
    require(!bus.unselected_data_read_word() && !rejects([&] { (void)bus.read(target); }).empty(),
            "No decoder contract invents a readable vacant word by default");
    bus.set_unselected_data_read_word(pattern);
    if (contract != functional) {
      require(!rejects([&] { (void)bus.read(target); }).empty() && bus.unselected_reads() == 0,
              "A configured word cannot bypass either strict diagnostic write contract");
    }
    bus.set_unselected_data_read_word(std::nullopt);
    require(!rejects([&] { (void)bus.read(target); }).empty(),
            "Removing the explicit word restores refusal");
  }
  DspBus bus;
  bus.set_write_contract(functional);
  bus.set_unselected_data_read_word(pattern);
  require(!rejects([&] { bus.set_unselected_data_read_word(word_mask + 1); }).empty() &&
              bus.unselected_data_read_word() == pattern,
          "A wider-than-word value rejects before mutating the decoder instance");
  for (const auto target : {DspAddress{},
                            DspAddress{.space = DspSpace::x},
                            DspAddress{.space = DspSpace::y},
                            DspAddress{.space = DspSpace::x, .address = rom_first},
                            DspAddress{.space = DspSpace::y, .address = rom_first},
                            DspAddress{.space = DspSpace::x, .address = internal_end},
                            DspAddress{.space = DspSpace::x, .address = reserved_io},
                            DspAddress{.space = DspSpace::y, .address = word_mask},
                            DspAddress{.space = DspSpace::x, .address = word_mask + 1}}) {
    require(!rejects([&] { (void)bus.read(target); }).empty() && bus.unselected_reads() == 0,
            "P, unknown selected RAM/ROM, I/O and addresses outside low128K stay strict");
  }
  bus.set_unselected_io_read_word(other);
  require(bus.read({.space = DspSpace::y, .address = word_mask}) == other,
          "The separate reserved-I/O word cannot be replaced by the vacant-data word");
  const DspMemoryAccess absent{.target = {.space = DspSpace::x, .address = vacant}};
  require(bus.memory_conflict({.core = absent, .dma = {.target = {}}}) ==
                  DspMemoryConflict::independent &&
              bus.unselected_reads() == 1,
          "Vacant read lookahead selects no module and cannot conflict with a real P fetch");
  require(!rejects([&] {
             (void)bus.memory_resource(
                 {.target = absent.target, .phase = DspMemoryPhase::peripheral_bit});
           }).empty(),
          "A word-read hypothesis cannot invent a peripheral bit-operation resource");
  const std::array known_rom{other};
  bus.install_rom({.space = DspSpace::x, .start = rom_first, .words = known_rom});
  bus.write({.space = DspSpace::x, .value = other});
  require(
      bus.read({.space = DspSpace::x}) == other &&
          bus.read({.space = DspSpace::x, .address = rom_first}) == other &&
          bus.unselected_reads() == 1,
      "Known selected RAM and ROM retain actual stored values independently of the decoder word");
}

void switching() {
  DspBus bus;
  bus.set_write_contract(functional);
  bus.set_unselected_data_read_word(other);
  bus.write({.space = DspSpace::x, .address = moved, .value = pattern});
  bus.observe_omr(layouts[1].omr);
  const DspAddress target{.space = DspSpace::x, .address = moved};
  for (unsigned cycle = 0; cycle < 3; ++cycle) {
    require(rejects([&] { (void)bus.read(target); }).contains("elapsed switch cycles") &&
                rejects([&] {
                  (void)bus.memory_resource({.target = target});
                }).contains("elapsed switch cycles") &&
                bus.unselected_reads() == 0,
            "A pending switch remains unknown even when its eventual address will be vacant");
    bus.advance_instruction_cycles(1);
  }
  require(bus.read(target) == other && bus.read({.address = switched_p}) == pattern,
          "A vacant X read does not expose the physical word now selected by P");
  bus.write({.space = DspSpace::x, .address = moved, .value = other});
  bus.observe_omr(layouts[0].omr);
  bus.advance_instruction_cycles(3);
  require(bus.read(target) == pattern &&
              rejects([&] {
                (void)bus.read({.space = DspSpace::x, .address = moved + 1});
              }).contains("Unknown DSP memory read"),
          "Vacant accesses neither overwrite hidden physical RAM nor initialize unknown neighbors");
}

native::NativeProgram program() {
  native::NativeProgram result;
  result.word(load_x1);
  result.word(vacant);
  for (unsigned i = 0; i < 16; ++i) { result.word(0); }
  return result;
}

void seed(OwnDspMachine &machine, const unsigned word) {
  machine.bus().set_write_contract(functional);
  machine.bus().set_unselected_data_read_word(word);
  native::reset_native(machine, program());
}

void native_read(const unsigned word) {
  OwnDspMachine machine(native::native_clocks);
  seed(machine, word);
  const auto saved = machine.checkpoint();
  const auto flags = machine.cpu().registers().sr;
  native::step_native(machine);
  const auto expected = machine.cpu().registers();
  require((expected.x >> word_bits) == word && expected.sr == flags &&
              expected.pc == native::boot + 2 && machine.bus().unselected_reads() == 1,
          "The actual native LOAD consumes precisely the configured word and preserves CCR");
  OwnDspMachine fresh(native::native_clocks);
  fresh.restore(saved);
  native::step_native(fresh);
  require(fresh.cpu().registers() == expected && fresh.bus().unselected_data_read_word() == word &&
              fresh.bus().unselected_reads() == 1,
          "A fresh owner restores the decoder and actually reexecutes the same native read");
  fresh.restore(saved);
  fresh.bus().set_unselected_data_read_word(std::nullopt);
  const auto stopped = fresh.run_unclocked();
  require(stopped.fault.kind == dsp56300::FaultKind::memory_read &&
              stopped.fault.address == vacant && stopped.fault.pc == native::boot &&
              fresh.cpu().progress().instructions == 0 && fresh.bus().unselected_reads() == 0,
          "The identical native instruction still fails visibly when the model is disabled");
}

void timed(const unsigned word) {
  OwnDspMachine machine(native::native_clocks);
  seed(machine, word);
  OwnDspTimeline timeline(
      machine, OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts, 0);
  constexpr unsigned limit = 64;
  for (unsigned cycle = 0; cycle < limit && machine.cpu().progress().instructions == 0; ++cycle) {
    static_cast<void>(timeline.advance_one(timeline.next_deadline(),
                                           OwnDspBoundary::through,
                                           {.cycle = cycle,
                                            .timing = DspDmaTiming::reference_sampled_cycles,
                                            .grant = DspDmaGrant::available}));
  }
  require(machine.cpu().progress().instructions == 1 &&
              (machine.cpu().registers().x >> word_bits) == word &&
              machine.bus().unselected_reads() == 1 && !timeline.faulted() &&
              machine.clock().elapsed_ticks() > machine.clock().hardware_reset_ticks(),
          "Actual fetch/data/retirement periods use the decoder without fabricating RAM");
}
}

int main() {
  try {
    guards();
    switching();
    for (const unsigned word : {0U, pattern, word_mask}) {
      sweep(word);
      native_read(word);
      timed(word);
    }
    std::cout << "Vacant data decoder: five maps, strict selected-memory guards, native reads, "
                 "fresh checkpoints and reference periods passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
