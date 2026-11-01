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
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
namespace native = test::dma_test;
constexpr auto functional = DspWriteContract::internal_writable_storage;
constexpr unsigned internal_end = 0x20000, rom_first = 0x4000, rom_end = 0xc000;
constexpr unsigned x_low_end = 0x3000, y_low_end = 0x2000;
constexpr unsigned pattern = 0x13579b, other = 0x2468ac;
constexpr unsigned moved = 0x10000, switched_p = 0x1000;
constexpr unsigned immediate_x0 = 0x44f400, store_x0 = 0x447000, load_x1 = 0x45f000;
constexpr unsigned reserved_io = 0xffff80;

struct Layout {
  unsigned omr, x_end, y_end;
};

// Independent intervals from UM tables3-1..3-6; the contract supplies no reads.
constexpr std::array layouts{Layout{.omr = 5, .x_end = 0x12000, .y_end = 0x16000},
                             Layout{.omr = 0x600085, .x_end = 0x10000, .y_end = 0x16000},
                             Layout{.omr = 0x400085, .x_end = 0x10000, .y_end = 0x14000},
                             Layout{.omr = 0x200085, .x_end = 0x10000, .y_end = 0x10000},
                             Layout{.omr = 0x000085, .x_end = 0x10000, .y_end = 0xe000}};

void sweep() {
  for (const auto layout : layouts) {
    DspBus bus;
    bus.set_write_contract(functional);
    bus.observe_omr(layout.omr);
    bus.advance_instruction_cycles(3);
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      const unsigned lower = space == DspSpace::x ? x_low_end : y_low_end;
      const unsigned upper = space == DspSpace::x ? layout.x_end : layout.y_end;
      const std::array rom{pattern};
      bus.install_rom({.space = space, .start = rom_first, .words = rom});
      bus.install_rom({.space = space, .start = rom_end - 1, .words = rom});
      unsigned selected = 0;
      unsigned vacant = 0;
      const auto count = bus.unselected_writes();
      for (unsigned address = 0; address < internal_end; ++address) {
        const bool ram = address < lower || (address >= rom_end && address < upper);
        const DspMemoryAccess access{.target = {.space = space, .address = address},
                                     .operation = DspMemoryOperation::write};
        const auto resource = bus.memory_resource(access);
        require(resource.selected == ram, "Every internal write selects exactly its declared RAM");
        bus.write({.space = space, .address = address, .value = other ^ address});
        if (ram) {
          ++selected;
          require(bus.read(access.target) == (other ^ address),
                  "Every selected cell stores the actual nonzero test word");
        } else {
          ++vacant;
          require(resource.bank == space && resource.word == address && resource.block_words == 0,
                  "An unselected write has a logical address and no allocated bank geometry");
        }
      }
      require(selected == lower + upper - rom_end && selected + vacant == internal_end &&
                  bus.unselected_writes() == count + vacant,
              "Complete interval census includes every RAM, ROM and vacant address once");
      require(bus.read({.space = space, .address = rom_first}) == pattern &&
                  bus.read({.space = space, .address = rom_end - 1}) == pattern,
              "Sweeping writes cannot alter either installed data-ROM endpoint");
      for (const auto address : {lower, upper, internal_end - 1}) {
        require(rejects([&] {
                  static_cast<void>(bus.read({.space = space, .address = address}));
                }).contains("Unmapped DSP memory read"),
                "No writable cell also means no invented readable RAM");
      }
      require(rejects([&] {
                static_cast<void>(bus.read({.space = space, .address = rom_first + 1}));
              }).contains("Unknown DSP memory read"),
              "Discarded ROM writes never initialize missing table words");
    }
  }
}

void guards() {
  DspBus bus;
  require(bus.write_contract() == DspWriteContract::strict &&
              rejects([&] {
                bus.write({.space = DspSpace::y, .address = y_low_end});
              }).contains("Unmapped DSP memory write"),
          "Strict mode remains the default decoder");
  bus.set_write_contract(functional);
  require(rejects([&] {
            bus.write({.space = DspSpace::y, .address = internal_end});
          }).contains("Unmapped DSP memory write") &&
              rejects([&] {
                bus.write({.address = x_low_end});
              }).contains("Unmapped DSP memory write") &&
              rejects([&] {
                bus.write({.space = DspSpace::x, .address = reserved_io});
              }).contains("Unimplemented DSP peripheral write") &&
              rejects([&] {
                bus.write({.space = DspSpace::y, .address = y_low_end, .value = 0x1000000});
              }).contains("exceeds 24 bits"),
          "High memory, P, reserved I/O and invalid words retain their actual failures");
  constexpr unsigned first_invalid = static_cast<unsigned>(DspWriteContract::functional_decode) + 1;
  for (unsigned raw = first_invalid; raw <= std::numeric_limits<std::uint8_t>::max(); ++raw) {
    const auto invalid = std::bit_cast<DspWriteContract>(static_cast<std::uint8_t>(raw));
    static_cast<void>(rejects([&] { bus.set_write_contract(invalid); }));
  }
  require(bus.write_contract() == functional, "Invalid policy cannot replace the active contract");
  bus.write({.space = DspSpace::y, .address = y_low_end, .value = pattern});
  require(bus.unselected_writes() == 1, "A committed unselected store precedes reset");
  bus.reset();
  require(bus.write_contract() == functional && bus.unselected_writes() == 0,
          "Hardware reset retains the decoder contract and starts a new observation epoch");
  bus.set_write_contract(DspWriteContract::strict);
  require(rejects([&] {
            bus.write({.space = DspSpace::y, .address = y_low_end});
          }).contains("Unmapped DSP memory write"),
          "Strict diagnostic selection restores refusal without any fabricated memory");
}

void switched_storage() {
  DspBus bus;
  bus.set_write_contract(functional);
  bus.write({.space = DspSpace::x, .address = moved, .value = pattern});
  bus.observe_omr(layouts[1].omr);
  require(rejects([&] {
            bus.write({.space = DspSpace::x, .address = moved, .value = other});
          }).contains("elapsed switch cycles") &&
              rejects([&] {
                static_cast<void>(
                    bus.memory_resource({.target = {.space = DspSpace::x, .address = moved},
                                         .operation = DspMemoryOperation::write}));
              }).contains("elapsed switch cycles"),
          "Functional write decoding cannot bypass a pending memory-map transition");
  bus.advance_instruction_cycles(3);
  bus.write({.space = DspSpace::x, .address = moved, .value = other});
  require(bus.read({.address = switched_p}) == pattern,
          "An unselected X write cannot overwrite the physical bank relocated to P");
  bus.write({.address = switched_p, .value = other});
  bus.observe_omr(layouts[0].omr);
  bus.advance_instruction_cycles(3);
  require(bus.read({.space = DspSpace::x, .address = moved}) == other,
          "Real P writes retain their physical effects when the X bank returns");
}

native::NativeProgram program() {
  native::NativeProgram result;
  // Independent FM MOVE #imm,X0; MOVE X0,X:abs; MOVE X:abs,X1; NOP.
  for (const auto word : {immediate_x0, pattern, store_x0, x_low_end, load_x1, x_low_end, 0U}) {
    result.word(word);
  }
  return result;
}

void synchronous() {
  OwnDspMachine machine(native::native_clocks);
  machine.bus().set_write_contract(functional);
  native::reset_native(machine, program());
  native::step_native(machine);
  const auto saved = machine.checkpoint();
  const auto before = machine.cpu().registers();
  unsigned writes = 0;
  const DspIoWriteObserver observer{
      .context = &writes,
      .completed = [](void *context, const DspStore &access, const unsigned pc) {
        require(access.space == DspSpace::x && access.address == x_low_end &&
                    access.value == pattern && pc == native::boot + 2,
                "The observer retains the actual instruction's unselected write");
        ++*static_cast<unsigned *>(context);
      }};
  const auto result = machine.run_unclocked(observer);
  auto expected = before;
  expected.pc += 2;
  require(result.dispatches == 1 && result.fault.kind == dsp56300::FaultKind::none && writes == 1 &&
              machine.cpu().registers() == expected && machine.cpu().progress().instructions == 2,
          "A real unselected store retires with only the independent MOVE PC effect");
  OwnDspMachine fresh(native::native_clocks);
  fresh.restore(machine.checkpoint());
  require(fresh.bus().unselected_writes() == 1 && fresh.cpu().registers() == expected,
          "A nonzero committed-write observation survives a fresh diagnostic checkpoint");
  fresh.restore(saved);
  require(fresh.bus().unselected_writes() == 0,
          "Restoring the actual earlier boundary also restores its earlier observation count");
  native::step_native(fresh);
  require(fresh.bus().write_contract() == functional && fresh.cpu().registers() == expected &&
              fresh.bus().unselected_writes() == 1,
          "A fresh chip restore retains the decoder and actual native continuation");
  const auto read = fresh.run_unclocked();
  require(read.fault.kind == dsp56300::FaultKind::memory_read && read.fault.address == x_low_end &&
              read.fault.pc == native::boot + 4 && fresh.cpu().progress().instructions == 2,
          "A later actual CPU read still refuses the absent cell without retirement");
}

void timed() {
  OwnDspMachine machine(native::native_clocks);
  machine.bus().set_write_contract(functional);
  native::reset_native(machine, program());
  OwnDspTimeline timeline(
      machine, OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts, 0);
  for (unsigned cycle = 0; cycle < 64 && machine.cpu().progress().instructions < 2; ++cycle) {
    static_cast<void>(timeline.advance_one(timeline.next_deadline(),
                                           OwnDspBoundary::through,
                                           {.cycle = cycle,
                                            .timing = DspDmaTiming::reference_sampled_cycles,
                                            .grant = DspDmaGrant::available}));
  }
  require(machine.cpu().progress().instructions == 2 &&
              machine.cpu().registers().pc == native::boot + 4 &&
              machine.cpu().registers().x == pattern && !timeline.faulted() &&
              machine.clock().elapsed_ticks() > machine.clock().hardware_reset_ticks(),
          "Actual F1/F2/native cycles retire the unselected write without a fabricated RAM bank");
}
}

int main() {
  try {
    sweep();
    guards();
    switched_storage();
    synchronous();
    timed();
    std::cout << "Functional DSP write decoding and native execution passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
