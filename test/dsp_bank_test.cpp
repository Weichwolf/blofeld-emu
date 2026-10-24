// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_dma.hpp"
#include "dma_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace blofeld::test::banks {
namespace {
using esai::rejects;
using esai::require;

void controller_reads() {
  // FM10.5/figure10-1: six four-register channels, four offsets and DSTR at the top.
  constexpr unsigned first_controller = 0xffffd8;
  constexpr unsigned last_controller = 0xfffff4;
  for (unsigned address = first_controller; address <= last_controller; ++address) {
    DspBus bus;
    const std::array core{DspMemoryAccess{.target = {.address = p_rom_begin}},
                          DspMemoryAccess{.target = {.space = DspSpace::x, .address = address}},
                          DspMemoryAccess{.target = {.space = DspSpace::y, .address = 0x20},
                                          .operation = DspMemoryOperation::write}};
    static_cast<void>(bus.advance_dma(inputs({.index = 0}), use(core)));
    bus.write({.space = DspSpace::x, .address = transfer::ring_start, .value = first});
    configure(bus, {.control = transfer::software_copy});
    const auto source = bus.advance_dma(inputs({.index = 1}), use(core));
    require(source.phase && source.phase->kind == read_phase && !source.word && !source.core_stall,
            "A controller read does not consume the DMA RAM source data path");
    const auto wait = bus.advance_dma(inputs({.index = 2, .grant = DspDmaGrant::wait}), use(core));
    require(wait.phase && wait.phase->value == first && !wait.word && !wait.core_stall,
            "Independent controller inspection cannot bypass a global destination wait");
    const auto destination = bus.advance_dma(inputs({.index = 3}), use(core));
    require(destination.word && destination.word->value == first && !destination.core_stall &&
                bus.read({.space = DspSpace::y, .address = transfer::destination}) == first &&
                bus.read({.space = DspSpace::x, .address = transfer::source0}) ==
                    transfer::ring_start + 1 &&
                bus.read({.space = DspSpace::x, .address = transfer::control0}) ==
                    (transfer::software_copy & ~transfer::enable),
            "Each declared controller read preserves actual DMA data and completion accounting");
    require(rejects([&] {
              static_cast<void>(bus.read({.space = DspSpace::x, .address = transfer::offset1}));
            }).contains("Unknown DSP DMA offset"),
            "Declaring register occupancy neither reads nor initializes unknown controller state");
  }
}

void unsupported_controller_use() {
  constexpr std::array cases{
      DspMemoryAccess{.target = {.space = DspSpace::y, .address = 0xffffef}},
      DspMemoryAccess{.target = {.space = DspSpace::x, .address = 0xffffd7}},
      DspMemoryAccess{.target = {.space = DspSpace::x, .address = 0xfffff5}},
      DspMemoryAccess{.target = {.space = DspSpace::x, .address = transfer::source0},
                      .operation = DspMemoryOperation::write},
      DspMemoryAccess{.target = {.space = DspSpace::x, .address = transfer::counter0},
                      .operation = DspMemoryOperation::write},
      DspMemoryAccess{.target = {.space = DspSpace::x, .address = transfer::control0},
                      .operation = DspMemoryOperation::write},
      DspMemoryAccess{.target = {.space = DspSpace::x, .address = transfer::status},
                      .operation = DspMemoryOperation::write}};
  for (const auto access : cases) {
    DspBus bus;
    const std::array core{access};
    const auto error =
        rejects([&] { static_cast<void>(bus.advance_dma(inputs({.index = 0}), use(core))); });
    require(error.contains("Unqualified DSP peripheral memory resource") &&
                error.contains(hex(access.target.address, 6)),
            "Wrong-bank, adjacent and controller-write occupancy fail before request sampling");
    static_cast<void>(bus.advance_dma(inputs({.index = 0}), use()));
  }
}

void memory_and_peripheral_phase(const DspAddress destination) {
  // FM10.2/APR23 section5: independent memory modules and peripheral data registers.
  constexpr unsigned value = 0x400;     // Also a valid XNADR payload.
  constexpr unsigned x_copy = 0x9802d0; // Software block, X->X, one linear word.
  constexpr unsigned y_copy = 0x9802d4; // Software block, X->Y, one linear word.
  const unsigned control = destination.space == DspSpace::x ? x_copy : y_copy;
  const std::array core{DspMemoryAccess{.target = {.address = p_rom_begin}},
                        DspMemoryAccess{.target = {.space = DspSpace::x, .address = 0x10}},
                        DspMemoryAccess{.target = {.space = DspSpace::y, .address = 0x20},
                                        .operation = DspMemoryOperation::write}};
  DspBus bus;
  static_cast<void>(bus.advance_dma(inputs({.index = 0}), use(core)));
  bus.write({.space = DspSpace::x, .address = transfer::ring_start, .value = value});
  configure(bus, {.destination = destination.address, .control = control});
  const auto read = bus.advance_dma(inputs({.index = 1}), use(core));
  require(read.phase && read.phase->kind == read_phase && !read.word && !read.core_stall,
          "Independent P/X/Y occupancy permits the actual source read");
  bus.write({.space = DspSpace::x, .address = transfer::ring_start, .value = first});
  const std::array collision{
      DspMemoryAccess{.target = destination, .operation = DspMemoryOperation::write}};
  require(rejects([&] {
            static_cast<void>(bus.advance_dma(inputs({.index = 2}), use(collision)));
          }).contains("Unqualified DSP peripheral memory resource write"),
          "A simultaneous core peripheral write still needs its own qualified collision model");
  const auto wait = bus.advance_dma(inputs({.index = 2, .grant = DspDmaGrant::wait}), use(core));
  require(wait.phase && wait.phase->kind == write_phase && wait.phase->value == value &&
              !wait.word && !wait.core_stall &&
              bus.read({.space = DspSpace::x, .address = transfer::source0}) ==
                  transfer::ring_start,
          "A waited peripheral phase retains its captured value without blocking memory core use");
  const auto completed = bus.advance_dma(inputs({.index = 3}), use(core));
  const auto target_space = destination.space == DspSpace::x ? DspDmaSpace::x : DspDmaSpace::y;
  require(completed.word && !completed.core_stall && completed.word->value == value &&
              completed.word->destination ==
                  DspDmaAddress{.space = target_space, .address = destination.address} &&
              bus.read({.space = DspSpace::x, .address = transfer::control0}) ==
                  (control & ~transfer::enable) &&
              bus.read({.space = DspSpace::x, .address = transfer::source0}) ==
                  transfer::ring_start + 1,
          "The real peripheral write completes once and retains ordinary DMA accounting");
  require(rejects([&] { static_cast<void>(bus.read(core.back().target)); }).contains("Unknown"),
          "Declaring memory occupancy does not execute or initialize a core operand");
}

void peripheral_destinations() {
  constexpr unsigned primary = 0xffffa0;
  constexpr unsigned secondary = 0xffff80;
  for (unsigned reg = 0; reg < 6; ++reg) {
    memory_and_peripheral_phase({.space = DspSpace::x, .address = primary + reg});
    memory_and_peripheral_phase({.space = DspSpace::y, .address = secondary + reg});
  }
  constexpr std::array dax_destinations{0xffffd1U, 0xffffd2U, 0xffffd3U};
  for (const auto reg : dax_destinations) {
    memory_and_peripheral_phase({.space = DspSpace::x, .address = reg});
  }
}

void forbidden_peripheral_phases() {
  struct Case {
    DspAddress target;
    unsigned control;
    bool source = false;
  };

  constexpr unsigned x_copy = 0x9802d0;
  constexpr unsigned y_copy = 0x9802d4;
  constexpr unsigned p_copy = 0x9802d8;
  constexpr std::array cases{
      Case{.target = {.space = DspSpace::x, .address = 0xffffa6}, .control = x_copy},
      Case{.target = {.space = DspSpace::y, .address = 0xffff86}, .control = y_copy},
      Case{.target = {.space = DspSpace::x, .address = 0xffffd0}, .control = x_copy},
      Case{.target = {.space = DspSpace::x, .address = 0xffffd4}, .control = x_copy},
      Case{.target = {.space = DspSpace::y, .address = 0xffffd2}, .control = y_copy},
      Case{
          .target = {.space = DspSpace::x, .address = 0xffff94}, .control = x_copy, .source = true},
      Case{.target = {.address = 0xffffa0}, .control = p_copy}};
  const std::array core{DspMemoryAccess{.target = {.address = p_rom_begin}}};
  for (const auto &item : cases) {
    DspBus bus;
    static_cast<void>(bus.advance_dma(inputs({.index = 0}), use(core)));
    bus.write({.space = DspSpace::x, .address = transfer::ring_start, .value = first});
    const Transfer setup{.source = item.source ? item.target.address : transfer::ring_start,
                         .destination = item.source ? transfer::destination : item.target.address,
                         .control = item.control};
    configure(bus, setup);
    if (!item.source) { static_cast<void>(bus.advance_dma(inputs({.index = 1}), use(core))); }
    const auto before_status = bus.read({.space = DspSpace::x, .address = transfer::status});
    for (const auto grant : {DspDmaGrant::wait, DspDmaGrant::available}) {
      const auto error = rejects([&] {
        static_cast<void>(
            bus.advance_dma(inputs({.index = item.source ? 1U : 2U, .grant = grant}), use(core)));
      });
      require(error.contains(item.target.space == DspSpace::p ? "Protected DSP ROM write"
                                                              : "Unsupported DSP DMA peripheral") &&
                  error.contains("channel=0") && error.contains(hex(item.target.address, 6)),
              "Neither a wait nor memory-only core occupancy qualifies forbidden DMA I/O");
      require(bus.read({.space = DspSpace::x, .address = transfer::source0}) == setup.source &&
                  bus.read({.space = DspSpace::x, .address = transfer::destination0}) ==
                      setup.destination &&
                  bus.read({.space = DspSpace::x, .address = transfer::control0}) ==
                      setup.control &&
                  bus.read({.space = DspSpace::x, .address = transfer::status}) == before_status,
              "Rejected phase admission retains the controller cycle, held word and endpoints");
    }
  }
}

void literal_resources() {
  struct Case {
    DspAddress target;
    DspMemoryResource expected;
  };

  constexpr std::array cases{
      Case{.target = {.space = DspSpace::x, .address = x_lower_last},
           .expected = {.bank = DspSpace::x, .word = x_lower_last, .block_words = 1024}},
      Case{.target = {.space = DspSpace::y, .address = y_lower_last},
           .expected = {.bank = DspSpace::y, .word = y_lower_last, .block_words = 1024}},
      Case{.target = {.space = DspSpace::x, .address = upper_begin},
           .expected = {.bank = DspSpace::x, .word = upper_begin, .block_words = 8192}},
      Case{.target = {.space = DspSpace::x, .address = x_upper_last},
           .expected = {.bank = DspSpace::x, .word = x_upper_last, .block_words = 8192}},
      Case{.target = {.space = DspSpace::y, .address = y_upper_last},
           .expected = {.bank = DspSpace::y, .word = y_upper_last, .block_words = 8192}},
      Case{.target = {.address = p_lower_last},
           .expected = {.word = p_lower_last, .block_words = 1024}},
      Case{.target = {.address = p_rom_begin}, .expected = {.rom = true, .word = p_rom_begin}},
      Case{.target = {.space = DspSpace::x, .address = 0x4000},
           .expected = {.bank = DspSpace::x, .rom = true, .word = 0x4000}}};
  DspBus bus;
  for (const auto &item : cases) {
    require(bus.memory_resource({.target = item.target}) == item.expected,
            "Resource identity/geometry matches the independent derivative memory map");
  }
  static_cast<void>(bus.memory_resource({.target = {.space = DspSpace::x, .address = 0x100}}));
  require(rejects([&] {
            static_cast<void>(bus.read({.space = DspSpace::x, .address = 0x100}));
          }).contains("Unknown"),
          "Resolving a physical resource never initializes its unknown contents");
  for (const auto address : {DspAddress{.address = 0x1000},
                             DspAddress{.space = DspSpace::x, .address = x_hole},
                             DspAddress{.space = DspSpace::y, .address = 0x2000}}) {
    static_cast<void>(
        rejects([&] { static_cast<void>(bus.memory_resource({.target = address})); }));
  }
  constexpr unsigned peripheral = 0xffff94;
  require(rejects([&] {
            static_cast<void>(
                bus.memory_resource({.target = {.space = DspSpace::x, .address = peripheral}}));
          }).contains("Unqualified DSP peripheral"),
          "The resolver does not guess peripheral-bus sharing or destructively read a FIFO");
  require(rejects([&] {
            static_cast<void>(
                bus.memory_resource({.target = {.space = DspSpace::x, .address = 0x4000},
                                     .operation = DspMemoryOperation::write}));
          }).contains("Protected DSP ROM write"),
          "The resource query retains strict ROM protection");
}

void partitions() {
  const DspBus bus;
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    const unsigned lower_blocks = space == DspSpace::x ? 12U : 8U;
    for (unsigned block = 0; block < lower_blocks; ++block) {
      const auto begin = block * 1024;
      const DspMemoryPair pair{
          .core = {.target = {.space = space, .address = begin}},
          .dma = {.target = {.space = space, .address = begin + section_last}}};
      require(bus.memory_conflict(pair) == DspMemoryConflict::protected_partition,
              "Every first/last word in each lower1K block has protected contention");
      if (block + 1 < lower_blocks) {
        require(
            bus.memory_conflict({.core = pair.core,
                                 .dma = {.target = {.space = space, .address = begin + 1024}}}) ==
                DspMemoryConflict::independent,
            "Adjacent lower banks can operate independently in the same data space");
      }
    }
    const unsigned upper_blocks = space == DspSpace::x ? 3U : 5U;
    for (unsigned block = 0; block < upper_blocks; ++block) {
      const auto base = upper_begin + (block * 8192);
      for (unsigned section = 0; section < 8; ++section) {
        const auto begin = base + (section * 1024);
        const DspMemoryAccess first_word{.target = {.space = space, .address = begin}};
        require(bus.memory_conflict(
                    {.core = first_word,
                     .dma = {.target = {.space = space, .address = begin + section_last}}}) ==
                    DspMemoryConflict::protected_partition,
                "Same1K subsection of each upper8K block remains protected");
        const auto other = base + (((section + 1) % 8) * 1024);
        require(bus.memory_conflict(
                    {.core = first_word, .dma = {.target = {.space = space, .address = other}}}) ==
                    DspMemoryConflict::unqualified,
                "Different1K subsections within one8K block cannot gain invented protection");
      }
    }
  }
  require(bus.memory_conflict({.core = {.target = {.space = DspSpace::x, .address = upper_begin}},
                               .dma = {.target = {.space = DspSpace::x, .address = upper_next}}}) ==
                  DspMemoryConflict::independent &&
              bus.memory_conflict({.core = {.target = {.space = DspSpace::x}},
                                   .dma = {.target = {.space = DspSpace::y}}}) ==
                  DspMemoryConflict::independent,
          "Different upper blocks and distinct X/Y storage are independent");
  require(bus.memory_conflict({.core = {.target = {.address = p_rom_begin}},
                               .dma = {.target = {.address = p_rom_begin + 1}}}) ==
              DspMemoryConflict::unqualified,
          "Unqualified ROM partition geometry never becomes a successful grant");
}

void program_partitions() {
  // DSP56371UM3.1: four lower P-RAM blocks of1K words; DCR DSS/DDS=2 selects P.
  constexpr unsigned program_copy = 0x88024a; // DE,DTM1,IRQ A,fixed P->fixed P.
  constexpr unsigned block_words = 1024;
  constexpr unsigned source_offset = 16;
  constexpr unsigned destination_offset = 32;
  for (unsigned block = 0; block < 4; ++block) {
    const auto source = (block * block_words) + source_offset;
    const auto destination = (((block + 1) % 4) * block_words) + destination_offset;
    for (const bool write_conflict : {false, true}) {
      DspBus bus;
      require(bus.memory_resource({.target = {.address = source}}) ==
                      DspMemoryResource{.word = source, .block_words = block_words} &&
                  bus.memory_conflict({.core = {.target = {.address = source}},
                                       .dma = {.target = {.address = destination}}}) ==
                      DspMemoryConflict::independent,
              "Every lower P-RAM block has its literal1K geometry and distinct neighbors");
      bus.write({.address = source, .value = first});
      bus.write({.address = destination, .value = second});
      static_cast<void>(bus.advance_dma(inputs({.index = 0}), use()));
      configure(bus, {.source = source, .destination = destination, .control = program_copy});
      const std::array source_use{DspMemoryAccess{.target = {.address = source + 1}}};
      const std::array destination_use{DspMemoryAccess{.target = {.address = destination + 1}}};
      const auto denied = bus.advance_dma(inputs({.index = 1, .requests = 1}), use(source_use));
      require(denied.phase && !denied.word && !denied.core_stall,
              "A pending P-RAM source phase waits for a core access to its same1K block");
      const auto captured =
          bus.advance_dma(inputs({.index = 2, .requests = 1}), use(destination_use));
      require(captured.phase && captured.phase->kind == read_phase && !captured.word &&
                  !captured.core_stall,
              "An independent P-RAM block permits the source read without stalling core");
      bus.write({.address = source, .value = 0});
      const auto waiting = bus.advance_dma(
          inputs({.index = 3, .requests = 1, .grant = DspDmaGrant::wait}), use(destination_use));
      require(waiting.core_stall && !waiting.word && waiting.phase &&
                  waiting.phase->value == first && bus.read({.address = destination}) == second,
              "A globally waiting P-RAM destination holds its partition and captured source");
      const auto complete = bus.advance_dma(inputs({.index = 4, .requests = 1}),
                                            use(write_conflict ? destination_use : source_use));
      require(complete.core_stall == write_conflict && complete.word &&
                  complete.word->source ==
                      DspDmaAddress{.space = DspDmaSpace::p, .address = source} &&
                  complete.word->destination ==
                      DspDmaAddress{.space = DspDmaSpace::p, .address = destination} &&
                  complete.word->value == first && bus.read({.address = destination}) == first,
              "A held write owns only its actual P-RAM partition, with no source reread");
    }
  }
}

void switched_alias() {
  DspBus bus;
  constexpr unsigned switched = 0x600085;
  constexpr unsigned physical = 0x10000;
  constexpr unsigned alias = 0x1000;
  bus.write({.space = DspSpace::x, .address = physical, .value = first});
  const auto before = bus.memory_resource({.target = {.space = DspSpace::x, .address = physical}});
  require(before == DspMemoryResource{.bank = DspSpace::x, .word = physical, .block_words = 8192},
          "The switchable block has a fixed physical X-bank identity");
  bus.observe_omr(switched);
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    require(rejects([&] {
              static_cast<void>(bus.memory_resource({.target = {.address = alias}}));
            }).contains("elapsed switch cycles"),
            "Pure resource queries do not consume the pending map-switch delay");
  }
  bus.advance_instruction_cycles(2);
  static_cast<void>(
      rejects([&] { static_cast<void>(bus.memory_resource({.target = {.address = alias}})); }));
  bus.advance_instruction_cycles(1);
  require(bus.memory_resource({.target = {.address = alias}}) == before &&
              bus.read({.address = alias}) == first,
          "The committed P alias names and reads the same physical X memory block");
  static_cast<void>(rejects([&] {
    static_cast<void>(bus.memory_resource({.target = {.space = DspSpace::x, .address = physical}}));
  }));
  require(bus.memory_conflict({.core = {.target = {.address = alias}},
                               .dma = {.target = {.address = alias + section_last}}}) ==
                  DspMemoryConflict::protected_partition &&
              bus.memory_conflict({.core = {.target = {.address = alias}},
                                   .dma = {.target = {.address = alias + 1024}}}) ==
                  DspMemoryConflict::unqualified,
          "P aliases use actual X1K subsection protection inside the shared8K block");
}
}
}

int main() {
  using namespace blofeld::test::banks;
  try {
    literal_resources();
    partitions();
    program_partitions();
    switched_alias();
    controller_reads();
    unsupported_controller_use();
    peripheral_destinations();
    forbidden_peripheral_phases();
    phase_checks();
    native_checks();
    std::cout
        << "Physical banks, selected DMA phase admission and native held-word replay passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
