// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dma.hpp"
#include "execution.hpp"

#include <array>
#include <cstddef>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
using blofeld::vi::dsp::Dma;
using dsp56300::Space;
using dsp56300::Word;
constexpr Word status_address = 0xfffff4, first_source = 0xffffef;
constexpr Word enable = 0x800000, software_block = 0x980000;
constexpr Word linear_xy = 0x2d4, fixed_xy = 0x244, three_d_xy = 0x604;
constexpr Word offset_zero = 0xfffff3, offset_one = 0xfffff2, offset_two = 0xfffff1;
constexpr Word all_done = 63, channel_zero_pending = 62;
constexpr Word linear_destination = 80, linear_source_end = 35, linear_destination_end = 83;
constexpr Word table_stride = 100, triple_source_end = 260;
constexpr Word dual_preset = 0x1002, second_row_source = 44, dual_source_end = 56;
constexpr unsigned audio_request = 22;
constexpr Word dax_audio = 0xffffd2, secondary_config_mask = 0x20004;
constexpr Word second_channel_source = 20;
constexpr Word activity_mask = 0xf00, channel_three_active = 0x700;
constexpr unsigned transfer_mode_shift = 19;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

Word read(const Dma &dma, const Word address) {
  Word value = 0;
  require(dma.read(address, value), "Read a defined DMA register");
  return value;
}

void put(Dma &dma, const Dma::Write access) {
  require(dma.write(access), "Write an implemented DMA register");
}

struct Configuration {
  unsigned channel = 0;
  Word source = 0, destination = 0, count = 0, control = 0;
};

void configure(Dma &dma, const Configuration config) {
  const auto base = first_source - (config.channel * 4);
  put(dma, {.address = base, .value = config.source});
  put(dma, {.address = base - 1, .value = config.destination});
  put(dma, {.address = base - 2, .value = config.count});
  put(dma, {.address = base - 3, .value = config.control});
}

struct Store {
  Dma::Address target;
  Word value, status;
};

struct Memory {
  std::vector<Store> stores;
  bool readable = true, writable = true;

  bool service(Dma &dma) {
    constexpr Word pc = 0x100;
    return dma.service(
        pc,
        [this](const Dma::Address source, Word &value) {
          value = source.address;
          return readable && source.space == Space::x;
        },
        [this, &dma](const Dma::Address target, const Word value) {
          if (!writable) { return false; }
          stores.push_back({.target = target, .value = value, .status = read(dma, status_address)});
          return true;
        });
  }
};

void reset_and_linear() {
  Dma dma;
  Word value = 0;
  require(read(dma, status_address) == all_done && read(dma, first_source - 3) == 0 &&
              !dma.read(first_source, value) && !dma.read(offset_zero, value),
          "Reset disables channels, marks them done and never invents operand/offset contents");
  configure(dma,
            {.source = 32,
             .destination = linear_destination,
             .count = 2,
             .control = software_block | linear_xy});
  require(dma.pending() && read(dma, status_address) == channel_zero_pending,
          "DE triggers a real pending software block and clears its done bit");
  Memory memory;
  require(memory.service(dma) && memory.stores.size() == 3 && !dma.pending(),
          "A two-count block transfers three real words");
  for (std::size_t index = 0; index < 3; ++index) {
    const auto &stored = memory.stores[index];
    require(stored.value == 32 + index && stored.target.address == linear_destination + index &&
                stored.target.space == Space::y,
            "Independent source/destination progression preserves every value");
  }
  require(read(dma, first_source) == linear_source_end &&
              read(dma, first_source - 1) == linear_destination_end &&
              read(dma, first_source - 2) == 2 && (read(dma, first_source - 3) & enable) == 0 &&
              read(dma, status_address) == all_done,
          "Only completed transfers reload DCO, disable a finite block and publish done");
}

void multidimensional() {
  constexpr std::array<Word, 12> expected{32, 33, 34, 44, 45, 46, 146, 147, 148, 158, 159, 160};
  constexpr std::array<Word, 3> presets{0x1042, 0x40042, 0x41002};
  for (unsigned mode = 0; mode < presets.size(); ++mode) {
    Dma dma;
    put(dma, {.address = offset_zero, .value = 10});
    put(dma, {.address = offset_one, .value = table_stride});
    configure(dma,
              {.source = 32,
               .destination = 7,
               .count = presets[mode],
               .control = software_block | three_d_xy | (mode << 4U)});
    Memory memory;
    require(memory.service(dma) && memory.stores.size() == expected.size(),
            "Each C/D/E layout expands to the same twelve-word matrix");
    for (std::size_t index = 0; index < expected.size(); ++index) {
      require(memory.stores[index].value == expected[index] &&
                  memory.stores[index].target.address == 7,
              "Triple counters apply the first/second offset at independent matrix boundaries");
    }
    require(read(dma, first_source) == triple_source_end &&
                read(dma, first_source - 2) == presets[mode],
            "The last triple transfer applies its final offset and reloads all fields");
  }
  Dma dma;
  put(dma, {.address = offset_two, .value = 10});
  constexpr Word dual_xy = 0x224;
  configure(dma, {.source = 32, .count = dual_preset, .control = software_block | dual_xy});
  Memory memory;
  require(memory.service(dma) && memory.stores.size() == 6 &&
              memory.stores[3].value == second_row_source &&
              read(dma, first_source) == dual_source_end,
          "Dual counters select their own DOR and reload the inner field");
}

void audio_ring_requests() {
  Dma dma;
  put(dma, {.address = offset_zero, .value = 1});
  constexpr Word negative_sixty_three = 0xffffc1;
  put(dma, {.address = offset_one, .value = negative_sixty_three});
  constexpr Word count = 63 << 6U;
  constexpr Word config = 0xaab604;
  constexpr Word source = 0x660;
  constexpr Word target = 0xffff80;
  configure(dma, {.source = source, .destination = target, .count = count, .control = config});
  Memory memory;
  require(!dma.pending() && memory.service(dma) && memory.stores.empty(),
          "Enabling a request-driven channel manufactures no transfers");
  for (unsigned index = 0; index < 128; ++index) {
    require(dma.trigger(audio_request) && memory.service(dma) &&
                memory.stores.size() == index + 1 &&
                memory.stores.back().value == source + (index % 64) &&
                memory.stores.back().target.address == target,
            "Each actual request transfers one ring word and rewinds after 64 words");
  }
  require(read(dma, first_source) == source && read(dma, first_source - 2) == count &&
              (read(dma, first_source - 3) & enable) != 0,
          "Repeated mode retains DE and reloads the circular source exactly");
  configure(dma,
            {.channel = 3,
             .source = source,
             .destination = dax_audio,
             .count = count,
             .control = config & ~secondary_config_mask});
  memory.stores.clear();
  require(
      dma.trigger(audio_request) && memory.service(dma) && memory.stores.size() == 2 &&
          memory.stores[0].target.space == Space::y && memory.stores[1].target.space == Space::x &&
          (memory.stores[0].status & activity_mask) == 0x100 &&
          (memory.stores[1].status & activity_mask) == channel_three_active,
      "Both matching channels run once, highest priority first, with distinct destination spaces");
}

void triggered_modes_and_chaining() {
  constexpr Word irq_enable = 0x400000;
  constexpr Word after_channel_zero = 4U << 11U;
  constexpr Word request_word = 1U << 19U;
  Dma chained;
  configure(chained,
            {.channel = 1,
             .source = 64,
             .control = enable | request_word | after_channel_zero | fixed_xy});
  configure(chained, {.source = 32, .control = software_block | irq_enable | fixed_xy});
  require(chained.interrupts() == 0, "Configuration never invents a completion interrupt");
  Memory chain_memory;
  require(chain_memory.service(chained) && chain_memory.stores.size() == 2 &&
              chain_memory.stores[0].value == 32 && chain_memory.stores[1].value == 64 &&
              chained.interrupts() == 1 && read(chained, status_address) == all_done,
          "An actual completed block triggers its dependent channel and latches its requested IRQ");
  Dma line;
  put(line, {.address = offset_zero, .value = 10});
  constexpr Word line_mode = 0x900204;
  configure(line, {.source = 32, .count = dual_preset, .control = line_mode});
  Memory line_memory;
  require(line.trigger(0) && line_memory.service(line) && line_memory.stores.size() == 3 &&
              read(line, first_source - 2) == 2 && (read(line, first_source - 3) & enable) != 0 &&
              line.trigger(0) && line_memory.service(line) && line_memory.stores.size() == 6 &&
              (read(line, first_source - 3) & enable) == 0,
          "Line mode consumes one inner row per request and stops at the full block boundary");
  for (const unsigned mode : {0U, 4U}) {
    Dma dma;
    configure(
        dma,
        {.source = 10, .count = 1, .control = enable | (mode << transfer_mode_shift) | linear_xy});
    Memory memory;
    require(
        !dma.pending() && dma.trigger(0) && memory.service(dma) && memory.stores.size() == 2 &&
            ((read(dma, first_source - 3) & enable) != 0) == (mode == 4),
        "External block modes consume the actual request and preserve DE only in repeated mode");
  }
  Dma wrapping;
  constexpr Word source_increment_xy = 0x254;
  configure(
      wrapping,
      {.source = dsp56300::word_mask, .count = 1, .control = software_block | source_increment_xy});
  Memory wrap_memory;
  require(wrap_memory.service(wrapping) && wrap_memory.stores.size() == 2 &&
              wrap_memory.stores[0].value == dsp56300::word_mask &&
              wrap_memory.stores[1].value == 0,
          "Address updates wrap at 24 guest bits without host overflow");
}

void arbitration_and_errors() {
  for (const bool continuous : {false, true}) {
    Dma dma;
    constexpr Word continuous_bit = 0x10000;
    const auto control = software_block | linear_xy | (continuous ? continuous_bit : 0);
    configure(dma, {.source = 10, .count = 2, .control = control});
    configure(dma,
              {.channel = 1,
               .source = second_channel_source,
               .count = 2,
               .control = software_block | linear_xy});
    Memory memory;
    require(memory.service(dma) && memory.stores.size() == 6 &&
                memory.stores[1].value == (continuous ? 11U : second_channel_source),
            "Equal priorities round-robin by word unless a continuous burst retains ownership");
  }
  Dma dma;
  configure(dma, {.source = 32, .count = 2, .control = software_block | fixed_xy});
  Memory memory;
  memory.writable = false;
  require(!memory.service(dma), "Reject an unavailable destination");
  const auto rejected = dma.failure();
  require(rejected && rejected->operation == blofeld::vi::Access::write && rejected->value == 32 &&
              read(dma, first_source) == 32 && read(dma, first_source - 2) == 2 &&
              read(dma, status_address) == channel_zero_pending,
          "A rejected destination never advances a pointer, decrements DCO or claims completion");
  dma.reset();
  memory.readable = false;
  configure(dma, {.source = 32, .control = software_block | fixed_xy});
  require(!memory.service(dma), "Reject an unavailable source");
  const auto missing = dma.failure();
  require(missing && missing->operation == blofeld::vi::Access::read && missing->address == 32 &&
              missing->pc == 0x100,
          "Unknown source faults preserve actual operand and execution context");
}
}

int main() try {
  reset_and_linear();
  multidimensional();
  audio_ring_requests();
  triggered_modes_and_chaining();
  arbitration_and_errors();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
