// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/interrupts.hpp"

#include <array>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::dsp::Interrupts;
using dsp56300::Word;
constexpr Word masked = 0x300, level_one_mask = 0x100, level_two_mask = 0x200;
constexpr Word dma0_vector = 0x18, timer0_vector = 0x54;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void mappings_and_masks() {
  struct Case {
    Word vector, address;
    unsigned shift;
  };

  constexpr auto core = Interrupts::core_address;
  constexpr auto peripheral = Interrupts::peripheral_address;
  constexpr std::array cases{Case{.vector = 0x10, .address = core, .shift = 0},
                             Case{.vector = 0x16, .address = core, .shift = 9},
                             Case{.vector = 0x18, .address = core, .shift = 12},
                             Case{.vector = 0x22, .address = core, .shift = 22},
                             Case{.vector = 0x30, .address = peripheral, .shift = 0},
                             Case{.vector = 0x44, .address = peripheral, .shift = 2},
                             Case{.vector = 0x28, .address = peripheral, .shift = 6},
                             Case{.vector = 0x58, .address = peripheral, .shift = 8},
                             Case{.vector = 0x78, .address = peripheral, .shift = 10},
                             Case{.vector = 0x68, .address = peripheral, .shift = 16}};
  for (const auto item : cases) {
    Interrupts controller;
    const std::array request{item.vector};
    require(!controller.select(request, 0),
            "Reset disables every source without discarding requests");
    require(controller.write({.address = item.address, .value = Word{2} << item.shift}) &&
                controller.select(request, level_one_mask) ==
                    Interrupts::Request{.vector = item.vector, .level = 1} &&
                !controller.select(request, level_two_mask) && !controller.select(request, masked),
            "Each primary register field maps IPL code 2 to CPU level 1 with its actual mask");
    require(controller.write({.address = item.address, .value = Word{3} << item.shift}) &&
                controller.select(request, level_two_mask) ==
                    Interrupts::Request{.vector = item.vector, .level = 2},
            "CPU mask equality admits a configured interrupt rather than dropping it");
  }
}

void external_modes() {
  Interrupts controller;
  require(!controller.external_edge(0x10), "Reset selects the level-sensitive external mode");
  for (const Word vector : {0x10U, 0x12U, 0x14U, 0x16U}) {
    const auto shift = ((vector - 0x10U) / 2) * 3;
    require(controller.write({.address = Interrupts::core_address, .value = Word{6} << shift}) &&
                controller.external_edge(vector) && !controller.external_edge(vector + 1),
            "IAL bit two selects the external negative-edge request independently of IPL");
  }
  require(!controller.external_edge(0) && !controller.external_edge(dma0_vector),
          "Non-external vectors have no external edge selector");
}

void priority_and_order() {
  Interrupts controller;
  require(controller.write({.address = Interrupts::core_address, .value = dsp56300::word_mask}) &&
              controller.write(
                  {.address = Interrupts::peripheral_address, .value = dsp56300::word_mask}),
          "Enable each priority group at the highest maskable level");

  struct Case {
    std::array<Word, 2> pending;
    Word expected;
  };

  constexpr std::array cases{Case{.pending = {0x54, 0x56}, .expected = 0x56},
                             Case{.pending = {0x5a, 0x54}, .expected = 0x54},
                             Case{.pending = {0x28, 0x44}, .expected = 0x44},
                             Case{.pending = {0x4c, 0x34}, .expected = 0x34},
                             Case{.pending = {0x40, 0x48}, .expected = 0x48},
                             Case{.pending = {0x78, 0x54}, .expected = 0x54},
                             Case{.pending = {0x68, 0x78}, .expected = 0x78},
                             Case{.pending = {0x22, 0x18}, .expected = 0x18}};
  for (const auto item : cases) {
    require(controller.select(item.pending, 0) ==
                Interrupts::Request{.vector = item.expected, .level = 2},
            "Equal IPL uses the documented peripheral order, including nonnumeric vector pairs");
  }
  constexpr Word dma_low_timer_high = 0x1000;
  constexpr Word timer_high = 0x300;
  require(controller.write({.address = Interrupts::core_address, .value = dma_low_timer_high}) &&
              controller.write({.address = Interrupts::peripheral_address, .value = timer_high}) &&
              controller.select(std::array<Word, 2>{dma0_vector, timer0_vector}, 0) ==
                  Interrupts::Request{.vector = timer0_vector, .level = 2},
          "Configured IPL precedes fixed source ordering");
  constexpr std::array<Word, 4> invalid{0, 4, 0x55, 0x24};
  require(!controller.select(invalid, 0),
          "Absent, reserved and odd vectors cannot become requests");
  controller.reset();
  require(!controller.select(std::array<Word, 1>{timer0_vector}, 0),
          "Reset restores disabled priority fields");
}
}

int main() try {
  mappings_and_masks();
  external_modes();
  priority_and_order();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
