// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hc12_minmax_fixture.hpp"
#include "mcu/hc12.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string_view>

namespace {
using namespace blofeld::test;
constexpr std::array flag_seeds{0U, 0xffU, 0x5aU, 0xa5U};
constexpr unsigned prefix = 0x18;

unsigned exhaustive_bytes() {
  MinmaxMemory memory;
  blofeld::Hc12 cpu(memory, BLOFELD_DEFAULT_SLA);
  unsigned cases = 0;
  for (const auto operation : minmax_operations) {
    if (operation.word) { continue; }
    const std::array code{static_cast<std::uint8_t>(prefix),
                          static_cast<std::uint8_t>(operation.opcode),
                          std::uint8_t{0}};
    memory.install(code);
    for (unsigned left = 0; left <= minmax_byte_mask; ++left) {
      for (unsigned right = 0; right <= minmax_byte_mask; ++right) {
        for (const auto flags : flag_seeds) {
          const MinmaxValues values{.left = left, .right = right, .flags = flags};
          memory.operand(right, false);
          cpu.set_state({.pc = minmax_entry,
                         .d = static_cast<std::uint16_t>(minmax_accumulator(operation, left)),
                         .x = minmax_source,
                         .y = minmax_initial_y,
                         .sp = minmax_initial_stack,
                         .ccr = static_cast<std::uint8_t>(flags)});
          cpu.step();
          try {
            check_minmax_result(memory, cpu, operation, values);
            require_minmax(
                cpu.state().pc == minmax_entry + code.size() && cpu.state().x == minmax_source &&
                    cpu.state().y == minmax_initial_y && cpu.state().sp == minmax_initial_stack,
                "Byte min/max must retain instruction length and base");
          } catch (const blofeld::Fault &error) {
            throw blofeld::Fault("Min/max opcode=" + blofeld::hex(operation.opcode, 2) + " left=" +
                                 blofeld::hex(left, 2) + " right=" + blofeld::hex(right, 2) +
                                 " flags=" + blofeld::hex(flags, 2) + ": " + error.what());
          }
          ++cases;
        }
      }
    }
  }
  return cases;
}
}

int main(const int argc, char **argv) {
  try {
    if (argc < 1 || argc > 2) { throw blofeld::Fault("Expected one optional min/max test mode"); }
    const std::string_view mode = argc == 1 ? "all" : argv[1];
    if (mode != "all" && mode != "bytes" && mode != "words" && mode != "timing" &&
        mode != "effects" && mode != "owner") {
      throw blofeld::Fault("Unknown min/max test mode");
    }
    unsigned bytes = 0;
    unsigned words = 0;
    unsigned timing = 0;
    unsigned effects = 0;
    unsigned machines = 0;
    if (mode == "all" || mode == "bytes") { bytes = exhaustive_bytes(); }
    if (mode == "all" || mode == "words") { words = minmax_word_cases(); }
    if (mode == "all" || mode == "timing") { timing = minmax_timing_cases(); }
    if (mode == "all" || mode == "effects") { effects = minmax_side_effect_cases(); }
    if (mode == "all" || mode == "owner") { machines = minmax_machine_cases(); }
    std::cout << "HC12 min/max: " << bytes << " byte cases, " << words << " word cases, " << timing
              << " timed cases, " << effects << " side-effect/fault cases, " << machines
              << " native owner cases passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
