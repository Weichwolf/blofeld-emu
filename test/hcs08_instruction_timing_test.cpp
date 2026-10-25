// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned prefix = 0x9e, default_secondary = 0xfe;

// Independently transcribed RM Rev.2 Table6-13, pp130/131; high nibble=bytes,
// low nibble=unstretched periods. Zero excludes undefined forms and STOP/WAIT.
// Primary9E uses FE. This table does not establish native semantic coverage.
constexpr std::array<unsigned, byte_values> primary{
    0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x35,
    0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25,
    0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23,
    0x25, 0x35, 0x35, 0x25, 0x25, 0x24, 0x25, 0x25, 0x25, 0x25, 0x25, 0x37, 0x25, 0x24, 0x36, 0x25,
    0x11, 0x34, 0x15, 0x11, 0x11, 0x33, 0x11, 0x11, 0x11, 0x11, 0x11, 0x24, 0x11, 0x11, 0x35, 0x11,
    0x11, 0x34, 0x16, 0x11, 0x11, 0x24, 0x11, 0x11, 0x11, 0x11, 0x11, 0x24, 0x11, 0x11, 0x25, 0x11,
    0x25, 0x35, 0x11, 0x25, 0x25, 0x33, 0x25, 0x25, 0x25, 0x25, 0x25, 0x37, 0x25, 0x24, 0x34, 0x25,
    0x14, 0x25, 0x11, 0x14, 0x14, 0x25, 0x14, 0x14, 0x14, 0x14, 0x14, 0x26, 0x14, 0x13, 0x25, 0x14,
    0x19, 0x16, 0x15, 0x1b, 0x11, 0x11, 0x13, 0x12, 0x13, 0x12, 0x13, 0x12, 0x11, 0,    0,    0,
    0x23, 0x23, 0x23, 0x23, 0x12, 0x12, 0x35, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x35, 0x11,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0,    0x25, 0x22, 0x22,
    0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x25, 0x23, 0x23,
    0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x36, 0x34, 0x34,
    0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x36, 0x34, 0x34,
    0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x25, 0x23, 0x23,
    0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x12, 0x13, 0x13, 0x13, 0x13, 0x13, 0x15, 0x13, 0x12};

constexpr std::array<unsigned, byte_values> secondary{
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0x36, 0x46, 0,    0x36, 0x36, 0,    0x36, 0x36, 0x36, 0x36, 0x36, 0x48, 0x36, 0x35, 0,    0x36,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0x25, 0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0x46, 0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0x35, 0,
    0x45, 0x45, 0x45, 0x45, 0x45, 0x45, 0x45, 0x45, 0x45, 0x45, 0x45, 0x45, 0,    0,    0x45, 0x45,
    0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0,    0,    0x34, 0x34,
    0,    0,    0,    0x36, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0x35, 0x35};

void known_form(Memory &memory, const unsigned packed, const std::span<const std::uint8_t> code) {
  constexpr unsigned mask = 15;
  const auto bytes = packed >> 4U;
  const Hcs08InstructionTiming expected{.bytes = bytes, .periods = packed & mask};
  for (const unsigned ccr : {flags_fixed, byte_mask}) {
    memory.install(entry, code);
    require(hcs08_instruction_timing(
                memory, {.pc = entry, .ccr = static_cast<std::uint8_t>(ccr)}) == expected,
            "Independent opcode matrix mismatch for " + hex(code[0]) + "/" + hex(code[1]));
  }
  for (unsigned missing = 0; missing < bytes; ++missing) {
    memory.install(entry, code);
    memory.known.reset(entry + missing);
    const auto fault = rejected([&] {
      static_cast<void>(hcs08_instruction_timing(memory, {.pc = entry, .ccr = flags_fixed}));
    });
    require(fault.contains("fetch address=" + hex(entry + missing)) &&
                fault.contains("pc=" + hex(entry)) && fault.contains("ccr=" + hex(flags_fixed)),
            "Every unknown opcode/operand byte fails with exact execution context");
  }
  for (unsigned suffix = 1; suffix < bytes; ++suffix) {
    const auto pc = static_cast<std::uint16_t>(space_bytes - suffix);
    memory.install(pc, code.first(suffix));
    require(rejected([&] {
              static_cast<void>(hcs08_instruction_timing(memory, {.pc = pc}));
            }).contains("instruction fetch wraps"),
            "Every truncated end-of-address-space instruction refuses wrapping");
  }
}

void matrix(const bool prefixed) {
  Memory memory;
  unsigned accepted = 0;
  for (unsigned opcode = 0; opcode < byte_values; ++opcode) {
    const auto byte = static_cast<std::uint8_t>(opcode);
    const std::array<std::uint8_t, 4> code =
        prefixed ? std::array<std::uint8_t, 4>{prefix, byte, 0, 0}
                 : std::array<std::uint8_t, 4>{byte, default_secondary, 0, 0};
    const auto packed = (prefixed ? secondary : primary).at(opcode);
    if (packed != 0) {
      known_form(memory, packed, code);
      ++accepted;
    } else {
      memory.install(entry, code);
      const auto fault = rejected([&] {
        static_cast<void>(hcs08_instruction_timing(memory, {.pc = entry, .ccr = flags_fixed}));
      });
      require(fault.contains("unsupported") && fault.contains("opcode=" + hex(opcode)) &&
                  fault.contains("pc=" + hex(entry)) && fault.contains("ccr=" + hex(flags_fixed)),
              "Undefined/held opcode forms fail visibly without guessed costs");
    }
  }
  constexpr unsigned primary_forms = 252;
  constexpr unsigned secondary_forms = 47;
  require(accepted == (prefixed ? secondary_forms : primary_forms) && memory.accesses.empty() &&
              memory.retired == 0,
          "Complete matrix classification performs no architectural data accesses or retirement");
}
} // namespace

int main() {
  try {
    matrix(false);
    matrix(true);
    std::cout << "HCS08 documented timing matrix: 252 primary classifications, 47 prefix forms; "
                 "all unknown-byte and fetch-wrap boundaries verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
