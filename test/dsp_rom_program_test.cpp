// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp_rom_program.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
using Program = blofeld::test::DspRomProgram;
using Register = blofeld::test::DspRomRegister;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

template <class Operation> void rejects(Operation operation) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw std::runtime_error("Invalid ROM fixture input was accepted");
}

void immediate_fields() {
  // FM table12-14, MOVE/MOVEC fields; independently encoded authored snippets
  // using the pinned NXP6.3.28 assembler. No manufacturer program is included.
  struct Encoding {
    unsigned selector;
    std::uint32_t opcode;
  };

  constexpr std::array<Encoding, 38> encodings{
      {{.selector = 4, .opcode = 0x44f400},  {.selector = 5, .opcode = 0x45f400},
       {.selector = 6, .opcode = 0x46f400},  {.selector = 7, .opcode = 0x47f400},
       {.selector = 8, .opcode = 0x50f400},  {.selector = 9, .opcode = 0x51f400},
       {.selector = 10, .opcode = 0x52f400}, {.selector = 11, .opcode = 0x53f400},
       {.selector = 12, .opcode = 0x54f400}, {.selector = 13, .opcode = 0x55f400},
       {.selector = 14, .opcode = 0x56f400}, {.selector = 15, .opcode = 0x57f400},
       {.selector = 16, .opcode = 0x60f400}, {.selector = 17, .opcode = 0x61f400},
       {.selector = 18, .opcode = 0x62f400}, {.selector = 19, .opcode = 0x63f400},
       {.selector = 20, .opcode = 0x64f400}, {.selector = 21, .opcode = 0x65f400},
       {.selector = 22, .opcode = 0x66f400}, {.selector = 23, .opcode = 0x67f400},
       {.selector = 24, .opcode = 0x70f400}, {.selector = 25, .opcode = 0x71f400},
       {.selector = 26, .opcode = 0x72f400}, {.selector = 27, .opcode = 0x73f400},
       {.selector = 28, .opcode = 0x74f400}, {.selector = 29, .opcode = 0x75f400},
       {.selector = 30, .opcode = 0x76f400}, {.selector = 31, .opcode = 0x77f400},
       {.selector = 32, .opcode = 0x05f420}, {.selector = 33, .opcode = 0x05f421},
       {.selector = 34, .opcode = 0x05f422}, {.selector = 35, .opcode = 0x05f423},
       {.selector = 36, .opcode = 0x05f424}, {.selector = 37, .opcode = 0x05f425},
       {.selector = 38, .opcode = 0x05f426}, {.selector = 39, .opcode = 0x05f427},
       {.selector = 57, .opcode = 0x05f439}, {.selector = 58, .opcode = 0x05f43a}}};
  constexpr unsigned datum = 0x123456;
  for (const auto [selector, opcode] : encodings) {
    Program program;
    program.immediate(static_cast<Register>(selector), datum);
    const std::array expected{opcode, datum};
    require(
        std::ranges::equal(program.words(), expected) && program.pc() == Program::boot + 2,
        "Full-word immediate must retain the manual selector fields and exact instruction width");
  }
}

void rejected_inputs() {
  Program program;
  constexpr unsigned datum = 0x123456;
  constexpr unsigned wide = 0x1000000;
  program.word(datum);
  constexpr std::array invalid_selectors{0U, 3U, 40U, 56U, 59U, 64U};
  for (const auto selector : invalid_selectors) {
    rejects([&] { program.immediate(static_cast<Register>(selector), 0); });
  }
  rejects([&] { program.word(wide); });
  rejects([&] { program.immediate(Register::x0, wide); });
  constexpr std::array invalid_suffix{0U, wide};
  rejects([&] { program.append(invalid_suffix); });
  const std::array expected{datum};
  require(std::ranges::equal(program.words(), expected) && program.pc() == Program::boot + 1,
          "Refused selectors and wide words cannot leave a partial encoded instruction");
}

void image_bounds() {
  Program program;
  const std::vector<std::uint32_t> prefix(Program::maximum_words - 1, 0);
  program.append(prefix);
  rejects([&] { program.immediate(Register::x0, 0); });
  require(program.words().size() == Program::maximum_words - 1 &&
              program.pc() == Program::boot + Program::maximum_words - 1,
          "A two-word instruction cannot partially cross the ROM limit");
  program.word(Program::word_mask);
  rejects([&] { program.word(0); });
  require(program.words().size() == Program::maximum_words &&
              program.pc() == Program::boot + Program::maximum_words &&
              program.words().back() == Program::word_mask,
          "The final full-width word is retained and overflow leaves the complete image unchanged");
}
}

int main() {
  try {
    immediate_fields();
    rejected_inputs();
    image_bounds();
    std::cout << "38 independent immediate encodings and atomic ROM bounds passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
