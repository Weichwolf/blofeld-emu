// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/memory.hpp"

#include <array>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::dsp::Memory;
using dsp56300::Space;
using dsp56300::Word;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void contents_and_boundaries() {
  Memory memory;
  Word value = 0;
  for (const auto space : {Space::program, Space::x, Space::y}) {
    require(!memory.read(space, 0, value), "Power-up RAM is unknown in each Harvard space");
  }
  constexpr Word datum = 0x923456;
  constexpr std::array<Word, 3> ends{0x1000, 0x3000, 0x2000};
  constexpr std::array spaces{Space::program, Space::x, Space::y};
  for (unsigned index = 0; index < spaces.size(); ++index) {
    const auto space = spaces[index];
    const auto last = ends[index] - 1;
    require(memory.write({.space = space, .address = last, .value = datum}) &&
                memory.read(space, last, value) && value == datum,
            "The final documented invariant RAM word is readable after a real store");
    require(!memory.write({.space = space, .address = ends[index], .value = datum}) &&
                !memory.read(space, ends[index], value),
            "A missing bank or reserved address cannot turn into extra RAM");
  }
  require(memory.write({.space = Space::x, .value = 1}) &&
              memory.write({.space = Space::y, .value = 2}) &&
              memory.write({.space = Space::program, .value = 3}),
          "Each Harvard space stores its own word at the same address");
  for (unsigned index = 0; index < spaces.size(); ++index) {
    constexpr std::array<Word, 3> expected{3, 1, 2};
    require(memory.read(spaces[index], 0, value) && value == expected[index],
            "Unrelated program/X/Y words never alias");
  }
  constexpr Word oversized = 0x1000000;
  require(!memory.write({.space = Space::x, .value = oversized}) &&
              memory.read(Space::x, 0, value) && value == 1,
          "An invalid 25-bit store leaves a known word unchanged");
}

void borrowed_windows() {
  Memory memory;
  constexpr Word address = 7;
  constexpr Word datum = 0x654321;
  const auto x = memory.windows(Space::x);
  const auto y = memory.windows(Space::y);
  require(x.size() == 1 && y.size() == 1 && x[0].address == 0 && y[0].address == 0 &&
              x[0].writable && y[0].writable && !x[0].aliases_program && !y[0].aliases_program,
          "Data grants are stable writable low RAM with no executable alias");
  require(memory.windows(Space::program).empty() && x[0].known[address] == 0 &&
              y[0].known[address] == 0,
          "Data windows do not grant unknown program bytes or invented initialized data");
  x[0].words[address] = datum;
  x[0].known[address] = 1;
  Word value = 0;
  require(memory.read(Space::x, address, value) && value == datum &&
              !memory.read(Space::y, address, value),
          "Native stores update the exact consumer-owned backing storage and its knowledge");
  require(memory.write({.space = Space::y, .address = address, .value = datum}) &&
              y[0].known[address] == 1 && y[0].words[address] == datum,
          "Callback and native stores share the same data without copying");
}
}

int main() try {
  contents_and_boundaries();
  borrowed_windows();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
