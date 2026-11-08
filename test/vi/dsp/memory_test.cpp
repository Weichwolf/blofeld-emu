// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/memory.hpp"

#include <array>
#include <cstddef>
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
    require(memory.write({.space = space, .address = ends[index], .value = datum}) ==
                    (space != Space::program) &&
                !memory.read(space, ends[index], value),
            "Discarding a reserved data store never creates extra readable RAM");
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
  require(x.size() == 2 && y.size() == 2 && x[0].address == 0 && y[0].address == 0 &&
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

void switched_maps() {
  struct Map {
    Word mode = 0, program_end = 0, x_end = 0, y_end = 0;
  };

  constexpr std::array maps{
      Map{.mode = 0, .program_end = 0x1000, .x_end = 0x12000, .y_end = 0x16000},
      Map{.mode = 0x80, .program_end = 0xb000, .x_end = 0x10000, .y_end = 0xe000},
      Map{.mode = 0x200080, .program_end = 0x9000, .x_end = 0x10000, .y_end = 0x10000},
      Map{.mode = 0x400080, .program_end = 0x5000, .x_end = 0x10000, .y_end = 0x14000},
      Map{.mode = 0x600080, .program_end = 0x3000, .x_end = 0x10000, .y_end = 0x16000}};
  constexpr std::size_t physical_words = std::size_t{88} * 1024;
  Memory memory;
  for (const auto map : maps) {
    memory.configure(map.mode);
    const auto x = memory.windows(Space::x);
    const auto y = memory.windows(Space::y);
    require(x[1].address + x[1].words.size() == map.x_end &&
                y[1].address + y[1].words.size() == map.y_end &&
                map.program_end + x[0].words.size() + x[1].words.size() + y[0].words.size() +
                        y[1].words.size() ==
                    physical_words,
            "Each table 3-6 mode partitions exactly the same 88K physical RAM words");
    require(memory.write({.space = Space::program, .address = map.program_end - 1, .value = 1}) &&
                !memory.write({.space = Space::program, .address = map.program_end, .value = 1}),
            "The program limit follows the selected banks");
    Word value = 0;
    require(memory.write({.space = Space::x, .address = map.x_end - 1, .value = 2}) &&
                memory.read(Space::x, map.x_end - 1, value) && value == 2 &&
                !memory.read(Space::x, map.x_end, value),
            "Data RAM ends at its actual bank limit, not at an allocation size");
  }
}

void retained_aliases() {
  constexpr Word x_address = 0x10003;
  constexpr Word p_address = 0x1003;
  constexpr Word switched = 0x600080;
  constexpr Word datum = 0x312456;
  Memory memory;
  require(memory.write({.space = Space::x, .address = x_address, .value = datum}),
          "Fill physical X RAM");
  memory.configure(switched);
  Word value = 0;
  require(!memory.read(Space::x, x_address, value) &&
              memory.read(Space::program, p_address, value) && value == datum &&
              memory.write({.space = Space::program, .address = p_address, .value = 7}),
          "The switched bank has one physical value and no simultaneous X alias");
  memory.configure(0);
  require(memory.read(Space::x, x_address, value) && value == 7 &&
              !memory.read(Space::program, p_address, value),
          "Switching back preserves the actual modified word");
  constexpr std::array<Word, 3> modes{0x80, 0x200080, 0x400080};
  constexpr std::array<Word, 3> y_addresses{0xe000, 0x10000, 0x14000};
  constexpr Word y_program = 0x3000;
  for (unsigned index = 0; index < modes.size(); ++index) {
    require(memory.write({.space = Space::y, .address = y_addresses[index], .value = datum}),
            "Fill a Y tail bank");
    memory.configure(modes[index]);
    require(memory.read(Space::program, y_program, value) && value == datum &&
                !memory.read(Space::y, y_addresses[index], value),
            "The selected physical Y tail becomes program RAM");
    memory.configure(0);
  }
}

void rom_and_reserved() {
  std::array<Word, 4> rom{1, 2, 3, 4};
  Memory memory({.x = rom, .y = {}});
  constexpr Word rom_first = 0x4000;
  constexpr Word reserved = 0x2000;
  constexpr Word peripheral = 0xffff80;
  Word value = 0;
  const auto windows = memory.windows(Space::x);
  require(windows.size() == 3 && !windows[1].writable && windows[1].known.empty() &&
              windows[1].words.data() == rom.data(),
          "Known immutable ROM is borrowed directly by native code");
  require(memory.read(Space::x, rom_first, value) && value == 1 &&
              memory.write({.space = Space::x, .address = rom_first, .value = 9}) &&
              memory.read(Space::x, rom_first, value) && value == 1 && rom[0] == 1,
          "Writes never replace supplied ROM contents");
  require(!memory.read(Space::y, rom_first, value) &&
              memory.write({.space = Space::y, .address = rom_first, .value = 0}) &&
              !memory.read(Space::y, rom_first, value),
          "Missing ROM stays unknown even after the original clear");
  require(
      memory.write({.space = Space::y, .address = reserved, .value = 0}) &&
          !memory.read(Space::y, reserved, value) &&
          !memory.write({.space = Space::x, .address = peripheral, .value = 0}),
      "The reserved-data sink never grants readable RAM or accepts unknown peripheral accesses");
  constexpr Word invalid = 0x1000000;
  std::array<Word, 1> invalid_rom{invalid};
  bool rejected = false;
  try {
    const Memory bad({.x = invalid_rom, .y = {}});
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Reject data ROM words wider than 24 bits before native execution");
}

}

int main() try {
  contents_and_boundaries();
  borrowed_windows();
  switched_maps();
  retained_aliases();
  rom_and_reserved();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
