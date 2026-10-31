// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_io_map.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
using namespace blofeld;
using blofeld::test::esai::rejects;
using blofeld::test::esai::require;

// Independent per-row transcription: tables3-7/3-8, lowest address first.
// M means assigned, including registers whose behavior remains unimplemented.
constexpr std::array<std::string_view, 8> x_rows{"RRMMMMMMMMMMMMMM",
                                                 "MMMMMRRRRRRRRRRR",
                                                 "MMMMMMMRMMMMRRRR",
                                                 "RRRMMMMMMMMMMMMM",
                                                 "RRRRRRRRRRRRRRRR",
                                                 "MMMMMMMMMMMMMMMM",
                                                 "MMMMMMMMMMMMMMMM",
                                                 "MMMMMMRRRRRMMMMM"};
constexpr std::array<std::string_view, 8> y_rows{"MMMMMMMRMMMMRRRR",
                                                 "RRRMMMMMMMMMMMMM",
                                                 "MMMMMMMMMMMMMMMM",
                                                 "MMMMMMMMMRRRRRRR",
                                                 "RRRRRRRRRRRRRRRR",
                                                 "RRRRRRRRRRRRRRRR",
                                                 "RRRRRRRRRRRRRRRR",
                                                 "RRRRRMMMRRRRRRRR"};
constexpr unsigned base = 0xffff80;
constexpr unsigned row_size = 16;
constexpr unsigned x_assigned = 86;
constexpr unsigned y_assigned = 52;

void all_slots() {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    const auto &rows = space == DspSpace::x ? x_rows : y_rows;
    unsigned count = 0;
    for (unsigned row = 0; row < rows.size(); ++row) {
      require(rows.at(row).size() == row_size, "Every manual row describes sixteen slots");
      for (unsigned column = 0; column < row_size; ++column) {
        const bool assigned = rows.at(row).at(column) == 'M';
        const auto expected = assigned ? DspIoSelection::assigned : DspIoSelection::reserved;
        require(dsp_io_selection({.space = space, .address = base + (row * row_size) + column}) ==
                    expected,
                "Every internal slot matches the independent derivative table");
        if (assigned) { ++count; }
      }
    }
    require(count == (space == DspSpace::x ? x_assigned : y_assigned),
            "Assigned totals include IDR, BIU, ROM patch and EFCOP registers");
  }
}

void boundaries_and_spaces() {
  constexpr unsigned one_past_word = 0x1000000;
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    require(dsp_io_selection({.space = space, .address = base - 1}) == DspIoSelection::outside &&
                dsp_io_selection({.space = space, .address = 0}) == DspIoSelection::outside,
            "Adjacent data addresses are outside the internal peripheral decoder");
    static_cast<void>(rejects(
        [&] { static_cast<void>(dsp_io_selection({.space = space, .address = one_past_word})); }));
  }
  for (unsigned offset = 0; offset < row_size * x_rows.size(); ++offset) {
    require(dsp_io_selection({.address = base + offset}) == DspIoSelection::outside,
            "Program ROM addresses never alias internal data-peripheral slots");
  }
}

void strict_bus_behavior() {
  DspBus bus;
  constexpr std::array reserved{DspAddress{.space = DspSpace::x, .address = base},
                                DspAddress{.space = DspSpace::x, .address = base + 1},
                                DspAddress{.space = DspSpace::y, .address = 0xffffb9}};
  for (const auto access : reserved) {
    static_cast<void>(rejects([&] { static_cast<void>(bus.read(access)); }));
    static_cast<void>(rejects(
        [&] { bus.write({.space = access.space, .address = access.address, .value = 0}); }));
    static_cast<void>(rejects([&] { static_cast<void>(bus.memory_resource({.target = access})); }));
  }
  constexpr unsigned patch_instruction = 0xffffa8;
  constexpr unsigned id_register = 0xfffff5;
  constexpr unsigned esai_control = 0xffff95;
  constexpr unsigned shi_clock = 0xffff90;
  require(dsp_io_selection({.space = DspSpace::y, .address = patch_instruction}) ==
                  DspIoSelection::assigned &&
              dsp_io_selection({.space = DspSpace::x, .address = id_register}) ==
                  DspIoSelection::assigned,
          "Unsupported documented registers are assigned, never classified as no-target writes");
  static_cast<void>(rejects(
      [&] { bus.write({.space = DspSpace::y, .address = patch_instruction, .value = 0}); }));
  static_cast<void>(rejects(
      [&] { static_cast<void>(bus.read({.space = DspSpace::x, .address = id_register})); }));
  require(bus.read({.space = DspSpace::y, .address = esai_control}) == 0 &&
              bus.read({.space = DspSpace::x, .address = shi_clock}) == 1,
          "Reserved/unsupported failures leave independent known ESAI/SHI reset registers intact");
}
}

int main() {
  try {
    all_slots();
    boundaries_and_spaces();
    strict_bus_behavior();
    std::cout << "DSP56371 internal I/O selection and strict unknown semantics passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
