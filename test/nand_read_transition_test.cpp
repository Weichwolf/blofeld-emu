// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_read_timing.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;

struct Vector {
  unsigned hz;
  std::uint64_t ce_release;
  std::uint64_t re_release;
  std::uint64_t access;
  std::uint64_t low;
  std::uint64_t hold;
};

constexpr std::array<Vector, 3> rows{
    {{.hz = 1000000000, .ce_release = 20, .re_release = 30, .access = 30, .low = 21, .hold = 15},
     {.hz = 1080000000, .ce_release = 22, .re_release = 33, .access = 33, .low = 23, .hold = 17},
     {.hz = 216000000, .ce_release = 5, .re_release = 7, .access = 7, .low = 5, .hold = 4}}};
constexpr std::uint64_t start = 1000;
constexpr std::uint64_t clearance = 100;
constexpr std::uint64_t margin = 10;
constexpr NandByte own{.value = 0xa3, .known = 0xff};

void require(const bool condition, const std::string_view label) {
  if (!condition) { throw Fault(std::string(label)); }
}

void query(const NandReadTiming &model, const std::optional<std::uint64_t> expected) {
  const auto copy = model;
  for (unsigned i = 0; i < 3; ++i) {
    require(model.next_transition() == expected && model == copy,
            "Query is complete-state read-only and deterministic");
  }
}

NandReadTiming initial(const Vector row, const bool ce) {
  NandReadTiming model(row.hz);
  query(model, std::nullopt);
  require(model.output(start) == NandReadOutput{}, "Unobserved output stays unknown");
  model.observe(start, {.ce_n = ce, .re_n = true});
  const auto due = start + (ce ? row.ce_release : row.re_release);
  query(model, due);
  require(model.output(due - 1) == NandReadOutput{},
          "Literal first release predecessor is unknown");
  query(model, due);
  require(model.output(due) == NandReadOutput{.driven = false},
          "Literal first release bound establishes Hi-Z");
  query(model, std::nullopt);
  if (ce) { model.observe(due, {.ce_n = false, .re_n = true}); }
  return model;
}

void selected_read(const Vector row, const bool ce) {
  auto model = initial(row, ce);
  unsigned calls = 0;
  const auto source = [&] {
    ++calls;
    return own;
  };
  const auto fall = start + clearance;
  model.observe(fall, {.ce_n = false, .re_n = false}, source);
  query(model, fall + row.access);
  require(model.output(fall + row.access - 1) == NandReadOutput{},
          "Literal access predecessor remains unknown");
  query(model, fall + row.access);
  require(model.output(fall + row.access) == NandReadOutput{.driven = true, .data = own},
          "Literal access event establishes driven own byte");
  query(model, std::nullopt);
  const auto rise = fall + row.access + margin;
  model.observe(rise, {.ce_n = false, .re_n = true}, source);
  query(model, rise + row.hold);
  require(model.output(rise + row.hold - 1) == NandReadOutput{.driven = true, .data = own},
          "Actual old byte survives through minimum hold");
  query(model, rise + row.hold);
  require(model.output(rise + row.hold) == NandReadOutput{},
          "Hold end loses guaranteed drive/data");
  query(model, rise + row.re_release);
  require(model.output(rise + row.re_release) == NandReadOutput{.driven = false},
          "RE release end establishes Hi-Z");
  query(model, std::nullopt);
  require(calls == 1, "Transition queries and time publications never consume another byte");
}

void overlapping_release(const Vector row, const bool ce_first) {
  auto model = initial(row, false);
  unsigned calls = 0;
  const auto fall = start + clearance;
  model.observe(fall, {.ce_n = false, .re_n = false}, [&] {
    ++calls;
    return own;
  });
  static_cast<void>(model.output(fall + row.access));
  const auto first = fall + row.access + margin;
  model.observe(first, {.ce_n = ce_first, .re_n = !ce_first});
  query(model, first + row.hold);
  model.observe(first + 1, {.ce_n = true, .re_n = true});
  const auto floating = first + row.ce_release + (ce_first ? 0U : 1U);
  query(model, first + row.hold);
  require(model.output(first + row.hold) == NandReadOutput{},
          "Second release cannot extend prior hold");
  query(model, floating);
  require(model.output(floating - 1) == NandReadOutput{} &&
              model.output(floating) == NandReadOutput{.driven = false},
          "Second release retains independently earliest literal release bound");
  query(model, std::nullopt);
  require(calls == 1, "Overlapping release never reads again");
}

void short_pulse(const Vector row) {
  auto model = initial(row, false);
  unsigned calls = 0;
  const auto fall = start + clearance;
  model.observe(fall, {.ce_n = false, .re_n = false}, [&] {
    ++calls;
    return own;
  });
  query(model, fall + row.access);
  const auto rise = fall + row.low;
  model.observe(rise, {.ce_n = false, .re_n = true});
  query(model, rise + row.re_release);
  require(model.output(fall + row.access) == NandReadOutput{},
          "Old pending access cannot publish after RE already rose");
  query(model, rise + row.re_release);
  require(model.output(rise + row.hold) == NandReadOutput{},
          "Unknown hold produces no separate visible transition");
  query(model, rise + row.re_release);
  require(model.output(rise + row.re_release) == NandReadOutput{.driven = false},
          "Short pulse only publishes eventual Hi-Z");
  query(model, std::nullopt);
  require(calls == 1, "Short pulse consumes only its actual admitted RE edge");
}
}

int main() {
  try {
    for (const auto row : rows) {
      for (const auto ce : {false, true}) { selected_read(row, ce); }
      for (const auto ce_first : {false, true}) { overlapping_release(row, ce_first); }
      short_pulse(row);
    }
    std::cout << "PASS NAND output next-transition: 15 directed waveforms at 3 literal "
                 "grids,read-only queries,access/hold/Hi-Z,overlap and cancelled early-access "
                 "bounds; no query source reads\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL NAND transitions: " << e.what() << '\n';
    return 1;
  }
}
