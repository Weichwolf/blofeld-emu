// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand_array.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <utility>

namespace {
using namespace blofeld;
constexpr unsigned page_index = 37, other_page = 72;
constexpr std::uint8_t first = 0x35, second = 0xa6, third = 0x5c, known = 0xff;
unsigned checks = 0;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
  ++checks;
}

void expect(const NandArray &array,
            const unsigned page,
            const unsigned column,
            const NandByte wanted) {
  require(array.cell({.page = page, .column = column}) == wanted, "Incorrect NAND byte");
}

void seed(NandArray &array, const std::uint8_t value) {
  array.observe_cell({.page = page_index}, {.value = value, .known = known});
  expect(array, page_index, 0, {.value = value, .known = known});
}

void mutations() {
  NandArray array;
  expect(array, page_index, 0, {});
  seed(array, first);
  const std::array<std::uint8_t, 3> bytes{first, second, third};
  array.observe_bytes({.page = page_index, .column = 1}, bytes);
  for (unsigned index = 0; index < bytes.size(); ++index) {
    expect(array, page_index, index + 1, {.value = bytes[index], .known = known});
  }
  array.observe_cell({.page = page_index, .column = 4}, {.value = 5, .known = 15});
  expect(array, page_index, 4, {.value = 5, .known = 15});
  for (unsigned index = 0; index < NandArray::block_pages; ++index) {
    array.observe_cell({.page = other_page + index}, {.value = second, .known = known});
  }
  expect(array, page_index, 0, {.value = first, .known = known});
  NandArray::Page replacement;
  replacement.cells.fill({.value = third, .known = known});
  replacement.programs = NandProgramCount{.main = 1, .spare = 2};
  array.restore(page_index, replacement);
  for (unsigned column = 0; column < NandArray::page_bytes; ++column) {
    expect(array, page_index, column, {.value = third, .known = known});
  }
  const auto history = array.snapshot(page_index).programs;
  require(history && history->spare == 2, "Program history changed");
  array.invalidate_page(page_index);
  expect(array, page_index, 0, {});
  array.erase_block(page_index / NandArray::block_pages);
  expect(array, page_index, 0, {.value = known, .known = known});
  array.invalidate_block(page_index / NandArray::block_pages);
  expect(array, page_index, 0, {});
  expect(array, other_page, 0, {.value = second, .known = known});
}

void copies() {
  NandArray source;
  seed(source, first);
  const NandArray constructed(source);
  source.observe_cell({.page = page_index}, {.value = second, .known = known});
  expect(constructed, page_index, 0, {.value = first, .known = known});
  NandArray assigned;
  seed(assigned, third);
  assigned = source;
  source.observe_cell({.page = page_index}, {.value = third, .known = known});
  expect(assigned, page_index, 0, {.value = second, .known = known});
  source.invalidate_page(page_index);
  expect(constructed, page_index, 0, {.value = first, .known = known});
  expect(assigned, page_index, 0, {.value = second, .known = known});
  const NandArray absent;
  assigned = absent;
  expect(assigned, page_index, 0, {});
}

void reuse_source(NandArray &source, const NandArray &destination, const std::uint8_t expected) {
  source = NandArray{};
  expect(source, page_index, 0, {});
  seed(source, third);
  expect(destination, page_index, 0, {.value = expected, .known = known});
}

void moves() {
  NandArray source;
  seed(source, first);
  NandArray constructed(std::move(source));
  expect(constructed, page_index, 0, {.value = first, .known = known});
  reuse_source(source, constructed, first);
  NandArray assigned;
  seed(assigned, second);
  assigned = std::move(constructed);
  expect(assigned, page_index, 0, {.value = first, .known = known});
  reuse_source(constructed, assigned, first);
  NandArray absent;
  assigned = std::move(absent);
  expect(assigned, page_index, 0, {});
}

void self_assignment(NandArray &destination, NandArray &source) {
  destination = source;
  expect(destination, page_index, 0, {.value = first, .known = known});
  destination = std::move(source);
  expect(destination, page_index, 0, {.value = first, .known = known});
}

void refusals() {
  NandArray array;
  seed(array, first);
  for (const auto location :
       std::array{NandLocation{.page = page_index, .column = NandArray::page_bytes},
                  NandLocation{.page = NandArray::pages}}) {
    bool refused = false;
    try {
      static_cast<void>(array.cell(location));
    } catch (const Fault &) { refused = true; }
    require(refused, "Invalid cached read accepted");
    expect(array, page_index, 0, {.value = first, .known = known});
  }
  bool refused = false;
  try {
    array.observe_cell({.page = page_index}, {.value = known, .known = 1});
  } catch (const Fault &) { refused = true; }
  require(refused, "Noncanonical cell accepted");
  expect(array, page_index, 0, {.value = first, .known = known});
  self_assignment(array, array);
}
}

int main() {
  try {
    mutations();
    copies();
    moves();
    refusals();
    std::cout << "NAND selected-page ownership/mutation/refusal checks=" << checks << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
