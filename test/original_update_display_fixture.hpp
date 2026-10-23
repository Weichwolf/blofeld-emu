// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "blofeld/mcu_bus.hpp"
#include "blofeld/st7565.hpp"
#include "original_update_forward_fixture.hpp"

#include <cstdint>
#include <string>

namespace blofeld::test::update_display {
using namespace blofeld;
using namespace blofeld::test::update_forward;
constexpr unsigned visible_columns = 128, visible_pages = 8, string_limit = 32;

inline void check_clear(const Forward &run) {
  blofeld::test::require(run.display->counts().writes == visible_columns * visible_pages &&
                             run.display->counts().reads == 0,
                         "Original error callback clears1024 cells without display reads");
  for (unsigned page = 0; page < visible_pages; ++page) {
    for (unsigned column = 0; column < St7565::columns; ++column) {
      const auto cell = run.display->controller().ram(page, column);
      blofeld::test::require(column < visible_columns ? cell.known == mask && cell.value == 0
                                                      : cell.known == 0,
                             "Literal blank frame and untouched spare columns must match");
    }
  }
  for (unsigned column = 0; column < St7565::columns; ++column) {
    blofeld::test::require(run.display->controller().ram(visible_pages, column).known == 0,
                           "Original error clear leaves the icon page unknown");
  }
}

inline std::string read_text(McuBus &bus, const unsigned address) {
  std::string result;
  for (unsigned n = 0; n < string_limit; ++n) {
    const auto value = bus.read(static_cast<std::uint16_t>(address + n));
    if (value == 0) { return result; }
    result.push_back(static_cast<char>(value));
  }
  throw Fault("Unbounded diagnostic text");
}

} // namespace blofeld::test::update_display
