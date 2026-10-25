// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_norm_fixture.hpp"
#include "keyboard_initial_fixture.hpp"
#include "lcd_fixture.hpp"
#include "mcu/mcu_bus.hpp"
#include "original_gpio_pair_fixture.hpp"

#include <cstdint>

namespace blofeld::test::keyboard_initial {
void Fixture::drawing() {
  const auto state = pair.mcu.cpu().state();
  if (state.pc == indicator_entry) {
    ++indicators;
    pair.trace << "INDICATOR index=" << indicators << '\n';
  }
  if (state.pc == clear_entry) { pair.trace << "CLEAR_ENTRY pc=" << state.pc << '\n'; }
  if (state.pc != text_entry && state.pc != rectangle_entry) { return; }
  const auto root = pair.mcu.bus().read_memory({.address = state.sp, .size = 2});
  if (root < target || root >= target + helper_bytes) { return; }
  const auto argument = [this, state](const unsigned offset) {
    return static_cast<unsigned>(
        pair.mcu.bus().read(static_cast<std::uint16_t>(state.sp + offset)));
  };
  if (state.pc == text_entry) {
    const auto pointer = pair.mcu.bus().read_memory(
        {.address = static_cast<std::uint16_t>(state.sp + 2), .size = 2});
    require(pointer >= target && pointer < target + helper_bytes,
            "Original text remains in copied helper");
    pair.trace << "TEXT index=" << texts << " pointer=" << pointer << " y=" << argument(4)
               << " x=" << argument(5) << " font=" << argument(6)
               << " mode=" << (state.d & byte_mask) << '\n';
    ++texts;
  } else {
    pair.trace << "RECTANGLE index=" << rectangles << " x=" << argument(5) << " y=" << argument(4)
               << " width=" << argument(3) << " height=" << argument(2)
               << " mode=" << (state.d & byte_mask) << '\n';
    ++rectangles;
  }
}
} // namespace blofeld::test::keyboard_initial
