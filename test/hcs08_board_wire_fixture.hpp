// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/hcs08_board.hpp"
#include "board/hcs08_board_wire.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <cstdint>
#include <initializer_list>

namespace blofeld::test::board_wire {
constexpr auto contract = Hcs08BoardContract::candidate_gt8a_reference_retirement;
constexpr auto normal = Hcs08ResetEntry::normal;
constexpr auto active = Hcs08ResetEntry::active_background;
constexpr auto high = Hcs08BdcDrive::high;
constexpr auto low = Hcs08BdcDrive::low;
constexpr auto released = Hcs08BdcDrive::released;
constexpr unsigned host_pc = 0x2000;
constexpr unsigned mask = 255;
constexpr Hcs08BoardWireInputs wire_inputs{.contract =
                                               Hcs08BdcWireContract::reference_command_delay,
                                           .initial_host = high,
                                           .clock = {.period_ticks = 1, .first_edge_ticks = 1}};

inline void clocks(Hcs08BoardWire &board, const unsigned count) {
  for (unsigned tick = 0; tick < count; ++tick) { board.clock_bdc(); }
}

// Independent literal protocol stimulus:ten-cycle sample,sixteen-cycle bit
// cell plus one synchronization edge. No target byte command is called here.
inline void send(Hcs08BoardWire &board, const std::uint8_t byte) {
  constexpr unsigned high_bit = 0x80;
  for (unsigned bit = 0; bit < 8; ++bit) {
    board.host_drive(low, {.pc = host_pc});
    clocks(board, 4);
    if (((static_cast<unsigned>(byte) << bit) & high_bit) != 0) {
      board.host_drive(high, {.pc = host_pc});
    }
    clocks(board, 9);
    board.host_drive(high, {.pc = host_pc});
    clocks(board, 4);
  }
}

inline void command(Hcs08BoardWire &board, const std::initializer_list<std::uint8_t> frame) {
  for (const auto byte : frame) { send(board, byte); }
  clocks(board, 16);
}

inline std::uint8_t receive(Hcs08BoardWire &board) {
  unsigned value = 0;
  for (unsigned bit = 0; bit < 8; ++bit) {
    board.host_drive(low, {.pc = host_pc});
    clocks(board, 3);
    board.host_drive(released, {.pc = host_pc});
    clocks(board, 8);
    value = ((value << 1U) | (board.level() ? 1U : 0U)) & mask;
    clocks(board, 6);
  }
  return static_cast<std::uint8_t>(value);
}

inline Hcs08FlashImage blank(const std::uint8_t security = 2) {
  Hcs08FlashImage image;
  image.data.fill(mask);
  image.known.set();
  image.fresh.set();
  constexpr unsigned vector = 0xfffe;
  image.known.reset(vector - hcs08_flash::first);
  image.known.reset(vector + 1 - hcs08_flash::first);
  image.fresh.reset(vector - hcs08_flash::first);
  image.fresh.reset(vector + 1 - hcs08_flash::first);
  image.data[hcs08_flash::nv_options - hcs08_flash::first] = security;
  image.fresh.reset(hcs08_flash::nv_options - hcs08_flash::first);
  return image;
}
} // namespace blofeld::test::board_wire
