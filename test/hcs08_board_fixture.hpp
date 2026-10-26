// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <cstdint>
#include <initializer_list>
#include <span>

namespace blofeld::test::board {
constexpr auto reference = Hcs08BoardContract::candidate_gt8a_reference_retirement;
constexpr unsigned entry = 0xe000, ram = 0x80, vector = 0xfffe, byte_mask = 0xff;

inline Hcs08FlashImage own_image(const std::span<const std::uint8_t> code,
                                 const std::uint8_t security = 2) {
  Hcs08FlashImage image;
  image.data.fill(byte_mask);
  image.known.set();
  image.fresh.set();
  const auto put = [&](const unsigned address, const std::uint8_t value) {
    const auto offset = address - hcs08_flash::first;
    image.data.at(offset) = value;
    image.fresh.reset(offset);
  };
  for (unsigned index = 0; index < code.size(); ++index) { put(entry + index, code[index]); }
  put(vector, entry >> 8U);
  put(vector + 1, 0);
  put(hcs08_flash::nv_options, security);
  return image;
}

inline void to(Hcs08Board &board, const std::uint64_t period) {
  while (board.state().completed_periods < period) { board.advance_reference_period(); }
}

inline void command(Hcs08Board &board, const std::initializer_list<std::uint8_t> frame) {
  for (const auto byte : frame) { board.receive_bdc(byte); }
  board.service_bdc(Hcs08BdcGrant::reference_instruction_boundary);
}

inline void same_memory(const Hcs08BoardMemory &first, const Hcs08BoardMemory &second) {
  using hcs08::require;
  for (unsigned address = Hcs08BoardMemory::ram_first; address < Hcs08BoardMemory::ram_end;
       ++address) {
    const auto actual = first.inspect_ram(static_cast<std::uint16_t>(address));
    const auto expected = second.inspect_ram(static_cast<std::uint16_t>(address));
    require(actual.value == expected.value && actual.known == expected.known,
            "Fresh joint board continuation preserves every RAM value and known mask");
  }
  require(
      first.flash_image() == second.flash_image() && first.flash_state() == second.flash_state() &&
          first.icg_state() == second.icg_state() && first.adc_state() == second.adc_state() &&
          first.timer_state() == second.timer_state() && first.cop_state() == second.cop_state() &&
          first.spi_state() == second.spi_state() && first.periods() == second.periods(),
      "Fresh joint continuation preserves all actual peripheral,NVM and clock-owner state");
}
} // namespace blofeld::test::board
