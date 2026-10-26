// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "board/hcs08_board_serial.hpp"
#include "common/bus.hpp"
#include "hcs08_board_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::vectors {

using blofeld::test::hcs08::require;
namespace fixture = blofeld::test::board;
constexpr unsigned vector = 0xffe0, direct_handler = 0xe200, moved_handler = 0xe100;
constexpr unsigned marker = 0x80, ordinary_read = 0x81, marker_moved = 1, marker_direct = 2;
constexpr unsigned setup_periods = 18, irq_periods = 11, isr_periods = 26, endpoint = 55;
constexpr unsigned caller_pc = 0xe00f, caller_sp = 0x47f, caller_a = 0x74, quiet_spi = 0x54;
constexpr unsigned caller_flags = 0x60, caller_known = 0xee, original_vector_high = 0xe2;
constexpr unsigned cold_h_known = 0xff00, cold_sp = 0xff;
constexpr auto observed = Hcs08BoardClockObservation::no_fault;
constexpr std::array<unsigned, 18> cuts{
    0, 1, 3, 5, 17, 18, 19, 23, 24, 25, 26, 27, 28, 29, 30, 45, 46, 54};

struct Configuration {
  unsigned protection, options, physical_vector, handler, marker_value;
};

// Literal GT8A protection examples plus FNORED/FPDIS/full-array exclusions.
constexpr std::array<Configuration, 6> cases{{{.protection = 0xfc,
                                               .options = 2,
                                               .physical_vector = 0xfde0,
                                               .handler = moved_handler,
                                               .marker_value = marker_moved},
                                              {.protection = 0xfa,
                                               .options = 2,
                                               .physical_vector = 0xfbe0,
                                               .handler = moved_handler,
                                               .marker_value = marker_moved},
                                              {.protection = 0xe0,
                                               .options = 2,
                                               .physical_vector = 0xe1e0,
                                               .handler = moved_handler,
                                               .marker_value = marker_moved},
                                              {.protection = 0xfc,
                                               .options = 0x42,
                                               .physical_vector = vector,
                                               .handler = direct_handler,
                                               .marker_value = marker_direct},
                                              {.protection = 0xff,
                                               .options = 2,
                                               .physical_vector = vector,
                                               .handler = direct_handler,
                                               .marker_value = marker_direct},
                                              {.protection = 0xde,
                                               .options = 2,
                                               .physical_vector = vector,
                                               .handler = direct_handler,
                                               .marker_value = marker_direct}}};

inline Hcs08FlashImage own_image(const Configuration config) {
  // Independent nominal foreground:3+2+2+4+2+3+1+1=18;CLI protects the NOP.
  constexpr std::array<std::uint8_t, 17> setup{0x45,
                                               4,
                                               0x80,
                                               0x94,
                                               0xa6,
                                               0x11,
                                               0xc7,
                                               0x18,
                                               2,
                                               0xa6,
                                               0x74,
                                               0xb7,
                                               0x28,
                                               0x9a,
                                               0x9d,
                                               0x20,
                                               0xfe};
  auto image = fixture::own_image(setup, static_cast<std::uint8_t>(config.options));
  const auto put = [&](const unsigned address, const std::uint8_t byte) {
    image.data.at(address - hcs08_flash::first) = byte;
    image.fresh.reset(address - hcs08_flash::first);
  };
  put(hcs08_flash::nv_protection, static_cast<std::uint8_t>(config.protection));
  put(vector, original_vector_high);
  put(vector + 1, 0);
  put(config.physical_vector, static_cast<std::uint8_t>(config.handler >> 8U));
  put(config.physical_vector + 1, 0);
  for (const unsigned handler : {moved_handler, direct_handler}) {
    // Marker/normal LDA from FFE0/SPI-disable/RTI:2+3+4+3+2+3+9=26.
    const auto value =
        static_cast<std::uint8_t>(handler == moved_handler ? marker_moved : marker_direct);
    const std::array<std::uint8_t, 16> isr{0xa6,
                                           value,
                                           0xb7,
                                           marker,
                                           0xc6,
                                           0xff,
                                           0xe0,
                                           0xb7,
                                           ordinary_read,
                                           0xa6,
                                           quiet_spi,
                                           0xb7,
                                           0x28,
                                           0x80,
                                           0x9d,
                                           0x9d};
    for (unsigned index = 0; index < isr.size(); ++index) { put(handler + index, isr[index]); }
  }
  return image;
}

inline Hcs08BoardCheckpoint reset(const Hcs08FlashImage &image) {
  Hcs08BoardInputs inputs;
  inputs.reset_observation = observed;
  const Hcs08Board board(image,
                         BLOFELD_HCS08_SLA,
                         fixture::reference,
                         inputs,
                         Hcs08RtiIrqContract::reference_immediate);
  require(board.cpu_state().pc == fixture::entry && board.cpu_state().a_known == 0 &&
              board.cpu_state().hx_known == cold_h_known && board.cpu_state().sp == cold_sp,
          "Own execution starts at a genuine cold reset;A/X remain unknown");
  return board.checkpoint();
}

inline Hcs08State caller() {
  return {.pc = caller_pc,
          .hx = caller_sp + 1,
          .sp = caller_sp,
          .a = caller_a,
          .ccr_known = caller_known,
          .ccr = caller_flags};
}

inline void to(Hcs08BoardSerial &board, const unsigned period, const Configuration config) {
  while (board.board_state().completed_periods < period) {
    require(!board.advance_reference_period(std::nullopt, observed),
            "TX-buffer-empty IRQ creates no physical SPI edge or MISO sample");
    if (board.board_state().completed_periods == setup_periods + irq_periods) {
      require(board.cpu_state().pc == config.handler &&
                  board.run_state().irq_periods == irq_periods,
              "Real logical SPI source fetches its literal physical vector and target");
    }
  }
}

inline void endpoint_checks(const Hcs08BoardSerial &board,
                            const Configuration config,
                            const Hcs08FlashImage &image) {
  require(board.cpu_state() == caller() && board.board_state().completed_periods == endpoint &&
              board.run_state().foreground_periods == setup_periods &&
              board.foreground_state().retired_instructions == 8 &&
              board.run_state().irq_periods == irq_periods &&
              board.run_state().program_periods == isr_periods &&
              board.run_state().completed_periods == endpoint,
          "Literal18+11+26periods and native RTI restore the independent caller state");
  const auto mark = board.memory().inspect_ram(marker);
  const auto direct = board.memory().inspect_ram(ordinary_read);
  require(
      mark.value == config.marker_value && mark.known == Hcs08KnownByte::all &&
          direct.value == original_vector_high && direct.known == Hcs08KnownByte::all &&
          board.memory().spi_state().control1 == quiet_spi &&
          board.clock_state().generated_edges == 0 && board.memory().flash_image() == image,
      "Only native ISR writes acknowledge SPI;ordinary CPU LDA still reads direct FFE0;NVM exact");
}

} // namespace blofeld::test::vectors
