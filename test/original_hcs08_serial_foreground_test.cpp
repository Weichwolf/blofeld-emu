// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "board/hcs08_board_serial.hpp"
#include "hcs08_board_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::board;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr unsigned extent = 3598, record_bytes = 514, calibration = 0xffbe, declared_trim = 0x80;
constexpr unsigned main_stack = 0x475, final_hx = 31250, full_flags = 0x6c, known_flags = 0xef;
constexpr unsigned first_cut = 22671, bounded_end = 22700, adc_left_at_bound = 165;
constexpr unsigned flash_pc = 0xea88, icg_pc = 0xe1b0, remaining_at_cut = 2;
constexpr unsigned declared_bus_hz = 12000000, declared_supply_uv = 3300000, partial_a = 0xfe;
constexpr std::array pages{0xe000U, 0xe200U, 0xe400U, 0xe600U, 0xe800U, 0xea00U, 0xfe00U};

Hcs08FlashImage image(const std::span<const std::uint8_t> records) {
  Hcs08FlashImage result;
  for (unsigned page = 0; page < pages.size(); ++page) {
    const auto first = page * record_bytes;
    const unsigned address = (static_cast<unsigned>(records[first]) << 8U) | records[first + 1];
    require(address == pages[page], "Pinned original page addresses retain physical mapping");
    for (unsigned offset = 0; offset < hcs08_flash::page_bytes; ++offset) {
      const auto target = address + offset - hcs08_flash::first;
      result.data[target] = records[first + 2 + offset];
      result.known.set(target);
    }
  }
  // Explicit per-unit diagnostic input only. The board never patches its image.
  result.data[calibration - hcs08_flash::first] = declared_trim;
  return result;
}

Hcs08BoardInputs
inputs(const std::optional<unsigned> phase, const bool reference, const bool analog) {
  Hcs08BoardInputs result;
  result.timer_phase = phase;
  if (reference) {
    result.adc_reference = Hcs08AdcReference{.bus_hz = declared_bus_hz,
                                             .supply_uv = declared_supply_uv,
                                             .low_uv = 0,
                                             .high_uv = declared_supply_uv};
  }
  if (analog) { result.analog[0] = 0; }
  return result;
}

Hcs08BoardSerial make_board(const Hcs08FlashImage &supplied,
                            const Hcs08BoardInputs &inputs,
                            const Hcs08SpiRunContract order) {
  const Hcs08Board initial(supplied, BLOFELD_HCS08_SLA, reference, inputs);
  return Hcs08BoardSerial(
      initial.checkpoint(),
      BLOFELD_HCS08_SLA,
      {.order = order,
       .launch = Hcs08SpiLaunchContract::reference_one_period,
       .program = Hcs08PrefetchContract::reference_peripheral_program_cycles,
       .foreground = Hcs08SpiForegroundContract::reference_instruction_retirement});
}

void to(Hcs08BoardSerial &board, const std::uint64_t period) {
  while (board.board_state().completed_periods < period) {
    static_cast<void>(board.advance_reference_period(std::nullopt));
  }
}

void pads(Hcs08BoardSerial &board) {
  board.attach_pad_source(Hcs08GpioPort::e, [](const unsigned mask) {
    return GpioPadSample{.known = mask, .levels = 8U & mask};
  });
}

void quarantine(Hcs08BoardSerial &board) {
  const auto before = board.board_state();
  const auto periods = board.memory().periods();
  const auto cpu = board.cpu_state();
  require(rejected([&] {
            static_cast<void>(board.advance_reference_period(std::nullopt));
          }).contains("coordinated recovery") &&
              rejected([&] {
                static_cast<void>(board.checkpoint());
              }).contains("coordinated recovery") &&
              board.board_state() == before && board.memory().periods() == periods &&
              board.cpu_state() == cpu,
          "Failed whole-board periods cannot consume another edge or produce successful snapshots");
}

void missing_inputs(const Hcs08FlashImage &supplied,
                    const unsigned missing,
                    const Hcs08SpiRunContract order) {
  const bool absent_reference = missing == 0;
  const bool absent_phase = missing == 1;
  auto board = make_board(supplied,
                          inputs(absent_phase ? std::nullopt : std::optional<unsigned>{0},
                                 !absent_reference,
                                 missing != 2),
                          order);
  pads(board);
  constexpr unsigned adc_period = 22690;
  constexpr unsigned phase_period = 22702;
  constexpr unsigned adc_retired = 6193;
  constexpr unsigned phase_retired = 6196;
  constexpr unsigned adc_pc = 0xe1bc;
  constexpr unsigned phase_pc = 0xe1c5;
  const auto period = absent_phase ? phase_period : adc_period;
  to(board, period - 1);
  const auto failure =
      rejected([&] { static_cast<void>(board.advance_reference_period(std::nullopt)); });
  constexpr std::array reasons{
      "unknown bus/supply/reference", "unknown prescaler phase", "unknown enabled analog pin"};
  const auto *const expected = reasons.at(missing);
  require(
      failure.contains(expected) && board.faulted() &&
          board.board_state().attempted_periods == period &&
          board.board_state().completed_periods == period - 1 &&
          board.board_state().phase == Hcs08BoardPhase::devices &&
          board.foreground_state().offered_periods == period - 1 &&
          board.foreground_state().retired_instructions ==
              (absent_phase ? phase_retired : adc_retired) &&
          board.cpu_state().pc == (absent_phase ? phase_pc : adc_pc) &&
          board.memory().flash_image() == supplied && board.memory().adc_state().conversions == 0 &&
          !board.memory().adc_state().result,
      "Independent original first-active-edge frontiers reject absent physical inputs before CPU "
      "progress");
  quarantine(board);
  std::cout << "MISSING kind=" << missing << " period=" << period
            << " retired=" << board.foreground_state().retired_instructions
            << " pc=" << board.cpu_state().pc << '\n';
}

void copied_prefix(Hcs08BoardSerial &board) {
  to(board, first_cut);
  require(board.foreground_state().remaining_periods == remaining_at_cut &&
              board.cpu_state().pc == icg_pc,
          "Joint cut lies in the actual first original ICG load's partial native budget");
  const auto saved = board.checkpoint();
  to(board, bounded_end);
  Hcs08BoardSerial fresh(saved, BLOFELD_HCS08_SLA);
  pads(fresh);
  to(fresh, bounded_end);
  require(board.cpu_state() == fresh.cpu_state() && board.run_state() == fresh.run_state() &&
              board.clock_state() == fresh.clock_state() &&
              board.foreground_state() == fresh.foreground_state() &&
              board.board_state() == fresh.board_state() &&
              board.bdc_state() == fresh.bdc_state() &&
              board.memory().adc_state().remaining == adc_left_at_bound &&
              board.memory().adc_state().conversions == 0,
          "Fresh owners reproduce the bounded original suffix and independently counted active ADC "
          "clocks");
  same_memory(board.memory(), fresh.memory());
}

struct Expected {
  unsigned phase, periods, retired, services;
};

constexpr std::array profiles{
    Expected{.phase = 0, .periods = 4022751, .retired = 1149066, .services = 285714},
    Expected{.phase = 127, .periods = 4022625, .retired = 1149030, .services = 285705}};

void declared_inputs(const Hcs08FlashImage &supplied,
                     const Expected expected,
                     const Hcs08SpiRunContract order) {
  std::cout << "BEGIN phase=" << expected.phase << '\n' << std::flush;
  auto board = make_board(supplied, inputs(expected.phase, true, true), order);
  pads(board);
  copied_prefix(board);
  const auto failure = rejected([&] { to(board, expected.periods); });
  require(failure.contains("unqualified idle FCBEF launch") && failure.contains("pc=0xea88") &&
              board.board_state().attempted_periods == expected.periods &&
              board.board_state().completed_periods == expected.periods - 1 &&
              board.board_state().phase == Hcs08BoardPhase::cpu &&
              board.foreground_state().executing_periods == expected.periods &&
              board.foreground_state().retired_instructions == expected.retired &&
              board.memory().periods().cop_services == expected.services &&
              board.memory().periods().service_known == partial_a &&
              board.cpu_state() == Hcs08State{.pc = flash_pc,
                                              .hx = final_hx,
                                              .sp = main_stack - 2,
                                              .a = byte_mask,
                                              .ccr_known = known_flags,
                                              .ccr = full_flags} &&
              board.memory().flash_image() == supplied && !board.memory().flash_state().active &&
              board.memory().flash_state().buffer == Hcs08FlashBuffer::empty,
          "Production owners reproduce independent original timer-loop totals and strict actual "
          "FSTATFF without a firmware patch");
  const auto adc = board.memory().adc_state();
  require(adc.complete && !adc.active && adc.result == 0 && adc.conversions == 1,
          "Only actual supplied device periods complete the declared stable-input ADC job");
  require(board.run_state().foreground_periods == expected.periods - 1 &&
              board.run_state().irq_periods == 0 && board.run_state().program_periods == 0 &&
              board.run_state().spi_periods ==
                  expected.periods - static_cast<unsigned>(
                                         order == Hcs08SpiRunContract::reference_cpu_before_spi) &&
              board.clock_state().generated_edges == 0 &&
              board.clock_state().completed_transfers == 0,
          "The unmodified original prefix uses the real foreground owner and never supplies SPI "
          "data or completion");
  quarantine(board);
  std::cout << "PROFILE phase=" << expected.phase << " periods=" << expected.periods
            << " retired=" << expected.retired << " services=" << expected.services
            << " pc=" << board.cpu_state().pc << '\n'
            << std::flush;
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Expected pinned original seven-page stream");
    std::ifstream input(arguments[1], std::ios::binary);
    std::vector<std::uint8_t> records(extent);
    input.read(reinterpret_cast<char *>(records.data()),
               static_cast<std::streamsize>(records.size()));
    require(input.gcount() == static_cast<std::streamsize>(records.size()) &&
                input.peek() == std::char_traits<char>::eof(),
            "Exact original record extent");
    const auto supplied = image(records);
    unsigned index = 0;
    for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                             Hcs08SpiRunContract::reference_spi_before_cpu}) {
      std::cout << "ORDER index=" << index++ << '\n';
      for (unsigned missing = 0; missing < 3; ++missing) {
        missing_inputs(supplied, missing, order);
      }
      for (const auto expected : profiles) { declared_inputs(supplied, expected, order); }
      std::cout
          << "Verified original reset through production memory,CPU,BDC and device clock owners.\n";
    }
    std::cout << "END orders=2\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
