// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "hcs08_board_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"

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
constexpr unsigned extent = 3598;
constexpr unsigned record_bytes = 514;
constexpr unsigned calibration = 0xffbe;
constexpr unsigned declared_trim = 0x80;
constexpr unsigned declared_bus_hz = 12000000;
constexpr unsigned declared_supply_uv = 3300000;
constexpr unsigned bounded_end = 22700;
constexpr unsigned adc_left_at_bound = 165;
constexpr unsigned reset_flags = 0x68;
constexpr unsigned low_byte = 0xff;
constexpr unsigned high_known = 0xff00;
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

void pads(Hcs08Board &board) {
  board.attach_pad_source(Hcs08GpioPort::e, [](const unsigned mask) {
    return GpioPadSample{.known = mask, .levels = 8U & mask};
  });
}

void reset_cuts(const Hcs08FlashImage &supplied) {
  constexpr std::array cuts{0U, 1U, 22671U, bounded_end};
  constexpr std::array causes{Hcs08ResetCause::warm, Hcs08ResetCause::power_on};
  for (const auto cause : causes) {
    const auto request = Hcs08BoardResetRequest{.cause = cause, .entry = Hcs08ResetEntry::normal};
    const Hcs08Board initial(supplied, BLOFELD_HCS08_SLA, reference, inputs(0, true, true));
    Hcs08Board baseline(
        initial.reset_checkpoint(request), BLOFELD_HCS08_SLA, inputs(0, true, true));
    pads(baseline);
    to(baseline, bounded_end);
    require(baseline.memory().adc_state().remaining == adc_left_at_bound &&
                baseline.memory().adc_state().conversions == 0 &&
                baseline.memory().flash_image() == supplied,
            "Independent original startup budget leaves165ADC periods and exact NVM");
    for (const auto cut : cuts) {
      Hcs08Board original(supplied, BLOFELD_HCS08_SLA, reference, inputs(0, true, true));
      pads(original);
      to(original, cut);
      const auto before = original.cpu_state();
      const auto periods = original.memory().periods();
      Hcs08Board fresh(
          original.reset_checkpoint(request), BLOFELD_HCS08_SLA, inputs(0, true, true));
      const auto expected_a = cause == Hcs08ResetCause::warm ? before.a : 0;
      const auto expected_known = cause == Hcs08ResetCause::warm ? before.a_known : 0;
      const auto expected_x = cause == Hcs08ResetCause::warm ? before.hx & low_byte : 0;
      const auto expected_x_known = cause == Hcs08ResetCause::warm ? before.hx_known & low_byte : 0;
      require(fresh.cpu_state() == Hcs08State{.pc = entry,
                                              .hx = static_cast<std::uint16_t>(expected_x),
                                              .sp = low_byte,
                                              .a = static_cast<std::uint8_t>(expected_a),
                                              .ccr_known = reset_flags,
                                              .ccr = reset_flags,
                                              .a_known = static_cast<std::uint8_t>(expected_known),
                                              .hx_known = static_cast<std::uint16_t>(
                                                  expected_x_known | high_known)} &&
                  fresh.run_state().remaining_periods == 0 &&
                  fresh.run_state().retired_instructions == 0 &&
                  fresh.memory().periods().completed == 0 && fresh.bdc_state() == Hcs08BdcState{},
              "Original reset reads its real vector,resets H/SP/CCR and retains only warm A/X");
      pads(fresh);
      to(fresh, bounded_end);
      require(fresh.cpu_state() == baseline.cpu_state() &&
                  fresh.run_state() == baseline.run_state() && fresh.state() == baseline.state() &&
                  original.cpu_state() == before && original.memory().periods() == periods,
              "Original startup after every logical reset cut consumes new full native budgets");
      same_memory(fresh.memory(), baseline.memory());
      std::cout << "RESET cause=" << static_cast<unsigned>(cause) << " cut=" << cut
                << " periods=" << fresh.state().completed_periods
                << " retired=" << fresh.run_state().retired_instructions
                << " pc=" << fresh.cpu_state().pc << '\n';
    }
  }
}

void failed_device_recovery(const Hcs08FlashImage &supplied) {
  constexpr unsigned attempted = 22690;
  constexpr unsigned retired = 6193;
  constexpr unsigned failure_pc = 0xe1bc;
  Hcs08Board original(supplied, BLOFELD_HCS08_SLA, reference, inputs(0, false, true));
  pads(original);
  require(rejected([&] { to(original, attempted); }).contains("unknown bus/supply/reference") &&
              original.state().completed_periods == attempted - 1 &&
              original.run_state().retired_instructions == retired &&
              original.cpu_state().pc == failure_pc,
          "Recovery starts at the independently counted first actual missing-ADC-input edge");
  const auto saved =
      original.reset_checkpoint({.cause = Hcs08ResetCause::warm, .entry = Hcs08ResetEntry::normal});
  Hcs08Board missing(saved, BLOFELD_HCS08_SLA, {});
  pads(missing);
  require(rejected([&] { to(missing, attempted); }).contains("unknown bus/supply/reference") &&
              missing.state().completed_periods == attempted - 1 &&
              missing.run_state().retired_instructions == retired &&
              missing.cpu_state().pc == failure_pc && missing.memory().flash_image() == supplied,
          "Warm reset invents neither the absent ADC clock/reference nor a conversion result");
  Hcs08Board observed(saved, BLOFELD_HCS08_SLA, inputs(0, true, true));
  pads(observed);
  to(observed, bounded_end);
  require(observed.memory().adc_state().remaining == adc_left_at_bound &&
              observed.memory().adc_state().conversions == 0 &&
              observed.memory().flash_image() == supplied && original.faulted(),
          "Only renewed explicit reference inputs permit a fresh original bounded startup");
  std::cout << "RECOVERY periods=" << observed.state().completed_periods
            << " retired=" << observed.run_state().retired_instructions
            << " pc=" << observed.cpu_state().pc << '\n';
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
    reset_cuts(supplied);
    failed_device_recovery(supplied);
    std::cout << "Verified original HCS08 warm/cold reset cuts and missing-input recovery.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
