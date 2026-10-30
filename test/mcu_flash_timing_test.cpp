// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_flash_timing.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr FlashTimeBasis initial{
    .timebase_hz = 60000000, .oscillator_hz = 12000000, .nvm_divisor = 72, .bus_period_ticks = 10};
constexpr unsigned words = 16384, bytes = 32768;
constexpr unsigned invalid_byte = 255;
constexpr std::uint64_t first_verify_ticks = 110;
constexpr std::uint64_t full_word_verify_ticks = 163940;
constexpr std::uint64_t full_byte_verify_ticks = 327780;
constexpr std::uint64_t program_ticks = 3490;
constexpr std::uint64_t transfer_ticks = 30;
constexpr std::uint64_t launch_ticks = 100;
constexpr std::uint64_t completed_tick = 3590;
constexpr unsigned at_completion_cycles = 349;
constexpr std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max();

void require(const bool value, const std::string &why) {
  if (!value) { throw Fault(why); }
}

void reject(const auto &action, const std::string_view why) {
  try {
    action();
  } catch (const Fault &error) {
    require(std::string_view(error.what()).contains(why),
            "Wrong timing rejection: " + std::string(error.what()));
    return;
  }
  throw Fault("Unsupported timing input was accepted: " + std::string(why));
}

void literal_equations() {
  struct Vector {
    FlashTimeBasis basis;
    std::uint64_t program, burst, row, sector, mass;
  };

  // Literal integer expectations; no conversion through floating microseconds.
  constexpr std::array<Vector, 4> vectors{{
      {.basis = initial,
       .program = 3490,
       .burst = 1530,
       .row = 50920,
       .sector = 1440000,
       .mass = 7200000},
      {.basis = {.timebase_hz = 120000000,
                 .oscillator_hz = 12000000,
                 .nvm_divisor = 60,
                 .bus_period_ticks = 5},
       .program = 5525,
       .burst = 2445,
       .row = 81320,
       .sector = 2400000,
       .mass = 12000000},
      {.basis = {.timebase_hz = 60000000,
                 .oscillator_hz = 12000000,
                 .nvm_divisor = 80,
                 .bus_period_ticks = 30},
       .program = 4350,
       .burst = 1870,
       .row = 62320,
       .sector = 1600000,
       .mass = 8000000},
      {.basis = {.timebase_hz = 240000060000000,
                 .oscillator_hz = 4000001,
                 .nvm_divisor = 24,
                 .bus_period_ticks = 120000000},
       .program = 15960000000,
       .burst = 6840000000,
       .row = 228000000000,
       .sector = 5760000000000,
       .mass = 28800000000000},
  }};
  for (const auto &v : vectors) {
    const auto program = flash_duration_ticks(v.basis, FlashTimedOperation::word_program);
    const auto burst = flash_duration_ticks(v.basis, FlashTimedOperation::same_row_burst);
    constexpr unsigned successors = 31;
    require(program == v.program && burst == v.burst && program + successors * burst == v.row,
            "Independent word/burst/row timing literals");
    require(flash_duration_ticks(v.basis, FlashTimedOperation::sector_erase) == v.sector &&
                flash_duration_ticks(v.basis, FlashTimedOperation::mass_erase) == v.mass,
            "Independent approximate erase-equation literals");
  }
}

void explicit_verify_units() {
  require(flash_duration_ticks(initial, FlashTimedOperation::verify_words, 1) ==
                  first_verify_ticks &&
              flash_duration_ticks(initial, FlashTimedOperation::verify_words, words) ==
                  full_word_verify_ticks &&
              flash_duration_ticks(initial, FlashTimedOperation::verify_bytes, bytes) ==
                  full_byte_verify_ticks,
          "Word text and table-consistent byte candidate retain different explicit limits");
  reject(
      [] { static_cast<void>(flash_duration_ticks(initial, FlashTimedOperation::verify_words)); },
      "scan count");
  reject(
      [] {
        static_cast<void>(
            flash_duration_ticks(initial, FlashTimedOperation::verify_words, words + 1));
      },
      "scan count");
  reject(
      [] {
        static_cast<void>(
            flash_duration_ticks(initial, FlashTimedOperation::verify_bytes, bytes + 1));
      },
      "scan count");
  reject(
      [] {
        static_cast<void>(flash_duration_ticks(initial, FlashTimedOperation::word_program, 1));
      },
      "non-verify");
  reject(
      [] {
        static_cast<void>(flash_duration_ticks(
            initial, std::bit_cast<FlashTimedOperation>(std::uint8_t{invalid_byte})));
      },
      "unsupported command");
}

void basis_boundaries() {
  constexpr std::array<FlashTimeBasis, 8> invalid{{
      {.timebase_hz = 60000000,
       .oscillator_hz = 12000000,
       .nvm_divisor = 0,
       .bus_period_ticks = 10},
      {.timebase_hz = 60000000,
       .oscillator_hz = 12000000,
       .nvm_divisor = 65,
       .bus_period_ticks = 10},
      {.timebase_hz = 60000000,
       .oscillator_hz = 12000000,
       .nvm_divisor = 520,
       .bus_period_ticks = 10},
      {.timebase_hz = 60000000,
       .oscillator_hz = 12000000,
       .nvm_divisor = 59,
       .bus_period_ticks = 10},
      {.timebase_hz = 60000000,
       .oscillator_hz = 12000000,
       .nvm_divisor = 88,
       .bus_period_ticks = 10},
      {.timebase_hz = 60000001,
       .oscillator_hz = 12000000,
       .nvm_divisor = 72,
       .bus_period_ticks = 10},
      {.timebase_hz = 60000000,
       .oscillator_hz = 12000000,
       .nvm_divisor = 72,
       .bus_period_ticks = 0},
      {.timebase_hz = 60000000,
       .oscillator_hz = 12000000,
       .nvm_divisor = 72,
       .bus_period_ticks = 61},
  }};
  for (const auto &basis : invalid) {
    reject(
        [&] { static_cast<void>(flash_duration_ticks(basis, FlashTimedOperation::word_program)); },
        "MCU flash timing");
  }
}

void clock_owned_deadlines() {
  constexpr unsigned transfer_cycles = 3;
  require(flash_transfer_ticks(
              initial, {.bus_cycles = transfer_cycles, .total_duration_ticks = program_ticks}) ==
                  transfer_ticks &&
              flash_deadline(launch_ticks, program_ticks) == completed_tick &&
              flash_deadline(maximum - program_ticks, program_ticks) == maximum,
          "Explicit transfer hypothesis and exact representable command deadlines");
  reject(
      [] {
        static_cast<void>(flash_transfer_ticks(
            initial, {.bus_cycles = 0, .total_duration_ticks = program_ticks}));
      },
      "hypothesis");
  reject(
      [] {
        static_cast<void>(flash_transfer_ticks(
            initial, {.bus_cycles = at_completion_cycles, .total_duration_ticks = program_ticks}));
      },
      "hypothesis");
  reject([] { static_cast<void>(flash_deadline(1, 0)); }, "zero-duration");
  reject([] { static_cast<void>(flash_deadline(maximum - (program_ticks - 1), program_ticks)); },
         "overflows");
}
}

int main() {
  try {
    literal_equations();
    explicit_verify_units();
    basis_boundaries();
    clock_owned_deadlines();
    std::cout << "UF32 flash timing:4literal bases,word/burst/row/erase equations,explicit scan "
                 "units,invalid inputs and deadline bounds passed;no autonomous native controller "
                 "claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
