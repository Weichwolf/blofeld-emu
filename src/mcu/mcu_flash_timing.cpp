// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/mcu_flash_timing.hpp"

#include "common/bus.hpp"

#include <cstdint>
#include <limits>

namespace blofeld {
namespace {
constexpr unsigned minimum_oscillator = 500000;
constexpr unsigned maximum_oscillator = 60000000;
constexpr unsigned minimum_bus = 1000000;
constexpr unsigned minimum_nvm = 150000;
constexpr unsigned maximum_nvm = 200000;
constexpr unsigned divider_mask_maximum = 64;
constexpr unsigned prescaler = 8;
constexpr unsigned flash_bytes = 32768;
constexpr unsigned flash_words = flash_bytes / 2;
constexpr unsigned verify_setup = 10;
constexpr unsigned single_nvm = 9;
constexpr unsigned single_bus = 25;
constexpr unsigned burst_nvm = 4;
constexpr unsigned burst_bus = 9;
constexpr unsigned sector_nvm = 4000;
constexpr unsigned mass_nvm = 20000;

std::uint64_t multiply(const std::uint64_t a, const std::uint64_t b) {
  if (b != 0 && a > std::numeric_limits<std::uint64_t>::max() / b) {
    throw Fault("MCU flash timing multiplication overflow");
  }
  return a * b;
}

void validate(const FlashTimeBasis &basis) {
  const bool divider =
      basis.nvm_divisor != 0 && (basis.nvm_divisor <= divider_mask_maximum ||
                                 (basis.nvm_divisor % prescaler == 0 &&
                                  basis.nvm_divisor / prescaler <= divider_mask_maximum));
  if (!divider) { throw Fault("MCU flash timing requires a representable loaded divider"); }
  if (basis.oscillator_hz < minimum_oscillator || basis.oscillator_hz > maximum_oscillator ||
      basis.timebase_hz == 0 || basis.timebase_hz % basis.oscillator_hz != 0) {
    throw Fault("MCU flash timing requires an exact supported oscillator basis");
  }
  if (basis.bus_period_ticks == 0 || basis.bus_period_ticks > basis.timebase_hz / minimum_bus) {
    throw Fault("MCU flash timing requires an exact bus period at least 1MHz");
  }
  if (basis.oscillator_hz < minimum_nvm * basis.nvm_divisor ||
      basis.oscillator_hz > maximum_nvm * basis.nvm_divisor) {
    throw Fault("MCU flash timing NVM clock is outside 150..200 kHz");
  }
}

std::uint64_t cycles(const FlashTimeBasis &basis, const unsigned nvm, const unsigned bus) {
  const auto nvm_period = multiply(basis.timebase_hz / basis.oscillator_hz, basis.nvm_divisor);
  const auto a = multiply(nvm_period, nvm);
  const auto b = multiply(basis.bus_period_ticks, bus);
  if (a > std::numeric_limits<std::uint64_t>::max() - b) {
    throw Fault("MCU flash timing duration overflow");
  }
  return a + b;
}

std::uint64_t verify(const FlashTimeBasis &basis, const unsigned scans, const unsigned maximum) {
  if (scans == 0 || scans > maximum) {
    throw Fault("MCU flash timing verify scan count is outside its explicit unit geometry");
  }
  return cycles(basis, 0, scans + verify_setup);
}
}

std::uint64_t flash_duration_ticks(const FlashTimeBasis &basis,
                                   const FlashTimedOperation operation,
                                   const unsigned verify_scan_count) {
  validate(basis);
  if (verify_scan_count != 0 && operation != FlashTimedOperation::verify_words &&
      operation != FlashTimedOperation::verify_bytes) {
    throw Fault("MCU flash timing rejects scan input for a non-verify command");
  }
  switch (operation) {
    case FlashTimedOperation::word_program: return cycles(basis, single_nvm, single_bus);
    case FlashTimedOperation::same_row_burst: return cycles(basis, burst_nvm, burst_bus);
    case FlashTimedOperation::sector_erase: return cycles(basis, sector_nvm, 0);
    case FlashTimedOperation::mass_erase: return cycles(basis, mass_nvm, 0);
    case FlashTimedOperation::verify_words: return verify(basis, verify_scan_count, flash_words);
    case FlashTimedOperation::verify_bytes: return verify(basis, verify_scan_count, flash_bytes);
    default: throw Fault("MCU flash timing rejects an unsupported command kind");
  }
}

std::uint64_t flash_deadline(const std::uint64_t start, const std::uint64_t delay) {
  if (delay == 0 || start > std::numeric_limits<std::uint64_t>::max() - delay) {
    throw Fault("MCU flash timing deadline is zero-duration or overflows");
  }
  return start + delay;
}

std::uint64_t flash_transfer_ticks(const FlashTimeBasis &basis,
                                   const FlashTransferHypothesis hypothesis) {
  validate(basis);
  const auto delay = multiply(basis.bus_period_ticks, hypothesis.bus_cycles);
  if (delay == 0 || delay >= hypothesis.total_duration_ticks) {
    throw Fault("MCU flash transfer hypothesis must be positive and precede completion");
  }
  return delay;
}
}
