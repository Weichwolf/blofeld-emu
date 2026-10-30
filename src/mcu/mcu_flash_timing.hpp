// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
// Exact digital clock basis. Bus frequency need not be an integer number of Hz.
struct FlashTimeBasis {
  std::uint64_t timebase_hz;
  std::uint32_t oscillator_hz;
  unsigned nvm_divisor;
  std::uint64_t bus_period_ticks;
  bool operator==(const FlashTimeBasis &) const = default;
};

enum class FlashTimedOperation : std::uint8_t {
  word_program,
  same_row_burst,
  sector_erase,
  mass_erase,
  verify_words,
  verify_bytes
};

// UF32 V01.05 A.2.1 nominal equations, not calibrated pulse boundaries. Erase
// equations are approximate; verify units remain explicit competing candidates.
[[nodiscard]] std::uint64_t flash_duration_ticks(const FlashTimeBasis &basis,
                                                 FlashTimedOperation operation,
                                                 unsigned verify_scan_count = 0);
[[nodiscard]] std::uint64_t flash_deadline(std::uint64_t start, std::uint64_t delay);

struct FlashTransferHypothesis {
  unsigned bus_cycles;
  std::uint64_t total_duration_ticks;
};

// Transfer latency is external hypothesis input; no undocumented value is defaulted.
[[nodiscard]] std::uint64_t flash_transfer_ticks(const FlashTimeBasis &basis,
                                                 FlashTransferHypothesis hypothesis);
}
