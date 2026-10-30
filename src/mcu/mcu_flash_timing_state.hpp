// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "mcu/mcu_flash_timing.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
enum class FlashTimingContract : std::uint8_t { unqualified, uf32_formula_candidate };
enum class FlashVerifyUnit : std::uint8_t { unqualified, words, bytes };

struct FlashTimingInputs {
  FlashTimingContract contract = FlashTimingContract::unqualified;
  // Undocumented transfer phase: required explicit hypothesis, never defaulted.
  std::optional<unsigned> transfer_bus_cycles = std::nullopt;
  FlashVerifyUnit verify_unit = FlashVerifyUnit::unqualified;
  bool operator==(const FlashTimingInputs &) const = default;
};

struct FlashClockBoundary {
  std::uint64_t ticks;
  std::uint64_t timebase_hz;
  std::uint32_t oscillator_hz;
  std::uint64_t bus_period_ticks;
  // The existing COP reference preempts events exactly at its reset boundary.
  bool include_endpoint = true;
};

// Timing metadata only. Command/address/data truth remains in McuFlash's slots.
struct FlashTimingState {
  FlashTimingInputs inputs;
  std::optional<FlashClockBoundary> clock;
  std::optional<std::uint64_t> transfer;
  std::optional<std::uint64_t> completion;
  std::optional<std::uint64_t> origin;
  std::optional<std::uint64_t> pending_duration;
  std::optional<FlashTimedOperation> pending_operation;
  bool faulted = false;
};
}
