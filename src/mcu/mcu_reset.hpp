// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
enum class McuResetSource : std::uint8_t { external, cop, power_on };
enum class McuResetPad : std::uint8_t { unknown, low, high };
enum class McuResetPhase : std::uint8_t { driving, settling, external_hold, ready, complete };

struct McuResetState {
  McuResetSource requested = McuResetSource::external;
  McuResetSource selected = McuResetSource::external;
  McuResetPhase phase = McuResetPhase::driving;
  unsigned synchronization_cycles = 0;
  std::uint64_t edges = 0;
  bool operator==(const McuResetState &) const = default;
};

// S12CRGUV1 5.2: explicit SYSCLK edges and observed RESET pad. The caller
// supplies the qualified 3..6-cycle synchronization delay, never a guessed default.
class McuResetSequence {
public:
  McuResetSequence(McuResetSource source, unsigned synchronization_cycles);
  void edge(McuResetPad pad);
  void complete();

  [[nodiscard]] McuResetState state() const { return state_; }

  [[nodiscard]] bool active() const;
  [[nodiscard]] bool drives_low() const;
  [[nodiscard]] std::uint16_t vector() const;

private:
  McuResetState state_;
};
}
