// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>

namespace blofeld {
// FM4-3/Table5-1 require three additional instruction cycles after an SC change.
// Only settled scalar AGU MOVE sampling is qualified; this is not a model of
// the physical sampling stage during those three cycles.
class DspCompatibilityMode {
public:
  [[nodiscard]] bool retire(const std::uint64_t cycles, const bool compatibility) noexcept {
    if (cycles < boundary_) { return false; }
    boundary_ = cycles;
    if (compatibility != requested_) {
      requested_ = compatibility;
      changed_ = cycles;
    }
    return true;
  }

  [[nodiscard]] std::optional<bool> sample(const std::uint64_t cycles,
                                           const bool compatibility) const noexcept {
    constexpr std::uint64_t settle_cycles = 3;
    if (cycles < boundary_ || compatibility != requested_ ||
        (changed_ && cycles - *changed_ < settle_cycles)) {
      return std::nullopt;
    }
    return requested_;
  }

  bool operator==(const DspCompatibilityMode &) const = default;

private:
  std::uint64_t boundary_ = 0;
  std::optional<std::uint64_t> changed_;
  bool requested_ = false;
};
}
