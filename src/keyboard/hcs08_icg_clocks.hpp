// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace blofeld {
enum class Hcs08IcgClockContract : std::uint8_t {
  unqualified = 0,
  candidate_gt8a_documented_routes = 1,
  unsupported = 255
};
enum class Hcs08IcgClockMode : std::uint8_t {
  unqualified = 0,
  off = 1,
  scm = 2,
  fei = 3,
  fbe = 4,
  fee = 5,
  unsupported = 255
};
enum class Hcs08IcgClockDomain : std::uint8_t {
  unqualified = 0,
  icgout = 1,
  bus = 2,
  fixed = 3,
  unsupported = 255
};
enum class Hcs08IcgFeePhase : std::uint8_t {
  unqualified = 0,
  initial_or_multiplier_relock = 1,
  after_first_lock = 2,
  unsupported = 255
};

struct Hcs08IcgFrequency {
  std::uint64_t numerator = 0, denominator = 1;
  bool operator==(const Hcs08IcgFrequency &) const = default;
};

struct Hcs08IcgTargetRatio {
  std::uint64_t numerator = 0, denominator = 1;
  bool operator==(const Hcs08IcgTargetRatio &) const = default;
};

struct Hcs08IcgClockPoint {
  Hcs08IcgClockMode mode = Hcs08IcgClockMode::unqualified;
  // Effective, synchronized hardware controls; not a pending software request.
  std::uint8_t control1 = 0, control2 = 0;
  std::optional<Hcs08IcgFrequency> dco = std::nullopt, external = std::nullopt;
  std::optional<bool> locked = std::nullopt;
  Hcs08IcgFeePhase fee_phase = Hcs08IcgFeePhase::unqualified;
  bool operator==(const Hcs08IcgClockPoint &) const = default;
};

// GT Rev1 ch9. Requested target and observed running source are separate facts.
// No oscillator, mode transition, LOCK/DCOS observation or CPU status is supplied.
class Hcs08IcgClocks {
public:
  explicit Hcs08IcgClocks(Hcs08IcgClockContract contract);
  void context(ExecutionContext value);
  [[nodiscard]] static std::optional<Hcs08IcgTargetRatio>
  target_ratio(std::uint8_t requested_control1, std::uint8_t requested_control2);
  // A disengaged result means a documented gated-off clock, never unknown Hz.
  [[nodiscard]] std::optional<Hcs08IcgFrequency> clock(Hcs08IcgClockDomain domain,
                                                       const Hcs08IcgClockPoint &point) const;

private:
  void validate(const Hcs08IcgClockPoint &point) const;
  [[nodiscard]] unsigned fee_divider(const Hcs08IcgClockPoint &point) const;
  [[nodiscard]] Hcs08IcgFrequency output(const Hcs08IcgClockPoint &point) const;
  [[nodiscard]] Hcs08IcgFrequency source(const std::optional<Hcs08IcgFrequency> &value,
                                         std::string_view name) const;
  [[nodiscard]] Hcs08IcgFrequency divide(Hcs08IcgFrequency value, unsigned divisor) const;
  [[noreturn]] void
  fail(const std::string &operation, unsigned address, const std::string &reason) const;
  ExecutionContext context_;
};
} // namespace blofeld
