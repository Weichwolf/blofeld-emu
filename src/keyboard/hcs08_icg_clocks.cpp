// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_icg_clocks.hpp"

#include "common/bus.hpp"

#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <string_view>

namespace blofeld {
namespace {
constexpr unsigned control_address = 0x48, status_address = 0x4a;
constexpr unsigned range = 0x40, mode_shift = 3, mode_mask = 3;
constexpr unsigned multiplier_shift = 4, field_mask = 7, internal_prescale = 64;
constexpr unsigned internal_divider = 7, bus_divider = 2, fixed_threshold = 4;

unsigned multiplier(const unsigned control2) {
  return 4 + (2 * ((control2 >> multiplier_shift) & field_mask));
}

unsigned divider(const unsigned control2) {
  return 1U << (control2 & field_mask);
}

unsigned external_prescale(const unsigned control1) {
  return (control1 & range) != 0 ? 1U : internal_prescale;
}
}

Hcs08IcgClocks::Hcs08IcgClocks(const Hcs08IcgClockContract contract) {
  if (contract != Hcs08IcgClockContract::candidate_gt8a_documented_routes) {
    fail("configure", control_address, "requires the explicit candidate clock-routing contract");
  }
}

void Hcs08IcgClocks::context(const ExecutionContext value) {
  context_ = value;
}

std::optional<Hcs08IcgTargetRatio>
Hcs08IcgClocks::target_ratio(const std::uint8_t requested_control1,
                             const std::uint8_t requested_control2) {
  const unsigned mode = (requested_control1 >> mode_shift) & mode_mask;
  if (mode == 0 || mode == 2) { return {}; }
  const unsigned prescale = mode == 1 ? internal_prescale : external_prescale(requested_control1);
  const unsigned numerator = prescale * multiplier(requested_control2);
  const unsigned denominator = divider(requested_control2) * (mode == 1 ? internal_divider : 1U);
  const unsigned common = std::gcd(numerator, denominator);
  return Hcs08IcgTargetRatio{.numerator = numerator / common, .denominator = denominator / common};
}

void Hcs08IcgClocks::validate(const Hcs08IcgClockPoint &point) const {
  switch (point.mode) {
    case Hcs08IcgClockMode::off:
    case Hcs08IcgClockMode::scm:
    case Hcs08IcgClockMode::fbe:
      if (point.locked == true) {
        fail("route", status_address, "LOCK must be clear in this mode");
      }
      return;
    case Hcs08IcgClockMode::fei: return;
    case Hcs08IcgClockMode::fee:
      switch (point.fee_phase) {
        case Hcs08IcgFeePhase::unqualified:
        case Hcs08IcgFeePhase::after_first_lock: return;
        case Hcs08IcgFeePhase::initial_or_multiplier_relock:
          if (point.locked == true) {
            fail("route",
                 status_address,
                 "LOCK conflicts with the initial/reconfigured half divider");
          }
          return;
        case Hcs08IcgFeePhase::unsupported:
        default: break;
      }
      fail("route", status_address, "unsupported FEE divider phase");
    case Hcs08IcgClockMode::unqualified:
    case Hcs08IcgClockMode::unsupported:
    default: break;
  }
  fail("route", status_address, "actual synchronized clock mode is unknown or unsupported");
}

unsigned Hcs08IcgClocks::fee_divider(const Hcs08IcgClockPoint &point) const {
  if (point.locked == true) { return 1; }
  switch (point.fee_phase) {
    case Hcs08IcgFeePhase::initial_or_multiplier_relock: return 2;
    case Hcs08IcgFeePhase::after_first_lock: return 1;
    case Hcs08IcgFeePhase::unqualified:
    case Hcs08IcgFeePhase::unsupported:
    default: break;
  }
  fail("route", status_address, "FEE initial/multiplier-change divider history is unknown");
}

Hcs08IcgFrequency Hcs08IcgClocks::source(const std::optional<Hcs08IcgFrequency> &value,
                                         const std::string_view name) const {
  if (!value) {
    fail("route", status_address, std::string(name) + " frequency has not been observed");
  }
  if (value->numerator == 0 || value->denominator == 0) {
    fail("route",
         status_address,
         std::string(name) + " requires a positive, nonstatic rational frequency");
  }
  const auto common = std::gcd(value->numerator, value->denominator);
  return {.numerator = value->numerator / common, .denominator = value->denominator / common};
}

Hcs08IcgFrequency Hcs08IcgClocks::divide(Hcs08IcgFrequency value, const unsigned divisor) const {
  if (divisor == 0) { fail("route", status_address, "zero clock divisor"); }
  const auto common = std::gcd(value.numerator, static_cast<std::uint64_t>(divisor));
  value.numerator /= common;
  const auto factor = divisor / common;
  if (factor == 0 || value.denominator > std::numeric_limits<std::uint64_t>::max() / factor) {
    fail("route", status_address, "rational clock denominator overflow");
  }
  value.denominator *= factor;
  return value;
}

Hcs08IcgFrequency Hcs08IcgClocks::output(const Hcs08IcgClockPoint &point) const {
  const unsigned reduction = divider(point.control2);
  if (point.mode == Hcs08IcgClockMode::fbe) {
    return divide(source(point.external, "external reference"), reduction);
  }
  const unsigned initial = point.mode == Hcs08IcgClockMode::fee ? fee_divider(point) : 1U;
  return divide(source(point.dco, "actual DCO"), reduction * initial);
}

std::optional<Hcs08IcgFrequency> Hcs08IcgClocks::clock(const Hcs08IcgClockDomain domain,
                                                       const Hcs08IcgClockPoint &point) const {
  switch (domain) {
    case Hcs08IcgClockDomain::icgout:
    case Hcs08IcgClockDomain::bus:
    case Hcs08IcgClockDomain::fixed: break;
    case Hcs08IcgClockDomain::unqualified:
    case Hcs08IcgClockDomain::unsupported:
    default: fail("route", status_address, "unknown clock consumer domain");
  }
  validate(point);
  if (point.mode == Hcs08IcgClockMode::off) { return {}; }
  if (domain == Hcs08IcgClockDomain::fixed) {
    if (point.mode == Hcs08IcgClockMode::scm || point.mode == Hcs08IcgClockMode::fei) { return {}; }
    const unsigned prescale = external_prescale(point.control1);
    const bool can_fix =
        point.mode == Hcs08IcgClockMode::fee &&
        prescale * multiplier(point.control2) >= fixed_threshold * divider(point.control2);
    if (can_fix) {
      if (!point.locked) { fail("route", status_address, "XCLK selection requires known LOCK"); }
      if (*point.locked) {
        return divide(source(point.external, "external reference"), bus_divider);
      }
    }
    return divide(output(point), bus_divider);
  }
  const auto value = output(point);
  return domain == Hcs08IcgClockDomain::bus ? divide(value, bus_divider) : value;
}

[[noreturn]] void Hcs08IcgClocks::fail(const std::string &operation,
                                       const unsigned address,
                                       const std::string &reason) const {
  throw Fault("HCS08 ICG clock " + operation + " address=" + hex(address) +
              " pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr) + " agent=CPU: " + reason);
}
} // namespace blofeld
