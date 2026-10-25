// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
namespace hcs08_icg {
constexpr std::uint16_t control1 = 0x48, control2 = 0x49;
constexpr std::uint16_t status1 = 0x4a, status2 = 0x4b;
constexpr std::uint16_t filter_upper = 0x4c, filter_lower = 0x4d, trim = 0x4e;
constexpr std::uint8_t reset_control1 = 0x44, all_known = 0xff;
constexpr std::uint16_t reset_filter = 0xc0;
}

enum class Hcs08IcgContract : std::uint8_t { unqualified, candidate_gt8a_no_clock_events };
enum class Hcs08IcgReset : std::uint8_t { unqualified, power_on, warm };

struct Hcs08IcgState {
  std::uint8_t control1_request = hcs08_icg::reset_control1;
  std::uint8_t control1_known = hcs08_icg::all_known;
  std::uint8_t control2_request = 0;
  std::optional<std::uint8_t> trim;
  std::optional<std::uint16_t> filter = hcs08_icg::reset_filter;
  bool control1_written = false;
  bool control2_written = false;
  bool reference_history_valid = true;
  bool operator==(const Hcs08IcgState &) const = default;
};

// Register/reset owner only: control requests are not LOCK/DCOS or clock rates.
// Live status/filter transactions and changed cross-domain requests remain strict
// until a clock/synchronization owner is connected. Copies retain no callbacks.
class Hcs08Icg {
public:
  Hcs08Icg(Hcs08IcgContract contract, Hcs08IcgReset cause);
  void context(ExecutionContext value);
  void reset(Hcs08IcgReset cause);
  [[noreturn]] void unqualified_clock_event();
  [[nodiscard]] Hcs08IcgState state() const;
  [[nodiscard]] std::uint8_t read(std::uint16_t address) const;
  void write(ByteWrite access);

private:
  void require_history(const std::string &operation, unsigned address) const;
  void write_control1(std::uint8_t value);
  void write_control2(std::uint8_t value);
  [[noreturn]] void
  fail(const std::string &operation, unsigned address, const std::string &reason) const;
  Hcs08IcgState state_;
  ExecutionContext context_;
};
} // namespace blofeld
