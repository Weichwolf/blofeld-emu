// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08_icg_events.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
enum class Hcs08IcgStatusContract : std::uint8_t { unqualified, candidate_gt8a_observed_bits };

struct Hcs08IcgStatusPoint {
  // Raw synchronized CLKST field (0..3), independent of requested CLKS.
  std::optional<std::uint8_t> mode = std::nullopt;
  std::optional<bool> reference_crystal = std::nullopt, locked = std::nullopt;
  std::optional<bool> external_stable = std::nullopt, dco_stable = std::nullopt;
  bool operator==(const Hcs08IcgStatusPoint &) const = default;
};

struct Hcs08IcgStatusState {
  Hcs08IcgStatusPoint point;
  Hcs08IcgEventState events;
  bool operator==(const Hcs08IcgStatusState &) const = default;
};

// Composes observed clock facts with owned fault-latch read/ack effects.
// No clock, detector, synchronization, elapsed time or CPU consumer is supplied.
class Hcs08IcgStatus {
public:
  explicit Hcs08IcgStatus(Hcs08IcgStatusContract contract);
  void context(ExecutionContext value);
  void reset();
  void observe(Hcs08IcgStatusPoint point);
  void control2(std::uint8_t value);
  void signal(Hcs08IcgFault event);
  [[nodiscard]] Hcs08KnownByte read_known(std::uint16_t address);
  [[nodiscard]] std::uint8_t read(std::uint16_t address);
  void write(ByteWrite access, std::uint8_t known = Hcs08KnownByte::all);
  [[nodiscard]] Hcs08IcgStatusState state() const;
  [[nodiscard]] Hcs08IcgFaultAction action() const;

private:
  [[nodiscard]] Hcs08KnownByte bits(std::uint16_t address) const;
  [[noreturn]] void
  fail(const std::string &operation, unsigned address, const std::string &reason) const;
  Hcs08IcgEvents events_{Hcs08IcgEventContract::candidate_gt8a_fault_latches};
  Hcs08IcgStatusPoint point_;
  ExecutionContext context_;
};
} // namespace blofeld
