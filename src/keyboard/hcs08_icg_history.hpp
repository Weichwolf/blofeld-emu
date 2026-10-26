// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08_icg_events.hpp"
#include "keyboard/hcs08_icg_status.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
enum class Hcs08IcgHistoryContract : std::uint8_t { unqualified, candidate_gt8a_unknown_events };

// Conservative event history, not an oscillator or a detector. An unobserved
// interval permits every fault-latch outcome and invalidates clock observations.
// Value copies preserve every possible read/clear sequence for joint checkpoints.
class Hcs08IcgHistory {
public:
  explicit Hcs08IcgHistory(Hcs08IcgHistoryContract contract);
  void context(ExecutionContext value);
  void reset();
  void observe(Hcs08IcgStatusPoint point);
  void control2(std::uint8_t value);
  void signal(Hcs08IcgFault event);
  void unobserved_interval();
  [[nodiscard]] Hcs08KnownByte read_known(std::uint16_t address);
  [[nodiscard]] std::uint8_t read(std::uint16_t address);
  void write(ByteWrite access, std::uint8_t known = Hcs08KnownByte::all);
  // Masking an IRQ cannot mask an unknown reset request.
  [[nodiscard]] Hcs08IcgFaultAction action(bool interrupt_masked) const;

private:
  [[nodiscard]] Hcs08IcgStatus &history(unsigned index);
  [[nodiscard]] const Hcs08IcgStatus &history(unsigned index) const;
  void append(const Hcs08IcgStatus &value);
  [[noreturn]] void
  fail(const std::string &operation, unsigned address, const std::string &reason) const;
  // Two source bits and one read-arm bit give at most eight distinct histories.
  static constexpr unsigned maximum_histories = 8;
  std::array<std::optional<Hcs08IcgStatus>, maximum_histories> histories_;
  unsigned size_ = 1;
  ExecutionContext context_;
};
} // namespace blofeld
