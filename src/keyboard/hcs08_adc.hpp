// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
namespace hcs08_adc {
constexpr unsigned control = 0x50, status = 0x51, high = 0x52, low = 0x53, pins = 0x54;
constexpr unsigned power = 0x80, right = 0x40, eight = 0x20, sign = 0x10;
constexpr unsigned complete = 0x80, interrupt = 0x40, continuous = 0x20, channel_mask = 0x1f;
constexpr unsigned channels = 8, maximum = 1023;
}

enum class Hcs08AdcContract : std::uint8_t { unqualified, candidate_gt8a_reference_stable_input };

struct Hcs08AdcReference {
  std::uint32_t bus_hz, supply_uv, low_uv, high_uv;
  bool operator==(const Hcs08AdcReference &) const = default;
};

struct Hcs08AdcState {
  std::uint8_t control = 0, status = 1, pins = 0;
  std::optional<std::uint16_t> result = std::nullopt;
  std::optional<Hcs08AdcReference> reference = std::nullopt;
  std::array<std::optional<std::uint32_t>, hcs08_adc::channels> inputs{};
  std::uint64_t conversions = 0;
  unsigned remaining = 0;
  bool active = false, complete = false;
  bool operator==(const Hcs08AdcState &) const = default;
};

// Ideal stable-input SAR reference; board clock, supply and input observations
// must be explicit. No polling, callback or injected result completes a job.
class Hcs08Adc {
public:
  explicit Hcs08Adc(Hcs08AdcContract contract = Hcs08AdcContract::unqualified);
  void context(ExecutionContext context);
  void reset();
  void observe_reference(Hcs08AdcReference reference);
  void observe_input(unsigned channel, std::optional<std::uint32_t> microvolts);
  void advance_bus(std::uint64_t periods);
  [[nodiscard]] static bool contains(unsigned address);
  [[nodiscard]] std::uint8_t read(std::uint16_t address);
  void write(ByteWrite access);
  [[nodiscard]] Hcs08AdcState state() const;
  [[nodiscard]] bool interrupt_requested() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] unsigned clock_divisor() const;

private:
  void healthy() const;
  void abort();
  [[nodiscard]] std::uint16_t reference_result() const;
  [[nodiscard]] unsigned continuous_periods() const;
  [[noreturn]] void
  fail(const std::string &operation, unsigned address, const std::string &reason) const;
  Hcs08AdcState state_;
  ExecutionContext context_;
  mutable bool healthy_ = true;
};
} // namespace blofeld
