// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
enum class Hcs08TpmContract : std::uint8_t { unqualified, candidate_gt8a_run_bus_edges };

struct Hcs08TpmState {
  std::uint16_t counter = 0;
  std::optional<std::uint16_t> read_latch = std::nullopt;
  std::optional<unsigned> prescaler_phase = std::nullopt;
  std::uint8_t control = 0, read_bytes = 0;
  bool overflow = false, overflow_read = false;
  bool operator==(const Hcs08TpmState &) const = default;
};

// TPM1 up-counter with MOD=0 and qualified run-mode BUSCLK edges. No oscillator,
// channel, PWM, external-clock synchronization or CPU interrupt is synthesized.
class Hcs08Tpm {
public:
  explicit Hcs08Tpm(Hcs08TpmContract contract, std::optional<unsigned> phase = std::nullopt);
  void context(ExecutionContext value);
  void reset(std::optional<unsigned> phase = std::nullopt);
  void observe_bus_phase(unsigned phase);
  void advance_bus(std::uint64_t periods);
  [[nodiscard]] Hcs08TpmState state() const;
  [[nodiscard]] bool interrupt_requested() const;
  [[nodiscard]] std::uint8_t read(std::uint16_t address);
  void write(ByteWrite access);
  static constexpr std::uint16_t status = 0x30, count_high = 0x31, count_low = 0x32;

private:
  void write_control(std::uint8_t value);
  [[noreturn]] void
  fail(const std::string &operation, unsigned address, const std::string &reason) const;
  Hcs08TpmState state_;
  ExecutionContext context_;
};
} // namespace blofeld
