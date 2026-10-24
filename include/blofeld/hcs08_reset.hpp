// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "blofeld/bus.hpp"
#include "blofeld/hcs08_bdc.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
enum class Hcs08ResetContract : std::uint8_t { unqualified, reference_bus_edges };
enum class Hcs08ResetDrive : std::uint8_t { unknown, low, released };
enum class Hcs08ModePad : std::uint8_t { unknown, low, high };
enum class Hcs08ResetPhase : std::uint8_t { driving, settling, external_hold, vector, ready };

struct Hcs08ResetState {
  std::uint64_t edges = 0;
  unsigned vector_periods = 0;
  Hcs08ResetPhase phase = Hcs08ResetPhase::driving;
  std::optional<bool> external_at_sample;
  std::optional<Hcs08BdcReset> mode;
  bool operator==(const Hcs08ResetState &) const = default;
};

// Explicit nominal 34-drive/38-settle/six-vector reference edge contract.
// Inputs describe an external low/released RESET driver and sampled BKGD/MS.
// Actual reset recognition, analog/setup/hold and target clock require the board.
// CPU, NVM/peripherals and BDC mode must be reset by their separate owner.
class Hcs08ResetSequence {
public:
  explicit Hcs08ResetSequence(Hcs08ResetContract contract);
  void edge(Hcs08ResetDrive reset, Hcs08ModePad mode, ExecutionContext context);
  [[nodiscard]] bool drives_low() const;
  [[nodiscard]] bool level(Hcs08ResetDrive reset) const;
  [[nodiscard]] Hcs08ResetState state() const;
  [[nodiscard]] bool faulted() const;

private:
  [[noreturn]] void fail(const std::string &reason) const;
  Hcs08ResetState state_;
  ExecutionContext context_;
  bool healthy_ = true;
};
}
