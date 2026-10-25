// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
struct Hcs08InstructionTiming {
  unsigned bytes = 0;
  unsigned periods = 0;
  bool operator==(const Hcs08InstructionTiming &) const = default;
};

// Documented unstretched S08CPUV2 costs for explicitly implemented forms only.
[[nodiscard]] Hcs08InstructionTiming hcs08_instruction_timing(const Bus &bus,
                                                              ExecutionContext context);

enum class Hcs08RunContract : std::uint8_t { unqualified, reference_instruction_retirement };

struct Hcs08RunState {
  std::uint64_t offered_periods = 0, executing_periods = 0, retired_instructions = 0;
  std::uint64_t code_revision = 0;
  unsigned remaining_periods = 0;
  Hcs08State pending;
  bool operator==(const Hcs08RunState &) const = default;
};

class Hcs08RunCheckpoint {
public:
  Hcs08RunCheckpoint(const Hcs08RunCheckpoint &) = default;
  Hcs08RunCheckpoint &operator=(const Hcs08RunCheckpoint &) = default;

private:
  friend class Hcs08RunClock;
  Hcs08RunCheckpoint() = default;
  Hcs08RunState state_;
  std::optional<Hcs08InstructionReservation> reservation_;
};

// Native architectural effects commit at the end of the documented instruction
// budget. This explicit reference contract does not qualify per-cycle bus
// transactions, prefetch, wait states or concurrent BDC arbitration.
class Hcs08RunClock {
public:
  Hcs08RunClock(Bus &bus, Hcs08Bdc &bdc, Hcs08RunContract contract);
  Hcs08RunClock(const Hcs08RunClock &) = delete;
  Hcs08RunClock &operator=(const Hcs08RunClock &) = delete;
  // One completed target bus period. The board orders CPU and BDC edge effects.
  void advance();
  [[nodiscard]] Hcs08RunState state() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] Hcs08RunCheckpoint checkpoint() const;
  void restore_checkpoint(const Hcs08RunCheckpoint &checkpoint);

private:
  void healthy() const;
  void check_reservation() const;
  [[noreturn]] void fail(const std::string &reason) const;
  Bus &bus_;
  Hcs08Bdc &bdc_;
  Hcs08RunState state_;
  std::optional<Hcs08InstructionReservation> reservation_;
  bool healthy_ = true;
};
} // namespace blofeld
