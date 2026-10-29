// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/nand.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
enum class NandOperationContract : std::uint8_t { unqualified, reference_maximum };

struct NandCompletionEvent {
  std::uint64_t ticks;
  NandPending operation;
  bool operator==(const NandCompletionEvent &) const = default;
};

// K9F1208U0C Rev1.1 p.13: explicit maximum-duration read/reset reference.
// Busy is recognized at the input latch (zero tWB, within its 100ns maximum).
// This is not measured silicon timing, R/B pull-up delay or board wiring.
// Owns the protocol so reset/power loss cannot leave stale read events behind.
// Copies are complete trusted value checkpoints, including cells and pending time.
class NandOperationClock {
public:
  NandOperationClock(Nand device, std::uint32_t timebase_hz, NandOperationContract contract);
  // Events precede inputs at the same tick. The board must interleave other
  // devices at next_event(); a larger advance still reports the exact event tick.
  [[nodiscard]] std::optional<NandCompletionEvent> advance(std::uint64_t ticks);
  [[nodiscard]] std::optional<NandCompletionEvent> next_event() const;

  [[nodiscard]] const Nand &device() const { return device_; }

  [[nodiscard]] std::uint64_t ticks() const { return ticks_; }

  [[nodiscard]] bool faulted() const { return !healthy_; }

  void observe_power_ready();
  void observe_identification(NandIdentification identification);
  void observe_wp(std::optional<bool> high);
  void observe_cell(NandLocation location, NandByte cell);
  void command(std::uint8_t value);
  void address(std::uint8_t value);
  void write(std::uint8_t value);
  [[nodiscard]] std::uint8_t read(std::uint8_t mask = Nand::byte_mask);
  // Only qualified selected RE rising / read-end CE-high observations.
  // Zero tRB (within its 100ns maximum); sequential tR / abort tCRY use maxima.
  void observe_read_end(NandReadEnd edge);
  // Program/erase duration and success remain external observations.
  void observe_completion(NandOutcome outcome);
  void power_loss();

private:
  void healthy() const;
  [[noreturn]] void fail(const std::string &reason) const;
  void schedule(NandPending operation, unsigned microseconds);
  [[nodiscard]] unsigned reset_duration() const;
  Nand device_;
  std::uint32_t timebase_hz_;
  std::uint64_t ticks_ = 0;
  std::optional<NandCompletionEvent> event_;
  bool healthy_ = true;
};
}
