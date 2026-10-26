// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08_spi.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
enum class Hcs08SpiClockContract : std::uint8_t {
  unqualified,
  reference_half_period_and_final_edge
};

struct Hcs08SpiClockState {
  std::uint64_t offered_periods = 0, active_periods = 0, generated_edges = 0;
  std::uint64_t completed_transfers = 0, aborted_transfers = 0;
  unsigned half_period = 0, remaining_periods = 0;
  bool active = false;
  bool operator==(const Hcs08SpiClockState &) const = default;
};

class Hcs08SpiClockCheckpoint {
public:
  Hcs08SpiClockCheckpoint(const Hcs08SpiClockCheckpoint &) = default;
  Hcs08SpiClockCheckpoint &operator=(const Hcs08SpiClockCheckpoint &) = default;

private:
  friend class Hcs08SpiClock;
  Hcs08SpiClockCheckpoint() = default;
  Hcs08SpiClockState state_;
  Hcs08SpiState peripheral_, expected_;
};

// Explicit reference: promotion at begin_transfer, first edge after one full
// half-period, completion at edge16. No physical write/start/SPRF phase claim.
// No callbacks or peers are retained; copies of all owners form a joint restore.
class Hcs08SpiClock {
public:
  explicit Hcs08SpiClock(Hcs08Spi &spi,
                         Hcs08SpiClockContract contract = Hcs08SpiClockContract::unqualified);
  Hcs08SpiClock(Hcs08Spi &spi, const Hcs08SpiClockCheckpoint &checkpoint, ExecutionContext context);
  Hcs08SpiClock(const Hcs08SpiClock &) = delete;
  Hcs08SpiClock &operator=(const Hcs08SpiClock &) = delete;
  // An explicit owner selects this promotion boundary; idle periods never do.
  void begin_transfer(ExecutionContext context);
  // One BUSCLK period, using the peer's input before this period's possible edge.
  // Publish returned pins to the peer even at completion: MOSI was valid at the
  // final sampling edge before the peripheral released its idle data output.
  [[nodiscard]] std::optional<Hcs08SpiPins> advance(ExecutionContext context,
                                                    std::optional<bool> miso);
  void reset(ExecutionContext context);
  [[nodiscard]] Hcs08SpiClockState state() const;
  [[nodiscard]] Hcs08SpiState peripheral_state() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] Hcs08SpiClockCheckpoint checkpoint(ExecutionContext context) const;
  void restore_checkpoint(const Hcs08SpiClockCheckpoint &checkpoint, ExecutionContext context);

private:
  void healthy(ExecutionContext context) const;
  void check_phase(ExecutionContext context) const;
  void check_counters(ExecutionContext context) const;
  [[noreturn]] void fail(ExecutionContext context,
                         const std::string &reason,
                         unsigned address = hcs08_spi::data) const;
  Hcs08Spi &spi_;
  Hcs08SpiClockState state_;
  Hcs08SpiState expected_;
  mutable bool healthy_ = true;
};
} // namespace blofeld
