// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_clock.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
enum class Hcs08SpiRunContract : std::uint8_t {
  unqualified,
  reference_cpu_before_spi,
  reference_spi_before_cpu
};
enum class Hcs08SpiLaunchContract : std::uint8_t {
  unqualified,
  reference_one_period,
  reference_two_periods
};
enum class Hcs08SpiRunPhase : std::uint8_t { boundary, cpu, spi, observe };

struct Hcs08SpiRunState {
  std::uint64_t attempted_periods = 0, completed_periods = 0;
  std::uint64_t irq_periods = 0, program_periods = 0, spi_periods = 0, launches = 0;
  unsigned launch_remaining = 0;
  std::optional<std::uint8_t> launch_byte;
  std::optional<Hcs08SpiPins> last_pins;
  Hcs08SpiRunPhase phase = Hcs08SpiRunPhase::boundary;
  bool operator==(const Hcs08SpiRunState &) const = default;
};

struct Hcs08SpiRunBoundary {
  Hcs08State cpu;
  std::optional<Hcs08IrqBusState> irq;
  std::optional<Hcs08PrefetchState> program;
  Hcs08SpiClockState spi;
  bool operator==(const Hcs08SpiRunBoundary &) const = default;
};

class Hcs08SpiRunCheckpoint {
public:
  Hcs08SpiRunCheckpoint(const Hcs08SpiRunCheckpoint &) = default;
  Hcs08SpiRunCheckpoint &operator=(const Hcs08SpiRunCheckpoint &) = default;

private:
  friend class Hcs08SpiRunClock;
  Hcs08SpiRunCheckpoint() = default;
  Hcs08SpiRunState state_;
  Hcs08SpiRunBoundary expected_;
  Hcs08SpiRunContract order_ = Hcs08SpiRunContract::unqualified;
  Hcs08SpiLaunchContract launch_ = Hcs08SpiLaunchContract::unqualified;
  Hcs08PrefetchContract program_ = Hcs08PrefetchContract::unqualified;
};

// The board wires CPU memory and SPI to the same target and owns external pins,
// ADC/timers and BDC transport. One explicit period advances IRQ/program and SPI
// once in the selected reference order. New queued data launches after one/two
// subsequent SPI stages; completion may arm a buffered successor in this period.
// Physical write/promotion/edge ordering and active BDC clocks are unqualified.
class Hcs08SpiRunClock {
public:
  Hcs08SpiRunClock(
      Hcs08Bdc &bdc,
      Hcs08SpiClock &spi,
      Hcs08SpiRunContract order,
      Hcs08SpiLaunchContract launch,
      Hcs08PrefetchContract program = Hcs08PrefetchContract::reference_peripheral_program_cycles);
  Hcs08SpiRunClock(const Hcs08SpiRunClock &) = delete;
  Hcs08SpiRunClock &operator=(const Hcs08SpiRunClock &) = delete;
  // Returns the actual edge pins even at byte completion. Failed later CPU
  // stages retain already committed pins in state().last_pins for diagnostics.
  [[nodiscard]] std::optional<Hcs08SpiPins> advance(std::optional<bool> miso);
  [[nodiscard]] Hcs08SpiRunState state() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] Hcs08SpiRunCheckpoint checkpoint() const;
  void restore_checkpoint(const Hcs08SpiRunCheckpoint &checkpoint);

private:
  [[nodiscard]] Hcs08SpiRunBoundary boundary() const;
  [[nodiscard]] ExecutionContext context() const;
  void healthy() const;
  void check_owner() const;
  void check_counters() const;
  void advance_cpu();
  void advance_spi(std::optional<bool> miso);
  void observe_launch();
  [[noreturn]] void fail(const std::string &reason) const;
  Hcs08Bdc &bdc_;
  Hcs08SpiClock &spi_;
  const Hcs08SpiRunContract order_;
  const Hcs08SpiLaunchContract launch_;
  const Hcs08PrefetchContract program_;
  Hcs08SpiRunState state_;
  Hcs08SpiRunBoundary expected_;
  mutable bool healthy_ = true;
};
} // namespace blofeld
