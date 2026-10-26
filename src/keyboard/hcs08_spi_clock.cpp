// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_spi_clock.hpp"

#include "common/bus.hpp"
#include "keyboard/hcs08_spi.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace blofeld {
Hcs08SpiClock::Hcs08SpiClock(Hcs08Spi &spi, const Hcs08SpiClockContract contract) : spi_(spi) {
  if (contract != Hcs08SpiClockContract::reference_half_period_and_final_edge) {
    fail({}, "requires explicit half-period/final-edge reference contract");
  }
  healthy({});
  check_phase({});
}

Hcs08SpiClock::Hcs08SpiClock(Hcs08Spi &spi,
                             const Hcs08SpiClockCheckpoint &checkpoint,
                             const ExecutionContext context)
    : spi_(spi) {
  restore_checkpoint(checkpoint, context);
}

[[noreturn]] void Hcs08SpiClock::fail(const ExecutionContext context,
                                      const std::string &reason,
                                      const unsigned address) const {
  healthy_ = false;
  throw Fault("HCS08 SPI clock operation address=" + hex(address) + " pc=" + hex(context.pc) +
              " ccr=" + hex(context.ccr) +
              " offered-periods=" + std::to_string(state_.offered_periods) +
              " active-periods=" + std::to_string(state_.active_periods) + ": " + reason);
}

void Hcs08SpiClock::healthy(const ExecutionContext context) const {
  if (!healthy_ || spi_.faulted()) {
    fail(context, "incomplete operation requires coordinated recovery");
  }
}

void Hcs08SpiClock::check_phase(const ExecutionContext context) const {
  const auto s = spi_.state();
  // A genuine register disable may await the next offered abort period.
  if (state_.active && !s.active && (s.control1 & hcs08_spi::enable) == 0) { return; }
  if (s.active != state_.active) { fail(context, "peripheral shifter has another phase owner"); }
  if (!state_.active) { return; }
  constexpr unsigned format = 0x1d;
  if (s.baud != expected_.baud) {
    fail(context, "active baud phase changed outside its clock owner", hcs08_spi::baud);
  }
  if ((s.control1 & format) != (expected_.control1 & format)) {
    fail(context, "active format phase changed outside its clock owner", hcs08_spi::control1);
  }
  if (s.shift_out != expected_.shift_out || s.shift_in != expected_.shift_in ||
      s.edges != expected_.edges || s.samples != expected_.samples || s.mosi != expected_.mosi) {
    fail(context, "shifter phase changed outside its clock owner");
  }
}

void Hcs08SpiClock::check_counters(const ExecutionContext context) const {
  constexpr auto limit = std::numeric_limits<std::uint64_t>::max();
  if (state_.offered_periods == limit || state_.active_periods == limit ||
      state_.generated_edges == limit || state_.completed_transfers == limit ||
      state_.aborted_transfers == limit) {
    fail(context, "period or event counter exhausted");
  }
}

void Hcs08SpiClock::begin_transfer(const ExecutionContext context) {
  healthy(context);
  healthy_ = false;
  check_phase(context);
  if (state_.active) { fail(context, "transfer already has a clock owner"); }
  spi_.context(context);
  spi_.transfer_to_shifter();
  state_.half_period = spi_.clock_divisor() / 2;
  state_.remaining_periods = state_.half_period;
  state_.active = true;
  expected_ = spi_.state();
  healthy_ = true;
}

std::optional<Hcs08SpiPins> Hcs08SpiClock::advance(const ExecutionContext context,
                                                   const std::optional<bool> miso) {
  healthy(context);
  healthy_ = false;
  check_counters(context);
  const auto current = spi_.state();
  if (state_.active && !current.active && (current.control1 & hcs08_spi::enable) == 0) {
    ++state_.offered_periods;
    ++state_.aborted_transfers;
    state_.active = false;
    state_.half_period = 0;
    state_.remaining_periods = 0;
    expected_ = current;
    healthy_ = true;
    return {};
  }
  check_phase(context);
  ++state_.offered_periods;
  if (!state_.active) {
    healthy_ = true;
    return {};
  }
  ++state_.active_periods;
  if (--state_.remaining_periods != 0) {
    healthy_ = true;
    return {};
  }
  spi_.context(context);
  spi_.edge(miso);
  const auto pins = spi_.pins();
  ++state_.generated_edges;
  constexpr unsigned final_edge = 16;
  if (spi_.state().edges == final_edge) {
    spi_.complete_transfer();
    ++state_.completed_transfers;
    state_.active = false;
    state_.remaining_periods = 0;
    state_.half_period = 0;
  } else {
    state_.remaining_periods = state_.half_period;
  }
  expected_ = spi_.state();
  healthy_ = true;
  return pins;
}

void Hcs08SpiClock::reset(const ExecutionContext context) {
  if (spi_.faulted() || spi_.state().active || (spi_.state().control1 & hcs08_spi::enable) != 0) {
    fail(context, "reset requires a separately reset or disabled healthy peripheral");
  }
  state_ = {};
  expected_ = spi_.state();
  healthy_ = true;
}

Hcs08SpiClockState Hcs08SpiClock::state() const {
  return state_;
}

Hcs08SpiState Hcs08SpiClock::peripheral_state() const {
  return spi_.state();
}

bool Hcs08SpiClock::faulted() const {
  return !healthy_ || spi_.faulted();
}

Hcs08SpiClockCheckpoint Hcs08SpiClock::checkpoint(const ExecutionContext context) const {
  healthy(context);
  check_phase(context);
  Hcs08SpiClockCheckpoint result;
  result.state_ = state_;
  result.peripheral_ = spi_.state();
  result.expected_ = expected_;
  return result;
}

void Hcs08SpiClock::restore_checkpoint(const Hcs08SpiClockCheckpoint &checkpoint,
                                       const ExecutionContext context) {
  if (spi_.faulted() || spi_.state() != checkpoint.peripheral_) {
    fail(context, "checkpoint requires the same separately restored peripheral state");
  }
  state_ = checkpoint.state_;
  expected_ = checkpoint.expected_;
  healthy_ = true;
  check_phase(context);
}
} // namespace blofeld
