// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_spi_run_clock.hpp"

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_clock.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace blofeld {
Hcs08SpiRunClock::Hcs08SpiRunClock(Hcs08Bdc &bdc,
                                   Hcs08SpiClock &spi,
                                   const Hcs08SpiRunContract order,
                                   const Hcs08SpiLaunchContract launch,
                                   const Hcs08PrefetchContract program)
    : bdc_(bdc), spi_(spi), order_(order), launch_(launch), program_(program) {
  expected_ = boundary();
  if (order != Hcs08SpiRunContract::reference_cpu_before_spi &&
      order != Hcs08SpiRunContract::reference_spi_before_cpu) {
    fail("requires an explicit CPU/SPI ordering reference");
  }
  if (launch != Hcs08SpiLaunchContract::reference_one_period &&
      launch != Hcs08SpiLaunchContract::reference_two_periods) {
    fail("requires an explicit one/two-period launch reference");
  }
  if (program != Hcs08PrefetchContract::reference_peripheral_program_cycles &&
      program != Hcs08PrefetchContract::reference_jump_program_cycles &&
      program != Hcs08PrefetchContract::reference_scanner_byte_cycles &&
      program != Hcs08PrefetchContract::reference_scanner_word_cycles &&
      program != Hcs08PrefetchContract::reference_scanner_move_cycles) {
    fail("requires a supported IRQ program contract");
  }
  healthy();
  check_owner();
}

[[noreturn]] void Hcs08SpiRunClock::fail(const std::string &reason) const {
  healthy_ = false;
  throw Fault("HCS08 CPU/SPI clock operation pc=" +
              (expected_.cpu.pc_known ? hex(expected_.cpu.pc) : "unknown") +
              " ccr=" + hex(expected_.cpu.ccr) +
              " attempted-periods=" + std::to_string(state_.attempted_periods) +
              " completed-periods=" + std::to_string(state_.completed_periods) +
              " phase=" + std::to_string(static_cast<unsigned>(state_.phase)) + ": " + reason);
}

ExecutionContext Hcs08SpiRunClock::context() const {
  const auto cpu = bdc_.cpu_state();
  return {.pc = cpu.pc, .ccr = cpu.ccr};
}

Hcs08SpiRunBoundary Hcs08SpiRunClock::boundary() const {
  return {.cpu = bdc_.cpu_state(),
          .irq = bdc_.irq_bus_state(),
          .program = bdc_.prefetch_state(),
          .spi = spi_.state()};
}

void Hcs08SpiRunClock::healthy() const {
  if (!healthy_ || bdc_.faulted() || spi_.faulted()) {
    fail("incomplete operation requires coordinated recovery");
  }
}

void Hcs08SpiRunClock::check_owner() const {
  if (bdc_.state().active) {
    fail("active background CPU/peripheral clock ordering is unqualified");
  }
  if (bdc_.instruction_reserved()) { fail("instruction budget has another clock owner"); }
  if (boundary() != expected_) { fail("CPU queue/IRQ or SPI clock has another phase owner"); }
  if (expected_.program && expected_.program->contract != program_) {
    fail("owned program queue contract differs from the selected program reference");
  }
  static_cast<void>(spi_.checkpoint(context()));
}

void Hcs08SpiRunClock::check_counters() const {
  constexpr auto limit = std::numeric_limits<std::uint64_t>::max();
  if (state_.attempted_periods == limit || state_.completed_periods == limit ||
      state_.irq_periods == limit || state_.program_periods == limit ||
      state_.spi_periods == limit || state_.launches == limit) {
    fail("period or event counter exhausted");
  }
}

void Hcs08SpiRunClock::advance_cpu() {
  state_.phase = Hcs08SpiRunPhase::cpu;
  auto irq = bdc_.irq_bus_state();
  constexpr unsigned complete_irq = 11;
  if (irq && irq->completed_cycles == complete_irq) {
    bdc_.start_irq_program(program_);
    irq.reset();
  }
  if (!irq) {
    const auto contract = bdc_.prefetch_state()
                              ? Hcs08IrqBusContract::reference_owned_program_handoff
                              : Hcs08IrqBusContract::reference_unstretched_sequence;
    if (bdc_.start_irq_bus(contract)) { irq = bdc_.irq_bus_state(); }
  }
  if (irq) {
    bdc_.advance_irq_bus();
    ++state_.irq_periods;
  } else if (bdc_.prefetch_state()) {
    bdc_.advance_irq_program();
    ++state_.program_periods;
  } else {
    fail("unowned foreground requires a qualified instruction clock or a real eligible IRQ");
  }
}

void Hcs08SpiRunClock::observe_launch() {
  const auto peripheral = spi_.peripheral_state();
  if (spi_.state().active || peripheral.active || !peripheral.transmit ||
      (peripheral.control1 & hcs08_spi::enable) == 0) {
    state_.launch_remaining = 0;
    state_.launch_byte.reset();
    return;
  }
  if (state_.launch_byte) {
    if (state_.launch_byte != peripheral.transmit) {
      fail("queued launch data changed before promotion");
    }
    return;
  }
  state_.launch_byte = peripheral.transmit;
  state_.launch_remaining = launch_ == Hcs08SpiLaunchContract::reference_one_period ? 1U : 2U;
}

void Hcs08SpiRunClock::advance_spi(const std::optional<bool> miso) {
  state_.phase = Hcs08SpiRunPhase::spi;
  const bool scheduled = state_.launch_byte.has_value();
  observe_launch();
  if (scheduled && state_.launch_byte) {
    const auto peripheral = spi_.peripheral_state();
    if (peripheral.transmit != state_.launch_byte || peripheral.active) {
      fail("pending promotion lost its actual queued byte or idle shifter");
    }
    if (--state_.launch_remaining == 0) {
      spi_.begin_transfer(context());
      ++state_.launches;
      state_.launch_byte.reset();
    }
  }
  state_.last_pins = spi_.advance(context(), miso);
  ++state_.spi_periods;
}

std::optional<Hcs08SpiPins> Hcs08SpiRunClock::advance(const std::optional<bool> miso) {
  healthy();
  healthy_ = false;
  check_counters();
  check_owner();
  ++state_.attempted_periods;
  state_.last_pins.reset();
  try {
    if (order_ == Hcs08SpiRunContract::reference_cpu_before_spi) {
      advance_cpu();
      advance_spi(miso);
    } else {
      advance_spi(miso);
      advance_cpu();
    }
    state_.phase = Hcs08SpiRunPhase::observe;
    observe_launch();
    ++state_.completed_periods;
    state_.phase = Hcs08SpiRunPhase::boundary;
    expected_ = boundary();
    healthy_ = true;
    return state_.last_pins;
  } catch (const Fault &error) { fail(error.what()); }
}

Hcs08SpiRunState Hcs08SpiRunClock::state() const {
  return state_;
}

bool Hcs08SpiRunClock::faulted() const {
  return !healthy_ || bdc_.faulted() || spi_.faulted();
}

Hcs08SpiRunCheckpoint Hcs08SpiRunClock::checkpoint() const {
  healthy();
  check_owner();
  Hcs08SpiRunCheckpoint result;
  result.state_ = state_;
  result.expected_ = expected_;
  result.order_ = order_;
  result.launch_ = launch_;
  result.program_ = program_;
  return result;
}

void Hcs08SpiRunClock::restore_checkpoint(const Hcs08SpiRunCheckpoint &checkpoint) {
  if (checkpoint.order_ != order_ || checkpoint.launch_ != launch_ ||
      checkpoint.program_ != program_) {
    fail("checkpoint ordering/launch/program contract mismatch");
  }
  healthy_ = false;
  if (bdc_.faulted() || spi_.faulted()) {
    fail("checkpoint requires separately restored healthy owners");
  }
  state_ = checkpoint.state_;
  expected_ = checkpoint.expected_;
  check_owner();
  healthy_ = true;
}
} // namespace blofeld
