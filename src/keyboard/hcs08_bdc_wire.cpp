// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_bdc_wire.hpp"

#include "common/bus.hpp"
#include "keyboard/hcs08_bdc.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned bit_cycles = 16, sample_cycle = 10, timeout_cycles = 512, sync_request = 128;
constexpr unsigned byte_bits = 8, bit_high = 0x80, byte_mask = 255;
}

Hcs08BdcWire::Hcs08BdcWire(Hcs08Bdc &bdc,
                           const Hcs08BdcWireContract contract,
                           const Hcs08BdcDrive initial_host,
                           const Hcs08BdcWireClock clock)
    : bdc_(bdc) {
  if (contract != Hcs08BdcWireContract::reference_command_delay) {
    fail("requires an explicit reference command-service delay contract");
  }
  if (clock.period_ticks == 0 || clock.first_edge_ticks == 0 ||
      clock.first_edge_ticks > clock.period_ticks ||
      clock.period_ticks > std::numeric_limits<std::uint64_t>::max() / timeout_cycles) {
    fail("invalid or unknown reference clock cadence/phase");
  }
  state_.period_ticks = clock.period_ticks;
  state_.next_clock_ticks = clock.first_edge_ticks;
  host_drive(initial_host, {});
}

void Hcs08BdcWire::healthy() const {
  if (!healthy_ || bdc_.faulted()) { fail("incomplete execution requires coordinated recovery"); }
}

[[noreturn]] void Hcs08BdcWire::fail(const std::string &reason) const {
  throw Fault("HCS08 BDC wire BKGD operation pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr) +
              " BDC-cycles=" + std::to_string(state_.cycles) + ": " + reason);
}

bool Hcs08BdcWire::resolved() const {
  if (state_.host == Hcs08BdcDrive::unknown) { fail("unknown host drive"); }
  if ((state_.host == Hcs08BdcDrive::low && state_.target == Hcs08BdcDrive::high) ||
      (state_.host == Hcs08BdcDrive::high && state_.target == Hcs08BdcDrive::low)) {
    fail("opposing active host/target drivers");
  }
  // Candidate GT8A BKGD's internal pull-up applies only after active drivers.
  return state_.host != Hcs08BdcDrive::low && state_.target != Hcs08BdcDrive::low;
}

void Hcs08BdcWire::host_drive(const Hcs08BdcDrive drive,
                              const ExecutionContext context,
                              const std::optional<std::uint64_t> time_ticks) {
  healthy();
  context_ = context;
  healthy_ = false;
  const auto time = time_ticks.value_or(state_.time_ticks);
  if (time < state_.time_ticks || time >= state_.next_clock_ticks) {
    fail("backward host time or an unobserved BDC clock edge");
  }
  state_.time_ticks = time;
  finish_bit(time);
  if (drive != Hcs08BdcDrive::released && drive != Hcs08BdcDrive::low &&
      drive != Hcs08BdcDrive::high) {
    fail("unknown or invalid host drive");
  }
  const bool before = state_.host == Hcs08BdcDrive::unknown || resolved();
  state_.host = drive;
  const bool after = resolved();
  if (state_.sync == Hcs08BdcSync::wait_release && after) {
    state_.sync = Hcs08BdcSync::wait_response;
    state_.sync_release.reset();
  } else if (before && !after && drive == Hcs08BdcDrive::low) {
    begin_bit();
  }
  healthy_ = true;
}

void Hcs08BdcWire::begin_bit() {
  if (state_.bit_pending || state_.perceived || state_.sync != Hcs08BdcSync::idle) {
    fail("host falling edge before the preceding bit/SYNC finished");
  }
  if (state_.service_due) { fail("host bit before the pending command-service boundary"); }
  const auto control = bdc_.state();
  state_.reply = control.reply_cursor != control.reply_size || state_.transmit_bits != 0;
  if (state_.reply && state_.transmit_bits == 0) { state_.outgoing = bdc_.read_reply(); }
  state_.last_fall = state_.time_ticks;
  state_.bit_pending = true;
}

void Hcs08BdcWire::receive_bit() {
  if (!state_.perceived) { fail("receive sample without a perceived host bit"); }
  state_.incoming = static_cast<std::uint8_t>(
      ((static_cast<unsigned>(state_.incoming) << 1U) | (resolved() ? 1U : 0U)) & byte_mask);
  ++state_.receive_bits;
  if (state_.receive_bits != byte_bits) { return; }
  bdc_.receive(state_.incoming);
  ++state_.received_bytes;
  state_.receive_bits = 0;
  state_.incoming = 0;
  const auto control = bdc_.state();
  if (control.received == control.expected) {
    const auto opcode = control.frame[0];
    const bool no_delay = opcode == hcs08_bdc::read_status || opcode == hcs08_bdc::write_control;
    if (!state_.last_fall) { fail("received byte without an observed host edge"); }
    state_.service_due =
        *state_.last_fall + (state_.period_ticks * (no_delay ? bit_cycles : 2 * bit_cycles));
  }
}

void Hcs08BdcWire::bit_clock() {
  if (state_.bit_pending) {
    state_.bit_pending = false;
    state_.perceived = state_.cycles;
  }
  if (!state_.perceived) { return; }
  const auto age = state_.cycles - *state_.perceived;
  if (state_.reply) {
    const bool one = (state_.outgoing & bit_high) != 0;
    if (age == 0 && !one) { state_.target = Hcs08BdcDrive::low; }
    constexpr unsigned one_pulse = 7;
    constexpr unsigned zero_pulse = 13;
    const auto pulse = one ? one_pulse : zero_pulse;
    if (age == pulse) { state_.target = Hcs08BdcDrive::high; }
    if (age == pulse + 1) { state_.target = Hcs08BdcDrive::released; }
  } else if (age == sample_cycle) {
    receive_bit();
  }
}

void Hcs08BdcWire::finish_bit(const std::uint64_t time_ticks) {
  if (!state_.perceived || !state_.last_fall ||
      time_ticks - *state_.last_fall < state_.period_ticks * bit_cycles) {
    return;
  }
  if (state_.target != Hcs08BdcDrive::released) {
    fail("nominal bit interval ended while the target still drives");
  }
  if (state_.reply) {
    state_.outgoing =
        static_cast<std::uint8_t>((static_cast<unsigned>(state_.outgoing) << 1U) & byte_mask);
    if (++state_.transmit_bits == byte_bits) {
      ++state_.transmitted_bytes;
      state_.transmit_bits = 0;
    }
  }
  state_.perceived.reset();
}

void Hcs08BdcWire::abort_communication() {
  bdc_.communication_timeout();
  state_.incoming = 0;
  state_.outgoing = 0;
  state_.receive_bits = 0;
  state_.transmit_bits = 0;
  state_.perceived.reset();
  state_.service_due.reset();
  state_.bit_pending = false;
  state_.reply = false;
  state_.target = Hcs08BdcDrive::released;
}

void Hcs08BdcWire::clock() {
  healthy();
  healthy_ = false;
  if (state_.cycles >= std::numeric_limits<std::uint64_t>::max() - timeout_cycles) {
    fail("clock horizon exhausted");
  }
  if (state_.next_clock_ticks >
      std::numeric_limits<std::uint64_t>::max() - (state_.period_ticks * timeout_cycles)) {
    fail("reference time horizon exhausted");
  }
  state_.time_ticks = state_.next_clock_ticks;
  state_.next_clock_ticks += state_.period_ticks;
  ++state_.cycles;
  finish_bit(state_.time_ticks);
  if (state_.last_fall && state_.sync == Hcs08BdcSync::idle &&
      state_.time_ticks - *state_.last_fall >= state_.period_ticks * sync_request &&
      state_.host == Hcs08BdcDrive::low) {
    abort_communication();
    state_.sync = Hcs08BdcSync::wait_release;
  }
  if (state_.last_fall && state_.sync == Hcs08BdcSync::idle &&
      state_.time_ticks - *state_.last_fall >= state_.period_ticks * timeout_cycles) {
    abort_communication();
    state_.last_fall.reset();
    ++state_.timeouts;
  }
  if (state_.sync != Hcs08BdcSync::idle) {
    sync_clock();
  } else {
    bit_clock();
    if (state_.service_due && state_.time_ticks >= *state_.service_due) {
      bdc_.service();
      state_.service_due.reset();
    }
  }
  static_cast<void>(resolved());
  healthy_ = true;
}

bool Hcs08BdcWire::level() const {
  healthy();
  return resolved();
}

Hcs08BdcDrive Hcs08BdcWire::target_drive() const {
  healthy();
  return state_.target;
}

Hcs08BdcWireState Hcs08BdcWire::state() const {
  return state_;
}

bool Hcs08BdcWire::faulted() const {
  return !healthy_ || bdc_.faulted();
}

Hcs08BdcWireCheckpoint Hcs08BdcWire::checkpoint() const {
  healthy();
  static_cast<void>(bdc_.checkpoint());
  Hcs08BdcWireCheckpoint checkpoint;
  checkpoint.state_ = state_;
  checkpoint.context_ = context_;
  return checkpoint;
}

void Hcs08BdcWire::restore_checkpoint(const Hcs08BdcWireCheckpoint &checkpoint) {
  static_cast<void>(bdc_.checkpoint());
  state_ = checkpoint.state_;
  context_ = checkpoint.context_;
  healthy_ = true;
}
} // namespace blofeld
