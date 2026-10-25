// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_tpm.hpp"

#include "common/bus.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned phase_limit = 128, word_values = 65536, byte_mask = 255;
constexpr unsigned bus_clock = 8, prescale_mask = 7, overflow_bit = 0x80, irq_bit = 0x40;
constexpr unsigned unsupported_modes = 0x30, selection_mask = 0x1f, control_mask = 0x7f;
constexpr unsigned modulus_high = 0x33, modulus_low = 0x34;
}

Hcs08Tpm::Hcs08Tpm(const Hcs08TpmContract contract, const std::optional<unsigned> phase) {
  if (contract != Hcs08TpmContract::candidate_gt8a_run_bus_edges) {
    fail("configure", status, "requires candidate GT8A run-mode bus-edge contract");
  }
  reset(phase);
}

void Hcs08Tpm::context(const ExecutionContext value) {
  context_ = value;
}

void Hcs08Tpm::reset(const std::optional<unsigned> phase) {
  if (phase && *phase >= phase_limit) { fail("reset", status, "invalid supplied divider phase"); }
  state_ = {.prescaler_phase = phase};
}

void Hcs08Tpm::observe_bus_phase(const unsigned phase) {
  if ((state_.control & bus_clock) != 0 || phase >= phase_limit) {
    fail("observe phase", status, "requires a disabled timer and a valid explicit phase");
  }
  state_.prescaler_phase = phase;
}

void Hcs08Tpm::advance_bus(const std::uint64_t periods) {
  if (periods == 0 || (state_.control & bus_clock) == 0) { return; }
  if (!state_.prescaler_phase) { fail("clock", status, "unknown prescaler phase"); }
  const unsigned divisor = 1U << (state_.control & prescale_mask);
  const auto partial = (*state_.prescaler_phase % divisor) + (periods % divisor);
  const auto increments = (periods / divisor) + (partial / divisor);
  state_.prescaler_phase = static_cast<unsigned>(partial % divisor);
  if (increments >= word_values - state_.counter) {
    state_.overflow = true;
    state_.overflow_read = false; // A newer event invalidates the older read/clear sequence.
  }
  state_.counter =
      static_cast<std::uint16_t>((state_.counter + increments % word_values) % word_values);
}

Hcs08TpmState Hcs08Tpm::state() const {
  return state_;
}

bool Hcs08Tpm::interrupt_requested() const {
  return state_.overflow && (state_.control & irq_bit) != 0;
}

std::uint8_t Hcs08Tpm::read(const std::uint16_t address) {
  if (address == status) {
    if (state_.overflow) { state_.overflow_read = true; }
    return static_cast<std::uint8_t>(state_.control | (state_.overflow ? overflow_bit : 0));
  }
  if (address == modulus_high || address == modulus_low) { return 0; }
  if (address != count_high && address != count_low) {
    fail("read", address, "channel or register owner is unimplemented");
  }
  if (!state_.read_latch) { state_.read_latch = state_.counter; }
  const auto result =
      address == count_high ? *state_.read_latch >> 8U : *state_.read_latch & byte_mask;
  state_.read_bytes |= static_cast<std::uint8_t>(address == count_high ? 1 : 2);
  if (state_.read_bytes == 3) {
    state_.read_latch.reset();
    state_.read_bytes = 0;
  }
  return static_cast<std::uint8_t>(result);
}

void Hcs08Tpm::write_control(const std::uint8_t value) {
  if ((value & unsupported_modes) != 0) {
    fail("write", status, "PWM or synchronized XCLK/external clock owner is unimplemented");
  }
  const bool active = (state_.control & bus_clock) != 0;
  if (active && (value & bus_clock) != 0 &&
      (value & selection_mask) != (state_.control & selection_mask)) {
    fail("write", status, "changed live prescaler requires qualified divider history");
  }
  if (active && (value & bus_clock) == 0) { state_.prescaler_phase.reset(); }
  if ((value & overflow_bit) == 0 && state_.overflow_read) {
    state_.overflow = false;
    state_.overflow_read = false;
  }
  state_.control = static_cast<std::uint8_t>(value & control_mask);
  state_.read_latch.reset();
  state_.read_bytes = 0;
}

void Hcs08Tpm::write(const ByteWrite access) {
  if (access.address == status) {
    write_control(access.value);
    return;
  }
  if (access.address == count_high || access.address == count_low) {
    state_.counter = 0;
    state_.read_latch.reset();
    state_.read_bytes = 0;
    // The manual defines counter/coherency reset, not a new prescaler phase.
    state_.prescaler_phase.reset();
    return;
  }
  fail("write", access.address, "modulo/channel owner is unimplemented");
}

[[noreturn]] void Hcs08Tpm::fail(const std::string &operation,
                                 const unsigned address,
                                 const std::string &reason) const {
  throw Fault("HCS08 TPM " + operation + " address=" + hex(address) + " pc=" + hex(context_.pc) +
              " ccr=" + hex(context_.ccr) + " agent=CPU: " + reason);
}
} // namespace blofeld
