// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_spi.hpp"

#include "common/bus.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
namespace spi = hcs08_spi;

namespace {
constexpr unsigned last_bit = 7, final_edge = 16, bit_count = 8;
constexpr unsigned prescale_shift = 4, divisor_mask = 7;
}

Hcs08Spi::Hcs08Spi(const Hcs08SpiContract contract) {
  if (contract != Hcs08SpiContract::candidate_gt8a_reference_master_edges) {
    fail("configure", spi::control1, "requires explicit candidate untimed master-edge contract");
  }
}

void Hcs08Spi::context(const ExecutionContext context) {
  context_ = context;
}

void Hcs08Spi::reset() {
  state_ = {};
  healthy_ = true;
}

bool Hcs08Spi::contains(const unsigned address) {
  return address >= spi::control1 && address <= spi::data;
}

void Hcs08Spi::healthy() const {
  if (!healthy_) {
    fail("access", spi::status, "incomplete operation requires coordinated recovery");
  }
}

[[noreturn]] void Hcs08Spi::fail(const std::string &operation,
                                 const unsigned address,
                                 const std::string &reason) const {
  healthy_ = false;
  throw Fault("HCS08 SPI " + operation + " address=" + hex(address) + " pc=" + hex(context_.pc) +
              " ccr=" + hex(context_.ccr) + " edges=" + std::to_string(state_.edges) +
              " samples=" + std::to_string(state_.samples) + ": " + reason);
}

bool Hcs08Spi::phase() const {
  return (state_.control1 & spi::phase) != 0;
}

bool Hcs08Spi::lsb_first() const {
  return (state_.control1 & spi::lsb_first) != 0;
}

void Hcs08Spi::drive_bit() {
  const auto bit = lsb_first() ? state_.samples : last_bit - state_.samples;
  state_.mosi = ((static_cast<unsigned>(state_.shift_out) >> bit) & 1U) != 0;
}

void Hcs08Spi::transfer_to_shifter() {
  healthy();
  if ((state_.control1 & (spi::enable | spi::master)) != (spi::enable | spi::master) ||
      state_.active || !state_.transmit) {
    fail("transfer", spi::data, "requires enabled idle master and genuine queued transmit byte");
  }
  state_.shift_out = *state_.transmit;
  state_.transmit.reset();
  state_.shift_in = 0;
  state_.edges = 0;
  state_.samples = 0;
  state_.active = true;
  state_.mosi.reset();
  if (!phase()) { drive_bit(); }
}

void Hcs08Spi::edge(const std::optional<bool> miso) {
  healthy();
  if (!state_.active || state_.edges == final_edge) {
    fail("SPSCK edge", spi::data, "requires active shifter and incomplete16-edge transfer");
  }
  const bool leading = (state_.edges & 1U) == 0;
  const bool sample = phase() ? !leading : leading;
  if (sample && !miso) { fail("MISO sample", spi::data, "unknown resolved input at sample edge"); }
  if (sample) {
    const auto bit = lsb_first() ? state_.samples : last_bit - state_.samples;
    state_.shift_in |= static_cast<std::uint8_t>(static_cast<unsigned>(*miso) << bit);
    ++state_.samples;
  } else if (state_.samples < bit_count) {
    drive_bit();
  }
  ++state_.edges;
}

void Hcs08Spi::complete_transfer() {
  healthy();
  if (!state_.active || state_.edges != final_edge || state_.samples != bit_count) {
    fail("completion", spi::status, "requires all16 actual clock edges and eight known samples");
  }
  if (!state_.receive_full) {
    state_.receive = state_.shift_in;
    state_.receive_full = true;
  } // Documented overrun discards new data without a separate error flag.
  state_.active = false;
  state_.mosi.reset(); // Idle data-pad level is not established by the transfer contract.
}

Hcs08SpiPins Hcs08Spi::pins() const {
  healthy();
  const bool idle = (state_.control1 & spi::polarity) != 0;
  return {.driven = (state_.control1 & spi::enable) != 0,
          .clock = idle != (state_.active && (state_.edges & 1U) != 0),
          .mosi = state_.mosi};
}

bool Hcs08Spi::interrupt_requested() const {
  healthy();
  return ((state_.control1 & spi::receive_irq) != 0 && state_.receive_full) ||
         ((state_.control1 & spi::transmit_irq) != 0 && !state_.transmit);
}

unsigned Hcs08Spi::clock_divisor() const {
  healthy();
  const unsigned prescaler = ((state_.baud >> prescale_shift) & divisor_mask) + 1U;
  const unsigned divider = 2U << (state_.baud & divisor_mask);
  return prescaler * divider;
}

Hcs08SpiState Hcs08Spi::state() const {
  return state_;
}

bool Hcs08Spi::faulted() const {
  return !healthy_;
}
} // namespace blofeld
