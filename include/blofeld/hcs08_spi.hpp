// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
namespace hcs08_spi {
constexpr unsigned control1 = 0x28, control2 = 0x29, baud = 0x2a, status = 0x2b, data = 0x2d;
constexpr unsigned receive_full = 0x80, transmit_empty = 0x20;
constexpr unsigned receive_irq = 0x80, enable = 0x40, transmit_irq = 0x20, master = 0x10;
constexpr unsigned polarity = 8, phase = 4, lsb_first = 1;
constexpr unsigned reset_control = 4;
}

enum class Hcs08SpiContract : std::uint8_t { unqualified, candidate_gt8a_reference_master_edges };

struct Hcs08SpiState {
  std::uint8_t control1 = hcs08_spi::reset_control, control2 = 0, baud = 0;
  std::optional<std::uint8_t> transmit;
  std::uint8_t receive = 0, shift_out = 0, shift_in = 0;
  unsigned edges = 0, samples = 0;
  bool active = false, receive_full = false, transmit_armed = false, receive_armed = false;
  std::optional<bool> mosi;
  bool operator==(const Hcs08SpiState &) const = default;
};

struct Hcs08SpiPins {
  bool driven = false;
  bool clock = false;
  std::optional<bool> mosi;
  // Original MODFEN=0 leaves SS with the separate GPIO owner.
  bool operator==(const Hcs08SpiPins &) const = default;
};

// S08SPIV3 master data path; bit events are separate from physical bus timing.
// No callbacks/peers are retained. Copies are trusted independent checkpoints.
class Hcs08Spi {
public:
  explicit Hcs08Spi(Hcs08SpiContract contract = Hcs08SpiContract::unqualified);
  void context(ExecutionContext context);
  void reset();
  [[nodiscard]] static bool contains(unsigned address);
  [[nodiscard]] std::uint8_t read(std::uint16_t address);
  void write(ByteWrite access);
  // The board must qualify the transfer latency (documented as within2 BUSCLK
  // cycles for idle SPI); this explicit event does not choose a physical phase.
  void transfer_to_shifter();
  // Exactly one actual master SPSCK transition. Unknown MISO fails at sampling
  // edges; shift edges need no input observation. This does not advance BUSCLK.
  void edge(std::optional<bool> miso);
  // A qualified owner supplies transfer completion after16 edges/8 samples.
  // No status read, elapsed-time guess or host byte injection completes a byte.
  void complete_transfer();
  [[nodiscard]] Hcs08SpiPins pins() const;
  [[nodiscard]] bool interrupt_requested() const;
  // Register-derived divisor only, not a calibrated active-clock phase.
  [[nodiscard]] unsigned clock_divisor() const;
  [[nodiscard]] Hcs08SpiState state() const;
  [[nodiscard]] bool faulted() const;

private:
  void healthy() const;
  void control(std::uint8_t value);
  [[nodiscard]] bool phase() const;
  [[nodiscard]] bool lsb_first() const;
  void drive_bit();
  [[noreturn]] void
  fail(const std::string &operation, unsigned address, const std::string &reason) const;
  Hcs08SpiState state_;
  ExecutionContext context_;
  mutable bool healthy_ = true;
};
} // namespace blofeld
