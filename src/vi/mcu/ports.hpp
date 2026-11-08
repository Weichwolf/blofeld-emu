// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/shi.hpp"
#include "mcu/boot_port.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace blofeld::vi::mcu {
enum class Port : std::uint8_t { a, b, e, t, m, j, p, q, r, s, u };

// MC9S12UF32 normal single-chip GPIO register contracts. Peripheral inputs are
// direct values; no electrical nets, voltage resolution or clock stepping.
class Ports {
public:
  struct Write {
    std::uint16_t offset = 0;
    std::uint8_t value = 0;
  };

  struct Input {
    std::uint8_t mask = 0, known = 0, value = 0;
  };

  explicit Ports(Shi &shi);
  [[nodiscard]] static bool contains(std::uint16_t offset) noexcept;
  [[nodiscard]] bool
  read(std::uint16_t offset, std::uint8_t &value, std::uint8_t mask = all_bits) const noexcept;
  [[nodiscard]] bool write(Write access) noexcept;
  void input(Port port, Input sample);

private:
  static constexpr std::uint8_t all_bits = 0xff;
  static constexpr std::uint8_t reset_pull = 0x90;
  static constexpr std::size_t port_count = 11;

  struct Registers {
    std::uint8_t latch = 0, known = 0, direction = 0, reduced = 0, pull = 0, polarity = 0;
    Input external;
  };

  struct Sample {
    unsigned known = 0, value = 0;
  };

  [[nodiscard]] Sample pins(std::size_t index) const noexcept;
  [[nodiscard]] bool write_control(Write access) noexcept;
  std::array<Registers, port_count> ports_{};
  BootPort boot_;
  std::uint8_t assignment_ = 0x10, mode_ = 0x80, pull_ = reset_pull, reduced_ = 0, routing_ = 0;
  bool assignment_written_ = false, mode_written_ = false;
};
}
