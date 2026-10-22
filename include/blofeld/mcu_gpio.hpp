// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/gpio_pads.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace blofeld {

enum class McuPort : std::uint8_t { a, b, e, t, m, j, p, q, r, s, u };

struct PinDrive {
  std::uint8_t mask = 0;
  std::uint8_t levels = 0;
  std::uint8_t floating = 0;
  std::uint8_t unknown = 0; // Strong peer drivers with an unobserved level.
};

struct PinFunction {
  std::uint8_t inputs = 0;
  std::uint8_t outputs = 0;
  std::uint8_t levels = 0;
  std::uint8_t unknown = 0; // Output pads whose peripheral latch is not known.

  bool operator==(const PinFunction &) const = default;
};

struct PinSample {
  std::uint8_t known = 0;
  std::uint8_t levels = 0;
  std::uint8_t conflicts = 0;
};

struct PinBiases {
  std::uint8_t up = 0;
  std::uint8_t down = 0;
};

struct PinOutputs {
  std::uint8_t mask = 0;
  std::uint8_t known = 0;
  std::uint8_t levels = 0;
  // Subset of mask with reduced output strength; never a weak pull or release.
  std::uint8_t reduced_drive = 0;

  bool operator==(const PinOutputs &) const = default;
};

// Logical GPIO and explicit external drives, with unknown inputs tracked.
// Electrical transition/synchronizer timing and alternate peripheral functions
// are not yet a complete board model.
class McuGpio {
public:
  McuGpio();
  void reset();
  void set_drive(McuPort port, PinDrive drive);
  void update_drive(McuPort port, std::uint8_t affected, PinDrive drive);
  void set_function(McuPort port, PinFunction function);
  void attach_pad_source(McuPort port, GpioPadReader reader);
  [[nodiscard]] std::uint8_t read_pad(McuPort port, std::uint8_t mask) const;
  [[nodiscard]] PinSample observe_pads(McuPort port, std::uint8_t mask) const;
  // Intended local drivers only; excludes peer levels and weak input pulls.
  [[nodiscard]] PinOutputs outputs(McuPort port) const;
  [[nodiscard]] PinBiases biases(McuPort port) const;
  [[nodiscard]] static std::string_view register_name(std::uint16_t offset);
  [[nodiscard]] std::uint8_t read(std::uint16_t offset) const;
  void write(ByteWrite write);

private:
  static constexpr std::size_t port_count = 11;

  struct PortState {
    std::uint8_t latch = 0;
    std::uint8_t latch_known = 0;
    std::uint8_t direction = 0;
    std::uint8_t reduced_drive = 0;
    std::uint8_t pull_enable = 0;
    std::uint8_t pull_down = 0;
    PinDrive external;
    PinFunction function{};
  };

  [[nodiscard]] std::uint8_t sample(McuPort port, std::uint8_t requested) const;
  [[nodiscard]] PinSample resolve(McuPort port, std::uint8_t requested) const;
  [[nodiscard]] unsigned pull_enable(McuPort port) const;
  [[nodiscard]] std::uint8_t data(McuPort port) const;
  [[nodiscard]] std::uint8_t tied_low(McuPort port) const;
  std::array<PortState, port_count> ports_{};
  std::array<GpioPadBinding, port_count> pads_{};
  std::uint8_t routing_ = 0;
  std::uint8_t assignment_e_ = 0;
  std::uint8_t pull_control_ = 0;
};

}
