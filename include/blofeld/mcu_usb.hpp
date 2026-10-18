// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace blofeld {

// USB20D6E2F configuration window, before MEN. Endpoint traffic and controller
// enable remain unsupported; this class must not fabricate completion flags.
class McuUsb {
public:
  void reset();
  [[nodiscard]] static std::string_view register_name(std::uint16_t offset);
  [[nodiscard]] static bool configuration_data(std::uint16_t offset);
  [[nodiscard]] std::uint8_t read(std::uint16_t offset) const;
  void write(ByteWrite access);
  void write_memory(MemoryWrite access);

private:
  [[nodiscard]] unsigned selected_register() const;
  std::array<std::uint32_t, 7> configuration_{};
  std::uint8_t selector_ = 0;
  std::uint8_t buffers_ = 0;
  std::optional<std::uint16_t> staged_high_;
};

}
