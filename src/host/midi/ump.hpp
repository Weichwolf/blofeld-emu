// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::host::midi {
struct Packet {
  std::array<std::uint8_t, 8> bytes{};
  std::uint64_t timestamp = 0;
  std::uint8_t size = 0;

  [[nodiscard]] std::span<const std::uint8_t> data() const noexcept {
    return std::span(bytes).first(size);
  }
};

class Ump {
public:
  explicit Ump(std::uint8_t group);
  [[nodiscard]] std::optional<Packet> decode(std::span<const std::uint32_t> words,
                                             std::uint64_t timestamp) noexcept;

  [[nodiscard]] const char *error() const noexcept { return error_; }

private:
  [[nodiscard]] bool voice(Packet &packet, std::uint8_t status) noexcept;
  [[nodiscard]] bool system(Packet &packet, std::uint8_t status) noexcept;
  [[nodiscard]] bool sysex(Packet &packet, std::span<const std::uint32_t> words) noexcept;
  [[nodiscard]] bool fail(const char *reason) noexcept;
  std::uint8_t group_;
  bool exclusive_ = false;
  const char *error_ = nullptr;
};
}
