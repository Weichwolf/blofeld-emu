// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>

namespace blofeld {
class McuBus;

// Full-speed root-port attachment at decoded reset/SOF boundaries. Device state,
// endpoint configuration and transfer completion remain owned by the MCU.
class UsbHostLink {
public:
  explicit UsbHostLink(std::uint64_t frame_ticks);
  [[nodiscard]] bool poll(McuBus &bus, std::uint64_t ticks);

private:
  enum class Stage : std::uint8_t { detached, debounce, reset, recovery, connected };
  std::uint64_t frame_ticks_;
  std::uint64_t since_ = 0;
  std::uint64_t frame_ = 0;
  std::optional<std::uint64_t> previous_;
  Stage stage_ = Stage::detached;
};
}
