// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace blofeld {
// Host-transport resource limits, not USB endpoint or device capacities.
inline constexpr std::size_t maximum_usb_transfer = std::size_t{1024} * 1024;
inline constexpr std::size_t maximum_usb_pending = 32;

struct UsbTransferRequest {
  std::uint32_t sequence = 0;
  std::uint8_t address =
      0; // Endpoint address includes its IN bit; both EP0 directions share a queue.
  std::uint32_t length = 0;
  std::array<std::uint8_t, 8> setup{};
  std::vector<std::uint8_t> output;
  bool short_not_ok = false;
  bool zero_packet = false;
};

enum class UsbTransferStatus : std::uint8_t { success, stalled, short_packet };

struct UsbTransferCompletion {
  std::uint32_t sequence = 0;
  UsbTransferStatus status = UsbTransferStatus::success;
  std::uint32_t transferred = 0;
  std::vector<std::uint8_t> input;
  bool input_direction = false;
  bool operator==(const UsbTransferCompletion &) const = default;
};

void validate_usb_transfer(const UsbTransferRequest &request);
}
