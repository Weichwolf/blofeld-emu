// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/usb_transfer.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>
#include <vector>

namespace blofeld {
struct UsbUnlinkRequest {
  std::uint32_t sequence = 0;
  std::uint32_t target = 0;
};

using UsbipCommand = std::variant<UsbTransferRequest, UsbUnlinkRequest>;
inline constexpr std::size_t usbip_transfer_header = 48;

// Transfer phase after a separately established USB/IP import. No descriptors,
// device-list success or TCP listener are fabricated by this framing layer.
class UsbipDecoder {
public:
  explicit UsbipDecoder(std::uint32_t device);
  void feed(std::span<const std::uint8_t> bytes);
  [[nodiscard]] std::size_t input_space() const;
  [[nodiscard]] std::optional<UsbipCommand> next();
  void finish();

  [[nodiscard]] bool faulted() const { return faulted_; }

private:
  void require_active() const;
  std::uint32_t device_;
  std::vector<std::uint8_t> bytes_;
  std::size_t consumed_ = 0;
  bool faulted_ = false;
  bool closed_ = false;
};

[[nodiscard]] std::vector<std::uint8_t>
encode_usbip_completion(const UsbTransferCompletion &completion);
[[nodiscard]] std::vector<std::uint8_t> encode_usbip_unlink(std::uint32_t sequence, bool cancelled);
}
