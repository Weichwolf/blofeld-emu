// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "usb/usb_transfer.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace blofeld::test::usbip_fixture {
using Bytes = std::vector<std::uint8_t>;
inline constexpr unsigned device = 0x00010002;
enum class Slot : std::uint8_t {
  command,
  sequence,
  device,
  direction,
  endpoint,
  flags,
  length,
  frame,
  packets,
  interval,
  setup0,
  setup1
};

inline void set(Bytes &bytes, const Slot slot, const std::uint32_t value) {
  const auto at = static_cast<std::size_t>(slot) * 4;
  constexpr unsigned byte_mask = 0xff;
  for (unsigned i = 0; i < 4; ++i) {
    bytes.at(at + i) = static_cast<std::uint8_t>((value >> ((3U - i) * 8U)) & byte_mask);
  }
}

inline Bytes words(const std::array<std::uint32_t, 12> &values) {
  Bytes bytes(values.size() * 4, 0);
  for (unsigned i = 0; i < values.size(); ++i) { set(bytes, static_cast<Slot>(i), values[i]); }
  return bytes;
}

inline Bytes submit(const UsbTransferRequest &request) {
  constexpr unsigned input_bit = 0x80;
  constexpr unsigned wire_input = 0x200;
  constexpr unsigned wire_zero = 0x40;
  const bool input = (request.address & input_bit) != 0;
  const unsigned flags = (input ? wire_input : 0) | (request.short_not_ok ? 1U : 0U) |
                         (request.zero_packet ? wire_zero : 0U);
  auto bytes = words({1,
                      request.sequence,
                      device,
                      input ? 1U : 0U,
                      request.address & 15U,
                      flags,
                      request.length,
                      0,
                      0,
                      0,
                      0,
                      0});
  constexpr unsigned setup_offset = 40;
  std::ranges::copy(request.setup, bytes.begin() + setup_offset);
  if (!input) { bytes.insert(bytes.end(), request.output.begin(), request.output.end()); }
  return bytes;
}

inline Bytes unlink(const unsigned sequence, const unsigned target) {
  return words({2, sequence, device, 0, 0, target, 0, 0, 0, 0, 0, 0});
}
}
