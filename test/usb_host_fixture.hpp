// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_usb.hpp"
#include "blofeld/usb_transfer.hpp"
#include "usb_endpoint_reset_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test::usb_host {
inline constexpr unsigned input_address = 0x81;
inline constexpr unsigned output_address = 2;
inline constexpr unsigned control_input = 0x80;
inline constexpr unsigned packet_bytes = 8;
inline constexpr unsigned endpoint_base = 0x314;
inline constexpr unsigned input_control = 0x318;
inline constexpr unsigned output_control = 0x31a;
inline constexpr unsigned status = 0x302;
inline constexpr unsigned mask = 0x304;
inline constexpr unsigned buffer_select = 0x30f;
inline constexpr unsigned local = 0x380;
inline constexpr unsigned ready = 0x7000;
inline constexpr unsigned acknowledged = 0x3000;
inline constexpr unsigned output_valid = 0x7000;
inline constexpr unsigned byte_mask = 0xff;
inline constexpr unsigned bulk_in_fields = 0xd1;
inline constexpr unsigned bulk_out_fields = 0xc2;
inline constexpr unsigned clear_transfer = 0x30;
inline constexpr ExecutionContext context{.pc = 0x8123, .ccr = 0x42};

inline void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

inline void reject(const auto &operation, const std::string_view text) {
  try {
    operation();
  } catch (const Fault &fault) {
    require(std::string_view(fault.what()).contains(text),
            "The negative control fails for its expected reason");
    return;
  }
  throw Fault("Unsupported USB host input reported success");
}

inline unsigned word(McuBus &bus, const unsigned address) {
  return static_cast<unsigned>(
      bus.read_memory({.address = static_cast<std::uint16_t>(address), .size = 2}));
}

inline void prepare(McuBus &bus, const unsigned packet = packet_bytes) {
  endpoint_reset::prepare(bus);
  bus.write(status + 1, 3); // Clear only the fixture's prior SETUP/SETOVR.
  bus.write(endpoint_reset::control, endpoint_reset::mce_high);
  const auto size = static_cast<std::uint32_t>(packet) << 19U;
  endpoint_reset::configure(bus, {.bank = 1, .value = size});
  endpoint_reset::configure(bus, {.bank = 2, .value = size | bulk_in_fields});
  endpoint_reset::configure(bus, {.bank = 3, .value = size | bulk_out_fields});
  bus.write(endpoint_reset::control, endpoint_reset::men_high);
  for (unsigned index = 0; index < 4; ++index) {
    bus.write(static_cast<std::uint16_t>(endpoint_base + (index * 2)), 0x10);
  }
}

inline void prime(McuBus &bus, const UsbBuffer buffer, const std::span<const std::uint8_t> bytes) {
  const auto index = static_cast<unsigned>(buffer);
  bus.write(buffer_select, static_cast<std::uint8_t>(index));
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    bus.write(static_cast<std::uint16_t>(local + i), bytes[i]);
  }
  bus.write_memory({.address = static_cast<std::uint16_t>(endpoint_base + (index * 2)),
                    .size = 2,
                    .value = ready | bytes.size()});
}

inline void release(McuBus &bus, const UsbBuffer buffer) {
  bus.write(static_cast<std::uint16_t>(endpoint_base + (static_cast<unsigned>(buffer) * 2)),
            clear_transfer);
}

inline std::vector<std::uint8_t>
received(McuBus &bus, const UsbBuffer buffer, const unsigned count) {
  bus.write(buffer_select, static_cast<std::uint8_t>(buffer));
  std::vector<std::uint8_t> result;
  result.reserve(count);
  for (unsigned i = 0; i < count; ++i) {
    result.push_back(bus.read(static_cast<std::uint16_t>(local + i)));
  }
  return result;
}

inline UsbTransferRequest
request(const std::uint32_t sequence, const std::uint8_t address, const std::uint32_t length) {
  return {.sequence = sequence,
          .address = address,
          .length = length,
          .setup = {},
          .output = {},
          .short_not_ok = false,
          .zero_packet = false};
}
}
