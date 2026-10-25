// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "usb/usb_device.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::usb_descriptor {
inline constexpr unsigned device_type = 1;
inline constexpr unsigned configuration_type = 2;
inline constexpr unsigned device_bytes = 18;
inline constexpr unsigned configuration_header_bytes = 9;
inline constexpr unsigned maximum_interfaces = 32;
inline constexpr unsigned maximum_descriptors = 1024;
[[nodiscard]] std::uint16_t little_word(std::span<const std::uint8_t> bytes, std::size_t offset);
void validate_device(std::span<const std::uint8_t> bytes);
[[nodiscard]] unsigned configuration_length(std::span<const std::uint8_t> bytes);
[[nodiscard]] std::vector<UsbInterfaceDescription>
parse_configuration(std::span<const std::uint8_t> bytes);
}
