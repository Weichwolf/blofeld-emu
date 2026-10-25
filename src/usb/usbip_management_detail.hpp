// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace blofeld::usbip_management {
inline constexpr unsigned version = 0x0111;
inline constexpr unsigned request_bit = 0x8000;
inline constexpr unsigned list = 5;
inline constexpr unsigned import = 3;
inline constexpr unsigned header_bytes = 8;
inline constexpr unsigned busid_bytes = 32;
inline constexpr unsigned path_bytes = 256;
[[nodiscard]] unsigned
network(std::span<const std::uint8_t> bytes, std::size_t offset, unsigned width);
enum class Width : std::uint8_t { word = 2, longword = 4 };
void append(std::vector<std::uint8_t> &bytes, std::uint32_t value, Width width);
[[nodiscard]] std::vector<std::uint8_t> header(unsigned operation, unsigned status);
void validate_busid(std::string_view busid);
}
