// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace blofeld::host {
// Extract exactly one suffix-matching member (ASCII case-insensitive) into
// memory. No filesystem extraction, ZIP64, encryption or multipart archives.
[[nodiscard]] std::vector<std::uint8_t>
zip_member(std::span<const std::uint8_t> input, std::string_view suffix, std::size_t maximum_bytes);
}
