// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "original_gpio_pair_fixture.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::test::gpio_parameter {
inline constexpr unsigned caller_entry_bytes = 35;

inline std::vector<std::uint8_t> caller_program(const gpio_pair::Case input,
                                                const std::span<const unsigned> values,
                                                const unsigned result_base) {
  std::vector<std::uint8_t> code;
  code.reserve((values.size() * caller_entry_bytes) + 2);
  for (unsigned n = 0; n < values.size(); ++n) {
    const auto result = result_base + n;
    // Own native caller: word parameter, location/kind, write(B=value), then
    // read(location/kind, D=parameter) and a native absolute result store.
    const std::array<std::uint8_t, caller_entry_bytes> entry{
        0xcc,
        static_cast<std::uint8_t>(input.parameter >> 8U),
        static_cast<std::uint8_t>(input.parameter & gpio_pair::mask),
        0x6c,
        0x80,
        0xc6,
        static_cast<std::uint8_t>(input.location),
        0x6b,
        0x82,
        0xc6,
        static_cast<std::uint8_t>(input.kind),
        0x6b,
        0x83,
        0xc6,
        static_cast<std::uint8_t>(values[n]),
        0x16,
        0xb7,
        0xa2,
        0xc6,
        static_cast<std::uint8_t>(input.location),
        0x6b,
        0x80,
        0x86,
        static_cast<std::uint8_t>(input.kind),
        0x6a,
        0x81,
        0xcc,
        static_cast<std::uint8_t>(input.parameter >> 8U),
        static_cast<std::uint8_t>(input.parameter & gpio_pair::mask),
        0x16,
        0xb6,
        0xdc,
        0x7b,
        static_cast<std::uint8_t>(result >> 8U),
        static_cast<std::uint8_t>(result & gpio_pair::mask)};
    code.insert(code.end(), entry.begin(), entry.end());
  }
  code.push_back(gpio_pair::branch_opcode);
  code.push_back(gpio_pair::branch_self);
  return code;
}
} // namespace blofeld::test::gpio_parameter
