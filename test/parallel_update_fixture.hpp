// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <span>
#include <vector>

namespace blofeld {
class Hc12;
class McuBus;

namespace test {
struct Transfer {
  std::span<const std::uint8_t> payload;
  std::optional<std::size_t> loss_at = std::nullopt;
  bool dispatch = false;
};

void boot(McuBus &bus, Hc12 &cpu, std::ostream &trace);
void transfer(McuBus &bus, Hc12 &cpu, const Transfer &request, std::ostream &trace);
void check_first_callback(McuBus &bus, Hc12 &cpu, std::ostream &trace);
std::span<const std::uint8_t> original_helper(const std::vector<std::uint8_t> &block,
                                              unsigned size);
}
}
