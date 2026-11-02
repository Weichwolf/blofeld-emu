// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/lcd_export.hpp"

#include <chrono>
#include <filesystem>

namespace blofeld::reference {
class Board;

// Read-only host presentation. Wall-clock polling never advances board clocks.
class HostDisplayPump {
public:
  explicit HostDisplayPump(const std::filesystem::path &directory);
  void poll(const Board &board);
  void capture(const Board &board);

private:
  LcdPublisher publisher_;
  std::chrono::steady_clock::time_point next_;
};
}
