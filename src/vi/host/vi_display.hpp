// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/lcd_export.hpp"

#include <chrono>
#include <filesystem>

namespace blofeld::vi {
class VirtualInstrument;

// Read-only host presentation. Wall-clock polling never advances board clocks.
class HostDisplayPump {
public:
  explicit HostDisplayPump(const std::filesystem::path &directory);
  void poll(const VirtualInstrument &board);
  void capture(const VirtualInstrument &board);

private:
  LcdPublisher publisher_;
  std::chrono::steady_clock::time_point next_;
};
}
