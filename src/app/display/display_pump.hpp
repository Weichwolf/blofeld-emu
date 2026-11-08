// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "app/display/lcd_export.hpp"

#include <chrono>
#include <filesystem>

namespace blofeld::vi {
class VirtualInstrument;
}

namespace blofeld::app {

// Read-only host presentation. Wall-clock polling never advances device clocks.
class DisplayPump {
public:
  explicit DisplayPump(const std::filesystem::path &directory);
  void poll(const vi::VirtualInstrument &instrument);
  void capture(const vi::VirtualInstrument &instrument);

private:
  LcdPublisher publisher_;
  std::chrono::steady_clock::time_point next_;
};
}
