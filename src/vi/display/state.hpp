// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/st7565.hpp"

#include <atomic>
#include <mutex>
#include <optional>

namespace blofeld::vi {
// One MCU owner captures complete controller state outside native execution.
// The app requests/takes snapshots; rendering and file I/O never hold this lock.
class DisplayState {
public:
  void request() noexcept { requested_.store(true, std::memory_order_release); }

  void capture(const St7565 &controller);
  [[nodiscard]] std::optional<St7565> take();

private:
  std::atomic<bool> requested_{true};
  std::mutex mutex_;
  std::optional<St7565> pending_;
};
}
