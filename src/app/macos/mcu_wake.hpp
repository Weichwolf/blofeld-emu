// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

namespace blofeld::vi {
struct CheckpointWait;
}

namespace blofeld::app {
class McuWake {
public:
  void start(std::chrono::nanoseconds device_period);
  [[nodiscard]] bool schedule() noexcept;
  void check() const;
  [[nodiscard]] bool wait_checkpoint(vi::CheckpointWait wait) noexcept;

#if defined(__aarch64__)
private:
  std::atomic<std::uint32_t> period_{0};
  std::atomic<std::uint64_t> budget_{0};
  bool scheduled_ = false;
  int schedule_status_ = 0;
  std::uint64_t checkpoint_ = 0;
  std::uint64_t checkpoint_deadline_ = 0;
#endif
};
}
