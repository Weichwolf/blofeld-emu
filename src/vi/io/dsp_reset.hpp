// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stop_token>

namespace blofeld::vi {
// One MCU publisher, one DSP execution owner. Epochs retain reset assertions
// across a quick assert/release; acknowledgement transfers peripheral ownership.
class DspReset {
public:
  struct State {
    std::uint64_t epoch = 0;
    bool held = false;
  };

  [[nodiscard]] State state() const noexcept;
  [[nodiscard]] bool pending(std::uint64_t epoch) const noexcept;
  [[nodiscard]] bool assert_reset() noexcept;
  // MCU owner, outside a native callback: release only after the old DSP instance
  // has retired and reset its peripherals. Cancellation never fabricates release.
  [[nodiscard]] bool release(const std::stop_token &stop);
  // DSP owner: acknowledge only after disposing of the previous Core.
  [[nodiscard]] bool acknowledge(std::uint64_t epoch) noexcept;
  [[nodiscard]] std::optional<State> wait_release(const std::stop_token &stop);

private:
  std::atomic<std::uint64_t> state_{0};
  std::mutex mutex_;
  std::condition_variable_any changed_;
  std::uint64_t acknowledged_ = 0;
};
}
