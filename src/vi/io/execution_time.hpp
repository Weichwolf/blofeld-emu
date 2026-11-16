// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <atomic>
#include <cstdint>
#include <optional>

namespace blofeld::vi {
class ExecutionTime {
public:
  void publish_mcu_wait(std::uint64_t ticks) noexcept;
  [[nodiscard]] std::optional<std::uint64_t> mcu_wait_time() const noexcept;
  void release_dsp(std::uint64_t origin) noexcept;
  [[nodiscard]] std::uint64_t dsp_origin() const noexcept;

private:
  std::atomic<std::uint64_t> first_wait_{0}, origin_{0};
  std::atomic<bool> waited_{false};
};
}
