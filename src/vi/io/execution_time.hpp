// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stop_token>

namespace blofeld::vi {
// Conservative grants in nominal 60 MHz units. Only actual shared transactions
// wait for the other owner; publishing completed MCU work never waits for DSP work.
class ExecutionTime {
public:
  [[nodiscard]] bool publish_mcu(std::uint64_t ticks) noexcept;
  [[nodiscard]] bool enter_mcu(std::uint64_t ticks, const std::stop_token &stop);
  void leave_mcu() noexcept;
  [[nodiscard]] std::optional<std::uint64_t> dsp_limit(std::uint64_t ticks,
                                                       const std::stop_token &stop);
  void hold_dsp() noexcept;
  void release_dsp(std::uint64_t origin) noexcept;
  [[nodiscard]] std::uint64_t dsp_origin() const noexcept;

private:
  mutable std::mutex mutex_;
  std::condition_variable_any changed_;
  std::uint64_t mcu_ = 0, dsp_ = 0, origin_ = 0;
  bool pending_ = false, granted_ = false, held_ = false;
};
}
