// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <atomic>
#include <cstdint>
#include <optional>
#include <stop_token>

namespace blofeld::vi {
// Conservative grants in nominal 60 MHz units. Only actual shared transactions
// wait for the other owner; publishing completed MCU work never waits for DSP work.
// One MCU owner and one DSP owner; reset follows their existing epoch handoff.
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
  void notify() noexcept;
  void signal() noexcept;
  template <typename Ready>
  [[nodiscard]] bool await(std::atomic<bool> &waiting, const std::stop_token &stop, Ready ready);

  std::atomic<std::uint64_t> mcu_{0};
  std::atomic<std::uint64_t> dsp_{0}, origin_{0}, transaction_{0}, changed_{0};
  std::atomic<bool> waiting_mcu_{false}, waiting_dsp_{false}, held_{false};
};
}
