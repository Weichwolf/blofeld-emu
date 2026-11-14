// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <atomic>
#include <cstdint>
#include <optional>
#include <stop_token>

namespace blofeld::vi {
// Conservative grants in nominal 60 MHz units. Only actual shared transactions
// and interrupt admission wait; publishing completed MCU work never waits for DSP work.
// One MCU owner and one DSP owner; reset follows their existing epoch handoff.
class ExecutionTime {
public:
  [[nodiscard]] bool publish_mcu(std::uint64_t ticks) noexcept;
  // One MCU owner records its first actual WAI; observing this never advances a clock.
  void publish_mcu_wait(std::uint64_t ticks) noexcept;
  [[nodiscard]] std::optional<std::uint64_t> mcu_wait_time() const noexcept;
  [[nodiscard]] bool enter_mcu(std::uint64_t ticks, const std::stop_token &stop);
  [[nodiscard]] bool wait_for_dsp(std::uint64_t ticks, const std::stop_token &stop);
  // WAI lends time without completed progress. The source yields on a real wake.
  [[nodiscard]] std::optional<std::uint64_t>
  idle_mcu(std::uint64_t now, std::uint64_t until, const std::stop_token &stop);
  [[nodiscard]] bool wake_mcu(std::uint64_t ticks) noexcept;
  void leave_mcu() noexcept;
  [[nodiscard]] std::optional<std::uint64_t> dsp_limit(std::uint64_t ticks,
                                                       const std::stop_token &stop);
  void hold_dsp() noexcept;
  void release_dsp(std::uint64_t origin) noexcept;
  [[nodiscard]] std::uint64_t dsp_origin() const noexcept;

private:
  void notify() noexcept;
  void signal() noexcept;
  [[nodiscard]] std::optional<std::uint64_t> idle_limit() const noexcept;
  template <typename Ready>
  [[nodiscard]] bool await(std::atomic<bool> &waiting, const std::stop_token &stop, Ready ready);

  // The MCU frontier and transaction handshake are genuinely shared. Keep
  // the DSP owner's progress writes and read-mostly reset state off their line.
  alignas(128) std::atomic<std::uint64_t> mcu_{0};
  std::atomic<std::uint64_t> first_wait_{0};
  std::atomic<bool> waited_{false};
  std::atomic<std::uint64_t> transaction_{0}, changed_{0};
  std::atomic<bool> waiting_mcu_{false}, waiting_dsp_{false};
  std::atomic<std::uint64_t> idle_{0}, idle_until_{0}, wake_at_{0};
  alignas(128) std::atomic<std::uint64_t> dsp_{0};
  alignas(128) std::atomic<std::uint64_t> origin_{0};
  std::atomic<bool> held_{false};
};
}
