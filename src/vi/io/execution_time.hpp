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
  enum class DspBoundary : std::uint8_t { exact, fractional, quiescent };
  enum class McuWork : std::uint8_t { waiting, running };

  struct DspGrant {
    enum class Kind : std::uint8_t { frontier, current };
    std::uint64_t until;
    Kind kind = Kind::frontier;
    bool operator==(const DspGrant &) const = default;
  };

  struct DspForecast {
    // Stopped-owner query: exclusive instrument-time bound under the current MCU no-output window.
    void *owner = nullptr;
    std::uint64_t (*query)(void *) = nullptr;
  };

  [[nodiscard]] bool publish_mcu(std::uint64_t ticks) noexcept;
  // One MCU owner records its first actual WAI; observing this never advances a clock.
  void publish_mcu_wait(std::uint64_t ticks) noexcept;
  [[nodiscard]] std::optional<std::uint64_t> mcu_wait_time() const noexcept;
  [[nodiscard]] bool enter_mcu(std::uint64_t ticks, const std::stop_token &stop);
  [[nodiscard]] bool wait_for_dsp(std::uint64_t ticks, const std::stop_token &stop);
  // Settle the actual receiver instant, retaining producer fractional phase.
  [[nodiscard]] bool settle_dsp(std::uint64_t ticks, const std::stop_token &stop);
  // Conditional no-output permission, ended by source progress, input or a joint no-event proof.
  [[nodiscard]] std::optional<std::uint64_t> idle_mcu(std::uint64_t now,
                                                      std::uint64_t until,
                                                      const std::stop_token &stop,
                                                      McuWork work = McuWork::waiting);
  [[nodiscard]] bool wake_mcu(std::uint64_t ticks) noexcept;
  void leave_mcu() noexcept;
  [[nodiscard]] std::optional<DspGrant> dsp_limit(std::uint64_t ticks,
                                                  const std::stop_token &stop,
                                                  DspBoundary boundary = DspBoundary::exact,
                                                  DspForecast forecast = {.owner = nullptr,
                                                                          .query = nullptr});
  void hold_dsp() noexcept;
  void release_dsp(std::uint64_t origin) noexcept;
  [[nodiscard]] std::uint64_t dsp_origin() const noexcept;

private:
  struct Cancel {
    ExecutionTime *owner;
    void operator()() const noexcept;
  };

  struct alignas(128) Cancellation {
    std::stop_token token;
    std::optional<std::stop_callback<Cancel>> callback;
    void bind(const std::stop_token &stop, ExecutionTime &owner);
  };

  struct IdleWindow {
    std::uint64_t until, generation;
  };

  void publish_dsp(std::uint64_t ticks, DspBoundary boundary);
  [[nodiscard]] bool publish_dsp_knowledge(std::uint64_t until) noexcept;
  [[nodiscard]] std::optional<DspGrant> execution_grant(std::uint64_t ticks,
                                                        DspBoundary boundary,
                                                        DspForecast forecast,
                                                        const std::stop_token &stop);
  [[nodiscard]] std::optional<DspGrant> forecast_grant(IdleWindow window,
                                                       std::uint64_t ticks,
                                                       DspForecast forecast,
                                                       const std::stop_token &stop);
  [[nodiscard]] bool dsp_ready(std::uint64_t ticks, DspBoundary boundary) const noexcept;
  void notify() noexcept;
  void signal() noexcept;
  [[nodiscard]] std::optional<IdleWindow> idle_window() const noexcept;
  template <typename Ready>
  [[nodiscard]] bool await(std::atomic<bool> &waiting, const std::stop_token &stop, Ready ready);

  // The MCU frontier and transaction handshake are genuinely shared. Keep
  // the DSP owner's progress writes and read-mostly reset state off their line.
  alignas(128) std::atomic<std::uint64_t> mcu_{0};
  std::atomic<std::uint64_t> first_wait_{0};
  std::atomic<bool> waited_{false};
  std::atomic<std::uint64_t> transaction_{0}, changed_{0};
  std::atomic<bool> waiting_mcu_{false}, waiting_dsp_{false};
  std::atomic<std::uint64_t> idle_{0}, idle_until_{0}, wake_at_{0}, settle_until_{0};
  alignas(128) std::atomic<std::uint64_t> dsp_{0}, dsp_known_{0};
  alignas(128) std::atomic<std::uint64_t> origin_{0};
  std::atomic<bool> held_{false};
  // Destroy registrations before the atomics their callbacks access.
  Cancellation mcu_cancellation_, dsp_cancellation_;
};
}
