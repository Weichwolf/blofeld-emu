// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stop_token>

namespace blofeld::vi {
// Functional low-byte Port Q/Port C exchange. Each producer owns its GPIO state;
// raw reads retain the current value. Fixed report blocks transfer actual DSP
// writes through bounded ownership; IRQ acknowledgements cannot erase a newer edge.
class PeerPort {
public:
  struct Sample {
    std::uint8_t mask = 0, known = 0, value = 0;
    bool operator==(const Sample &) const = default;
  };

  // MCU owner publishes ordered data/direction changes; full transport waits
  // for actual space without advancing device time. Equal states need no slot.
  [[nodiscard]] bool mcu_data(Sample value, std::uint64_t ticks, const std::stop_token &stop = {});
  void dsp_data(Sample value) noexcept;
  [[nodiscard]] Sample mcu_data() const noexcept;
  // DSP owner applies only changes at or before its actual instrument time.
  // Repeated reads retain the same sample; reading never consumes future data.
  [[nodiscard]] Sample sample_mcu_data(std::uint64_t ticks) noexcept;
  // MCU owner: coalesce inputs while reset is asserted, then publish the retained
  // value at release. DSP owner retires the old prefix before acknowledging reset.
  void hold_mcu_data() noexcept;
  [[nodiscard]] bool release_mcu_data(std::uint64_t ticks, const std::stop_token &stop);
  void retire_mcu_data() noexcept;
  [[nodiscard]] Sample dsp_data() const noexcept;
  // Actual PDRC data writes, distinct from direction/mux/latch publication.
  [[nodiscard]] bool dsp_write(Sample value) noexcept;
  [[nodiscard]] std::optional<Sample> read_dsp_data(std::uint8_t mask) noexcept;

  [[nodiscard]] const char *error() const noexcept {
    return error_.load(std::memory_order_acquire);
  }

  // J0 -> configured IRQA; E9 -> XIRQ. Announced report commands stay pending
  // until the actual MCU strobe or DSP withdrawal. An eligible ordinary notification
  // survives host batching only within the same MCU interrupt-admission window.
  // MCU owner publishes admission at native mask/return boundaries. Masked pulses
  // are never retained for a later unmask; this is transport, not a pin latch.
  [[nodiscard]] bool mcu_interrupt_ready(bool ready) noexcept;
  [[nodiscard]] bool mcu_strobe(bool active) noexcept;
  [[nodiscard]] bool dsp_ready(bool active) noexcept;
  [[nodiscard]] std::optional<std::uint64_t> dsp_interrupt(bool edge) const noexcept;
  [[nodiscard]] std::optional<std::uint64_t> mcu_interrupt() const noexcept;
  [[nodiscard]] bool acknowledge_dsp(std::uint64_t observed) noexcept;
  [[nodiscard]] bool acknowledge_mcu(std::uint64_t observed) noexcept;
  // DSP owner after retiring its old Core; request counters remain monotonic.
  void reset_dsp() noexcept;

private:
  struct Input {
    std::uint64_t ticks = 0;
    Sample data;
  };

  struct Inputs {
    static constexpr std::size_t capacity = 256;
    std::array<Input, capacity> entries{};
    alignas(128) std::atomic<std::uint64_t> published{0};
    std::uint64_t last_tick = 0; // MCU owner.
    bool held = false;
    std::atomic<std::uint64_t> reset_through{0};
    alignas(128) std::atomic<std::uint64_t> consumed{0};
    Sample sample; // DSP owner; retained through DSP reset.
    std::atomic<bool> waiting{false};
    std::atomic<std::uint64_t> changed{0};
  };

  [[nodiscard]] bool wait_input_space(std::uint64_t published, const std::stop_token &stop);
  [[nodiscard]] bool
  append_mcu_data(Sample value, std::uint64_t ticks, const std::stop_token &stop);
  void release_input_space(std::uint64_t consumed) noexcept;

  struct Reply {
    std::mutex mutex;
    std::array<Sample, 16> bytes{};
    unsigned written = 0, read = 0;
    std::atomic<bool> active{false}, recording{false}, announced{false};
  };

  [[nodiscard]] bool start_reply() noexcept;
  [[nodiscard]] bool fail(const char *message) noexcept;

  struct Request {
    std::atomic<std::uint64_t> posted{0}, accepted{0};
    std::atomic<bool> active{false};
    [[nodiscard]] bool update(bool value) noexcept;
    [[nodiscard]] std::optional<std::uint64_t> pending(bool edge) const noexcept;
    [[nodiscard]] bool acknowledge(std::uint64_t observed) noexcept;
  };

  Inputs inputs_;
  std::atomic<std::uint32_t> mcu_{0}, dsp_{0};
  Request to_dsp_, to_mcu_;
  std::atomic<std::uint64_t> mcu_admission_{0};
  Reply reply_;
  std::atomic<const char *> error_{nullptr};
};
}
