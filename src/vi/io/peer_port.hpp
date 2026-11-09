// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>

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

  void mcu_data(Sample value) noexcept;
  void dsp_data(Sample value) noexcept;
  [[nodiscard]] Sample mcu_data() const noexcept;
  [[nodiscard]] Sample dsp_data() const noexcept;
  // Actual PDRC data writes, distinct from direction/mux/latch publication.
  [[nodiscard]] bool dsp_write(Sample value) noexcept;
  [[nodiscard]] std::optional<Sample> read_dsp_data(std::uint8_t mask) noexcept;

  [[nodiscard]] const char *error() const noexcept {
    return error_.load(std::memory_order_acquire);
  }

  // Active-low requests: J0 -> configured IRQA edge/level, E9 -> level-sensitive XIRQ.
  [[nodiscard]] bool mcu_strobe(bool active) noexcept;
  [[nodiscard]] bool dsp_ready(bool active) noexcept;
  [[nodiscard]] std::optional<std::uint64_t> dsp_interrupt(bool edge) const noexcept;
  [[nodiscard]] std::optional<std::uint64_t> mcu_interrupt() const noexcept;
  [[nodiscard]] bool acknowledge_dsp(std::uint64_t observed) noexcept;
  [[nodiscard]] bool acknowledge_mcu(std::uint64_t observed) noexcept;
  // DSP owner after retiring its old Core; request counters remain monotonic.
  void reset_dsp() noexcept;

private:
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

  std::atomic<std::uint32_t> mcu_{0}, dsp_{0};
  Request to_dsp_, to_mcu_;
  Reply reply_;
  std::atomic<const char *> error_{nullptr};
};
}
