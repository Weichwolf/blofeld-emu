// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <atomic>
#include <cstdint>
#include <optional>

namespace blofeld::vi {
// Functional low-byte Port Q/Port C exchange. Each producer owns its GPIO state;
// reads retain the current value. IRQ acknowledgements cannot erase a newer edge.
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

  // Active-low functional requests: J0 -> IRQA, E9 -> XIRQ. Release is inactive.
  [[nodiscard]] bool mcu_strobe(bool active) noexcept;
  [[nodiscard]] bool dsp_ready(bool active) noexcept;
  [[nodiscard]] std::optional<std::uint64_t> dsp_interrupt(bool edge) const noexcept;
  [[nodiscard]] std::optional<std::uint64_t> mcu_interrupt() const noexcept;
  [[nodiscard]] bool acknowledge_dsp(std::uint64_t observed) noexcept;
  [[nodiscard]] bool acknowledge_mcu(std::uint64_t observed) noexcept;
  // DSP owner after retiring its old Core; request counters remain monotonic.
  void reset_dsp() noexcept;

private:
  struct Request {
    std::atomic<std::uint64_t> posted{0}, accepted{0};
    std::atomic<bool> active{false};
    [[nodiscard]] bool update(bool value) noexcept;
    [[nodiscard]] std::optional<std::uint64_t> pending(bool edge) const noexcept;
    [[nodiscard]] bool acknowledge(std::uint64_t observed) noexcept;
  };

  std::atomic<std::uint32_t> mcu_{0}, dsp_{0};
  Request to_dsp_, to_mcu_;
};
}
