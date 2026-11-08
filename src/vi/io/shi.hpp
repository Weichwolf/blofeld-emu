// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stop_token>

namespace blofeld::vi {
// DSP56371UM 7.4: completed SPI word transactions and DSP-visible registers.
// One DSP owner calls read/write; one external owner calls select/transfer.
class Shi {
public:
  struct Write {
    std::uint32_t address = 0, value = 0;
  };

  struct Mode {
    unsigned bits = 0;
    bool falling = false;
    bool continuous = false;
  };

  struct Transfer {
    bool accepted = false;
    // Unpacked bus word, or unknown after an underrun; never a fabricated zero.
    std::optional<std::uint32_t> output;
  };

  [[nodiscard]] bool read(std::uint32_t address, std::uint32_t &value) noexcept;
  [[nodiscard]] bool write(Write access) noexcept;
  [[nodiscard]] std::optional<Mode> select(bool active) noexcept;
  // MCU owner, outside native callbacks: wait for the DSP-written receive mode.
  [[nodiscard]] std::optional<Mode> wait_mode(const std::stop_token &stop);
  [[nodiscard]] Transfer transfer(std::uint32_t word) noexcept;
  // DSP owner after its previous Core has stopped; preserves the external selection.
  void reset() noexcept;

private:
  static constexpr std::uint32_t reset_status = 0x8200;
  void publish() noexcept;
  void prime_transmit() noexcept;
  [[nodiscard]] std::optional<Mode> mode() const noexcept;
  std::mutex mutex_;
  std::condition_variable_any ready_;
  std::array<std::uint32_t, 10> fifo_{};
  std::size_t head_ = 0, count_ = 0;
  std::uint32_t control_ = 0, clock_ = 1, errors_ = 0;
  std::optional<std::uint32_t> transmit_, shifting_;
  bool observed_overrun_ = false, observed_underrun_ = false, selected_ = false;
  bool phase_zero_complete_ = false;
  // Reset: HIDLE and HTDE set. Polls require one acquire load, no mutex.
  std::atomic<std::uint32_t> status_{reset_status};
};
}
