// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

namespace blofeld::vi {
// DSP56371UM 7.4: SPI receive transactions and the DSP-visible FIFO.
// One DSP owner calls read/write; one MCU owner calls select/receive.
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

  [[nodiscard]] bool read(std::uint32_t address, std::uint32_t &value) noexcept;
  [[nodiscard]] bool write(Write access) noexcept;
  [[nodiscard]] std::optional<Mode> select(bool active) noexcept;
  [[nodiscard]] bool receive(std::uint32_t word) noexcept;

private:
  static constexpr std::uint32_t reset_status = 0x8200;
  void publish() noexcept;
  std::mutex mutex_;
  std::array<std::uint32_t, 10> fifo_{};
  std::size_t head_ = 0, count_ = 0;
  std::uint32_t control_ = 0, clock_ = 0, errors_ = 0;
  bool observed_overrun_ = false, selected_ = false;
  // Reset: HIDLE and HTDE set. Polls require one acquire load, no mutex.
  std::atomic<std::uint32_t> status_{reset_status};
};
}
