// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/media_image.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::vi {
// Decoded page read/program and block erase. One DSP execution owner; image data is direct.
// Identification remains an explicit failure until bound.
class MemoryController {
public:
  struct Write {
    std::uint32_t controls = 0;
    std::uint8_t value = 0;
    bool known = false;
    bool driven = true;
  };

  explicit MemoryController(MediaImage *image = nullptr) : image_(image) { reset(); }

  ~MemoryController() { reset(); }

  MemoryController(const MemoryController &) = delete;
  MemoryController &operator=(const MemoryController &) = delete;
  void reset() noexcept;
  [[nodiscard]] bool write(Write access) noexcept;

  [[nodiscard]] bool pending() const noexcept { return operation_ != Operation::none; }

  [[nodiscard]] std::uint64_t epoch() const noexcept { return epoch_; }

  // Nominal VM operation durations in DSP reference-clock periods, independent
  // of host work and PLL multiplication. These are not fitted chip timings.
  [[nodiscard]] unsigned reference_ticks() const noexcept;

  [[nodiscard]] bool complete();
  [[nodiscard]] bool ready(bool &value) const noexcept;

  [[nodiscard]] std::optional<std::uint8_t> output() const noexcept;

  [[nodiscard]] const char *error() const noexcept { return error_; }

private:
  enum class Phase : std::uint8_t {
    idle,
    address,
    read,
    program_address,
    program_data,
    erase_address,
    erase_confirm,
    end
  };
  enum class Operation : std::uint8_t { none, read, program, erase, reset };
  [[nodiscard]] bool latch(Write access) noexcept;
  [[nodiscard]] bool read_output() noexcept;
  [[nodiscard]] bool command(std::uint8_t value) noexcept;
  [[nodiscard]] bool confirm(Operation operation) noexcept;
  void interrupt_write() noexcept;
  [[nodiscard]] bool address(std::uint8_t value) noexcept;
  [[nodiscard]] std::uint8_t status() const noexcept;
  [[nodiscard]] bool fail(const char *message) noexcept;
  MediaImage *image_;
  const char *error_ = nullptr;
  Phase phase_ = Phase::idle;
  Operation operation_ = Operation::none;
  std::uint32_t controls_ = 0, page_ = 0, column_ = 0, pointer_ = 0;
  unsigned addresses_ = 0, program_size_ = 0;
  std::array<std::uint8_t, MediaImage::page_bytes> program_{};
  std::uint64_t epoch_ = 0;
  bool status_mode_ = false, failed_ = false;
  std::optional<std::uint8_t> output_;
  std::span<const std::uint8_t> buffer_;
};
}
