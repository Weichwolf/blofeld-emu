// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/media_image.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::vi {
// Decoded page-read transactions. One DSP execution owner; image data is direct.
// Program/erase/ID/status commands remain explicit failures until bound.
class MemoryController {
public:
  struct Write {
    std::uint32_t controls = 0;
    std::uint8_t value = 0;
    bool known = false;
  };

  explicit MemoryController(const MediaImage *image = nullptr) : image_(image) { reset(); }

  void reset() noexcept;
  [[nodiscard]] bool write(Write access) noexcept;

  [[nodiscard]] bool pending() const noexcept { return pending_; }

  [[nodiscard]] bool complete() noexcept;
  [[nodiscard]] bool ready(bool &value) const noexcept;

  [[nodiscard]] std::optional<std::uint8_t> output() const noexcept { return output_; }

  [[nodiscard]] const char *error() const noexcept { return error_; }

private:
  enum class Phase : std::uint8_t { idle, address, read, end };
  [[nodiscard]] bool latch(Write access) noexcept;
  [[nodiscard]] bool fail(const char *message) noexcept;
  const MediaImage *image_;
  const char *error_ = nullptr;
  Phase phase_ = Phase::idle;
  std::uint32_t controls_ = 0, page_ = 0, column_ = 0;
  unsigned addresses_ = 0;
  bool pending_ = false, ready_ = true, reset_pending_ = false;
  std::optional<std::uint8_t> output_;
  std::span<const std::uint8_t> buffer_;
};
}
