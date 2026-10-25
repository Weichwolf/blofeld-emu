// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <cstdint>
#include <optional>
#include <string_view>

namespace blofeld {

struct FlashOptions {
  std::uint8_t protection = 0;
  std::uint8_t security = 0;
};

// FTS32K normal-mode configuration only. No command completion, programming,
// erase, BDM or physical power-loss behavior is implied. McuFlash owns key access.
class McuFlashConfig {
public:
  explicit McuFlashConfig(FlashOptions options);
  void reset(FlashOptions options);
  [[nodiscard]] static std::string_view register_name(std::uint16_t offset);
  [[nodiscard]] std::uint8_t read(std::uint16_t offset) const;
  void write(ByteWrite access);
  [[nodiscard]] bool secured() const;
  [[nodiscard]] bool backdoor_enabled() const;
  [[nodiscard]] bool protected_byte(std::uint16_t physical_offset) const;
  [[nodiscard]] bool whole_array_unprotected() const;
  [[nodiscard]] std::optional<unsigned> oscillator_divisor() const;

private:
  friend class McuFlash;
  void unsecure(); // Only the owning, successfully matched key sequence may call this.
  void write_protection(std::uint8_t value);
  std::uint8_t divider_ = 0;
  FlashOptions options_;
};

}
