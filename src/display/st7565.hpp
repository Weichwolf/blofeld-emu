// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld {

struct LcdByte {
  std::uint8_t value = 0;
  std::uint8_t known = 0;
};

struct St7565State {
  std::uint8_t page = 0;
  std::uint8_t column = 0;
  std::uint8_t start_line = 0;
  std::uint8_t volume = 32;
  std::uint8_t power = 0;
  std::uint8_t static_mode = 0;
  std::optional<std::uint8_t> resistor;
  std::optional<std::uint8_t> booster;
  bool display_on = false;
  bool reverse = false;
  bool adc_reverse = false;
  bool all_points = false;
  bool bias = false;
  bool common_reverse = false;
  bool static_on = false;
  bool read_modify_write = false;
};

// Transaction reference: callers supply completed, qualified bus cycles.
// No board wiring, busy latency, analog drive or pad timing is inferred.
class St7565 {
public:
  static constexpr unsigned columns = 132;
  static constexpr unsigned pages = 9;
  void hardware_reset();
  void command(std::uint8_t value);
  void write_data(std::uint8_t value);
  [[nodiscard]] LcdByte read_output() const;
  [[nodiscard]] LcdByte read_data();
  [[nodiscard]] LcdByte ram(unsigned page, unsigned column) const;
  [[nodiscard]] St7565State state() const;

private:
  enum class Parameter : std::uint8_t { none, volume, indicator, booster };
  void require_reset() const;
  void software_reset();
  void set_address(std::uint8_t value);
  void set_mode(std::uint8_t value);
  void set_parameter(std::uint8_t value);
  void advance_column();
  [[nodiscard]] static std::size_t address(unsigned page, unsigned column);
  static constexpr std::size_t ram_bytes = static_cast<std::size_t>(columns) * pages;
  std::array<std::uint8_t, ram_bytes> bytes_{};
  std::bitset<ram_bytes> known_;
  St7565State state_;
  LcdByte holder_;
  Parameter parameter_ = Parameter::none;
  std::uint8_t saved_column_ = 0;
  bool dummy_ = true;
  bool reset_observed_ = false;
};

}
