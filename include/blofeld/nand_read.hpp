// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>

namespace blofeld {

// Explicit K9F1208U0C single-page read reference, with observed completion events.
// No board attachment, erased defaults, write commands or elapsed time are implied.
class NandRead {
public:
  static constexpr unsigned page_bytes = 528;
  static constexpr unsigned main_bytes = 512;
  static constexpr unsigned pages = 131072;

  void observe_power_ready();
  void observe_bytes(unsigned page, unsigned column, std::span<const std::uint8_t> bytes);
  void command(std::uint8_t value);
  void address(std::uint8_t value);
  void observe_completion();
  [[nodiscard]] std::uint8_t read();

  [[nodiscard]] std::optional<bool> ready() const { return ready_; }

  [[nodiscard]] unsigned page() const { return page_; }

  [[nodiscard]] unsigned column() const { return column_; }

private:
  enum class Phase : std::uint8_t { cold, idle, address, read_pending, reset_pending, data, end };

  struct Page {
    std::array<std::uint8_t, page_bytes> bytes{};
    std::array<bool, page_bytes> known{};
  };

  [[noreturn]] void fail(const std::string &operation, const std::string &reason) const;
  std::map<unsigned, Page> contents_;
  Phase phase_ = Phase::cold;
  std::optional<bool> ready_;
  unsigned page_ = 0;
  unsigned column_ = 0;
  unsigned address_cycles_ = 0;
  std::uint8_t command_ = 0;
};

}
