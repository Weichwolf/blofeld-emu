// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_system_options.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
enum class Hcs08CopContract : std::uint8_t { unqualified, candidate_gt8a_run_bus_edges };

struct Hcs08CopState {
  std::optional<std::uint8_t> reset_status = std::nullopt;
  std::uint32_t elapsed = 0;
  std::uint8_t options = Hcs08SystemOptions::reset_value;
  bool options_written = false, reset_requested = false;
  bool operator==(const Hcs08CopState &) const = default;
};

// Run-mode bus-clock watchdog and SRS/SOPT registers. Timeout is a reset request;
// the board must perform coordinated CPU/peripheral/NVM reset before resuming.
class Hcs08Cop {
public:
  explicit Hcs08Cop(Hcs08CopContract contract);
  void context(ExecutionContext value);
  void reset(std::optional<std::uint8_t> value = std::nullopt);
  // Return consumed periods, stopping exactly at the first reset request.
  [[nodiscard]] std::uint64_t advance_bus(std::uint64_t periods);
  [[nodiscard]] Hcs08CopState state() const;
  [[nodiscard]] std::uint8_t read(std::uint16_t address) const;
  void write(ByteWrite access);
  static constexpr std::uint16_t status = 0x1800;

private:
  void healthy(const std::string &operation, unsigned address) const;
  [[noreturn]] void
  fail(const std::string &operation, unsigned address, const std::string &reason) const;
  Hcs08SystemOptions options_{Hcs08FlashContract::candidate_gt8a};
  std::optional<std::uint8_t> reset_status_;
  std::uint32_t elapsed_ = 0;
  bool reset_requested_ = false;
  ExecutionContext context_;
};
} // namespace blofeld
