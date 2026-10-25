// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/nand_array.hpp"
#include "storage/nand_image.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace blofeld {

enum class NandOutcome : std::uint8_t { pass, fail };
enum class NandPending : std::uint8_t { none, read, reset, program, erase };

// Explicit K9F1208U0C reference milestones. No erased image, physical wiring,
// write permission, block condition, timing or successful outcome is presumed.
class Nand {
public:
  static constexpr unsigned page_bytes = NandArray::page_bytes;
  static constexpr unsigned main_bytes = NandArray::main_bytes;
  static constexpr unsigned pages = NandArray::pages;
  static constexpr unsigned block_pages = NandArray::block_pages;
  static constexpr unsigned status_bits = 0xc1;
  static constexpr std::uint8_t byte_mask = 0xff;

  Nand() = default;
  explicit Nand(const NandImage &image);
  [[nodiscard]] NandImage persistent_image() const;

  void observe_power_ready();
  void observe_bytes(unsigned page, unsigned column, std::span<const std::uint8_t> bytes);
  void observe_cell(NandLocation location, NandByte cell);
  void observe_block(unsigned block, NandBlock state);
  void observe_wp(std::optional<bool> high);
  void command(std::uint8_t value);
  void address(std::uint8_t value);
  void write(std::uint8_t value);
  void observe_completion();
  void observe_completion(NandOutcome outcome);
  void power_loss();
  [[nodiscard]] std::uint8_t read(std::uint8_t mask = byte_mask);
  [[nodiscard]] NandByte status() const;
  [[nodiscard]] NandByte cell(NandLocation location) const;
  [[nodiscard]] NandPending pending() const;

  [[nodiscard]] std::optional<bool> ready() const { return ready_; }

  [[nodiscard]] unsigned page() const { return page_; }

  [[nodiscard]] unsigned column() const { return column_; }

  [[nodiscard]] bool status_mode() const { return status_mode_; }

private:
  enum class Phase : std::uint8_t {
    cold,
    idle,
    read_address,
    read_pending,
    reset_pending,
    data,
    end,
    program_address,
    program_data,
    program_pending,
    erase_address,
    erase_pending
  };

  struct Program {
    std::array<std::uint8_t, page_bytes> bytes{};
    unsigned start = 0;
    unsigned size = 0;
    std::optional<NandArray::Page> before;
  };

  [[noreturn]] void fail(const std::string &operation, const std::string &reason) const;
  void check_observation() const;
  void check_write_permission(bool erase) const;
  void begin_program();
  void begin_erase();
  void confirm_program();
  void confirm_erase();
  void complete_program();
  void abort_active();
  void address_page(std::uint8_t value);
  void address_erase(std::uint8_t value);
  void begin_read(std::uint8_t value);
  NandArray contents_;
  Phase phase_ = Phase::cold;
  std::optional<bool> ready_;
  std::optional<bool> wp_high_;
  std::optional<NandOutcome> outcome_;
  std::optional<unsigned> pointer_;
  Program program_;
  unsigned page_ = 0;
  unsigned column_ = 0;
  unsigned address_cycles_ = 0;
  std::uint8_t command_ = 0;
  bool status_mode_ = false;
  bool reset_status_ = false;
};
}
