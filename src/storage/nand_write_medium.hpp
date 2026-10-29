// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_operation_clock.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace blofeld {
// Explicit reference cell physics and maximum-duration write/verify controller.
// No external pass/fail completion entry point. Copies own complete pending work.
class NandWriteMedium {
public:
  NandWriteMedium(Nand nand, std::uint32_t timebase, NandCellProfile profile);

  [[nodiscard]] const Nand &device() const { return clock_.device(); }

  [[nodiscard]] std::uint64_t ticks() const { return clock_.ticks(); }

  [[nodiscard]] bool faulted() const { return bad_ || clock_.faulted(); }

  [[nodiscard]] std::optional<NandCompletionEvent> next_event() const;
  [[nodiscard]] std::optional<NandCompletionEvent> advance(std::uint64_t at);
  void observe_power_ready();
  void observe_identification(NandIdentification identification);
  void observe_wp(std::optional<bool> high);
  void command(std::uint8_t value);
  void address(std::uint8_t value);
  void write(std::uint8_t value);
  [[nodiscard]] std::uint8_t read(std::uint8_t mask = Nand::byte_mask);
  void observe_read_end(NandReadEnd edge);
  void power_loss();

private:
  using Page = std::array<NandByte, Nand::page_bytes>;

  struct Job {
    NandCompletionEvent event;
    unsigned first_page = 0;
    unsigned column = 0;
    std::vector<std::uint8_t> data;
    std::vector<Page> before;
  };

  [[noreturn]] void fail(const std::string &reason) const;
  void healthy() const;
  [[nodiscard]] Job prepare(NandPending kind) const;
  void execute(const Job &job);
  static void merge(std::optional<bool> &verified, std::optional<bool> next);
  NandOperationClock clock_;
  std::optional<NandCellModel> cells_;
  std::uint32_t timebase_;
  std::vector<std::uint8_t> staged_;
  unsigned program_page_ = 0;
  unsigned program_column_ = 0;
  std::optional<Job> job_;
  bool bad_ = false;
};
}
