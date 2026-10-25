// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_file.hpp"
#include "storage/nand_image.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>

namespace blofeld {

// Single-writer persistent device. Mutations publish a cold power-loss
// projection before becoming visible. Host IO is outside the future audio path.
class NandStore {
public:
  explicit NandStore(const std::filesystem::path &path);
  NandStore(const std::filesystem::path &path, const NandImage &initial);
  ~NandStore();
  NandStore(const NandStore &) = delete;
  NandStore &operator=(const NandStore &) = delete;
  void set_observer(NandFileObserver observer);
  void command(std::uint8_t value);
  void address(std::uint8_t value);
  void write(std::uint8_t value);
  void observe_power_ready();
  void observe_wp(std::optional<bool> high);
  void observe_bytes(unsigned page, unsigned column, std::span<const std::uint8_t> bytes);
  void observe_cell(NandLocation location, NandByte cell);
  void observe_block(unsigned block, NandBlock state);
  void observe_completion();
  void observe_completion(NandOutcome outcome);
  void power_loss();
  [[nodiscard]] std::uint8_t read(std::uint8_t mask = Nand::byte_mask);
  [[nodiscard]] const Nand &device() const;
  [[nodiscard]] bool faulted() const;

  [[nodiscard]] std::optional<bool> ready() const { return device().ready(); }

  [[nodiscard]] unsigned page() const { return device().page(); }

  [[nodiscard]] unsigned column() const { return device().column(); }

  [[nodiscard]] NandPending pending() const { return device().pending(); }

  [[nodiscard]] bool status_mode() const { return device().status_mode(); }

  [[nodiscard]] NandByte status() const { return device().status(); }

  [[nodiscard]] NandByte cell(NandLocation location) const { return device().cell(location); }

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
