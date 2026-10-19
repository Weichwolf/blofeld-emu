// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

namespace blofeld {

// Immutable trusted in-process state. Not a file format or a whole-board snapshot.
class McuCheckpoint {
public:
  McuCheckpoint(const McuCheckpoint &) = default;
  McuCheckpoint &operator=(const McuCheckpoint &) = default;
  McuCheckpoint(McuCheckpoint &&) noexcept = default;
  McuCheckpoint &operator=(McuCheckpoint &&) noexcept = default;
  ~McuCheckpoint();

private:
  friend class McuMachine;
  struct Storage;
  explicit McuCheckpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

// Single-threaded MCU owner. Restore replaces bus/CPU objects, invalidating
// references to them. External observers must be explicitly reattached.
class McuMachine {
public:
  McuMachine(const McuFlashImage &image, const std::filesystem::path &specification);
  ~McuMachine();
  McuMachine(const McuMachine &) = delete;
  McuMachine &operator=(const McuMachine &) = delete;
  [[nodiscard]] McuBus &bus();
  [[nodiscard]] Hc12 &cpu();
  [[nodiscard]] const McuBus &bus() const;
  [[nodiscard]] const Hc12 &cpu() const;
  [[nodiscard]] McuCheckpoint checkpoint() const;
  // Strong exception guarantee; also permits recovery after a failed CPU step.
  void restore(const McuCheckpoint &checkpoint);

private:
  void check_specification() const;
  struct State;
  std::filesystem::path specification_;
  std::vector<std::uint8_t> specification_bytes_;
  std::unique_ptr<State> state_;
};

}
