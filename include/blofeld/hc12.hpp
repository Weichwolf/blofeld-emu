// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"

#include <filesystem>
#include <memory>
#include <string>

namespace blofeld {

struct CpuState {
  std::uint16_t pc = 0;
  std::uint16_t d = 0;
  std::uint16_t x = 0;
  std::uint16_t y = 0;
  std::uint16_t sp = 0;
  std::uint8_t ccr = 0;
};

// Native SLEIGH semantics execute original HC12 instructions. Peripheral and
// cycle timing are supplied by the board model, not by the instruction decoder.
class Hc12 {
public:
  explicit Hc12(Bus &bus, const std::filesystem::path &specification);
  ~Hc12();
  Hc12(const Hc12 &) = delete;
  Hc12 &operator=(const Hc12 &) = delete;
  void reset();
  void set_state(const CpuState &state);
  [[nodiscard]] CpuState state() const;
  [[nodiscard]] std::string disassemble() const;
  void step();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace blofeld
