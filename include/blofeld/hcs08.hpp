// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace blofeld {
struct Hcs08State {
  static constexpr unsigned fixed_ccr = 0x60;
  std::uint16_t pc = 0;
  std::uint16_t hx = 0;
  std::uint16_t sp = 0;
  std::uint8_t a = 0;
  std::uint8_t ccr = fixed_ccr; // S08CPUV2: bits6/5 are permanently one.
  bool operator==(const Hcs08State &) const = default;
};

// Native instruction engine, separate from target memory/peripherals and BDC.
// Entry registers are explicit debug inputs, not invented power-on values.
// Timing, interrupts, reset sequencing and BDC privileges require their owners.
class Hcs08 {
public:
  explicit Hcs08(Bus &bus, const std::filesystem::path &specification);
  ~Hcs08();
  Hcs08(const Hcs08 &) = delete;
  Hcs08 &operator=(const Hcs08 &) = delete;
  void set_state(const Hcs08State &state);
  [[nodiscard]] Hcs08State state() const;
  [[nodiscard]] std::string disassemble() const;
  // BGND is an observed CPU request, not a fabricated active-BDC status.
  [[nodiscard]] bool background_requested() const;
  [[nodiscard]] bool faulted() const;
  void step();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace blofeld
