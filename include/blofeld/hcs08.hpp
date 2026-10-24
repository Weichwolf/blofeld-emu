// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace blofeld {
struct Hcs08State {
  static constexpr unsigned fixed_ccr = 0x60;
  static constexpr std::uint8_t all_ccr = 0xff;
  std::uint16_t pc = 0;
  std::uint16_t hx = 0;
  std::uint16_t sp = 0;
  std::uint8_t a = 0;
  std::uint8_t ccr_known = all_ccr;
  std::uint8_t ccr = fixed_ccr; // S08CPUV2: bits6/5 are permanently one.
  bool operator==(const Hcs08State &) const = default;
};

// Trusted native instruction-boundary state. Memory, clocks and external peers
// must be restored separately by the owner before further execution.
class Hcs08Checkpoint {
public:
  Hcs08Checkpoint(const Hcs08Checkpoint &) = default;
  Hcs08Checkpoint &operator=(const Hcs08Checkpoint &) = default;

private:
  friend class Hcs08;
  Hcs08Checkpoint() = default;
  static constexpr unsigned register_bytes = 64;
  std::array<std::uint8_t, register_bytes> registers_{};
  std::uint16_t pc_ = 0;
  bool background_ = false;
  std::uint8_t ccr_known_ = Hcs08State::all_ccr;
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
  // Warm architectural reset after the board has completed its six reset periods.
  // A/X retain supplied pre-reset state; H/SP/I/vector reset, other CCR bits unknown.
  // Peripheral reset and pin/clock sequencing belong to the board owner.
  void reset();
  [[nodiscard]] Hcs08State state() const;
  [[nodiscard]] std::string disassemble() const;
  // BGND is an observed CPU request, not a fabricated active-BDC status.
  [[nodiscard]] bool background_requested() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] Hcs08Checkpoint checkpoint() const;
  void restore_checkpoint(const Hcs08Checkpoint &checkpoint);
  void step();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace blofeld
