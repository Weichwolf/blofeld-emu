// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace blofeld {

enum class Hc12ResetSource : std::uint8_t { external, cop };

struct CpuState {
  std::uint16_t pc = 0;
  std::uint16_t d = 0;
  std::uint16_t x = 0;
  std::uint16_t y = 0;
  std::uint16_t sp = 0;
  std::uint8_t ccr = 0;
};

struct CpuStep {
  bool instruction_executed = true;
  std::optional<std::uint16_t> interrupt_vector = std::nullopt;
  bool stack_reused = false;
};

// Trusted opaque instruction-boundary state, including native scratch registers.
class Hc12Checkpoint {
public:
  Hc12Checkpoint(const Hc12Checkpoint &) = default;
  Hc12Checkpoint &operator=(const Hc12Checkpoint &) = default;

private:
  friend class Hc12;
  Hc12Checkpoint() = default;
  static constexpr unsigned register_bytes = 256;
  std::array<std::uint8_t, register_bytes> registers_{};
  std::uint16_t pc_ = 0;
  bool waiting_ = false;
  bool defer_irq_ = false;
};

// Native SLEIGH semantics execute original HC12 instructions. Peripheral and
// cycle timing are supplied by the board model, not by the instruction decoder.
class Hc12 {
public:
  explicit Hc12(Bus &bus, const std::filesystem::path &specification);
  ~Hc12();
  Hc12(const Hc12 &) = delete;
  Hc12 &operator=(const Hc12 &) = delete;
  void reset(Hc12ResetSource source = Hc12ResetSource::external);
  // Debug register replacement cancels WAI and interrupt deferral.
  void set_state(const CpuState &state);
  [[nodiscard]] CpuState state() const;
  [[nodiscard]] bool waiting() const;
  [[nodiscard]] std::string disassemble() const;
  [[nodiscard]] std::optional<InterruptRequest> pending_interrupt() const;
  // A failed unretired step cannot resume its partial CPU/bus prefix. Reset,
  // explicit debug state or coordinated restore must establish a fresh boundary.
  CpuStep step();

private:
  friend class McuMachine;
  [[nodiscard]] Hc12Checkpoint checkpoint() const;
  void restore_checkpoint(const Hc12Checkpoint &checkpoint);
  [[nodiscard]] bool executing() const;
  void begin_wait_clock();
  void end_wait_clock(bool healthy);
  void enter_interrupt(InterruptRequest request, std::optional<std::uint16_t> reuse_stack);
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace blofeld
