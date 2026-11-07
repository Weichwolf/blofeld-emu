// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "cpu12/memory.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
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
  std::uint32_t retired = 0;
  bool instruction_executed = true;
  std::optional<std::uint16_t> interrupt_vector = std::nullopt;
  bool stack_reused = false;
};

// Native CPU12 adapter; McuMachine owns device time and peripherals.
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
  // explicit debug state must establish a fresh boundary.
  CpuStep step();
  CpuStep run_until(std::uint64_t cycle_deadline);
  [[nodiscard]] std::uint64_t cycles() const;

private:
  friend class McuMachine;
  [[nodiscard]] bool executing() const;
  void bind_memory(cpu12::Memory memory);
  void set_clock_observer(std::function<void(std::uint64_t, ExecutionContext, bool)> observer);
  void begin_wait_clock();
  void end_wait_clock(bool healthy);
  void enter_interrupt(InterruptRequest request, std::optional<std::uint16_t> reuse_stack);
  CpuStep execute(std::optional<std::uint64_t> deadline);
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace blofeld
