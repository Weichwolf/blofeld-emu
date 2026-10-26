// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "history_startup_memory.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_cop.hpp"
#include "keyboard/hcs08_known_interrupts.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::test::clock_prefix {
struct Profile {
  std::optional<unsigned> phase;
  bool adc_reference = false;
};

// Declared ADC/phase references are diagnostic inputs, never a Blofeld clock
// calibration. Each supplied BUSCLK period drives actual device owners once.
class Memory final : public Bus, public Hcs08KnownMemory, public Hcs08KnownInterrupts {
public:
  Memory(std::span<const std::uint8_t> pages, Profile input);
  icg_history::Memory core;
  Hcs08Cop cop{Hcs08CopContract::candidate_gt8a_run_bus_edges};
  Profile profile;
  unsigned services = 0;
  std::uint8_t service_known = Hcs08KnownByte::all;
  std::uint64_t attempted = 0, completed = 0;
  bool failed = false;

  void devices(const Hcs08State &state);
  void begin_instruction(ExecutionContext value) override;
  void begin_reset(ExecutionContext value) override;
  void end_reset() override;
  void retire_instruction() override;
  [[nodiscard]] std::optional<InterruptRequest> interrupt_request(std::uint8_t ccr) const override;
  [[nodiscard]] std::optional<InterruptRequest>
  interrupt_request(const Hcs08State &state) const override;
  [[nodiscard]] std::uint8_t read(std::uint16_t address) override;
  [[nodiscard]] Hcs08KnownByte read_known_byte(std::uint16_t address) override;
  void write(std::uint16_t address, std::uint8_t value) override;
  void write_known_byte(ByteWrite access, std::uint8_t mask) override;
  [[nodiscard]] std::uint8_t peek(std::uint16_t address) const override;
  [[nodiscard]] std::uint64_t revision() const override;
  [[nodiscard]] std::uint64_t code_revision() const override;
  void note_code_byte(std::uint16_t address) const override;
  void check_execute(InstructionFetch fetch) const override;

private:
  void configure_adc();
  void service(ByteWrite access, std::uint8_t mask);
};
} // namespace blofeld::test::clock_prefix
