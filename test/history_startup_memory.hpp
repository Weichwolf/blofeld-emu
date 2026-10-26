// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_icg.hpp"
#include "keyboard/hcs08_icg_history.hpp"
#include "keyboard/hcs08_known_interrupts.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_tpm.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::test::icg_history {
// Untimed original-prefix diagnostic. No DCO rate, timer count or reset-to-
// instrument success is supplied. The first live TPM read must refuse.
class Memory final : public Bus, public Hcs08KnownMemory, public Hcs08KnownInterrupts {
public:
  explicit Memory(std::span<const std::uint8_t> pages);
  hcs08_clock_startup::ApplicationMemory data;
  Hcs08IcgHistory history{Hcs08IcgHistoryContract::candidate_gt8a_unknown_events};
  Hcs08Tpm timer{Hcs08TpmContract::candidate_gt8a_run_bus_edges};

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
};
} // namespace blofeld::test::icg_history
