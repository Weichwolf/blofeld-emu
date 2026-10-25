// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
namespace hcs08_bdc {
constexpr std::uint8_t go = 0x08, background = 0x90;
constexpr std::uint8_t read_status = 0xe4, write_control = 0xc4;
constexpr std::uint8_t read_byte = 0xe0, write_byte = 0xc0;
constexpr std::uint8_t write_a = 0x48, write_ccr = 0x49, write_pc = 0x4b;
constexpr std::uint8_t write_hx = 0x4c, write_sp = 0x4f;
constexpr std::uint8_t read_a = 0x68, read_ccr = 0x69, read_pc = 0x6b;
constexpr std::uint8_t read_hx = 0x6c, read_sp = 0x6f;
}

// Target mapping owns BDC privilege/security, memory timing and side effects.
// This is deliberately not the CPU's unqualified generic Bus interface.
class Hcs08BdcMemory {
public:
  virtual ~Hcs08BdcMemory() = default;
  [[nodiscard]] virtual bool secured(ExecutionContext context) const = 0;
  virtual std::uint8_t read(std::uint16_t address, ExecutionContext context) = 0;
  virtual void write(ByteWrite access, ExecutionContext context) = 0;
};

enum class Hcs08BdcReset : std::uint8_t { normal, bkgd_held_low };
enum class Hcs08BdcGrant : std::uint8_t { unqualified, reference_instruction_boundary };

struct Hcs08BdcState {
  std::array<std::uint8_t, 4> frame{};
  std::array<std::uint8_t, 2> reply{};
  unsigned received = 0, expected = 0, reply_size = 0, reply_cursor = 0;
  std::uint64_t serviced = 0;
  std::uint8_t control = 0;
  bool active = false;
  bool operator==(const Hcs08BdcState &) const = default;
};

// The owning board must restore CPU, memory and external transport separately.
class Hcs08BdcCheckpoint {
public:
  Hcs08BdcCheckpoint(const Hcs08BdcCheckpoint &) = default;
  Hcs08BdcCheckpoint &operator=(const Hcs08BdcCheckpoint &) = default;

private:
  friend class Hcs08Bdc;
  Hcs08BdcCheckpoint() = default;
  Hcs08BdcState state_;
};

// Byte transaction layer only: no pin decoding, clocks, delays or guessed ACKs.
// A complete frame requires an explicit service boundary before any effects.
class Hcs08Bdc {
public:
  Hcs08Bdc(Hcs08 &cpu,
           Hcs08BdcMemory &memory,
           Hcs08FlashContract contract,
           Hcs08BdcReset sampled_reset);
  Hcs08Bdc(const Hcs08Bdc &) = delete;
  Hcs08Bdc &operator=(const Hcs08Bdc &) = delete;
  void receive(std::uint8_t byte);
  void service(Hcs08BdcGrant grant = Hcs08BdcGrant::unqualified);
  [[nodiscard]] std::uint8_t read_reply();
  // The clock owner, not a status read, establishes the documented timeout.
  void communication_timeout();
  void step_cpu();
  [[nodiscard]] bool service_interrupt();
  [[nodiscard]] bool start_irq_bus(Hcs08IrqBusContract contract);
  void advance_irq_bus();
  [[nodiscard]] std::optional<Hcs08IrqBusState> irq_bus_state() const;
  void start_irq_program(Hcs08PrefetchContract contract);
  void advance_irq_program();
  [[nodiscard]] std::optional<Hcs08PrefetchState> prefetch_state() const;
  [[nodiscard]] std::optional<Hcs08InstructionReservation>
  reserve_instruction(Hcs08InstructionContract contract);
  [[nodiscard]] bool instruction_reserved() const;
  [[nodiscard]] bool reservation_matches(const Hcs08InstructionReservation &reservation) const;
  void retire_reserved_instruction(const Hcs08InstructionReservation &reservation);
  [[nodiscard]] std::uint8_t status() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] Hcs08BdcState state() const;
  [[nodiscard]] Hcs08State cpu_state() const;
  [[nodiscard]] Hcs08BdcCheckpoint checkpoint() const;
  void restore_checkpoint(const Hcs08BdcCheckpoint &checkpoint);

private:
  void healthy() const;
  void complete_cpu_step();
  [[noreturn]] void fail(const std::string &operation, const std::string &reason) const;
  [[nodiscard]] unsigned frame_size(std::uint8_t command) const;
  [[nodiscard]] std::uint16_t argument() const;
  void execute(Hcs08BdcGrant grant);
  void register_access();
  void control_write();
  void reply_byte(std::uint8_t byte);
  void reply_word(std::uint16_t word);
  Hcs08 &cpu_;
  Hcs08BdcMemory &memory_;
  Hcs08BdcState state_;
  bool healthy_ = true;
};
} // namespace blofeld
