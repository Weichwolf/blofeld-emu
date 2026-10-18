// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace blofeld {

struct Fault : std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct ByteWrite {
  std::uint16_t address = 0;
  std::uint8_t value = 0;
};

struct InstructionFetch {
  std::uint16_t address = 0;
  unsigned length = 0;
};

struct ExecutionContext {
  static constexpr std::uint8_t initial_ccr = 0xd0;
  std::uint16_t pc = 0;
  std::uint8_t ccr = initial_ccr;
};

enum class InterruptKind : std::uint8_t { maskable, xirq };

struct InterruptRequest {
  std::uint16_t vector = 0;
  InterruptKind kind = InterruptKind::maskable;
};

class Bus {
public:
  virtual ~Bus() = default;
  virtual std::uint8_t read(std::uint16_t address) = 0;
  virtual void write(std::uint16_t address, std::uint8_t value) = 0;
  [[nodiscard]] virtual std::uint8_t peek(std::uint16_t address) const = 0;
  [[nodiscard]] virtual std::uint64_t revision() const = 0;
  virtual void check_execute(InstructionFetch fetch) const = 0;

  virtual void begin_instruction(ExecutionContext /*context*/) {}

  [[nodiscard]] virtual std::optional<InterruptRequest>
  interrupt_request(std::uint8_t /*ccr*/) const {
    return {};
  }

  virtual void acknowledge_interrupt(std::uint16_t /*vector*/) {}

  // Commit effects that become visible after the current instruction's final
  // bus transaction. This is not an instruction-to-cycle timing approximation.
  virtual void retire_instruction() {}
};

std::string hex(std::uint64_t value, unsigned width = 4);

} // namespace blofeld
