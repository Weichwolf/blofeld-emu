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

// Logical CPU operand widths, not promises about physical bus-cycle timing.
struct MemoryRead {
  std::uint16_t address = 0;
  unsigned size = 1;
};

struct MemoryWrite {
  std::uint16_t address = 0;
  unsigned size = 1;
  std::uint64_t value = 0;
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
  [[nodiscard]] virtual std::uint64_t read_memory(MemoryRead access);
  virtual void write_memory(MemoryWrite access);
  [[nodiscard]] virtual std::uint8_t peek(std::uint16_t address) const = 0;
  [[nodiscard]] virtual std::uint64_t revision() const = 0;

  // Backends may conservatively track bytes read by the instruction decoder.
  // The default keeps the original all-write invalidation contract.
  [[nodiscard]] virtual std::uint64_t code_revision() const { return revision(); }

  virtual void note_code_byte(std::uint16_t /*address*/) const {}

  virtual void check_execute(InstructionFetch fetch) const = 0;

  virtual void begin_instruction(ExecutionContext /*context*/) {}

  virtual void begin_execution(InstructionFetch /*fetch*/) {}

  virtual void begin_reset(ExecutionContext /*context*/) {}

  virtual void end_reset() {}

  virtual void begin_interrupt(InterruptRequest /*request*/, bool /*reuse_stack*/) {}

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
