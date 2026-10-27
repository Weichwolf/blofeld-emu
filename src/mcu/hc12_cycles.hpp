// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "common/masked_bus.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string_view>

namespace blofeld {

struct Hc12CyclePlan {
  std::string_view slots;
  unsigned bytes = 0;
};

// S12CPUV2 HCS12 cycle sequences, assuming un-stretched internal memory.
// Unsupported instructions reject; this is not a complete prefetch/bus model.
[[nodiscard]] Hc12CyclePlan hc12_cycle_plan(const Bus &bus, ExecutionContext context);

enum class Hc12WordPart : std::uint8_t { whole, high, low };
enum class Hc12ReadTiming : std::uint8_t { unqualified, single_cycle, byte_cycles };

struct Hc12BusCycle {
  char kind = 'f'; // S12CPUV2 section 6.6 cycle code.
  std::optional<std::uint16_t> address = std::nullopt;
  unsigned size = 0;
  // Expanded reads retain the native slot kind and expose each physical byte.
  Hc12WordPart word_part = Hc12WordPart::whole;
  ExecutionContext context{};
};

// The caller qualifies memory and advances one bus cycle per callback. Optional
// read geometry expands a native word into two separately clocked/latched bytes.
class Hc12CycleBus final : public MaskedByteBus {
public:
  using Clock = std::function<void(Hc12BusCycle)>;
  using ReadTiming = std::function<Hc12ReadTiming(MemoryRead)>;
  Hc12CycleBus(Bus &bus, Clock clock, ReadTiming read_timing = {});
  std::uint8_t read(std::uint16_t address) override;
  void write(std::uint16_t address, std::uint8_t value) override;
  [[nodiscard]] std::uint64_t read_memory(MemoryRead transfer) override;
  [[nodiscard]] std::uint8_t read_masked_byte(MaskedByteRead access) override;
  void write_memory(MemoryWrite transfer) override;
  [[nodiscard]] std::uint8_t peek(std::uint16_t address) const override;
  [[nodiscard]] std::uint64_t revision() const override;
  [[nodiscard]] std::uint64_t code_revision() const override;
  void note_code_byte(std::uint16_t address) const override;
  void check_execute(InstructionFetch fetch) const override;
  void begin_instruction(ExecutionContext context) override;
  void begin_execution(InstructionFetch fetch) override;
  void begin_reset(ExecutionContext context) override;
  void end_reset() override;
  void begin_interrupt(InterruptRequest request, bool reuse_stack) override;
  void retire_instruction() override;
  [[nodiscard]] std::optional<InterruptRequest> interrupt_request(std::uint8_t ccr) const override;
  void acknowledge_interrupt(std::uint16_t vector) override;
  // Caller-owned WAI/peripheral time, outside an executing instruction.
  void advance_idle(unsigned cycles);

  [[nodiscard]] std::uint64_t cycles() const { return cycles_; }

  [[nodiscard]] bool faulted() const { return faulted_; }

private:
  friend class McuMachine;
  [[nodiscard]] std::uint64_t checkpoint_cycles() const;
  void restore_boundary(std::uint64_t cycles, ExecutionContext context);
  void healthy() const;
  void start(std::string_view slots);
  void tick(Hc12BusCycle cycle);
  [[nodiscard]] char access(MemoryRead transfer, bool write);
  [[nodiscard]] std::uint64_t read_byte(Hc12BusCycle cycle);
  [[nodiscard]] std::uint64_t read_word_bytes(MemoryRead transfer, char kind);
  [[nodiscard]] std::uint64_t read_operand(MemoryRead transfer, std::optional<std::uint8_t> mask);
  void finish();
  [[noreturn]] void fail(std::string_view reason) const;
  Bus &bus_;
  Clock clock_;
  ReadTiming read_timing_;
  ExecutionContext context_{};
  std::string_view slots_;
  std::size_t next_ = 0;
  std::uint64_t cycles_ = 0;
  bool active_ = false;
  mutable bool faulted_ = false;
};

}
