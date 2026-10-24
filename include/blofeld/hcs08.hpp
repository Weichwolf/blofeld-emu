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
struct Hcs08State {
  static constexpr unsigned fixed_ccr = 0x60;
  static constexpr std::uint8_t all_ccr = 0xff;
  static constexpr std::uint8_t all_a = 0xff;
  static constexpr std::uint16_t all_hx = 0xffff;
  std::uint16_t pc = 0;
  std::uint16_t hx = 0;
  std::uint16_t sp = 0;
  std::uint8_t a = 0;
  std::uint8_t ccr_known = all_ccr;
  std::uint8_t ccr = fixed_ccr; // S08CPUV2: bits6/5 are permanently one.
  bool pc_known = true;         // False after active-background reset until a qualified PC write.
  std::uint8_t a_known = all_a;
  std::uint16_t hx_known = all_hx;
  bool operator==(const Hcs08State &) const = default;
};

enum class Hcs08IrqBusContract : std::uint8_t { unqualified, reference_unstretched_sequence };

// Completed user-view bus cycles, not internal pipeline register timing. The
// vector bytes are observed separately before the free cycle makes the target
// available. Prefetched bytes stay owned until a qualified queue consumer exists.
struct Hcs08IrqBusState {
  Hcs08State entry;
  std::uint16_t admitted_vector = 0, vector = 0, target = 0;
  std::array<std::uint8_t, 2> vector_bytes{};
  std::array<std::uint8_t, 3> prefetched{};
  unsigned completed_cycles = 0, attempted_cycles = 0, vector_reads = 0, prefetched_bytes = 0;
  unsigned ready_program_bytes = 0;
  bool program_byte_in_flight = false;
  bool vector_selected = false, target_known = false, acknowledged = false;
  bool operator==(const Hcs08IrqBusState &) const = default;
};

// Trusted native instruction/IRQ-bus boundary state. Memory, clocks and external peers
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
  bool pc_known_ = true;
  std::uint8_t ccr_known_ = Hcs08State::all_ccr;
  std::uint8_t a_known_ = Hcs08State::all_a;
  std::uint16_t hx_known_ = Hcs08State::all_hx;
  std::optional<Hcs08IrqBusState> irq_bus_;
};

enum class Hcs08ResetEntry : std::uint8_t { unqualified, normal, active_background };
enum class Hcs08ResetCause : std::uint8_t { unqualified, warm, power_on };

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
  // Architectural reset after the board has completed its reset sequence.
  // Warm retains healthy A/X knowledge; power_on starts with unknown A/X.
  // H/SP/I reset, other CCR bits unknown. A fresh CPU permits only power_on.
  // Normal entry fetches the vector. Active background entry precedes that fetch;
  // its numerical PC is unqualified and remains explicitly unknown.
  // Peripheral reset and pin/clock sequencing belong to the board owner.
  void reset(Hcs08ResetEntry mode = Hcs08ResetEntry::normal,
             Hcs08ResetCause cause = Hcs08ResetCause::warm);
  [[nodiscard]] Hcs08State state() const;
  [[nodiscard]] std::string disassemble() const;
  // BGND is an observed CPU request, not a fabricated active-BDC status.
  [[nodiscard]] bool background_requested() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] Hcs08Checkpoint checkpoint() const;
  void restore_checkpoint(const Hcs08Checkpoint &checkpoint);
  // Normal-running architectural boundary, separate from its eleven-cycle/queue
  // timing owner. Only a real eligible bus request can create a stack frame.
  // Entry does not retire an instruction or clear peripheral source flags.
  [[nodiscard]] std::optional<std::uint16_t> pending_interrupt() const;
  [[nodiscard]] bool service_interrupt();
  // Real sssssvvfppp accesses, one explicitly supplied bus period per advance.
  // No instruction retires. Ordinary stepping refuses the retained IRQ queue;
  // consuming/invalidation of that queue is not yet qualified.
  [[nodiscard]] bool start_irq_bus(Hcs08IrqBusContract contract);
  void advance_irq_bus();
  [[nodiscard]] std::optional<Hcs08IrqBusState> irq_bus_state() const;
  // Executes one instruction; an eligible unserviced IRQ refuses before fetch.
  void step();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace blofeld
