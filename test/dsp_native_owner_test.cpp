// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_native_timed_execution.hpp"
#include "dsp56300/core.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
unsigned assertions = 0;
constexpr unsigned source_address = 8, destination_address = 12;
constexpr unsigned high_word = 0x923456, low_word = 0xabcdef;
constexpr std::uint64_t initial_a = 0x5a112233445566, initial_b = 0xa5f1e2d3c4b5a6;
constexpr unsigned normal_omr = 0x305, initial_status = 0x300;

void require(const bool condition, const std::string_view message) {
  ++assertions;
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

class Ram final : public dsp56300::Io {
public:
  explicit Ram(const std::span<const unsigned> program) : known_(program.size()) {
    if (known_ > code_.size()) { throw std::invalid_argument("Oversized authored program"); }
    std::ranges::copy(program, code_.begin());
    x_[source_address] = high_word;
    y_[source_address] = low_word;
  }

  [[nodiscard]] dsp56300::Buffers storage() noexcept {
    return {.program = std::span(code_).first(known_), .x = x_, .y = y_};
  }

  [[nodiscard]] std::optional<unsigned>
  inspect_program(const unsigned address) const noexcept override {
    return address < known_ ? std::optional(code_.at(address)) : std::nullopt;
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision_; }

  void replace_first(dsp56300::Core &cpu, const unsigned word) {
    const std::array words{word};
    cpu.load(dsp56300::Space::program, 0, words);
    ++revision_;
  }

  bool read(const dsp56300::Space space,
            const unsigned address,
            unsigned /*pc*/,
            unsigned &value) noexcept override {
    ++effects_;
    if (space == dsp56300::Space::program) {
      if (address >= known_) { return false; }
      value = code_.at(address);
      return true;
    }
    if ((space != dsp56300::Space::x && space != dsp56300::Space::y) || address >= x_.size()) {
      return false;
    }
    value = (space == dsp56300::Space::x ? x_ : y_).at(address);
    return true;
  }

  bool write(const dsp56300::Space space,
             const unsigned address,
             unsigned /*pc*/,
             const unsigned value) noexcept override {
    ++effects_;
    if ((space != dsp56300::Space::x && space != dsp56300::Space::y) || address >= x_.size()) {
      return false;
    }
    (space == dsp56300::Space::x ? x_ : y_).at(address) = value;
    return true;
  }

  bool bit(dsp56300::Space /*space*/,
           unsigned /*address*/,
           unsigned /*pc*/,
           unsigned /*bit*/,
           dsp56300::BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  [[nodiscard]] std::uint64_t clock() const noexcept { return clock_; }

  [[nodiscard]] unsigned effects() const noexcept { return effects_; }

  [[nodiscard]] unsigned read_x(const unsigned address) const { return x_.at(address); }

  void advance_clock(const std::uint64_t units) { clock_ += units; }

private:
  std::array<unsigned, 64> x_{}, y_{};
  std::uint64_t clock_ = 0;
  unsigned effects_ = 0;

  std::array<unsigned, 16> code_{};
  std::size_t known_;
  std::uint64_t revision_ = 0;
};

dsp56300::Registers seed() {
  dsp56300::Registers result;
  result.a = initial_a;
  result.b = initial_b;
  result.x = 1;
  result.y = 2;
  result.omr = normal_omr;
  result.sr = initial_status;
  result.r[0] = source_address;
  result.r[1] = destination_address;
  result.m.fill(dsp56300::word_mask);
  return result;
}

blofeld::DspTimedInstruction
offer(blofeld::DspNativeTimedExecution &timing, dsp56300::Core &cpu, const Ram &memory) {
  const auto actual = timing.run(cpu, memory, {});
  if (!actual.next || !actual.next_cycles) {
    throw std::runtime_error("Missing genuine Core offer");
  }
  return timing.prepare_owner(cpu, actual.next.value(), actual.next_cycles.value());
}

dsp56300::InstructionProgress stage(dsp56300::Core &cpu,
                                    const blofeld::DspTimedInstruction instruction) {
  auto actual =
      cpu.begin_instruction(instruction.words, dsp56300::InstructionPublication::pcu_updates);
  constexpr unsigned maximum_effects = 64;
  for (unsigned i = 0; actual.pending && i < maximum_effects; ++i) {
    actual = cpu.advance_instruction(actual.pending->ticket);
  }
  return actual;
}

unsigned long_load(const unsigned selector) {
  constexpr unsigned base = 0x40d800;
  constexpr unsigned high_selector_shift = 17;
  return base | ((selector & 3U) << 16) | ((selector & 4U) << high_selector_shift);
}

void native_long_views() {
  constexpr std::uint64_t raw = 0x923456abcdef;
  constexpr std::uint64_t full = 0xff923456abcdef;
  constexpr std::uint64_t high_accumulator = 0xff923456000000;
  constexpr std::uint64_t low_accumulator = 0xffabcdef000000;
  constexpr std::array<std::array<std::uint64_t, 2>, 8> accumulators{
      {{0x5a923456abcdef, initial_b},
       {initial_a, 0xa5923456abcdef},
       {initial_a, initial_b},
       {initial_a, initial_b},
       {full, initial_b},
       {initial_a, full},
       {high_accumulator, low_accumulator},
       {low_accumulator, high_accumulator}}};
  for (unsigned selector = 0; selector < 8; ++selector) {
    const std::array program{long_load(selector), 0U};
    Ram memory(program);
    dsp56300::Core cpu(memory.storage(), &memory);
    cpu.set_registers(seed());
    blofeld::DspNativeTimedExecution timing(cpu.progress(),
                                            blofeld::DspDependencyPolicy::conservative);
    const auto instruction = offer(timing, cpu, memory);
    require(instruction.dependency_units == 3, "Conservative long transfer keeps the full pre-gap");
    memory.advance_clock(instruction.base_units + instruction.dependency_units);
    require(timing.run_owner(cpu, memory, instruction),
            "A genuine bounded Core instruction retires");
    auto expected = seed();
    expected.a = accumulators.at(selector)[0];
    expected.b = accumulators.at(selector)[1];
    if (selector == 2) { expected.x = raw; }
    if (selector == 3) { expected.y = raw; }
    ++expected.r[0];
    expected.pc = 1;
    require(cpu.registers() == expected &&
                cpu.progress() == dsp56300::Progress{.instructions = 1, .estimated_cycles = 1} &&
                memory.effects() == 0 && memory.clock() == 4,
            "Independent long destination, flags, one AGU update and real retirement remain exact");
    bool refused = false;
    try {
      static_cast<void>(timing.run_owner(cpu, memory, instruction));
    } catch (const blofeld::Fault &) { refused = true; }
    require(refused && cpu.progress().instructions == 1,
            "A consumed reservation cannot retire twice");
    const auto following = offer(timing, cpu, memory);
    require(following.dependency_units == 3,
            "The actual native load retains its complete post-gap");
    memory.advance_clock(following.base_units + following.dependency_units);
    require(timing.run_owner(cpu, memory, following) && memory.clock() == 8 &&
                cpu.registers().pc == 2 && cpu.jit_statistics().native_move_instructions == 1 &&
                cpu.jit_statistics().native_alu_instructions == 2 &&
                cpu.jit_statistics().helper_instructions == 0,
            "Post-gap NOP and original long RAM read execute natively without a staged receipt");
  }
}

void clocked_known_dependency() {
  constexpr unsigned load_a = 0x56e000;
  constexpr unsigned store_a = 0x566100;
  const std::array program{load_a, store_a};
  Ram memory(program);
  dsp56300::Core cpu(memory.storage(), &memory);
  cpu.set_registers(seed());
  blofeld::DspNativeTimedExecution timing(cpu.progress());
  const auto first = timing.run(cpu, memory, {.instructions = 1, .cycles = 16});
  require(first.execution.dispatches == 1, "Initial RAM load retires through the existing prefix");
  memory.advance_clock(first.cycles);
  const auto following = offer(timing, cpu, memory);
  require(following.dependency_units == 1,
          "Matching accumulator store still requires one interlock");
  memory.advance_clock(following.base_units + following.dependency_units);
  require(
      timing.run_owner(cpu, memory, following) && memory.clock() == 3 &&
          memory.read_x(destination_address) == high_word && memory.effects() == 0 &&
          cpu.progress() == dsp56300::Progress{.instructions = 2, .estimated_cycles = 2} &&
          cpu.jit_statistics().native_move_instructions == 2 &&
          cpu.jit_statistics().helper_instructions == 0,
      "A charged dependency no longer forces an otherwise genuine native RAM store into replay");
}

void changed_captured_code() {
  constexpr std::uint64_t expected_x = 0x923456abcdef;
  const std::array program{long_load(2)};
  Ram memory(program);
  dsp56300::Core cpu(memory.storage(), &memory);
  cpu.set_registers(seed());
  blofeld::DspNativeTimedExecution timing(cpu.progress(),
                                          blofeld::DspDependencyPolicy::conservative);
  const auto instruction = offer(timing, cpu, memory);
  memory.replace_first(cpu, 0);
  const auto before = cpu.registers();
  memory.advance_clock(instruction.base_units + instruction.dependency_units);
  require(!timing.run_owner(cpu, memory, instruction) && cpu.registers() == before &&
              cpu.progress().instructions == 0,
          "Live code substitution cannot replace the previously captured reserved opcode");
  const auto actual = stage(cpu, instruction);
  require(actual.retired && cpu.registers().x == expected_x && memory.effects() == 2,
          "The original captured instruction remains available to its genuine staged owner");
  timing.complete_owner(cpu, instruction);
}

void invalid_owner_clocks_and_real_fault() {
  const std::array program{long_load(0)};
  Ram memory(program);
  dsp56300::Core cpu(memory.storage(), &memory);
  auto initial = seed();
  constexpr unsigned port = 0xffff90;
  initial.r[0] = port;
  cpu.set_registers(initial);
  blofeld::DspNativeTimedExecution timing(cpu.progress(),
                                          blofeld::DspDependencyPolicy::conservative);
  const auto instruction = offer(timing, cpu, memory);
  auto forged = instruction;
  --forged.dependency_units;
  bool refused = false;
  try {
    static_cast<void>(timing.run_owner(cpu, memory, forged));
  } catch (const blofeld::Fault &) { refused = true; }
  require(refused && cpu.registers() == initial && memory.effects() == 0,
          "A shortened declared gap is refused before any CPU or bus effect");
  memory.advance_clock(instruction.base_units + instruction.dependency_units);
  require(!timing.run_owner(cpu, memory, instruction) && cpu.registers() == initial &&
              memory.effects() == 0 && cpu.progress().instructions == 0 &&
              cpu.fault().kind == dsp56300::FaultKind::none,
          "A real peripheral address remains owned with no speculative callback or AGU update");
  const auto actual = stage(cpu, instruction);
  require(!actual.retired && actual.fault.kind == dsp56300::FaultKind::memory_read &&
              actual.fault.space == dsp56300::Space::x && actual.fault.address == port &&
              actual.fault.pc == 0 && memory.effects() == 1,
          "The genuine owned read still fails visibly with its actual address and originating PC");
}
}

int main() {
  try {
    native_long_views();
    clocked_known_dependency();
    changed_captured_code();
    invalid_owner_clocks_and_real_fault();
    std::cout << "PASS bounded native owners: " << assertions << " checks\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
