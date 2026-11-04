// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_native_timed_execution.hpp"
#include "dsp56300/core.hpp"
#include "dsp_native_timed_fixture.hpp"

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
using namespace blofeld::test::timed;
unsigned assertions = 0;
constexpr unsigned output_register = 0xffff90;

void require(const bool condition, const std::string_view message) {
  ++assertions;
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

class Memory final : public dsp56300::Io {
public:
  using OutputBlock = std::array<unsigned, windows.size()>;

  explicit Memory(const std::span<const unsigned> supplied = program)
      : known_words_(supplied.size()) {
    require(supplied.size() <= source_.size(), "Bounded authored code image");
    std::ranges::copy(supplied, source_.begin());
  }

  [[nodiscard]] std::uint64_t clock() const noexcept { return clock_; }

  void advance_clock(const std::uint64_t units) { clock_ += units; }

  [[nodiscard]] unsigned output_count() const noexcept { return output_count_; }

  [[nodiscard]] const OutputBlock &outputs() const noexcept { return outputs_; }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision_; }

  void patch(const unsigned address, const unsigned value) {
    require(address < known_words_ && value <= dsp56300::word_mask, "Bounded actual code mutation");
    source_.at(address) = value;
    ++revision_;
  }

  [[nodiscard]] std::optional<unsigned>
  inspect_program(const unsigned address) const noexcept override {
    return address < known_words_ ? std::optional{source_.at(address)} : std::nullopt;
  }

  bool read(const dsp56300::Space space,
            const unsigned address,
            unsigned /*pc*/,
            unsigned &value) noexcept override {
    if (space == dsp56300::Space::program) {
      const auto word = inspect_program(address);
      if (!word) { return false; }
      value = *word;
      return true;
    }
    return false;
  }

  bool write(const dsp56300::Space space,
             const unsigned address,
             unsigned /*pc*/,
             const unsigned value) noexcept override {
    if (space != dsp56300::Space::x || address != output_register ||
        output_count_ == windows.size()) {
      return false;
    }
    outputs_.at(output_count_++) = value;
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

private:
  static constexpr unsigned capacity = 512;
  std::array<unsigned, capacity> source_{};
  std::size_t known_words_;
  std::uint64_t clock_ = 0;
  OutputBlock outputs_{};
  unsigned output_count_ = 0;
  std::uint64_t revision_ = 0;
};

void stage(dsp56300::Core &cpu, const blofeld::DspTimedInstruction &instruction) {
  auto result =
      cpu.begin_instruction(instruction.words, dsp56300::InstructionPublication::pcu_updates);
  constexpr unsigned maximum_effects = 256;
  for (unsigned effect = 0; result.pending && effect < maximum_effects; ++effect) {
    result = cpu.advance_instruction(result.pending->ticket);
  }
  require(result.retired && !result.pending && result.fault.kind == dsp56300::FaultKind::none,
          "Every owner instruction completes through real staged Core effects");
}

unsigned next_anchor(const unsigned completed) {
  auto next = static_cast<unsigned>(starts.size());
  for (const auto &window : windows) {
    if (window.first + 1 > completed) { next = std::min(next, window.first + 1); }
    if (window.last + 1 > completed) { next = std::min(next, window.last + 1); }
  }
  return next;
}

void reference_windows() {
  Memory memory;
  dsp56300::Core cpu(memory);
  blofeld::DspNativeTimedExecution timing(cpu.progress());
  std::array<std::uint64_t, starts.size()> anchors{};
  unsigned completed = 0;
  unsigned groups = 0;
  unsigned stalls = 0;
  while (completed < starts.size()) {
    const auto end = next_anchor(completed);
    const auto before_outputs = memory.output_count();
    constexpr unsigned generous_cycle_budget = 1024;
    const auto result =
        timing.run(cpu, memory, {.instructions = end - completed, .cycles = generous_cycle_budget});
    require(result.execution.fault.kind == dsp56300::FaultKind::none,
            "Native prefix executes actual known code without a fault");
    require(memory.output_count() == before_outputs,
            "CPU-only native prefixes stop before the genuine peripheral write");
    if (result.execution.dispatches != 0) {
      memory.advance_clock(result.cycles);
      completed += result.execution.dispatches;
      groups += result.execution.dispatches > 1 ? 1U : 0U;
    } else {
      if (!result.next || !result.next_cycles) { throw std::runtime_error("Missing Core preview"); }
      const auto instruction = timing.prepare_owner(cpu, *result.next, *result.next_cycles);
      memory.advance_clock(instruction.base_units + instruction.dependency_units);
      stalls += instruction.dependency_units;
      stage(cpu, instruction);
      timing.complete_owner(cpu, instruction);
      ++completed;
    }
    require(cpu.progress().instructions == completed,
            "Every timing transition is backed by actual Core retirement");
    if (completed == end) { anchors.at(completed - 1) = memory.clock(); }
  }
  for (const auto &window : windows) {
    require(anchors.at(window.last) - anchors.at(window.first) == window.clocks,
            "Actual native/staged execution retains the independent NXP quiet-anchor window");
  }
  const std::array<unsigned, windows.size()> expected_outputs{1, 1, 1, 0};
  require(memory.output_count() == windows.size() && memory.outputs() == expected_outputs &&
              stalls == 1,
          "Four real peripheral writes and exactly the one independently required stall");
  require(groups != 0 && cpu.jit_statistics().native_alu_instructions != 0,
          "The dependency owner preserves actual native multi-instruction execution");
  constexpr unsigned nominal_cycles = 115;
  require(cpu.progress().estimated_cycles == nominal_cycles && memory.clock() == nominal_cycles + 1,
          "Independent base-cost total stays distinct from actual dependency clocks");
  constexpr unsigned expected_status = 0xc00310;
  constexpr unsigned expected_omr = 0x305;
  constexpr std::uint64_t inserted_bit = std::uint64_t{1} << 24;
  const auto registers = cpu.registers();
  require(registers.pc == program.size() && registers.a == 0 && registers.b == inserted_bit &&
              registers.x == 1 && registers.sr == expected_status && registers.omr == expected_omr,
          "Independent normal-NXP register/status values accompany the timing expectations");
}

unsigned timed_step(blofeld::DspNativeTimedExecution &timing, dsp56300::Core &cpu, Memory &memory) {
  constexpr unsigned cycle_budget = 1024;
  const auto actual = timing.run(cpu, memory, {.instructions = 1, .cycles = cycle_budget});
  require(actual.execution.fault.kind == dsp56300::FaultKind::none,
          "A bounded authored step keeps the genuine Core result");
  if (actual.execution.dispatches != 0) {
    require(actual.execution.dispatches == 1,
            "The explicit one-instruction test budget is retained");
    memory.advance_clock(actual.cycles);
    return 0;
  }
  if (!actual.next || !actual.next_cycles) {
    throw std::runtime_error("Missing actual next opcode");
  }
  const auto owned = timing.prepare_owner(cpu, *actual.next, *actual.next_cycles);
  memory.advance_clock(owned.base_units + owned.dependency_units);
  stage(cpu, owned);
  timing.complete_owner(cpu, owned);
  return owned.dependency_units;
}

void checkpoint_and_mutation() {
  Memory memory;
  dsp56300::Core cpu(memory);
  blofeld::DspNativeTimedExecution timing(cpu.progress());
  constexpr unsigned producer_index = 21;
  constexpr unsigned consumer_index = 22;
  for (unsigned index = 0; index <= producer_index; ++index) {
    static_cast<void>(timed_step(timing, cpu, memory));
  }
  const auto saved_cpu = cpu.checkpoint();
  const auto saved_timing = timing;
  const auto saved_memory = memory;
  require(timed_step(timing, cpu, memory) == 1 && memory.output_count() == 1 &&
              memory.outputs().front() == 1,
          "The actual dependent MOVEP needs one clock and writes the INSERT result");
  const auto expected_registers = cpu.registers();
  const auto expected_progress = cpu.progress();
  cpu.restore(saved_cpu);
  timing = saved_timing;
  memory = saved_memory;
  require(timed_step(timing, cpu, memory) == 1 && cpu.registers() == expected_registers &&
              cpu.progress() == expected_progress,
          "A real Core checkpoint and copied timing history retain the independent dependency");
  cpu.restore(saved_cpu);
  timing = saved_timing;
  memory = saved_memory;
  constexpr unsigned movep_b1 = 0x04cd90;
  memory.patch(starts.at(consumer_index), movep_b1);
  require(
      timed_step(timing, cpu, memory) == 0 && memory.output_count() == 1 &&
          memory.outputs().front() == 0,
      "A real same-cost opcode replacement reads B1, with no fitted GPIO delay or stale metadata");
}

void ownership_guards() {
  Memory memory;
  dsp56300::Core cpu(memory);
  blofeld::DspNativeTimedExecution timing(cpu.progress());
  const auto initial = cpu.progress();
  const auto empty = timing.run(cpu, memory, {});
  require(empty.execution.dispatches == 0 && cpu.progress() == initial &&
              memory.output_count() == 0,
          "Zero execution budgets retain real CPU and peripheral state");
  const auto outside = cpu.run(1);
  require(outside.dispatches == 1, "The guard observes an actual untracked Core instruction");
  bool refused = false;
  try {
    static_cast<void>(timing.run(cpu, memory, {.instructions = 1, .cycles = 2}));
  } catch (const blofeld::Fault &) { refused = true; }
  require(refused && cpu.progress() == outside.progress,
          "Untracked execution refuses before another CPU instruction or fabricated history");
  cpu.reset();
  timing.reset(cpu.progress());
  require(timed_step(timing, cpu, memory) == 0 && cpu.progress().instructions == 1,
          "Only a real reset plus fresh timing history restarts the authored program");
}

void repeat_latch() {
  constexpr unsigned entry = 0x100;
  constexpr unsigned jump = 0x0c0100;
  constexpr unsigned repeat_three = 0x0603a0;
  constexpr unsigned increment_b = 0x000009;
  std::array<unsigned, entry + 3> code{};
  code.front() = jump;
  code.at(entry) = repeat_three;
  Memory memory(code);
  dsp56300::Core cpu(memory);
  const auto boot = cpu.run(2);
  require(boot.dispatches == 2 && cpu.registers().pc == entry + 1 && cpu.registers().lc == 3,
          "A real reset JMP and REP establish the authored repeat context");
  blofeld::DspNativeTimedExecution timing(cpu.progress());
  constexpr unsigned budget = 64;
  const auto first = timing.run(cpu, memory, {.instructions = budget, .cycles = budget});
  require(first.execution.dispatches == 1 && first.cycles == 1 && cpu.registers().pc == entry + 1,
          "The first unlatched repeat body cannot consume a sequential successor");
  memory.patch(entry + 1, increment_b);
  const auto remaining = timing.run(cpu, memory, {.instructions = budget, .cycles = budget});
  require(remaining.execution.dispatches == 2 && remaining.cycles == 2 &&
              cpu.registers().pc == entry + 2 && cpu.registers().b == 0,
          "Two actual native NOP repetitions retain the real Core latch after a P-word mutation");
  constexpr unsigned expected_retirements = 5;
  constexpr unsigned expected_base_cycles = 11;
  require(cpu.progress().instructions == expected_retirements &&
              cpu.progress().estimated_cycles == expected_base_cycles,
          "JMP3 + REP5 + three NOPs is independent of the later program bytes");
}

void loop_endpoints() {
  constexpr unsigned entry = 0x100;
  constexpr unsigned jump = 0x0c0100;
  constexpr unsigned relative_loop_three = 0x060390;
  constexpr unsigned end_displacement = 3;
  std::array<unsigned, entry + 5> code{};
  code.front() = jump;
  code.at(entry) = relative_loop_three;
  code.at(entry + 1) = end_displacement;
  Memory memory(code);
  dsp56300::Core cpu(memory);
  const auto boot = cpu.run(2);
  require(boot.dispatches == 2 && cpu.registers().pc == entry + 2 && cpu.registers().lc == 3,
          "Independently assembled DOR starts a three-iteration, two-NOP body");
  blofeld::DspNativeTimedExecution timing(cpu.progress());
  constexpr unsigned repetitions = 3;
  constexpr unsigned budget = 64;
  for (unsigned iteration = 0; iteration < repetitions; ++iteration) {
    const auto actual = timing.run(cpu, memory, {.instructions = budget, .cycles = budget});
    require(actual.execution.dispatches == 2 && actual.cycles == 2,
            "The timed native block ends at the actual DO endpoint on every iteration");
    const auto expected_pc = iteration + 1 < repetitions ? entry + 2 : entry + 4;
    require(cpu.registers().pc == expected_pc,
            "Native loop control, not a host PC write, selects the next block");
  }
  constexpr unsigned expected_retirements = 8;
  constexpr unsigned expected_base_cycles = 14;
  constexpr unsigned loop_flag = 1U << 15;
  require(cpu.progress().instructions == expected_retirements &&
              cpu.progress().estimated_cycles == expected_base_cycles && cpu.registers().sp == 0 &&
              cpu.registers().lc == 0 && (cpu.registers().sr & loop_flag) == 0,
          "JMP3 + DOR5 + six NOPs restores the real outer loop and stack state");
}

void explicit_control_tail() {
  constexpr unsigned jump_to_six = 0x0c0006;
  constexpr unsigned generous_budget = 64;
  const std::array<unsigned, 8> code{0, jump_to_six, 0, 0, 0, 0, 0, 0};
  Memory memory(code);
  dsp56300::Core cpu(memory);
  blofeld::DspNativeTimedExecution timing(cpu.progress());
  const auto warm = timing.run(cpu, memory, {.instructions = 1, .cycles = 1});
  require(warm.execution.dispatches == 1 && cpu.registers().pc == 1,
          "A real NOP warms its cost before the nonsequential control test");
  const auto before = cpu.jit_statistics();
  const auto actual =
      timing.run(cpu, memory, {.instructions = generous_budget, .cycles = generous_budget});
  require(actual.execution.dispatches == 1 && actual.cycles == 3 && cpu.registers().pc == 6,
          "A short native JMP terminates the timing plan before any target instructions");
  require(cpu.jit_statistics().helper_instructions == before.helper_instructions,
          "A qualified control tail retains native execution rather than forcing a slow helper");
}

void captured_offer_guard() {
  Memory memory;
  dsp56300::Core cpu(memory);
  blofeld::DspNativeTimedExecution timing(cpu.progress());
  const auto actual = timing.run(cpu, memory, {});
  if (!actual.next || !actual.next_cycles) {
    throw std::runtime_error("Missing actual owner offer");
  }
  auto changed_opcode = *actual.next;
  auto changed_literal = *actual.next;
  constexpr unsigned substituted_jump = 0x0c0006;
  changed_opcode.opcode = substituted_jump;
  if (!changed_literal.extension) { throw std::runtime_error("Missing authored immediate"); }
  changed_literal.extension = *changed_literal.extension ^ 1U;
  const auto before = cpu.progress();
  const std::array forged{
      blofeld::DspTimedInstruction{
          .words = changed_opcode, .base_units = *actual.next_cycles, .dependency_units = 0},
      blofeld::DspTimedInstruction{
          .words = changed_literal, .base_units = *actual.next_cycles, .dependency_units = 0},
      blofeld::DspTimedInstruction{
          .words = *actual.next, .base_units = *actual.next_cycles + 1, .dependency_units = 0}};
  for (const auto &attempt : forged) {
    bool refused = false;
    try {
      static_cast<void>(timing.prepare_owner(cpu, attempt.words, attempt.base_units));
    } catch (const blofeld::Fault &) { refused = true; }
    require(refused && cpu.progress() == before && memory.output_count() == 0,
            "The owner cannot substitute opcode, extension or base cost for the actual Core offer");
  }
  const auto legitimate = timing.prepare_owner(cpu, *actual.next, *actual.next_cycles);
  stage(cpu, legitimate);
  timing.complete_owner(cpu, legitimate);
  require(cpu.progress().instructions == 1,
          "Refused substitution preserves the genuine captured instruction and its retirement");
}
}

int main() {
  try {
    reference_windows();
    checkpoint_and_mutation();
    ownership_guards();
    repeat_latch();
    loop_endpoints();
    explicit_control_tail();
    captured_offer_guard();
    std::cout << "PASS timed native prefixes: " << assertions << " checks\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
