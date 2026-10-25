// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hc12_minmax_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace blofeld::test {
namespace {
constexpr unsigned page_prefix = 0x18;
constexpr unsigned initial_x = 0x1357;
constexpr std::array seeds{0U, 0xffU, 0x5aU, 0xa5U};
enum class Accumulator : std::uint8_t { none, a, b, d };

struct Mode {
  std::array<std::uint8_t, 3> operand{};
  unsigned bytes = 1;
  unsigned group = 0;
  unsigned reg = 0;
  int offset = 0;
  Accumulator accumulator = Accumulator::none;
  int update = 0;
  bool pre = false;
  bool indirect = false;
};

constexpr std::array modes{
    Mode{.operand = {0}},
    Mode{.operand = {0x40}, .reg = 1},
    Mode{.operand = {0x80}, .reg = 2},
    Mode{.operand = {0xc0}, .reg = 3},
    Mode{.operand = {0xd0}, .reg = 3, .offset = -16},
    Mode{.operand = {0x20}, .update = 1, .pre = true},
    Mode{.operand = {0x2f}, .update = -1, .pre = true},
    Mode{.operand = {0x37}, .update = 8},
    Mode{.operand = {0x38}, .update = -8},
    Mode{.operand = {0x60}, .reg = 1, .update = 1, .pre = true},
    Mode{.operand = {0xaf}, .reg = 2, .update = -1, .pre = true},
    Mode{.operand = {0x77}, .reg = 1, .update = 8},
    Mode{.operand = {0xb8}, .reg = 2, .update = -8},
    Mode{.operand = {0xe4}, .accumulator = Accumulator::a},
    Mode{.operand = {0xed}, .reg = 1, .accumulator = Accumulator::b},
    Mode{.operand = {0xf6}, .reg = 2, .accumulator = Accumulator::d},
    Mode{.operand = {0xfc}, .reg = 3, .accumulator = Accumulator::a},
    Mode{.operand = {0xfd}, .reg = 3, .accumulator = Accumulator::b},
    Mode{.operand = {0xfe}, .reg = 3, .accumulator = Accumulator::d},
    Mode{.operand = {0xe0, 0x81}, .bytes = 2, .group = 1, .offset = 129},
    Mode{.operand = {0xe1, 0x81}, .bytes = 2, .group = 1, .offset = -127},
    Mode{.operand = {0xf8, 0x81}, .bytes = 2, .group = 1, .reg = 3, .offset = 129},
    Mode{.operand = {0xf9, 0x81}, .bytes = 2, .group = 1, .reg = 3, .offset = -127},
    Mode{.operand = {0xe2, 1, 0x23}, .bytes = 3, .group = 2, .offset = 291},
    Mode{.operand = {0xf2, 0xfe, 0xdc}, .bytes = 3, .group = 2, .reg = 2, .offset = -292},
    Mode{.operand = {0xfa, 1, 0x23}, .bytes = 3, .group = 2, .reg = 3, .offset = 291},
    Mode{.operand = {0xfa, 0xfe, 0xdc}, .bytes = 3, .group = 2, .reg = 3, .offset = -292},
    Mode{.operand = {0xe7}, .group = 3, .accumulator = Accumulator::d, .indirect = true},
    Mode{.operand = {0xff}, .group = 3, .reg = 3, .accumulator = Accumulator::d, .indirect = true},
    Mode{.operand = {0xe3, 1, 0x23}, .bytes = 3, .group = 4, .offset = 291, .indirect = true},
    Mode{.operand = {0xfb, 0xfe, 0xdc},
         .bytes = 3,
         .group = 4,
         .reg = 3,
         .offset = -292,
         .indirect = true}};

unsigned wrap(const int value) {
  return static_cast<unsigned>(value + static_cast<int>(minmax_word_mask + 1)) & minmax_word_mask;
}

void assign_base(CpuState &state, const Mode mode, const unsigned value) {
  switch (mode.reg) {
    case 0: state.x = static_cast<std::uint16_t>(value); break;
    case 1: state.y = static_cast<std::uint16_t>(value); break;
    case 2: state.sp = static_cast<std::uint16_t>(value); break;
    case 3: break;
    default: throw Fault("Invalid own min/max index register");
  }
}

int displacement(const Mode mode, const unsigned d) {
  if (mode.update != 0) { return mode.pre ? mode.update : 0; }
  switch (mode.accumulator) {
    case Accumulator::none: return mode.offset;
    case Accumulator::a: return static_cast<int>(d >> 8U);
    case Accumulator::b: return static_cast<int>(d & minmax_byte_mask);
    case Accumulator::d: return static_cast<int>(d);
    default: throw Fault("Invalid own min/max accumulator");
  }
}

struct Prepared {
  CpuState before;
  CpuState after;
  std::optional<unsigned> pointer;
  unsigned bytes;
};

Prepared prepare(MinmaxMemory &memory,
                 const Mode mode,
                 const MinmaxOperation operation,
                 const MinmaxValues values,
                 const unsigned alignment) {
  std::array<std::uint8_t, 5> code{page_prefix, static_cast<std::uint8_t>(operation.opcode)};
  std::ranges::copy(std::span(mode.operand).first(mode.bytes), code.begin() + 2);
  const auto length = mode.bytes + 2;
  memory.install(std::span(code).first(length), alignment);
  const auto d = minmax_accumulator(operation, values.left);
  CpuState before{.pc = static_cast<std::uint16_t>(minmax_entry + alignment),
                  .d = static_cast<std::uint16_t>(d),
                  .x = initial_x,
                  .y = minmax_initial_y,
                  .sp = minmax_initial_stack,
                  .ccr = static_cast<std::uint8_t>(values.flags)};
  const auto offset = displacement(mode, d);
  const auto desired = mode.indirect ? minmax_pointer : minmax_source;
  const auto base = mode.reg == 3 ? before.pc + length : wrap(static_cast<int>(desired) - offset);
  assign_base(before, mode, base);
  const auto address = wrap(static_cast<int>(base) + offset);
  std::optional<unsigned> pointer;
  memory.source = address;
  if (mode.indirect) {
    pointer = address;
    memory.source = minmax_source;
    memory.bytes.at(address) = minmax_source >> 8U;
    memory.bytes.at(address + 1) = minmax_source & minmax_byte_mask;
  }
  memory.operand(values.right, operation.word);
  auto after = before;
  after.pc = static_cast<std::uint16_t>(before.pc + length);
  assign_base(after, mode, wrap(static_cast<int>(base) + mode.update));
  return {.before = before, .after = after, .pointer = pointer, .bytes = length};
}

std::string_view slots(const Mode mode, const MinmaxOperation operation) {
  // Literal HCS12COREUG instruction-table rows, independent of the production plan.
  constexpr std::array<std::array<std::string_view, 4>, 5> table{
      {{"OrPf", "OrPw", "ORPf", "ORPW"},
       {"OrPO", "OrPwO", "ORPO", "ORPWO"},
       {"OfrPP", "OfrPwP", "OfRPP", "OfRPWP"},
       {"OfIfrPf", "OfIfrPw", "OfIfRPf", "OfIfRPW"},
       {"OfIPrPf", "OfIPrPw", "OfIPRPf", "OfIPRPW"}}};
  return table.at(mode.group).at((operation.word ? 2U : 0U) + (operation.memory ? 1U : 0U));
}

void check_clocks(const std::vector<Hc12BusCycle> &clocks,
                  const MinmaxMemory &memory,
                  const Prepared &prepared,
                  const std::string_view expected) {
  require_minmax(clocks.size() == expected.size(), "Min/max literal cycle count differs");
  for (unsigned index = 0; index < clocks.size(); ++index) {
    const auto &cycle = clocks.at(index);
    require_minmax(cycle.kind == expected[index] && cycle.context.pc == prepared.before.pc &&
                       cycle.context.ccr == prepared.before.ccr &&
                       cycle.word_part == Hc12WordPart::whole,
                   "Min/max manual slot, context and logical word phases must agree");
    if (cycle.kind == 'I') {
      require_minmax(cycle.address == prepared.pointer && cycle.size == 2,
                     "Min/max pointer read must use its independently calculated word address");
    } else if (cycle.kind == 'r' || cycle.kind == 'w' || cycle.kind == 'R' || cycle.kind == 'W') {
      const unsigned width = cycle.kind == 'R' || cycle.kind == 'W' ? 2U : 1U;
      require_minmax(cycle.address == memory.source && cycle.size == width,
                     "Min/max native operand must retain manual width and address");
    } else {
      require_minmax(!cycle.address && cycle.size == 0,
                     "Min/max cannot invent prefetch data addresses");
    }
  }
}

void check_one_timed(MinmaxMemory &memory,
                     Hc12 &cpu,
                     std::vector<Hc12BusCycle> &clocks,
                     const Mode mode,
                     const MinmaxOperation operation,
                     const MinmaxValues values,
                     const unsigned alignment) {
  const auto prepared = prepare(memory, mode, operation, values, alignment);
  clocks.clear();
  cpu.set_state(prepared.before);
  cpu.step();
  check_minmax_result(memory, cpu, operation, values, mode.indirect ? 1U : 0U);
  const auto state = cpu.state();
  require_minmax(state.pc == prepared.after.pc && state.x == prepared.after.x &&
                     state.y == prepared.after.y && state.sp == prepared.after.sp,
                 "Min/max must apply one native index update and the full instruction length");
  if (prepared.pointer) {
    require_minmax(memory.accesses[0] == MinmaxAccess{.address = *prepared.pointer,
                                                      .size = 2,
                                                      .value = minmax_source},
                   "Min/max indirect operand has exactly one preceding pointer read");
  }
  check_clocks(clocks, memory, prepared, slots(mode, operation));
}
}

unsigned minmax_word_cases() {
  constexpr std::array<unsigned, 16> values{0,
                                            1,
                                            2,
                                            0x7f,
                                            0x80,
                                            0xff,
                                            0x100,
                                            0x101,
                                            0x7ffe,
                                            0x7fff,
                                            0x8000,
                                            0x8001,
                                            0xff00,
                                            0xfffe,
                                            0xffff,
                                            0x5555};
  MinmaxMemory memory;
  Hc12 cpu(memory, BLOFELD_DEFAULT_SLA);
  unsigned cases = 0;
  for (const auto operation : minmax_operations) {
    if (!operation.word) { continue; }
    const std::array<std::uint8_t, 3> code{
        page_prefix, static_cast<std::uint8_t>(operation.opcode), 0};
    memory.install(code);
    for (const auto left : values) {
      for (const auto right : values) {
        for (const auto flags : seeds) {
          memory.operand(right, true);
          cpu.set_state({.pc = minmax_entry,
                         .d = static_cast<std::uint16_t>(left),
                         .x = minmax_source,
                         .y = minmax_initial_y,
                         .sp = minmax_initial_stack,
                         .ccr = static_cast<std::uint8_t>(flags)});
          cpu.step();
          check_minmax_result(
              memory, cpu, operation, {.left = left, .right = right, .flags = flags});
          require_minmax(cpu.state().pc == minmax_entry + code.size() &&
                             cpu.state().x == minmax_source && cpu.state().y == minmax_initial_y &&
                             cpu.state().sp == minmax_initial_stack,
                         "Word min/max must retain registers and its instruction length");
          ++cases;
        }
      }
    }
  }
  return cases;
}

namespace {
unsigned timed_flags(MinmaxMemory &memory,
                     Hc12 &cpu,
                     std::vector<Hc12BusCycle> &clocks,
                     const Mode mode,
                     const MinmaxOperation operation,
                     const MinmaxValues values) {
  unsigned cases = 0;
  for (const auto flags : seeds) {
    auto item = values;
    item.flags = flags;
    for (const auto alignment : {0U, 1U}) {
      check_one_timed(memory, cpu, clocks, mode, operation, item, alignment);
      ++cases;
    }
  }
  return cases;
}

unsigned timed_inputs(MinmaxMemory &memory,
                      Hc12 &cpu,
                      std::vector<Hc12BusCycle> &clocks,
                      const Mode mode,
                      const MinmaxOperation operation) {
  constexpr std::array<unsigned, 5> inputs{0, 1, 0x7f, 0x80, 0xff};
  unsigned cases = 0;
  for (const auto left : inputs) {
    for (const auto right : inputs) {
      constexpr unsigned repeated_byte = 0x101;
      const MinmaxValues values{.left = operation.word ? left << 8U : left,
                                .right = operation.word ? right * repeated_byte : right,
                                .flags = 0};
      cases += timed_flags(memory, cpu, clocks, mode, operation, values);
    }
  }
  return cases;
}
}

unsigned minmax_timing_cases() {
  MinmaxMemory memory;
  std::vector<Hc12BusCycle> clocks;
  Hc12CycleBus bus(memory, [&clocks](const auto cycle) { clocks.push_back(cycle); });
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  unsigned cases = 0;
  for (const auto mode : modes) {
    for (const auto operation : minmax_operations) {
      cases += timed_inputs(memory, cpu, clocks, mode, operation);
    }
  }
  return cases;
}
}
