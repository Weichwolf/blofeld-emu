// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned daa = 0x72, add = 0xab, adc = 0xa9, stack = 0x240, index = 0x9345;
constexpr unsigned decimal_limit = 100, decimal_base = 10, nibble_bits = 4, nibble_mask = 15;
constexpr unsigned decimal_carry = 0x60, both_corrections = 0x66, low_correction = 6;

struct Row {
  unsigned c, high_first, high_last, h, low_first, low_last, correction, carry;
};

// Independent literal transcription of Family RM Rev2 printed247 Table A-2.
constexpr std::array<Row, 9> rows{{{.c = 0,
                                    .high_first = 0,
                                    .high_last = 9,
                                    .h = 0,
                                    .low_first = 0,
                                    .low_last = 9,
                                    .correction = 0,
                                    .carry = 0},
                                   {.c = 0,
                                    .high_first = 0,
                                    .high_last = 8,
                                    .h = 0,
                                    .low_first = 10,
                                    .low_last = 15,
                                    .correction = low_correction,
                                    .carry = 0},
                                   {.c = 0,
                                    .high_first = 0,
                                    .high_last = 9,
                                    .h = 1,
                                    .low_first = 0,
                                    .low_last = 3,
                                    .correction = low_correction,
                                    .carry = 0},
                                   {.c = 0,
                                    .high_first = 10,
                                    .high_last = 15,
                                    .h = 0,
                                    .low_first = 0,
                                    .low_last = 9,
                                    .correction = decimal_carry,
                                    .carry = 1},
                                   {.c = 0,
                                    .high_first = 9,
                                    .high_last = 15,
                                    .h = 0,
                                    .low_first = 10,
                                    .low_last = 15,
                                    .correction = both_corrections,
                                    .carry = 1},
                                   {.c = 0,
                                    .high_first = 10,
                                    .high_last = 15,
                                    .h = 1,
                                    .low_first = 0,
                                    .low_last = 3,
                                    .correction = both_corrections,
                                    .carry = 1},
                                   {.c = 1,
                                    .high_first = 0,
                                    .high_last = 2,
                                    .h = 0,
                                    .low_first = 0,
                                    .low_last = 9,
                                    .correction = decimal_carry,
                                    .carry = 1},
                                   {.c = 1,
                                    .high_first = 0,
                                    .high_last = 2,
                                    .h = 0,
                                    .low_first = 10,
                                    .low_last = 15,
                                    .correction = both_corrections,
                                    .carry = 1},
                                   {.c = 1,
                                    .high_first = 0,
                                    .high_last = 3,
                                    .h = 1,
                                    .low_first = 0,
                                    .low_last = 3,
                                    .correction = both_corrections,
                                    .carry = 1}}};

std::optional<Row> row(const unsigned a, const unsigned flags) {
  std::optional<Row> found;
  for (const auto &candidate : rows) {
    if (((flags & flag_c) != 0) == (candidate.c != 0) &&
        ((flags & flag_h) != 0) == (candidate.h != 0) && a / 16 >= candidate.high_first &&
        a / 16 <= candidate.high_last && a % 16 >= candidate.low_first &&
        a % 16 <= candidate.low_last) {
      require(!found, "Independent decimal table rows cannot overlap");
      found = candidate;
    }
  }
  return found;
}

Hcs08State input(const unsigned a, const unsigned flags) {
  return {.pc = entry,
          .hx = index,
          .sp = stack,
          .a = static_cast<std::uint8_t>(a),
          .ccr = static_cast<std::uint8_t>(flags)};
}

Hcs08State result(Hcs08State state, const unsigned a, const bool carry, const unsigned bytes) {
  state.pc = static_cast<std::uint16_t>(state.pc + bytes);
  state.a = static_cast<std::uint8_t>(a);
  state.ccr = static_cast<std::uint8_t>((state.ccr & (flags_fixed | flag_h | flag_i)) |
                                        (carry ? flag_c : 0U) | (a == 0 ? flag_z : 0U) |
                                        ((a & flag_v) != 0 ? flag_n : 0U));
  state.ccr_known =
      static_cast<std::uint8_t>((state.ccr_known | flag_n | flag_z | flag_c) & ~flag_v);
  state.ccr &= state.ccr_known;
  return state;
}

void table(const std::string &specification) {
  Memory memory;
  constexpr std::array<std::uint8_t, 1> code{daa};
  memory.install(entry, code);
  Hcs08 cpu(memory, specification);
  unsigned accepted = 0;
  unsigned refused = 0;
  for (unsigned a = 0; a < byte_values; ++a) {
    for (unsigned flags = 0; flags < byte_values; ++flags) {
      if ((flags & flags_fixed) != flags_fixed) { continue; }
      const auto before = input(a, flags);
      cpu.set_state(before);
      const auto retired = memory.retired;
      if (const auto expected = row(a, flags)) {
        cpu.step();
        require(cpu.state() == result(before,
                                      (a + expected->correction) & byte_mask,
                                      expected->carry != 0,
                                      1) &&
                    memory.retired == retired + 1 && !cpu.faulted(),
                "DAA must match the independent literal table, flags and preserved registers");
        ++accepted;
      } else {
        const auto reason = rejected([&] { cpu.step(); });
        require(reason.contains("HCS08 DAA input") && reason.contains("pc=0x8000") &&
                    reason.contains("agent=CPU") && cpu.faulted() && cpu.state() == before &&
                    memory.retired == retired,
                "Unspecified decimal input refuses with context before architectural effects");
        ++refused;
      }
      require(memory.accesses.empty(), "Inherent DAA cannot access data memory");
    }
  }
  constexpr unsigned accepted_count = 6144;
  constexpr unsigned refused_count = 10240;
  require(accepted == accepted_count && refused == refused_count,
          "All256A times64CCR:384 legal and640 unspecified A/H/C inputs,16 other flag seeds");
}

unsigned bcd(const unsigned decimal) {
  return ((decimal / decimal_base) << nibble_bits) | (decimal % decimal_base);
}

void sum(Hcs08 &cpu, Memory &memory, const unsigned left, const unsigned right, const unsigned c) {
  const auto initial = input(bcd(left), flags_fixed | flag_v | flag_i | flag_n | flag_z | c);
  cpu.set_state(initial);
  cpu.step();
  const auto binary = cpu.state();
  cpu.step();
  const unsigned decimal = left + right + c;
  auto expected = result(initial, bcd(decimal % decimal_limit), decimal >= decimal_limit, 3);
  expected.ccr = static_cast<std::uint8_t>(
      expected.ccr |
      ((left % decimal_base + right % decimal_base + c) > nibble_mask ? flag_h : 0U));
  require(cpu.state() == expected && binary.pc == entry + 2 && memory.accesses.empty(),
          "Actual native ADD/ADC then DAA must equal independent decimal integer addition");
}

void additions(const std::string &specification) {
  Memory memory;
  Hcs08 cpu(memory, specification);
  unsigned count = 0;
  for (unsigned right = 0; right < decimal_limit; ++right) {
    const std::array<std::uint8_t, 3> code{add, static_cast<std::uint8_t>(bcd(right)), daa};
    memory.install(entry, code);
    for (unsigned left = 0; left < decimal_limit; ++left) {
      sum(cpu, memory, left, right, 0);
      ++count;
    }
    const std::array<std::uint8_t, 3> carry_code{adc, static_cast<std::uint8_t>(bcd(right)), daa};
    memory.install(entry, carry_code);
    for (unsigned left = 0; left < decimal_limit; ++left) {
      for (unsigned c = 0; c < 2; ++c) {
        sum(cpu, memory, left, right, c);
        ++count;
      }
    }
  }
  constexpr unsigned cases = 30000;
  require(count == cases && memory.retired == 2 * cases,
          "10000ADD and20000ADC independent decimal sums execute both native instructions");
}

void missing_inputs(const std::string &specification) {
  Memory memory;
  constexpr std::array<std::uint8_t, 1> code{daa};
  memory.install(entry, code);
  Hcs08 cpu(memory, specification);
  for (unsigned bit = 0; bit < 10; ++bit) {
    auto before = input(0, flags_fixed);
    if (bit < 8) {
      before.a_known = static_cast<std::uint8_t>(byte_mask ^ (1U << bit));
    } else {
      const unsigned flag = bit == 8 ? flag_h : flag_c;
      before.ccr_known = static_cast<std::uint8_t>(byte_mask ^ flag);
    }
    cpu.set_state(before);
    require(rejected([&] { cpu.step(); }).contains("requires known A/H/C") && cpu.faulted() &&
                cpu.state() == before && memory.retired == 0 && memory.accesses.empty(),
            "Every missing actual A/H/C input bit must refuse before native arithmetic");
    require(rejected([&] {
              static_cast<void>(cpu.checkpoint());
            }).contains("healthy completed native boundary"),
            "Faulted decimal execution requires coordinated recovery");
  }
}

void unrelated_knowledge(const std::string &specification) {
  Memory memory;
  constexpr std::array<std::uint8_t, 3> code{daa, 0x29, 0}; // BVS consumes undefined V.
  memory.install(entry, code);
  Hcs08 cpu(memory, specification);
  for (const unsigned mask : {0U, 0xffU, 0xff00U, 0xffffU}) {
    auto before = input(0, flags_fixed);
    before.hx_known = static_cast<std::uint16_t>(mask);
    before.hx &= before.hx_known;
    before.ccr_known = flags_fixed | flag_h | flag_c;
    cpu.set_state(before);
    cpu.step();
    require(cpu.state() == result(before, 0, false, 1),
            "DAA defines N/Z/C, leaves V unknown and preserves unrelated I/H/X knowledge");
    if (mask == word_mask) {
      require(rejected([&] { cpu.step(); }).contains("partial-CCR") && cpu.faulted(),
              "A native branch cannot consume the undefined DAA overflow flag");
    }
  }
}

void checkpoint_sums(const std::string &specification) {
  constexpr unsigned high_result = operand + 1;
  constexpr unsigned low_expected = 0x27;
  constexpr unsigned final_flags = 0x6b;
  constexpr unsigned left_low = 0x58;
  // Own native two-byte BCD program:9958+0069=10027,including decimal carry.
  constexpr std::array<std::uint8_t, 13> code{
      add, 0x69, daa, 0xb7, operand, 0xa6, 0x99, adc, 0, daa, 0xb7, high_result, 0x82};
  for (const unsigned cut : {1U, 2U, 5U, 6U}) {
    Memory memory;
    memory.install(entry, code);
    memory.writable.set(operand);
    memory.writable.set(high_result);
    Hcs08 cpu(memory, specification);
    cpu.set_state(input(left_low, flags_fixed | flag_i));
    for (unsigned n = 0; n < cut; ++n) { cpu.step(); }
    const auto saved = cpu.checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      Memory restored = memory;
      Hcs08 fresh(restored, specification);
      fresh.restore_checkpoint(saved);
      require(fresh.state() == cpu.state(),
              "Fresh CPU owner restores actual DAA boundary and undefined flag knowledge");
      while (!fresh.background_requested()) { fresh.step(); }
      const Hcs08State expected{
          .pc = entry + code.size(), .hx = index, .sp = stack, .a = 0, .ccr = final_flags};
      require(
          fresh.state() == expected && restored.retired == 8 &&
              restored.bytes[operand] == low_expected && restored.bytes[high_result] == 0 &&
              restored.accesses ==
                  std::vector<Access>{{.address = operand, .value = low_expected, .write = true},
                                      {.address = high_result, .value = 0, .write = true}},
          "Eight fresh BCD carry suffixes retain literal state and both actual native stores");
    }
  }
}
}

int main(const int count, const char *const *const arguments) {
  try {
    require(count == 1 || count == 2, "Expected at most one native specification path");
    const std::string specification = count == 2 ? arguments[1] : BLOFELD_HCS08_SLA;
    table(specification);
    additions(specification);
    missing_inputs(specification);
    unrelated_knowledge(specification);
    checkpoint_sums(specification);
    std::cout << "HCS08 DAA:6144 table cases,10240 explicit refusals,30000 native ADD/ADC sums; "
                 "unknown inputs/V consumers,unrelated knowledge,8 fresh BCD carry suffixes\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
