// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
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
using namespace test::hcs08;
constexpr unsigned stack = 0x240, index = 0x334, accumulator = 0x7f, displacement = 4;
constexpr unsigned data_value = 0x81, sign_bit = 0x80, vnz = 0x86, vnzc = 0x87, arithmetic = 0x97;
constexpr unsigned known_base = 0xec, initial_flags = 0xec, flag_variants = 8;
constexpr unsigned jump_target = 0x9000, high_mask = 0xff00, unknown_half = 0xef, nibble_mask = 15;
constexpr int signed_minimum = -128, signed_maximum = 127;

enum class Kind : std::uint8_t {
  bit_set,
  higher,
  lower_same,
  unequal,
  jump,
  push_a,
  push_x,
  pull_a,
  load_a,
  compare_indexed,
  load_x,
  compare_stack,
  shift,
  rotate,
  add
};

struct Operation {
  Kind kind;
  std::array<std::uint8_t, 3> code;
  unsigned length, defined;
};

// Own single-instruction programs; opcodes/effects come from Family details.
constexpr std::array<Operation, 15> operations{
    {{.kind = Kind::bit_set, .code = {0x0e, operand, displacement}, .length = 3, .defined = flag_c},
     {.kind = Kind::higher, .code = {0x22, displacement, 0}, .length = 2, .defined = 0},
     {.kind = Kind::lower_same, .code = {0x23, displacement, 0}, .length = 2, .defined = 0},
     {.kind = Kind::unequal, .code = {0x26, displacement, 0}, .length = 2, .defined = 0},
     {.kind = Kind::jump, .code = {0xcc, 0x90, 0}, .length = 3, .defined = 0},
     {.kind = Kind::push_a, .code = {0x87, 0, 0}, .length = 1, .defined = 0},
     {.kind = Kind::push_x, .code = {0x89, 0, 0}, .length = 1, .defined = 0},
     {.kind = Kind::pull_a, .code = {0x86, 0, 0}, .length = 1, .defined = 0},
     {.kind = Kind::load_a, .code = {0xe6, displacement, 0}, .length = 2, .defined = vnz},
     {.kind = Kind::compare_indexed, .code = {0xf1, 0, 0}, .length = 1, .defined = vnzc},
     {.kind = Kind::load_x, .code = {0x9e, 0xee, displacement}, .length = 3, .defined = vnz},
     {.kind = Kind::compare_stack,
      .code = {0x9e, 0xe1, displacement},
      .length = 3,
      .defined = vnzc},
     {.kind = Kind::shift, .code = {0x9e, 0x68, displacement}, .length = 3, .defined = vnzc},
     {.kind = Kind::rotate, .code = {0x9e, 0x69, displacement}, .length = 3, .defined = vnzc},
     {.kind = Kind::add, .code = {0x9e, 0xeb, displacement}, .length = 3, .defined = arithmetic}}};

unsigned data_address(const Kind kind) {
  switch (kind) {
    case Kind::bit_set: return operand;
    case Kind::pull_a: return stack + 1;
    case Kind::load_a: return index + displacement;
    case Kind::compare_indexed: return index;
    case Kind::higher:
    case Kind::lower_same:
    case Kind::unequal:
    case Kind::jump:
    case Kind::push_a:
    case Kind::push_x:
    case Kind::load_x:
    case Kind::compare_stack:
    case Kind::shift:
    case Kind::rotate:
    case Kind::add: return stack + displacement;
    default: throw Fault("Invalid own opcode kind");
  }
}

bool branch(const Kind kind, const unsigned flags) {
  switch (kind) {
    case Kind::higher: return (flags & (flag_c | flag_z)) == 0;
    case Kind::lower_same: return (flags & (flag_c | flag_z)) != 0;
    case Kind::unequal: return (flags & flag_z) == 0;
    case Kind::bit_set:
    case Kind::jump:
    case Kind::push_a:
    case Kind::push_x:
    case Kind::pull_a:
    case Kind::load_a:
    case Kind::compare_indexed:
    case Kind::load_x:
    case Kind::compare_stack:
    case Kind::shift:
    case Kind::rotate:
    case Kind::add:
    default: throw Fault("Own branch expectation requires a conditional branch");
  }
}

// Enumerate every possible CCR completion instead of duplicating admission logic.
std::optional<unsigned> resolved(const Hcs08State before, const auto &projection) {
  std::optional<unsigned> result;
  for (unsigned flags = 0; flags < byte_values; ++flags) {
    if ((flags & before.ccr_known) != before.ccr) { continue; }
    const auto value = projection(flags);
    if (result && *result != value) { return {}; }
    result = value;
  }
  require(result.has_value(), "Own CCR input must have at least one valid completion");
  return result;
}

unsigned nz(const unsigned byte) {
  return (byte == 0 ? flag_z : 0U) | ((byte & sign_bit) != 0 ? flag_n : 0U);
}

int signed_byte(const unsigned byte) {
  return byte < sign_bit ? static_cast<int>(byte)
                         : static_cast<int>(byte) - static_cast<int>(byte_values);
}

unsigned arithmetic_flags(const unsigned a, const unsigned b, const bool add) {
  const int signed_result = add ? signed_byte(a) + signed_byte(b) : signed_byte(a) - signed_byte(b);
  const unsigned wide = add ? a + b : a + byte_values - b;
  unsigned flags = nz(wide & byte_mask);
  if (signed_result < signed_minimum || signed_result > signed_maximum) { flags |= flag_v; }
  if (add && (a & nibble_mask) + (b & nibble_mask) > nibble_mask) { flags |= flag_h; }
  if (add ? wide >= byte_values : a < b) { flags |= flag_c; }
  return flags;
}

struct Expected {
  Hcs08State cpu;
  std::vector<Access> accesses;
  unsigned status = 0;
};

std::optional<Expected>
control_effect(const Operation operation, const Hcs08State before, const unsigned data) {
  Expected expected{.cpu = before, .accesses = {}};
  expected.cpu.pc = static_cast<std::uint16_t>(entry + operation.length);
  const auto address = data_address(operation.kind);
  switch (operation.kind) {
    case Kind::higher:
    case Kind::lower_same:
    case Kind::unequal: {
      const auto target = resolved(before, [&](const unsigned flags) {
        return entry + operation.length + (branch(operation.kind, flags) ? displacement : 0U);
      });
      if (!target) { return {}; }
      expected.cpu.pc = static_cast<std::uint16_t>(*target);
      break;
    }
    case Kind::jump: expected.cpu.pc = jump_target; break;
    case Kind::push_a:
    case Kind::push_x: {
      const unsigned value = operation.kind == Kind::push_a ? before.a : before.hx & byte_mask;
      expected.accesses.push_back({.address = stack, .value = value, .write = true});
      --expected.cpu.sp;
      break;
    }
    case Kind::bit_set:
      expected.accesses.push_back({.address = address, .value = data, .write = false});
      if ((data & sign_bit) != 0) {
        expected.cpu.pc += displacement;
        expected.status = flag_c;
      }
      break;
    case Kind::pull_a:
    case Kind::load_a:
    case Kind::compare_indexed:
    case Kind::load_x:
    case Kind::compare_stack:
    case Kind::shift:
    case Kind::rotate:
    case Kind::add:
      expected.accesses.push_back({.address = address, .value = data, .write = false});
      break;
    default: throw Fault("Invalid own control-effect kind");
  }
  return expected;
}

std::optional<Expected>
expectation(const Operation operation, const Hcs08State before, const unsigned data) {
  const auto control = control_effect(operation, before, data);
  if (!control) { return {}; }
  auto expected = *control;
  const auto address = data_address(operation.kind);
  unsigned status = expected.status;
  switch (operation.kind) {
    case Kind::pull_a:
      ++expected.cpu.sp;
      expected.cpu.a = static_cast<std::uint8_t>(data);
      break;
    case Kind::load_a:
      expected.cpu.a = static_cast<std::uint8_t>(data);
      status = nz(data);
      break;
    case Kind::load_x:
      expected.cpu.hx = static_cast<std::uint16_t>((before.hx & high_mask) | data);
      status = nz(data);
      break;
    case Kind::compare_stack:
    case Kind::compare_indexed: status = arithmetic_flags(before.a, data, false); break;
    case Kind::add:
      expected.cpu.a = static_cast<std::uint8_t>((before.a + data) & byte_mask);
      status = arithmetic_flags(before.a, data, true);
      break;
    case Kind::shift:
    case Kind::rotate: {
      const auto value = resolved(before, [&](const unsigned flags) {
        const unsigned incoming = operation.kind == Kind::rotate ? flags & flag_c : 0;
        return ((data << 1U) | incoming) & byte_mask;
      });
      if (!value) { return {}; }
      status = nz(*value);
      const bool carry = (data & sign_bit) != 0;
      if (carry) { status |= flag_c; }
      if (carry != ((*value & sign_bit) != 0)) { status |= flag_v; }
      expected.accesses.push_back({.address = address, .value = *value, .write = true});
      break;
    }
    case Kind::bit_set:
    case Kind::higher:
    case Kind::lower_same:
    case Kind::unequal:
    case Kind::jump:
    case Kind::push_a:
    case Kind::push_x: break;
    default: throw Fault("Invalid own data-effect kind");
  }
  expected.cpu.ccr_known = static_cast<std::uint8_t>(before.ccr_known | operation.defined);
  expected.cpu.ccr = static_cast<std::uint8_t>(((before.ccr & ~operation.defined) | status) &
                                               expected.cpu.ccr_known);
  return expected;
}

void check(Memory &memory,
           Hcs08 &cpu,
           const Operation operation,
           const Hcs08State supplied,
           const unsigned data) {
  const auto address = data_address(operation.kind);
  memory.bytes[address] = static_cast<std::uint8_t>(data);
  memory.known.set(address);
  memory.accesses.clear();
  cpu.set_state(supplied);
  const auto before = cpu.state();
  const auto retired = memory.retired;
  const auto expected = expectation(operation, before, data);
  if (!expected) {
    require(rejected([&] { cpu.step(); }).contains("partial-CCR") && cpu.faulted() &&
                cpu.state() == before && memory.retired == retired && memory.accesses.empty(),
            "Unresolved branch/rotate input refuses before any architectural or RAM effects");
    return;
  }
  cpu.step();
  require(cpu.state() == expected->cpu && memory.accesses == expected->accesses &&
              memory.retired == retired + 1 && !cpu.faulted(),
          "Independent opcode=" + hex(operation.code[0]) + " CCR=" + hex(before.ccr) +
              " known=" + hex(before.ccr_known) + " operand=" + hex(data) +
              " checks complete CPU state, knowledge and ordered memory effects");
}

void flags_and_dependencies() {
  for (const auto operation : operations) {
    Memory memory;
    memory.writable.set();
    memory.install(entry, operation.code);
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
    for (unsigned subset = 0; subset < flag_variants; ++subset) {
      const unsigned known = known_base | (subset & 3U) | ((subset & 4U) != 0 ? flag_h : 0U);
      for (unsigned seed = 0; seed < flag_variants; ++seed) {
        const unsigned flags = initial_flags | (seed & 3U) | ((seed & 4U) != 0 ? flag_h : 0U);
        check(memory,
              cpu,
              operation,
              {.pc = entry,
               .hx = index,
               .sp = stack,
               .a = accumulator,
               .ccr_known = static_cast<std::uint8_t>(known),
               .ccr = static_cast<std::uint8_t>(flags)},
              data_value);
        if (operation.kind == Kind::bit_set || operation.kind == Kind::pull_a ||
            operation.kind == Kind::load_a || operation.kind == Kind::load_x) {
          check(memory,
                cpu,
                operation,
                {.pc = entry,
                 .hx = index,
                 .sp = stack,
                 .a = accumulator,
                 .ccr_known = static_cast<std::uint8_t>(known),
                 .ccr = static_cast<std::uint8_t>(flags)},
                0);
        }
      }
    }
  }
}

void arithmetic_boundaries() {
  constexpr std::array<unsigned, 7> values{0, 1, 15, 16, 127, 128, 255};
  for (const auto operation : operations) {
    if (operation.kind != Kind::add && operation.kind != Kind::compare_stack &&
        operation.kind != Kind::compare_indexed && operation.kind != Kind::shift &&
        operation.kind != Kind::rotate) {
      continue;
    }
    Memory memory;
    memory.writable.set();
    memory.install(entry, operation.code);
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
    for (const auto a : values) {
      for (const auto data : values) {
        for (const unsigned carry : {0U, flag_c}) {
          check(memory,
                cpu,
                operation,
                {.pc = entry,
                 .hx = index,
                 .sp = stack,
                 .a = static_cast<std::uint8_t>(a),
                 .ccr_known = unknown_half,
                 .ccr = static_cast<std::uint8_t>(flags_fixed | flag_i | carry)},
                data);
        }
      }
    }
  }
}

void missing_operand() {
  Memory memory;
  constexpr std::array<std::uint8_t, 3> add{0x9e, 0xeb, displacement};
  memory.install(entry, add);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = entry,
                 .hx = index,
                 .sp = stack,
                 .a = accumulator,
                 .ccr_known = unknown_half,
                 .ccr = flags_fixed | flag_i});
  const auto before = cpu.state();
  require(rejected([&] { cpu.step(); }).contains("read address=0x0244") && cpu.faulted() &&
              cpu.state() == before && memory.retired == 0 && memory.accesses.empty(),
          "A failed actual operand read cannot define H or retire a prefixed ADD");
}
} // namespace

int main() {
  try {
    flags_and_dependencies();
    arithmetic_boundaries();
    missing_operand();
    std::cout << "HCS08 ADC-path partial CCR dependencies and native effects pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
