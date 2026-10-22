// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "hc12_cycle_fixture.hpp"
#include "hc12_indexed_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <string_view>

namespace {
using namespace blofeld::test;

constexpr std::array operations{
    IndexedOperation{.opcode = 0xa6, .destination = 0, .word = false, .store = false, .lea = false},
    IndexedOperation{.opcode = 0xec, .destination = 0, .word = true, .store = false, .lea = false},
    IndexedOperation{.opcode = 0x6a, .destination = 0, .word = false, .store = true, .lea = false},
    IndexedOperation{.opcode = 0x6c, .destination = 0, .word = true, .store = true, .lea = false},
    IndexedOperation{.opcode = 0x1a, .destination = 1, .word = true, .store = false, .lea = true},
    IndexedOperation{.opcode = 0x19, .destination = 2, .word = true, .store = false, .lea = true},
    IndexedOperation{.opcode = 0x1b, .destination = 3, .word = true, .store = false, .lea = true}};

bool same_state(const blofeld::CpuState &left, const blofeld::CpuState &right) {
  return left.pc == right.pc && left.d == right.d && left.x == right.x && left.y == right.y &&
         left.sp == right.sp && left.ccr == right.ccr;
}

blofeld::CpuState initial_state(const IndexedInput &input, const unsigned flags) {
  constexpr unsigned initial_x = 0x1357;
  constexpr unsigned initial_y = 0x2468;
  constexpr unsigned initial_sp = 0x3579;
  blofeld::CpuState state{.pc = indexed_entry,
                          .d = static_cast<std::uint16_t>(input.d),
                          .x = initial_x,
                          .y = initial_y,
                          .sp = initial_sp,
                          .ccr = static_cast<std::uint8_t>(flags)};
  if (input.base_register != 3) {
    set_indexed_register(state, {.index = input.base_register, .value = input.base});
  }
  return state;
}

struct ExpectedOperand {
  unsigned address;
  unsigned data;
  unsigned flags;
};

blofeld::CpuState expected_state(const IndexedInput &input,
                                 const IndexedOperation &operation,
                                 const ExpectedOperand operand) {
  auto state = initial_state(input, operand.flags);
  state.pc = static_cast<std::uint16_t>(indexed_entry + 1 + input.bytes);
  if (input.update != 0) {
    set_indexed_register(
        state, {.index = input.base_register, .value = indexed_sum(input.base, input.update)});
  }
  if (operation.lea) {
    set_indexed_register(state, {.index = operation.destination - 1, .value = operand.address});
    return state;
  }
  if (!operation.store) {
    state.d = static_cast<std::uint16_t>(
        operation.word ? operand.data : (operand.data << 8U) | (input.d & indexed_byte_mask));
  }
  constexpr unsigned preserved_flags = 0xf1;
  constexpr unsigned negative = 8;
  constexpr unsigned zero = 4;
  const unsigned sign = operation.word ? 0x8000U : 0x80U;
  state.ccr = static_cast<std::uint8_t>((operand.flags & preserved_flags) |
                                        ((operand.data & sign) != 0 ? negative : 0U) |
                                        (operand.data == 0 ? zero : 0U));
  return state;
}

struct RejectedWord {
  unsigned flags;
  char kind;
};

bool reject_wrapped_word(CycleFixture &fixture,
                         const IndexedInput &input,
                         const IndexedOperation &operation,
                         const RejectedWord point) {
  const auto memory = fixture.memory.bytes;
  const auto slots = indexed_slots(input, operation);
  try {
    fixture.cpu.step();
  } catch (const blofeld::Fault &error) {
    const std::string reason = error.what();
    require_cycle(reason.contains("Unsupported MCU operand range at 0xffff size=2") &&
                      reason.contains("pc=0x8000") &&
                      reason.contains("ccr=" + blofeld::hex(point.flags, 2)) &&
                      fixture.bus.faulted(),
                  "Unqualified wrapping word must fail with its exact range and CPU context");
    require_cycle(fixture.memory.bytes == memory && !fixture.clocks.empty(),
                  "Rejected wrapping word must preserve memory and expose completed clocks");
    const auto failed_slot = slots.find(point.kind);
    require_cycle(failed_slot != std::string_view::npos && fixture.clocks.size() == failed_slot + 1,
                  "Wrapping pointer/data must reject at its independently expected bus slot");
    for (unsigned index = 0; index < fixture.clocks.size(); ++index) {
      require_cycle(fixture.clocks[index].kind == slots[index],
                    "Rejected word must retain its exact manual cycle prefix");
    }
    const auto &last = fixture.clocks.back();
    require_cycle(last.address == indexed_word_mask && last.size == 2,
                  "Rejected word reports its actual logical pointer/data access");
    fixture.cpu.set_state(initial_state(input, point.flags));
    const auto cycles = fixture.bus.cycles();
    bool rejected = false;
    try {
      fixture.cpu.step();
    } catch (const blofeld::Fault &retry) {
      rejected = std::string(retry.what()).contains("requires coordinated recovery");
    }
    require_cycle(rejected && fixture.bus.cycles() == cycles && fixture.memory.bytes == memory,
                  "A failed timed word must quarantine execution without more memory or clocks");
    return false;
  }
  throw blofeld::Fault("Unqualified wrapping word was silently accepted");
}

bool check_case(CycleFixture &fixture,
                const IndexedInput &input,
                const IndexedOperation &operation,
                const unsigned flags) {
  std::array<std::uint8_t, 4> code{operation.opcode};
  std::ranges::copy(std::span(input.operand).first(input.bytes), code.begin() + 1);
  fixture.install(std::span(code).first(input.bytes + 1), initial_state(input, flags));
  fixture.clocks.clear();
  const auto base = input.base_register == 3 ? indexed_entry + input.bytes + 1 : input.base;
  const auto update_offset = input.pre ? input.update : 0;
  const auto pointer = indexed_sum(base, input.update != 0 ? update_offset : input.offset);
  const auto address =
      indexed_indirect(input.kind) ? indexed_memory(fixture.memory, pointer, true) : pointer;
  const bool wrapping_pointer = indexed_indirect(input.kind) && pointer == indexed_word_mask;
  const bool wrapping_data = !operation.lea && operation.word && address == indexed_word_mask;
  if (wrapping_pointer || wrapping_data) {
    const char data_kind = operation.store ? 'W' : 'R';
    return reject_wrapped_word(
        fixture, input, operation, {.flags = flags, .kind = wrapping_pointer ? 'I' : data_kind});
  }
  const auto before_first = fixture.memory.bytes.at(address);
  const auto second_address = (address + 1) & indexed_word_mask;
  const auto before_second = fixture.memory.bytes.at(second_address);
  const auto stored = operation.word ? input.d : input.d >> 8U;
  const auto data =
      operation.store ? stored : indexed_memory(fixture.memory, address, operation.word);
  const auto expected =
      expected_state(input, operation, {.address = address, .data = data, .flags = flags});
  fixture.cpu.step();
  require_cycle(same_state(fixture.cpu.state(), expected),
                "Indexed CPU result, length, flags or retained registers differ");
  check_indexed_clocks(fixture, input, operation, pointer, address);
  if (operation.store) {
    require_cycle(indexed_memory(fixture.memory, address, operation.word) == data,
                  "Indexed store did not write the independently calculated bytes");
  } else {
    require_cycle(fixture.memory.bytes.at(address) == before_first &&
                      fixture.memory.bytes.at(second_address) == before_second,
                  "Indexed load/LEA unexpectedly modified data");
  }
  fixture.memory.write(static_cast<std::uint16_t>(address), before_first);
  fixture.memory.write(static_cast<std::uint16_t>(second_address), before_second);
  return true;
}

class Matrix {
public:
  Matrix() { reset(); }

  void reset() {
    fixture_ = std::make_unique<CycleFixture>();
    for (unsigned address = 0; address <= indexed_word_mask; ++address) {
      constexpr unsigned multiplier = 73;
      constexpr unsigned seed = 21;
      fixture_->memory.bytes.at(address) =
          static_cast<std::uint8_t>((address * multiplier + seed) & indexed_byte_mask);
    }
  }

  void check(const IndexedInput &input) {
    postbytes_.at(input.operand[0]) = true;
    for (const auto &operation : operations) {
      if (operation.lea && indexed_indirect(input.kind)) { continue; }
      constexpr unsigned alternating_flags = 0x5a;
      constexpr unsigned inverse_flags = 0xa5;
      for (const auto flags : {0U, indexed_byte_mask, alternating_flags, inverse_flags}) {
        try {
          if (!check_case(*fixture_, input, operation, flags)) {
            ++rejections_;
            reset();
          }
        } catch (const std::exception &error) {
          throw blofeld::Fault("Own indexed case opcode=" + blofeld::hex(operation.opcode, 2) +
                               " post=" + blofeld::hex(input.operand[0], 2) +
                               " base=" + blofeld::hex(input.base) +
                               " offset=" + std::to_string(input.offset) +
                               " update=" + std::to_string(input.update) +
                               " d=" + blofeld::hex(input.d) + ": " + error.what());
        }
        ++cases_;
      }
    }
  }

  [[nodiscard]] unsigned cases() const { return cases_; }

  [[nodiscard]] unsigned rejections() const { return rejections_; }

  void check_postbytes() const {
    require_cycle(std::ranges::all_of(postbytes_, [](const bool covered) { return covered; }),
                  "The fixed matrix must execute every indexed postbyte");
  }

private:
  std::unique_ptr<CycleFixture> fixture_;
  unsigned cases_ = 0;
  unsigned rejections_ = 0;
  std::array<bool, indexed_byte_mask + 1> postbytes_{};
};

IndexedInput
constant_input(const unsigned reg, const unsigned base, const int offset, const IndexedKind kind) {
  IndexedInput input{.kind = kind, .base_register = reg, .base = base, .offset = offset};
  const auto bits = indexed_sum(0, offset);
  if (kind == IndexedKind::short_offset) {
    constexpr unsigned five_bit_mask = 0x1f;
    input.operand[0] = static_cast<std::uint8_t>((reg << 6U) | (bits & five_bit_mask));
  } else {
    constexpr unsigned prefix = 0xe0;
    input.operand[0] = static_cast<std::uint8_t>(prefix | (reg << 3U));
    if (kind == IndexedKind::offset9) {
      input.bytes = 2;
      input.operand[0] |= static_cast<std::uint8_t>((bits >> 8U) & 1U);
      input.operand[1] = static_cast<std::uint8_t>(bits & indexed_byte_mask);
    } else {
      input.bytes = 3;
      input.operand[0] |= static_cast<std::uint8_t>(kind == IndexedKind::indirect16 ? 3U : 2U);
      input.operand[1] = static_cast<std::uint8_t>(bits >> 8U);
      input.operand[2] = static_cast<std::uint8_t>(bits & indexed_byte_mask);
    }
  }
  return input;
}

void constants(Matrix &matrix) {
  constexpr unsigned ordinary_base = 0x4000;
  constexpr std::array boundaries{-32768,
                                  -257,
                                  -256,
                                  -17,
                                  -16,
                                  -1,
                                  0,
                                  1,
                                  15,
                                  16,
                                  static_cast<int>(indexed_byte_mask),
                                  256,
                                  32767};
  for (unsigned reg = 0; reg < 4; ++reg) {
    for (int offset = -16; offset < 16; ++offset) {
      matrix.check(constant_input(reg, ordinary_base, offset, IndexedKind::short_offset));
    }
    for (int offset = -256; offset < 256; ++offset) {
      matrix.check(constant_input(reg, ordinary_base, offset, IndexedKind::offset9));
    }
    for (const auto base : {0U, 1U, indexed_word_mask, ordinary_base}) {
      for (const auto offset : boundaries) {
        matrix.check(constant_input(reg, base, offset, IndexedKind::offset16));
        matrix.check(constant_input(reg, base, offset, IndexedKind::indirect16));
        if (offset >= -16 && offset <= 15) {
          matrix.check(constant_input(reg, base, offset, IndexedKind::short_offset));
        }
        if (offset >= -256 && offset <= static_cast<int>(indexed_byte_mask)) {
          matrix.check(constant_input(reg, base, offset, IndexedKind::offset9));
        }
      }
    }
  }
}

void automatic(Matrix &matrix) {
  constexpr unsigned prefix = 0x20;
  constexpr unsigned post_bit = 0x10;
  constexpr unsigned negative_code = 16;
  constexpr unsigned ordinary_base = 0x4000;
  for (unsigned reg = 0; reg < 3; ++reg) {
    for (const auto base : {0U, 1U, indexed_word_mask, ordinary_base}) {
      for (int update = -8; update <= 8; ++update) {
        if (update == 0) { continue; }
        for (const auto pre : {false, true}) {
          const auto encoded = update < 0 ? negative_code - static_cast<unsigned>(-update)
                                          : static_cast<unsigned>(update - 1);
          matrix.check({.operand = {static_cast<std::uint8_t>((reg << 6U) | prefix |
                                                              (pre ? 0U : post_bit) | encoded)},
                        .base_register = reg,
                        .base = base,
                        .update = update,
                        .pre = pre});
        }
      }
    }
  }
}

void byte_accumulators(Matrix &matrix, const unsigned reg) {
  constexpr unsigned prefix = 0xe4;
  constexpr unsigned ordinary_base = 0x4000;
  for (unsigned accumulator = 0; accumulator < 2; ++accumulator) {
    for (unsigned value = 0; value <= indexed_byte_mask; ++value) {
      matrix.check({.operand = {static_cast<std::uint8_t>(prefix | (reg << 3U) | accumulator)},
                    .base_register = reg,
                    .base = ordinary_base,
                    .offset = static_cast<int>(value),
                    .d = accumulator == 0 ? (value << 8U) | indexed_byte_mask
                                          : (indexed_byte_mask << 8U) | value});
    }
  }
}

void word_accumulators(Matrix &matrix, const unsigned reg) {
  constexpr unsigned prefix = 0xe4;
  constexpr unsigned ordinary_base = 0x4000;
  constexpr std::array word_boundaries{
      0U, 1U, 0x7fU, 0x80U, 0xffU, 0x100U, 0x7fffU, 0x8000U, 0xff00U, 0xffffU};
  for (const auto base : {0U, 1U, indexed_word_mask, ordinary_base}) {
    for (const auto value : word_boundaries) {
      for (const auto indirect : {false, true}) {
        matrix.check(
            {.operand = {static_cast<std::uint8_t>(prefix | (reg << 3U) | (indirect ? 3U : 2U))},
             .kind = indirect ? IndexedKind::indirect_d : IndexedKind::short_offset,
             .base_register = reg,
             .base = base,
             .offset = static_cast<int>(value),
             .d = value});
      }
    }
  }
}

void accumulators(Matrix &matrix) {
  for (unsigned reg = 0; reg < 4; ++reg) {
    byte_accumulators(matrix, reg);
    word_accumulators(matrix, reg);
  }
}
}

int main() {
  try {
    Matrix matrix;
    constants(matrix);
    automatic(matrix);
    accumulators(matrix);
    matrix.check_postbytes();
    std::cout << "HC12 indexed addresses, flags, register updates and manual bus slots: "
              << matrix.cases() << " fixed cases passed (" << matrix.rejections()
              << " strict word-wrap rejections)\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
