// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_norm_expectations.hpp"
#include "dsp_saturation_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <string_view>

namespace blofeld::test::saturation {
namespace {
namespace native = dma_test;
using Program = native::NativeProgram;
constexpr std::size_t batch_limit = 1800;
constexpr unsigned bfu_left_word = 0x0c1d02;  // FM13-15: BFU ASL #1,A,A, not single-bit ASL A.
constexpr unsigned bfu_right_word = 0x0c1c02; // FM13-17: BFU ASR #1,A,A.
constexpr unsigned bfu_source_b = 0x80;
// FM12-21/23 and the instruction encodings: these literals avoid the native
// assembler's ambiguous multiply aliases and unsupported reversed operands.
constexpr unsigned signed_product_word = 0x2000d0;    // QQQ=101: Y0,X0, commutative signed product.
constexpr unsigned immediate_product_word = 0x0141c0; // qq=00: immediate,X0.
constexpr unsigned power_product_word = 0x0100d0;     // QQ=01: X0,#n.
constexpr unsigned mixed_multiply_word = 0x01278d;    // QQQQ=1101: X0,Y0, signed/unsigned.
constexpr unsigned mixed_accumulate_word = 0x01268d;
constexpr unsigned double_accumulate_word = 0x01248d; // ss=00, QQQQ=1101.
// FM13-118/131: long-immediate moves to A0/A1/A2/B0/B1/B2/X0/X1/Y1/SR/Y0/R0.
constexpr std::array operand_seeds{0x50f400U,
                                   0x54f400U,
                                   0x52f400U,
                                   0x51f400U,
                                   0x55f400U,
                                   0x53f400U,
                                   0x44f400U,
                                   0x45f400U,
                                   0x47f400U,
                                   0x05f439U,
                                   0x46f400U,
                                   0x60f400U};

struct Encoding {
  Operation operation;
  unsigned a;
  unsigned b;
};

// FM13 instruction diagrams and table12-18 (NE=0010). The two columns name
// destination A/B; EXTRACTU reads the other accumulator in each column.
constexpr std::array scalar_encodings{
    Encoding{.operation = Operation::add, .a = 0x200040, .b = 0x200048},
    Encoding{.operation = Operation::subtract, .a = 0x200044, .b = 0x20004c},
    Encoding{.operation = Operation::adc, .a = 0x200021, .b = 0x200029},
    Encoding{.operation = Operation::sbc, .a = 0x200025, .b = 0x20002d},
    Encoding{.operation = Operation::add_left, .a = 0x200012, .b = 0x20001a},
    Encoding{.operation = Operation::add_right, .a = 0x200002, .b = 0x20000a},
    Encoding{.operation = Operation::subtract_left, .a = 0x200016, .b = 0x20001e},
    Encoding{.operation = Operation::subtract_right, .a = 0x200006, .b = 0x20000e},
    Encoding{.operation = Operation::absolute, .a = 0x200026, .b = 0x20002e},
    Encoding{.operation = Operation::negate, .a = 0x200036, .b = 0x20003e},
    Encoding{.operation = Operation::shift_left, .a = 0x200032, .b = 0x20003a},
    Encoding{.operation = Operation::shift_right, .a = 0x200022, .b = 0x20002a},
    Encoding{.operation = Operation::round, .a = 0x200011, .b = 0x200019},
    Encoding{.operation = Operation::transfer, .a = 0x200001, .b = 0x200009},
    Encoding{.operation = Operation::increment, .a = 0x000008, .b = 0x000009},
    Encoding{.operation = Operation::decrement, .a = 0x00000a, .b = 0x00000b},
    Encoding{.operation = Operation::normalize, .a = 0x01d815, .b = 0x01d81d},
    Encoding{.operation = Operation::conditional_transfer, .a = 0x022000, .b = 0x022008},
    Encoding{.operation = Operation::extract, .a = 0x0c1a98, .b = 0x0c1a89},
    Encoding{.operation = Operation::insert, .a = 0x0c1b68, .b = 0x0c1b69},
    Encoding{.operation = Operation::self_move, .a = 0x21ce00, .b = 0x21ef00}};

bool product_operation(const Operation operation) {
  return operation == Operation::multiply || operation == Operation::multiply_round ||
         operation == Operation::accumulate || operation == Operation::accumulate_round ||
         operation == Operation::double_accumulate;
}

unsigned mixed_product(const Input &input) {
  const bool accumulate =
      input.operation == Operation::accumulate || input.operation == Operation::accumulate_round;
  const bool dmac = input.operation == Operation::double_accumulate;
  auto word = accumulate ? mixed_accumulate_word : mixed_multiply_word;
  if (dmac) { word = double_accumulate_word; }
  if (input.signedness == Signedness::signed_unsigned && dmac) { word |= 0x100; }
  if (input.signedness == Signedness::unsigned_unsigned) {
    constexpr unsigned double_unsigned_bits =
        0x140; // s/S in FM13-55: both source operands unsigned.
    word |= dmac ? double_unsigned_bits : 0x40U;
  }
  if (input.destination_b) { word |= 0x20; }
  if (input.negative_product) { word |= 0x10; }
  return word;
}

void emit_product(Program &code, const Input &input) {
  const bool accumulate =
      input.operation == Operation::accumulate || input.operation == Operation::accumulate_round;
  if (input.operation == Operation::double_accumulate ||
      input.signedness != Signedness::signed_signed) {
    code.word(mixed_product(input));
    return;
  }
  auto word = signed_product_word;
  if (input.form == ProductForm::immediate) { word = immediate_product_word; }
  if (input.form == ProductForm::power) { word = power_product_word | (input.power << 8); }
  if (accumulate) { word |= 2; }
  if (input.operation == Operation::multiply_round ||
      input.operation == Operation::accumulate_round) {
    word |= 1;
  }
  if (input.destination_b) { word |= 8; }
  if (input.negative_product) { word |= 4; }
  code.word(word);
  if (input.form == ProductForm::immediate) { code.word(input.y); }
}

std::string product_instruction(const Input &input, const std::string &destination) {
  std::string name;
  if (input.operation == Operation::multiply) {
    name = "mpy";
  } else if (input.operation == Operation::accumulate) {
    name = "mac";
  } else if (input.operation == Operation::multiply_round) {
    name = "mpyr";
  } else if (input.operation == Operation::accumulate_round) {
    name = "macr";
  } else if (input.operation == Operation::double_accumulate) {
    name = "dmac";
  } else {
    throw Fault("Expected own product instruction");
  }
  if (input.signedness == Signedness::signed_unsigned) { name += "su"; }
  if (input.signedness == Signedness::unsigned_unsigned) { name += "uu"; }
  if (input.operation == Operation::double_accumulate &&
      input.signedness == Signedness::signed_signed) {
    name += "ss";
  }
  if (input.form == ProductForm::immediate) {
    return name + "i #>" + number(input.y) + ",x0," + destination;
  }
  const char *const first = input.negative_product ? " -x0," : " x0,";
  const auto second = input.form == ProductForm::power ? '#' + std::to_string(input.power) : "y0";
  return name + first + second + ',' + destination;
}

void seed(Program &code, const Input &input) {
  const auto a = input.destination_b ? input.source : input.destination;
  const auto b = input.destination_b ? input.destination : input.source;
  const std::array words{static_cast<unsigned>(a & word_mask),
                         static_cast<unsigned>((a >> word_bits) & word_mask),
                         static_cast<unsigned>(a >> data_bits),
                         static_cast<unsigned>(b & word_mask),
                         static_cast<unsigned>((b >> word_bits) & word_mask),
                         static_cast<unsigned>(b >> data_bits),
                         input.x,
                         input.y,
                         input.y ^ word_mask,
                         masked_status | input.status,
                         input.y,
                         counter_seed};
  for (std::size_t index = 0; index < words.size(); ++index) {
    code.word(operand_seeds.at(index));
    code.word(words.at(index));
  }
}

unsigned scalar_opcode(const Input &input) {
  for (const auto &encoding : scalar_encodings) {
    if (encoding.operation == input.operation) {
      return input.destination_b ? encoding.b : encoding.a;
    }
  }
  throw Fault("Expected a scalar saturation fixture operation");
}

void emit(Program &code, const Input &input) {
  if (product_operation(input.operation)) {
    emit_product(code, input);
    return;
  }
  if (input.operation == Operation::bfu_left || input.operation == Operation::bfu_right) {
    auto word = input.operation == Operation::bfu_left ? bfu_left_word : bfu_right_word;
    if (input.destination_b) { word |= bfu_source_b | 1U; }
    code.word(word);
    return;
  }
  code.word(scalar_opcode(input));
}

void verify(OwnDspMachine &machine, const Input &input) {
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  require(before.a == (input.destination_b ? input.source : input.destination) &&
              before.b == (input.destination_b ? input.destination : input.source) &&
              (before.x & word_mask) == input.x && (before.y & word_mask) == input.y &&
              (before.x >> word_bits) == input.y &&
              (before.y >> word_bits) == (input.y ^ word_mask) &&
              before.sr == (masked_status | input.status) && before.r.at(0) == counter_seed,
          "Native setup supplies independent operand values, modes, flags and counter");
  const auto wanted = expected(input);
  auto cpu = before;
  if (input.destination_b) {
    cpu.b = wanted.value;
  } else {
    cpu.a = wanted.value;
  }
  cpu.pc += input.form == ProductForm::immediate ? 2U : 1U;
  cpu.sr = (cpu.sr & ~wanted.flag_mask) | wanted.flags;
  cpu.r.at(0) = static_cast<unsigned>(std::int64_t{counter_seed} + wanted.counter_delta);
  native::step_native(machine);
  auto after = machine.cpu().registers();
  const auto observed_status = after.sr;
  if (!wanted.qualified_carry) {
    cpu.sr &= ~carry;
    after.sr &= ~carry;
  }
  const auto context = instruction(input) + " D=" + number(input.destination) +
                       " S=" + number(input.source) + " X0=" + number(input.x) +
                       " Y0=" + number(input.y) + " incoming SR=" + number(input.status) +
                       " expected value=" + number(wanted.value) +
                       " observed value=" + number(input.destination_b ? after.b : after.a) +
                       " expected SR=" + number(cpu.sr) + " observed SR=" + number(observed_status);
  require(after == cpu && machine.cpu().progress().instructions == progress.instructions + 1,
          "Independent arithmetic, literal saturation table and qualified whole-state outcome: " +
              context);
}
}

std::string instruction(const Input &input) {
  const std::string destination = input.destination_b ? "b" : "a";
  const std::string source = input.destination_b ? "a" : "b";
  switch (input.operation) {
    case Operation::add: return "add x0," + destination;
    case Operation::subtract: return "sub x0," + destination;
    case Operation::adc: return "adc x," + destination;
    case Operation::sbc: return "sbc x," + destination;
    case Operation::add_left: return "addl " + source + ',' + destination;
    case Operation::add_right: return "addr " + source + ',' + destination;
    case Operation::subtract_left: return "subl " + source + ',' + destination;
    case Operation::subtract_right: return "subr " + source + ',' + destination;
    case Operation::absolute: return "abs " + destination;
    case Operation::negate: return "neg " + destination;
    case Operation::increment: return "inc " + destination;
    case Operation::decrement: return "dec " + destination;
    case Operation::shift_left: return "asl " + destination;
    case Operation::shift_right: return "asr " + destination;
    case Operation::bfu_left: return "asl #1," + destination + ',' + destination;
    case Operation::bfu_right: return "asr #1," + destination + ',' + destination;
    case Operation::normalize: return "norm r0," + destination;
    case Operation::multiply:
    case Operation::accumulate:
    case Operation::multiply_round:
    case Operation::accumulate_round:
    case Operation::double_accumulate: return product_instruction(input, destination);
    case Operation::round: return "rnd " + destination;
    case Operation::extract: return "extractu x0," + source + ',' + destination;
    case Operation::insert: return "insert x0,x1," + destination;
    case Operation::transfer: return "tfr " + source + ',' + destination;
    case Operation::conditional_transfer: return "tne " + source + ',' + destination;
    case Operation::self_move: return "move " + destination + ',' + destination;
    default: throw Fault("Unknown own saturation fixture operation");
  }
}

void run(const std::span<const Input> inputs) {
  for (std::size_t start = 0; start < inputs.size(); start += batch_limit) {
    const auto count = std::min(batch_limit, inputs.size() - start);
    const auto batch = inputs.subspan(start, count);
    Program code;
    for (const auto &input : batch) {
      seed(code, input);
      emit(code, input);
    }
    OwnDspMachine machine(native::native_clocks);
    native::reset_native(machine, code);
    for (const auto &input : batch) {
      for (unsigned step = 0; step < preamble_steps; ++step) { native::step_native(machine); }
      verify(machine, input);
    }
  }
  std::cout << inputs.size() << " independent saturation/result/flag/state cases passed.\n";
}
}

int main(const int count, char **arguments) {
  try {
    blofeld::test::require(count == 2, "Usage: dsp-saturation-test scope");
    const auto inputs = blofeld::test::saturation::cases(std::string_view(arguments[1]));
    blofeld::test::saturation::run(inputs);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
