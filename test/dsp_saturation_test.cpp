// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_operand_fixture.hpp"
#include "dsp_saturation_fixture.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <string_view>

namespace blofeld::test::saturation {
namespace {
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

void emit_product(DspProgram &code, const Input &input) {
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

void seed(DspProgram &code, const Input &input) {
  operands::seed_operands(code,
                          {.a = input.destination_b ? input.source : input.destination,
                           .b = input.destination_b ? input.destination : input.source,
                           .control = input.x,
                           .source = input.y,
                           .status = input.status});
  code.immediate("y0", input.y);
  code.immediate("r0", counter_seed);
}

void emit(DspProgram &code, const Input &input) {
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
  code.emit(instruction(input));
}

void verify(DspMachine &machine, const Input &input) {
  const auto before = machine.cpu().debug_state();
  require(before.cpu.a == (input.destination_b ? input.source : input.destination) &&
              before.cpu.b == (input.destination_b ? input.destination : input.source) &&
              (before.cpu.x & word_mask) == input.x && (before.cpu.y & word_mask) == input.y &&
              (before.cpu.x >> word_bits) == input.y &&
              (before.cpu.y >> word_bits) == (input.y ^ word_mask) &&
              before.cpu.sr == (masked_status | input.status) && before.cpu.r.at(0) == counter_seed,
          "Native setup supplies independent operand values, modes, flags and counter");
  const auto wanted = expected(input);
  auto cpu = before.cpu;
  if (input.destination_b) {
    cpu.b = wanted.value;
  } else {
    cpu.a = wanted.value;
  }
  cpu.pc += input.form == ProductForm::immediate ? 2U : 1U;
  cpu.sr = (cpu.sr & ~wanted.flag_mask) | wanted.flags;
  cpu.r.at(0) = static_cast<unsigned>(std::int64_t{counter_seed} + wanted.counter_delta);
  machine.cpu().step();
  auto after = machine.cpu().debug_state();
  const auto observed_status = after.cpu.sr;
  if (!wanted.qualified_carry) {
    cpu.sr &= ~carry;
    after.cpu.sr &= ~carry;
  }
  const auto context =
      instruction(input) + " D=" + number(input.destination) + " S=" + number(input.source) +
      " X0=" + number(input.x) + " Y0=" + number(input.y) + " incoming SR=" + number(input.status) +
      " expected value=" + number(wanted.value) +
      " observed value=" + number(input.destination_b ? after.cpu.b : after.cpu.a) +
      " expected SR=" + number(cpu.sr) + " observed SR=" + number(observed_status);
  require(after.cpu == cpu && after.native_instructions == before.native_instructions + 1 &&
              after.stack_high == before.stack_high && after.stack_low == before.stack_low &&
              after.ep == before.ep && after.sz == before.sz && after.sc == before.sc,
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
  DspMachine machine;
  for (std::size_t start = 0; start < inputs.size(); start += batch_limit) {
    const auto count = std::min(batch_limit, inputs.size() - start);
    const auto batch = inputs.subspan(start, count);
    DspProgram code;
    for (const auto &input : batch) {
      seed(code, input);
      emit(code, input);
    }
    machine.cpu().reset_spi_boot();
    code.install(machine.bus());
    for (const auto &input : batch) {
      for (unsigned step = 0; step < preamble_steps; ++step) { machine.cpu().step(); }
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
