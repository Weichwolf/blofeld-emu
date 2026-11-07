// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_norm_expectations.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test {
namespace {
namespace native = dma_test;
using Program = native::NativeProgram;
constexpr unsigned transfer_address = 0x100, transfer_second_address = 0x200;
constexpr unsigned move_sr = 0x05f439;
// FM13-118/131/REP: named seed and directed-program instruction words.
constexpr unsigned move_m0 = 0x05f420;
constexpr unsigned move_m1 = 0x05f421;
constexpr unsigned move_m4 = 0x05f424;
constexpr unsigned move_r0 = 0x60f400;
constexpr unsigned move_r4 = 0x64f400;
constexpr unsigned move_x0 = 0x44f400;
constexpr unsigned move_y0 = 0x46f400;
constexpr unsigned move_x1 = 0x45f400;
constexpr unsigned move_y1 = 0x47f400;
constexpr unsigned store_x0 = 0x446000;
constexpr unsigned store_y0 = 0x4e6400;
constexpr unsigned repeat_a = 0x06ce20;
constexpr unsigned add_with_old_a = 0x21c640;
constexpr unsigned move_a_to_x0 = 0x21c400;
constexpr unsigned move_a_to_sr = 0x04ceb9;

// DSP56300FM13-118/131: literal long-immediate data/control fields.
constexpr std::array accumulator_seeds{
    0x50f400U, 0x54f400U, 0x52f400U, 0x51f400U, 0x55f400U, 0x53f400U};

void immediate(Program &program, const unsigned opcode, const unsigned value) {
  program.word(opcode);
  program.word(value);
}

void set_accumulator(Program &program, const bool accumulator_b, const std::uint64_t value) {
  const unsigned first = accumulator_b ? 3U : 0U;
  immediate(program, accumulator_seeds[first], static_cast<unsigned>(value & word_mask));
  immediate(program,
            accumulator_seeds[first + 1],
            static_cast<unsigned>((value >> word_bits) & word_mask));
  immediate(program, accumulator_seeds[first + 2], static_cast<unsigned>(value >> data_bits));
}

void run_to(DspMachine &machine, const unsigned pc) {
  unsigned count = 0;
  while (machine.cpu().registers().pc != pc) {
    constexpr unsigned instruction_limit = 100;
    require(++count <= instruction_limit, "Native transfer diagnostic exceeded its setup limit");
    native::step_native(machine);
  }
}

constexpr unsigned seed_x = 0x123456;
constexpr unsigned seed_y = 0x654321;
constexpr unsigned original_flags = carry | overflow | zero | negative | unnormalized | extension;
constexpr std::uint64_t half = 0x400000000000;

enum class Form : std::uint8_t {
  reg_x,
  reg_y,
  store_x,
  store_y,
  long_store,
  control,
  program,
  peripheral,
  raw_high,
  raw_low,
  raw_extension,
  raw_long,
  replace_source,
  load_x,
  load_y,
  dual_store,
  bit_test,
};

struct Input {
  std::uint64_t value = 0;
  unsigned status = 0;
  bool source_b = false;
  Form form = Form::reg_x;
};

struct BusResult {
  unsigned high = 0;
  unsigned low = 0;
  unsigned status = 0;
};

// FM 3.1.6 and 5.4.1.2: pre-scaling growth; then signed scaling/limiting.
// A/B contribute when they drive a data bus. Captured MCPX pm8_mixed_dir
// distinguishes this from inspecting the untouched accumulator unconditionally.
BusResult expected(const Input input) {
  constexpr unsigned ordinary_growth_high = 46;
  unsigned growth_high = ordinary_growth_high;
  if ((input.status & scale_up) != 0) { --growth_high; }
  if ((input.status & scale_down) != 0) { ++growth_high; }
  auto status = input.status;
  if ((((input.value >> growth_high) ^ (input.value >> (growth_high - 1))) & 1U) != 0) {
    status |= sticky;
  }
  constexpr std::uint64_t full_range = std::uint64_t{1} << 56;
  auto scaled = (input.value & sign_bit) != 0 ? -static_cast<std::int64_t>(full_range - input.value)
                                              : static_cast<std::int64_t>(input.value);
  if ((input.status & scale_up) != 0) { scaled *= 2; }
  if ((input.status & scale_down) != 0) { scaled >>= 1; }
  constexpr std::int64_t maximum = 0x7fffffffffff;
  constexpr std::int64_t minimum = -0x800000000000;
  if (scaled > maximum) {
    status |= limit;
    scaled = maximum;
  }
  if (scaled < minimum) {
    status |= limit;
    scaled = minimum;
  }
  const auto bits = static_cast<std::uint64_t>(scaled);
  return {.high = static_cast<unsigned>((bits >> word_bits) & word_mask),
          .low = static_cast<unsigned>(bits & word_mask),
          .status = status};
}

std::string instruction(const Input input) {
  const std::string src = input.source_b ? "b" : "a";
  const std::string other = input.source_b ? "a" : "b";
  switch (input.form) {
    case Form::reg_x: return "move " + src + ",x0";
    case Form::reg_y: return "move " + src + ",y1";
    case Form::store_x: return "move " + src + ",x:(r0)+";
    case Form::store_y: return "move " + src + ",y:(r4)+";
    case Form::long_store: return "move " + src + ",l:(r0)+";
    case Form::control: return "move " + src + ",m1";
    case Form::program: return "move " + src + ",p:(r0)+";
    case Form::peripheral: return "movep " + src + ",x:<<$fffffc";
    case Form::raw_high: return "move " + src + "1,x0";
    case Form::raw_low: return "move " + src + "0,x0";
    case Form::raw_extension: return "move " + src + "2,x0";
    case Form::raw_long: return "move " + src + "10,l:(r0)+";
    case Form::replace_source: return "move " + src + ",x:(r0)+ x0," + src;
    case Form::load_x: return "move x:(r0)+,x0 " + src + ",y0";
    case Form::load_y: return "move " + src + ",x0 y:(r4)+,y1";
    case Form::dual_store: return "move " + src + ",x:(r0)+ " + other + ",y:(r4)+";
    case Form::bit_test: return "btst #0," + src;
    default: throw Fault("Unknown own transfer form");
  }
}

void emit_instruction(Program &program, const Input input) {
  // FM13-118..131/13-26, independently cross-checked with assembler6.3.28.
  // This table follows Form order; the assembler is never a runtime dependency.
  constexpr std::array opcodes{std::array{0x21c400U,
                                          0x21c700U,
                                          0x565800U,
                                          0x5e5c00U,
                                          0x485800U,
                                          0x04cea1U,
                                          0x07588eU,
                                          0x08ce3cU,
                                          0x218400U,
                                          0x210400U,
                                          0x214400U,
                                          0x405800U,
                                          0x081800U,
                                          0x109800U,
                                          0x11dc00U,
                                          0xbb1800U,
                                          0x0bce60U},
                               std::array{0x21e400U,
                                          0x21e700U,
                                          0x575800U,
                                          0x5f5c00U,
                                          0x495800U,
                                          0x04cfa1U,
                                          0x07588fU,
                                          0x08cf3cU,
                                          0x21a400U,
                                          0x212400U,
                                          0x216400U,
                                          0x415800U,
                                          0x091800U,
                                          0x129800U,
                                          0x19dc00U,
                                          0xbe1800U,
                                          0x0bcf60U}};
  program.word(opcodes[input.source_b ? 1U : 0U].at(static_cast<unsigned>(input.form)));
}

unsigned initialize(Program &program, const Input input) {
  immediate(program, move_m0, word_mask);
  immediate(program, move_m1, word_mask);
  immediate(program, move_m4, word_mask);
  immediate(program, move_r0, transfer_address);
  immediate(program, move_r4, transfer_second_address);
  immediate(program, move_x0, seed_x);
  immediate(program, move_y0, seed_y);
  immediate(program, move_x1, 0);
  immediate(program, move_y1, 0);
  program.word(store_x0);
  program.word(store_y0);
  set_accumulator(program, input.source_b, input.value);
  set_accumulator(program, !input.source_b, 0);
  immediate(program, move_sr, input.status);
  const auto pc = program.pc();
  emit_instruction(program, input);
  return pc;
}

void check_data(DspBus &bus,
                const dsp56300::Registers &state,
                const Input input,
                const BusResult result) {
  const auto x = static_cast<unsigned>(state.x & word_mask);
  const auto y0 = static_cast<unsigned>(state.y & word_mask);
  const auto y1 = static_cast<unsigned>(state.y >> word_bits);
  const auto x_at = [&bus](const unsigned address) {
    return bus.read({.space = DspSpace::x, .address = address});
  };
  const auto y_at = [&bus](const unsigned address) {
    return bus.read({.space = DspSpace::y, .address = address});
  };
  bool correct = true;
  switch (input.form) {
    case Form::reg_x: correct = x == result.high; break;
    case Form::reg_y: correct = y1 == result.high; break;
    case Form::store_x:
    case Form::replace_source: correct = x_at(transfer_address) == result.high; break;
    case Form::store_y: correct = y_at(transfer_second_address) == result.high; break;
    case Form::long_store:
      correct = x_at(transfer_address) == result.high && y_at(transfer_address) == result.low;
      break;
    case Form::control: correct = state.m[1] == result.high; break;
    case Form::program:
      correct = bus.read({.space = DspSpace::p, .address = transfer_address}) == result.high;
      break;
    case Form::peripheral: correct = bus.once_latch() == result.high; break;
    case Form::raw_high: correct = x == ((input.value >> word_bits) & word_mask); break;
    case Form::raw_low: correct = x == (input.value & word_mask); break;
    case Form::raw_extension: {
      const auto byte = static_cast<unsigned>(input.value >> data_bits);
      constexpr unsigned byte_sign = 0x80;
      constexpr unsigned extension_bits = 0xffff00;
      correct = x == (byte | ((byte & byte_sign) != 0 ? extension_bits : 0));
      break;
    }
    case Form::raw_long:
      correct = x_at(transfer_address) == ((input.value >> word_bits) & word_mask) &&
                y_at(transfer_address) == (input.value & word_mask);
      break;
    case Form::load_x: correct = x == seed_x && y0 == result.high; break;
    case Form::load_y: correct = x == result.high && y1 == seed_y; break;
    case Form::dual_store:
      correct = x_at(transfer_address) == result.high && y_at(transfer_second_address) == 0;
      break;
    case Form::bit_test: break;
    default: throw Fault("Unknown own transfer result form");
  }
  require(correct, "Transfer data mismatch: " + instruction(input));
}

bool raw_form(const Form form) {
  return form == Form::raw_high || form == Form::raw_low || form == Form::raw_extension ||
         form == Form::raw_long;
}

void check(DspBus &bus,
           const dsp56300::Registers &before,
           const dsp56300::Registers &after,
           const Input input) {
  auto result = expected(input);
  if (raw_form(input.form)) { result.status = input.status; }
  if (input.form == Form::bit_test) {
    // BTST has the standard S/L updates and reports the scaled/limited bit.
    result.status = (result.status & ~carry) | (result.high & carry);
  }
  const auto context =
      instruction(input) + " value=" + number(input.value) + " sr=" + number(input.status);
  require(after.sr == result.status,
          context + " flags=" + number(after.sr) + " expected=" + number(result.status));
  auto a = before.a;
  auto b = before.b;
  if (input.form == Form::replace_source) {
    (input.source_b ? b : a) = std::uint64_t{seed_x} << word_bits;
  }
  require(after.a == a && after.b == b && after.sp == before.sp && after.omr == before.omr &&
              after.lc == before.lc && after.la == before.la && after.vba == before.vba,
          context + " unexpectedly changed an accumulator/control register: A=" + number(after.a) +
              " wanted=" + number(a) + " B=" + number(after.b) + " wanted=" + number(b) +
              " X=" + number(after.x) + " OMR=" + number(after.omr));
  auto registers = before.r;
  if (input.form == Form::store_x || input.form == Form::long_store ||
      input.form == Form::program || input.form == Form::raw_long ||
      input.form == Form::replace_source || input.form == Form::load_x ||
      input.form == Form::dual_store) {
    ++registers[0];
  }
  if (input.form == Form::store_y || input.form == Form::load_y || input.form == Form::dual_store) {
    ++registers[4];
  }
  auto modifiers = before.m;
  if (input.form == Form::control) { modifiers[1] = result.high; }
  require(after.r == registers && after.n == before.n && after.m == modifiers,
          context + " unexpectedly changed address state");
  require(after.pc == before.pc + 1, context + " invalid length");
  const auto form = input.form;
  const bool writes_x0 = form == Form::reg_x || form == Form::raw_high || form == Form::raw_low ||
                         form == Form::raw_extension || form == Form::load_y;
  require((after.x >> word_bits) == (before.x >> word_bits) &&
              (writes_x0 || (after.x & word_mask) == (before.x & word_mask)) &&
              (form == Form::load_x || (after.y & word_mask) == (before.y & word_mask)) &&
              (form == Form::reg_y || form == Form::load_y ||
               (after.y >> word_bits) == (before.y >> word_bits)),
          context + " changed an unselected data register");
  check_data(bus, after, input, result);
}

void run_batch(DspMachine &machine, const std::span<const Input> batch) {
  Program program;
  std::vector<unsigned> pcs;
  for (const auto input : batch) { pcs.push_back(initialize(program, input)); }
  native::reset_native(machine, program);
  for (std::size_t index = 0; index < batch.size(); ++index) {
    run_to(machine, pcs[index]);
    const auto before = machine.cpu().registers();
    const auto retired = machine.cpu().progress().instructions;
    native::step_native(machine);
    check(machine.bus(), before, machine.cpu().registers(), batch[index]);
    require(machine.cpu().progress().instructions == retired + 1,
            "Each literal transfer retires exactly once through the native owner");
  }
}

void test_matrix() {
  constexpr std::array<std::uint64_t, 12> values{0,
                                                 1,
                                                 0x1fffffffffff,
                                                 0x200000000000,
                                                 0x400000abcdef,
                                                 0x600000123456,
                                                 0x7fffffffffff,
                                                 0x800000000000,
                                                 0x1000000000000,
                                                 0xff800000000000,
                                                 0xffffffffffffff,
                                                 0x80000000000000};
  std::vector<Input> inputs;
  for (const auto value : values) {
    for (const unsigned scaling : {0U, scale_down, scale_up}) {
      for (const bool source_b : {false, true}) {
        for (const unsigned preserved : {0U, sticky, limit, sticky | limit}) {
          for (unsigned form = 0; form <= static_cast<unsigned>(Form::bit_test); ++form) {
            inputs.push_back(
                {.value = value,
                 .status = masked_status | scaling | (source_b ? original_flags : 0U) | preserved,
                 .source_b = source_b,
                 .form = static_cast<Form>(form)});
          }
        }
      }
    }
  }
  constexpr unsigned expected_cases = 4896;
  require(inputs.size() == expected_cases, "Transfer matrix must cover each declared combination");
  DspMachine machine(native::native_clocks);
  constexpr std::size_t batch_size = 32;
  for (std::size_t offset = 0; offset < inputs.size(); offset += batch_size) {
    run_batch(machine,
              std::span(inputs).subspan(offset, std::min(batch_size, inputs.size() - offset)));
  }
  std::cout << inputs.size() << " independent accumulator-transfer vectors passed\n";
}

void test_repeat_count_transfer() {
  DspMachine machine(native::native_clocks);
  for (const unsigned scaling : {0U, scale_down, scale_up}) {
    Program program;
    set_accumulator(program, false, half);
    immediate(program, move_sr, masked_status | scaling);
    const auto prefix = program.pc();
    program.word(repeat_a);
    const auto body = program.pc();
    program.word(0x000000);
    native::reset_native(machine, program);
    run_to(machine, prefix);
    const auto wanted = expected({.value = half, .status = masked_status | scaling});
    native::step_native(machine);
    require(machine.cpu().registers().pc == body && machine.cpu().registers().lc == wanted.high &&
                machine.cpu().registers().sr == wanted.status &&
                machine.cpu().registers().a == half,
            "REP accumulator count uses the standard pre-scaling S/L transfer behavior");
    native::step_native(machine);
    require(machine.cpu().registers().lc == wanted.high - 1 &&
                machine.cpu().registers().pc == body &&
                machine.cpu().registers().sr == wanted.status,
            "REP retires one native body instruction after the scaled count transfer");
  }
}

void test_source_timing() {
  Program program;
  set_accumulator(program, false, half);
  set_accumulator(program, true, 0);
  immediate(program, move_x0, half >> word_bits);
  immediate(program, move_sr, masked_status);
  const auto add = program.pc();
  program.word(add_with_old_a);
  DspMachine machine(native::native_clocks);
  native::reset_native(machine, program);
  run_to(machine, add);
  native::step_native(machine);
  require(machine.cpu().registers().a == half * 2 &&
              machine.cpu().registers().y == (half >> word_bits) &&
              machine.cpu().registers().sr == (masked_status | extension | sticky),
          "Parallel transfer reads pre-ALU accumulator and growth before ADD overwrites it");
}

void test_unselected_growth_and_sr_write() {
  Program program;
  set_accumulator(program, false, 0);
  set_accumulator(program, true, half);
  immediate(program, move_sr, masked_status);
  const auto transfer = program.pc();
  program.word(move_a_to_x0);
  set_accumulator(program, false, half);
  // Scaled result is itself a legal SR value, including explicitly cleared S.
  constexpr unsigned replacement = 0x400000;
  program.word(move_a_to_sr);
  DspMachine machine(native::native_clocks);
  native::reset_native(machine, program);
  run_to(machine, transfer);
  native::step_native(machine);
  require(machine.cpu().registers().sr == masked_status && machine.cpu().registers().x == 0,
          "Unselected accumulator growth does not set S on another accumulator's transfer");
  for (unsigned count = 0; count < 4; ++count) { native::step_native(machine); }
  require(machine.cpu().registers().sr == replacement,
          "Explicit MOVEC to SR replaces computed transfer flags with the moved word");
}
}
}

int main() {
  try {
    blofeld::test::test_matrix();
    blofeld::test::test_repeat_count_transfer();
    blofeld::test::test_source_timing();
    blofeld::test::test_unselected_growth_and_sr_write();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
