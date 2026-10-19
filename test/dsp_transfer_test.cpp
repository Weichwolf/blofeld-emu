// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_transfer_fixture.hpp"

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
  if ((input.status & scale_up) != 0) { ++growth_high; }
  if ((input.status & scale_down) != 0) { --growth_high; }
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

void emit_instruction(DspProgram &program, const Input input) {
  // FM 13-120, 13-124 and 13-129. The helper assembler silently drops
  // a second XR/YR register move, so encode these complete parallel words.
  constexpr unsigned class_two_x = 0x081800;
  constexpr unsigned class_one_x = 0x109800;
  constexpr unsigned class_one_y = 0x11dc00;
  constexpr unsigned dual_ab = 0xbb1800;
  constexpr unsigned dual_ba = 0xbe1800;
  constexpr unsigned class_two_b = 0x010000;
  constexpr unsigned class_one_x_b = 0x020000;
  constexpr unsigned class_one_y_b = 0x080000;
  if (input.form == Form::replace_source) {
    program.word(class_two_x | (input.source_b ? class_two_b : 0));
  } else if (input.form == Form::load_x) {
    program.word(class_one_x | (input.source_b ? class_one_x_b : 0));
  } else if (input.form == Form::load_y) {
    program.word(class_one_y | (input.source_b ? class_one_y_b : 0));
  } else if (input.form == Form::dual_store) {
    program.word(input.source_b ? dual_ba : dual_ab);
  } else {
    program.emit(instruction(input));
  }
}

unsigned initialize(DspProgram &program, const Input input) {
  program.immediate("m0", word_mask);
  program.immediate("m1", word_mask);
  program.immediate("m4", word_mask);
  program.immediate("r0", transfer_address);
  program.immediate("r4", transfer_second_address);
  program.immediate("x0", seed_x);
  program.immediate("y0", seed_y);
  program.immediate("x1", 0);
  program.immediate("y1", 0);
  program.emit("move x0,x:(r0)");
  program.emit("move y0,y:(r4)");
  set_accumulator(program, input.source_b ? "b" : "a", input.value);
  set_accumulator(program, input.source_b ? "a" : "b", 0);
  program.immediate("sr", input.status);
  const auto pc = program.pc();
  emit_instruction(program, input);
  return pc;
}

void check_data(DspBus &bus, const DspState &state, const Input input, const BusResult result) {
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

void check(DspBus &bus, const DspState &before, const DspState &after, const Input input) {
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

void run_batch(DspBus &bus, DspCore &cpu, const std::span<const Input> batch) {
  DspProgram program;
  std::vector<unsigned> pcs;
  for (const auto input : batch) { pcs.push_back(initialize(program, input)); }
  program.install(bus);
  cpu.reset_spi_boot();
  for (std::size_t index = 0; index < batch.size(); ++index) {
    run_to(cpu, pcs[index]);
    const auto before = cpu.state();
    cpu.step();
    check(bus, before, cpu.state(), batch[index]);
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
  DspBus bus;
  DspCore cpu(bus);
  constexpr std::size_t batch_size = 32;
  for (std::size_t offset = 0; offset < inputs.size(); offset += batch_size) {
    run_batch(
        bus, cpu, std::span(inputs).subspan(offset, std::min(batch_size, inputs.size() - offset)));
  }
  std::cout << inputs.size() << " independent accumulator-transfer vectors passed\n";
}

void test_repeat_count_transfer() {
  DspBus bus;
  DspCore cpu(bus);
  for (const unsigned scaling : {0U, scale_down, scale_up}) {
    DspProgram program;
    set_accumulator(program, "a", half);
    program.immediate("sr", masked_status | scaling);
    const auto prefix = program.pc();
    program.emit("rep a");
    const auto body = program.pc();
    program.emit("nop");
    program.install(bus);
    cpu.reset_spi_boot();
    run_to(cpu, prefix);
    const auto wanted = expected({.value = half, .status = masked_status | scaling});
    cpu.step();
    require(cpu.state().pc == body && cpu.state().lc == wanted.high &&
                cpu.state().sr == wanted.status && cpu.state().a == half,
            "REP accumulator count uses the standard pre-scaling S/L transfer behavior");
    cpu.step();
    require(cpu.state().lc == wanted.high - 1 && cpu.state().pc == body &&
                cpu.state().sr == wanted.status,
            "REP retires one native body instruction after the scaled count transfer");
  }
}

void test_source_timing() {
  DspProgram program;
  set_accumulator(program, "a", half);
  set_accumulator(program, "b", 0);
  program.immediate("x0", half >> word_bits);
  program.immediate("sr", masked_status);
  const auto add = program.pc();
  program.emit("add x0,a a,y0");
  DspBus bus;
  program.install(bus);
  DspCore cpu(bus);
  run_to(cpu, add);
  cpu.step();
  require(cpu.state().a == half * 2 && cpu.state().y == (half >> word_bits) &&
              cpu.state().sr == (masked_status | extension | sticky),
          "Parallel transfer reads pre-ALU accumulator and growth before ADD overwrites it");
}

void test_unselected_growth_and_sr_write() {
  DspProgram program;
  set_accumulator(program, "a", 0);
  set_accumulator(program, "b", half);
  program.immediate("sr", masked_status);
  const auto transfer = program.pc();
  program.emit("move a,x0");
  set_accumulator(program, "a", half);
  // Scaled result is itself a legal SR value, including explicitly cleared S.
  constexpr unsigned replacement = 0x400000;
  program.emit("move a,sr");
  DspBus bus;
  program.install(bus);
  DspCore cpu(bus);
  run_to(cpu, transfer);
  cpu.step();
  require(cpu.state().sr == masked_status && cpu.state().x == 0,
          "Unselected accumulator growth does not set S on another accumulator's transfer");
  for (unsigned count = 0; count < 4; ++count) { cpu.step(); }
  require(cpu.state().sr == replacement,
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
