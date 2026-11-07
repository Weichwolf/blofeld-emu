// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_io.hpp"
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
constexpr unsigned transfer_address = 0x100;
constexpr unsigned vsl_base = 0x0ac0c0;
constexpr unsigned source_bit = 16;
constexpr unsigned ea_bit = 8;
constexpr unsigned insert_bit = 4;
constexpr unsigned address_offset = 3;
constexpr unsigned neighbor_cookie = 0x5a7bc9;
constexpr unsigned matrix_stride = 16;
// FM12-21/22,13-118/131: literal long-immediate register seeds.
constexpr unsigned move_r0 = 0x60f400;
constexpr unsigned move_n0 = 0x70f400;
constexpr unsigned move_m0 = 0x05f420;
constexpr unsigned move_sr = 0x05f439;
constexpr unsigned register_shift = 16;
constexpr std::array a_seeds{0x50f400U, 0x54f400U, 0x52f400U};
constexpr std::array b_seeds{0x51f400U, 0x55f400U, 0x53f400U};

void immediate(Program &program, const unsigned opcode, const unsigned value) {
  program.word(opcode);
  program.word(value);
}

void set_accumulator(Program &program, const bool accumulator_b, const std::uint64_t value) {
  const auto &seeds = accumulator_b ? b_seeds : a_seeds;
  immediate(program, seeds[0], static_cast<unsigned>(value & word_mask));
  immediate(program, seeds[1], static_cast<unsigned>((value >> word_bits) & word_mask));
  immediate(program, seeds[2], static_cast<unsigned>(value >> data_bits));
}

void run_to(DspMachine &machine, const unsigned pc) {
  unsigned count = 0;
  while (machine.cpu().registers().pc != pc) {
    constexpr unsigned instruction_limit = 100;
    require(++count <= instruction_limit, "Native VSL diagnostic exceeded its setup limit");
    native::step_native(machine);
  }
}

struct Input {
  std::uint64_t value = 0;
  unsigned status = 0;
  unsigned reg = 0;
  unsigned mode = 0;
  bool source_b = false;
  bool insert = false;
};

unsigned opcode(const Input input) {
  const unsigned encoded_reg = input.mode == 6 ? 0 : input.reg;
  return vsl_base | (static_cast<unsigned>(input.source_b) << source_bit) |
         ((input.mode * 8 + encoded_reg) << ea_bit) |
         (static_cast<unsigned>(input.insert) << insert_bit);
}

unsigned target(const Input input, const unsigned base) {
  if (input.mode == 5) { return base + address_offset; }
  if (input.mode == 7) { return base - 1; }
  return base;
}

unsigned next_r(const Input input, const unsigned base) {
  switch (input.mode) {
    case 0: return base - address_offset;
    case 1: return base + address_offset;
    case 2:
    case 7: return base - 1;
    case 3: return base + 1;
    default: return base;
  }
}

unsigned initialize(Program &program, const Input input, const unsigned base) {
  set_accumulator(program, input.source_b, input.value);
  set_accumulator(program, !input.source_b, other_accumulator);
  for (unsigned reg = 0; reg < 8; ++reg) {
    immediate(
        program, move_r0 | (reg << register_shift), reg == input.reg ? base : address_seed + reg);
    immediate(program, move_n0 | (reg << register_shift), address_offset);
    immediate(program, move_m0 | reg, word_mask);
  }
  immediate(program, move_sr, input.status);
  const auto pc = program.pc();
  program.word(opcode(input));
  if (input.mode == 6) { program.word(base); }
  return pc;
}

void seed_neighbors(DspBus &bus, const unsigned address) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned offset : {0U, 1U, 2U}) {
      bus.write({.space = space, .address = address + offset - 1, .value = neighbor_cookie});
    }
  }
}

void check_words(DspBus &bus, const Input input, const unsigned address) {
  // FM 13-182: raw 24-bit MSP, and LSP shifted once with opcode-bit insertion.
  const auto high = static_cast<unsigned>((input.value >> word_bits) & word_mask);
  const auto low = static_cast<unsigned>(
      ((input.value & word_mask) * 2 + static_cast<unsigned>(input.insert)) & word_mask);
  require(bus.read({.space = DspSpace::x, .address = address}) == high &&
              bus.read({.space = DspSpace::y, .address = address}) == low,
          "VSL raw pair mismatch: opcode=" + number(opcode(input)) +
              " value=" + number(input.value) + " sr=" + number(input.status));
}

void check(DspMachine &machine,
           const dsp56300::Registers &before,
           const Input input,
           const unsigned base) {
  auto &bus = machine.bus();
  const auto after = machine.cpu().registers();
  auto expected = before;
  expected.r[input.reg] = next_r(input, base);
  expected.pc += input.mode == 6 ? 2U : 1U;
  require(after == expected,
          "VSL must update its exact EA/PC and preserve every other register, flag and stack cell");
  check_words(bus, input, target(input, base));
  const auto address = target(input, base);
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    require(bus.read({.space = space, .address = address - 1}) == neighbor_cookie &&
                bus.read({.space = space, .address = address + 1}) == neighbor_cookie,
            "VSL modified adjacent memory");
  }
}

void run_batch(const std::span<const Input> batch) {
  Program program;
  DspMachine machine(native::native_clocks);
  std::vector<unsigned> pcs;
  for (std::size_t index = 0; index < batch.size(); ++index) {
    const auto base = transfer_address + (static_cast<unsigned>(index) * matrix_stride);
    pcs.push_back(initialize(program, batch[index], base));
    seed_neighbors(machine.bus(), target(batch[index], base));
  }
  native::reset_native(machine, program);
  for (std::size_t index = 0; index < batch.size(); ++index) {
    run_to(machine, pcs[index]);
    const auto before = machine.cpu().registers();
    native::step_native(machine);
    check(machine,
          before,
          batch[index],
          transfer_address + (static_cast<unsigned>(index) * matrix_stride));
  }
}

void append_modes(std::vector<Input> &inputs, const Input base) {
  for (unsigned reg = 0; reg < 8; ++reg) {
    for (unsigned mode = 0; mode < 8; ++mode) {
      auto input = base;
      input.reg = reg;
      input.mode = mode;
      inputs.push_back(input);
    }
  }
}

void test_matrix() {
  constexpr std::array<std::uint64_t, 6> values{
      0, 0x7f400000923456, 0x00800000800000, 0x80000000000000, 0xff800000ffffff, 0xffffffffffffff};
  std::vector<Input> inputs;
  for (const auto value : values) {
    for (const unsigned scaling : {0U, scale_down, scale_up}) {
      for (const unsigned flags : {0U, 0xffU}) {
        for (const bool source_b : {false, true}) {
          for (const bool insert : {false, true}) {
            append_modes(inputs,
                         {.value = value,
                          .status = masked_status | scaling | flags,
                          .source_b = source_b,
                          .insert = insert});
          }
        }
      }
    }
  }
  constexpr unsigned expected_cases = 9216;
  require(inputs.size() == expected_cases, "VSL matrix must cover each declared combination");
  constexpr std::size_t batch_size = 32;
  for (std::size_t offset = 0; offset < inputs.size(); offset += batch_size) {
    run_batch(std::span(inputs).subspan(offset, std::min(batch_size, inputs.size() - offset)));
  }
  std::cout << inputs.size()
            << " independent VSL value/scaling/flags/A-B/insert/Rn/EA vectors passed\n";
}

void test_modulo_and_repeat() {
  Program program;
  constexpr std::uint64_t accumulator = 0x7f400000923456;
  constexpr unsigned boundary = transfer_address + 3;
  set_accumulator(program, false, accumulator);
  immediate(program, move_r0, boundary);
  immediate(program, move_m0, 3);
  constexpr unsigned status = masked_status | scale_down | carry | zero;
  immediate(program, move_sr, status);
  constexpr unsigned repeat_three = 0x0603a0;
  program.word(repeat_three);
  const auto body = program.pc();
  program.word(vsl_base | (3U * 8U << ea_bit) | (1U << insert_bit));
  DspMachine machine(native::native_clocks);
  native::reset_native(machine, program);
  run_to(machine, body);
  for (const unsigned address : {boundary, transfer_address, transfer_address + 1}) {
    native::step_native(machine);
    check_words(machine.bus(), {.value = accumulator, .insert = true}, address);
    require(machine.cpu().registers().sr == status && machine.cpu().registers().a == accumulator,
            "Repeated VSL preserves raw source and flags");
  }
  require(machine.cpu().registers().r[0] == transfer_address + 2 &&
              machine.cpu().registers().pc == body + 1 && machine.cpu().registers().lc == 0,
          "REP VSL advances modulo address once per native iteration and retires normally");
}

void expect_fault(DspMachine &machine, const dsp56300::Fault expected) {
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  bool rejected = false;
  try {
    native::step_native(machine);
  } catch (const Fault &) { rejected = true; }
  const auto observed = machine.cpu().fault();
  require(rejected && observed == expected && machine.cpu().registers() == before &&
              machine.cpu().progress().instructions == progress.instructions,
          "VSL fault must retain exact kind/space/address/value/PC without retirement: kind=" +
              std::to_string(static_cast<unsigned>(observed.kind)) +
              " address=" + number(observed.address) + " value=" + number(observed.value) +
              " pc=" + number(observed.pc));
}

void check_bus_fault(const DspMachine &machine, const std::string &detail) {
  const auto fault = machine.cpu().fault();
  const auto failure = machine.io().failure();
  if (!failure || !failure->cause) { throw Fault("VSL bus fault lost its board exception"); }
  require(failure->space == fault.space && failure->address == fault.address &&
              failure->pc == fault.pc &&
              failure->operation == (fault.kind == dsp56300::FaultKind::memory_write
                                         ? DspIo::Operation::write
                                         : DspIo::Operation::read),
          "Rejected VSL bus operation must retain its board context and cause");
  try {
    std::rethrow_exception(failure->cause);
  } catch (const Fault &error) {
    require(std::string{error.what()}.contains(detail),
            "Rejected VSL bus operation must retain its original board diagnostic");
  }
}

void test_faults() {
  constexpr unsigned reserved_address = 0x3000;
  constexpr unsigned rom_address = 0x4000;
  for (const unsigned address : {reserved_address, rom_address}) {
    Program program;
    set_accumulator(program, false, 0);
    const auto pc = program.pc();
    program.word(vsl_base | (6U * 8U << ea_bit));
    program.word(address);
    DspMachine machine(native::native_clocks);
    native::reset_native(machine, program);
    run_to(machine, pc);
    expect_fault(machine,
                 {.kind = dsp56300::FaultKind::memory_write,
                  .space = dsp56300::Space::x,
                  .address = address,
                  .pc = pc});
    check_bus_fault(machine, address == reserved_address ? "0x003000" : "ROM");
  }
  for (const bool invalid_immediate : {false, true}) {
    Program program;
    const auto opcode = vsl_base | ((6U * 8U + (invalid_immediate ? 4U : 0U)) << ea_bit);
    program.word(opcode);
    // Supply the extension for the reserved EA form so its rejection cannot be
    // confused with the separate unknown-program-sample case.
    if (invalid_immediate) { program.word(transfer_address); }
    DspMachine machine(native::native_clocks);
    native::reset_native(machine, program);
    expect_fault(machine,
                 {.kind = invalid_immediate ? dsp56300::FaultKind::invalid_instruction
                                            : dsp56300::FaultKind::memory_read,
                  .address = invalid_immediate ? native::boot : native::boot + 1,
                  .value = invalid_immediate ? opcode : 0,
                  .pc = native::boot});
    if (!invalid_immediate) { check_bus_fault(machine, "P:"); }
  }
}

void test_supported_cache_control() {
  Program program;
  constexpr unsigned pflush = 3;
  program.word(pflush);
  DspMachine machine(native::native_clocks);
  native::reset_native(machine, program);
  auto expected = machine.cpu().registers();
  ++expected.pc;
  native::step_native(machine);
  require(machine.cpu().registers() == expected,
          "Supported PFLUSH must retire while preserving every other architectural register");
}
}
}

int main() {
  try {
    blofeld::test::test_matrix();
    blofeld::test::test_modulo_and_repeat();
    blofeld::test::test_faults();
    blofeld::test::test_supported_cache_control();
    std::cout << "Native VSL REP/modulo, four contextual refusals and supported PFLUSH passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
