// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
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
constexpr unsigned vsl_base = 0x0ac0c0;
constexpr unsigned source_bit = 16;
constexpr unsigned ea_bit = 8;
constexpr unsigned insert_bit = 4;
constexpr unsigned address_offset = 3;
constexpr unsigned neighbor_cookie = 0x5a7bc9;
constexpr unsigned matrix_stride = 16;

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

unsigned initialize(DspProgram &program, const Input input, const unsigned base) {
  set_accumulator(program, input.source_b ? "b" : "a", input.value);
  set_accumulator(program, input.source_b ? "a" : "b", other_accumulator);
  for (unsigned reg = 0; reg < 8; ++reg) {
    const auto name = std::to_string(reg);
    program.immediate("r" + name, reg == input.reg ? base : address_seed + reg);
    program.immediate("n" + name, address_offset);
    program.immediate("m" + name, word_mask);
  }
  program.immediate("sr", input.status);
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

void check(DspBus &bus,
           const DspState &before,
           const DspState &after,
           const Input input,
           const unsigned base) {
  auto addresses = before.r;
  addresses[input.reg] = next_r(input, base);
  require(after.r == addresses && after.n == before.n && after.m == before.m,
          "VSL effective-address update must happen exactly once");
  require(after.a == before.a && after.b == before.b && after.sr == before.sr &&
              after.x == before.x && after.y == before.y && after.sp == before.sp &&
              after.lc == before.lc && after.la == before.la && after.omr == before.omr,
          "VSL must preserve both accumulators, every status bit and unselected state");
  require(after.pc == before.pc + (input.mode == 6 ? 2U : 1U),
          "VSL PC must consume its optional absolute effective-address extension");
  check_words(bus, input, target(input, base));
  const auto address = target(input, base);
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    require(bus.read({.space = space, .address = address - 1}) == neighbor_cookie &&
                bus.read({.space = space, .address = address + 1}) == neighbor_cookie,
            "VSL modified adjacent memory");
  }
}

void run_batch(DspBus &bus, DspCore &cpu, const std::span<const Input> batch) {
  DspProgram program;
  std::vector<unsigned> pcs;
  for (std::size_t index = 0; index < batch.size(); ++index) {
    const auto base = transfer_address + (static_cast<unsigned>(index) * matrix_stride);
    pcs.push_back(initialize(program, batch[index], base));
    seed_neighbors(bus, target(batch[index], base));
  }
  program.install(bus);
  cpu.reset_spi_boot();
  for (std::size_t index = 0; index < batch.size(); ++index) {
    run_to(cpu, pcs[index]);
    const auto before = cpu.state();
    cpu.step();
    check(bus,
          before,
          cpu.state(),
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
  DspBus bus;
  DspCore cpu(bus);
  constexpr std::size_t batch_size = 32;
  for (std::size_t offset = 0; offset < inputs.size(); offset += batch_size) {
    run_batch(
        bus, cpu, std::span(inputs).subspan(offset, std::min(batch_size, inputs.size() - offset)));
  }
  std::cout << inputs.size()
            << " independent VSL value/scaling/flags/A-B/insert/Rn/EA vectors passed\n";
}

void test_modulo_and_repeat() {
  DspProgram program;
  constexpr std::uint64_t accumulator = 0x7f400000923456;
  constexpr unsigned boundary = transfer_address + 3;
  set_accumulator(program, "a", accumulator);
  program.immediate("r0", boundary);
  program.immediate("m0", 3);
  constexpr unsigned status = masked_status | scale_down | carry | zero;
  program.immediate("sr", status);
  program.emit("rep #3");
  const auto body = program.pc();
  program.word(vsl_base | (3U * 8U << ea_bit) | (1U << insert_bit));
  DspBus bus;
  program.install(bus);
  DspCore cpu(bus);
  run_to(cpu, body);
  for (const unsigned address : {boundary, transfer_address, transfer_address + 1}) {
    cpu.step();
    check_words(bus, {.value = accumulator, .insert = true}, address);
    require(cpu.state().sr == status && cpu.state().a == accumulator,
            "Repeated VSL preserves raw source and flags");
  }
  require(cpu.state().r[0] == transfer_address + 2 && cpu.state().pc == body + 1 &&
              cpu.state().lc == 0,
          "REP VSL advances modulo address once per native iteration and retires normally");
}

void expect_fault(DspCore &cpu, const std::string &operation, const std::string &detail) {
  try {
    cpu.step();
    throw Fault("Expected strict VSL/unsupported-instruction failure");
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(message.contains(operation) && message.contains(detail) && message.contains("pc="),
            "Missing original instruction fault/context: " + message);
  }
}

void test_faults() {
  constexpr unsigned reserved_address = 0x3000;
  constexpr unsigned rom_address = 0x4000;
  for (const unsigned address : {reserved_address, rom_address}) {
    DspProgram program;
    set_accumulator(program, "a", 0);
    const auto pc = program.pc();
    program.word(vsl_base | (6U * 8U << ea_bit));
    program.word(address);
    DspBus bus;
    program.install(bus);
    DspCore cpu(bus);
    run_to(cpu, pc);
    expect_fault(cpu, "write", address == reserved_address ? "0x003000" : "ROM");
  }
  for (const bool invalid_immediate : {false, true}) {
    DspProgram program;
    program.word(vsl_base | ((6U * 8U + (invalid_immediate ? 4U : 0U)) << ea_bit));
    DspBus bus;
    program.install(bus);
    DspCore cpu(bus);
    expect_fault(cpu,
                 invalid_immediate ? "Unsupported native DSP opcode" : "read",
                 invalid_immediate ? "DSP" : "P:");
  }
  DspProgram program;
  constexpr unsigned pflush = 3;
  program.word(pflush);
  DspBus bus;
  program.install(bus);
  DspCore cpu(bus);
  expect_fault(cpu, "Unimplemented DSP instruction", "PFLUSH");
}
}
}

int main() {
  try {
    blofeld::test::test_matrix();
    blofeld::test::test_modulo_and_repeat();
    blofeld::test::test_faults();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
