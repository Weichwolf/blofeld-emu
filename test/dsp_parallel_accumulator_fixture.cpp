// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_parallel_accumulator_fixture.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <string>

namespace blofeld::test::parallel_accumulator {
namespace {
std::int64_t signed_value(const std::uint64_t value) {
  const auto positive = static_cast<std::int64_t>(value);
  return (value & sign_bit) != 0 ? positive - static_cast<std::int64_t>(accumulator_range)
                                 : positive;
}

void initialize(DspProgram &program, const Input input) {
  program.immediate("omr", reset_omr);
  program.immediate("sr", masked_status);
  constexpr std::array names{"x0", "x1", "y0", "y1"};
  constexpr std::array seeds{0x654321U, 0x123456U, 0x2468acU, 0x13579bU};
  for (unsigned index = 0; index < names.size(); ++index) {
    program.immediate(names.at(index), seeds.at(index));
  }
  for (const bool b : {false, true}) {
    const auto value = b ? input.value.b : input.value.a;
    for (unsigned part = 0; part < 3; ++part) {
      program.immediate(std::string(b ? "b" : "a") + std::to_string(part),
                        (value >> (word_bits * part)) & (part == 2 ? extension_mask : word_mask));
    }
  }
  const auto index = std::to_string(input.reg);
  program.immediate("r" + index, pointer);
  program.immediate("n" + index, offset);
  program.immediate("m" + index, word_mask);
  program.immediate("sr", input.status);
  for (unsigned quiet = 0; quiet < 8; ++quiet) { program.word(0); }
}

void cells(DspMachine &machine, const Input input, const bool unknown) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto location : {address(input) - 1, address(input), address(input) + 1}) {
      const bool selected = (space == DspSpace::y) == input.y && location == address(input);
      if (selected && unknown) { continue; }
      machine.bus().write(
          {.space = space, .address = location, .value = selected ? input.word : canary});
    }
  }
}
}

Sum addition(const Pair input, const unsigned status) {
  const auto mathematical = signed_value(input.a) + signed_value(input.b);
  auto result = static_cast<std::uint64_t>(mathematical) & accumulator_mask;
  const auto bound = static_cast<std::int64_t>(sign_bit);
  bool overflowed = mathematical < -bound || mathematical >= bound;
  if ((status & saturation_mode) != 0) {
    // FM3.2.3/Table3-1: sign,extension LSB and fraction MSB select saturation.
    const auto selector =
        ((result >> 55U) << 2U) | (((result >> 48U) & 1U) << 1U) | ((result >> 47U) & 1U);
    constexpr std::array action{0U, 1U, 1U, 1U, 2U, 2U, 2U, 0U};
    const auto selected = action.at(selector);
    overflowed = selected != 0;
    constexpr std::uint64_t positive_limit = 0x007fffffffffff;
    constexpr std::uint64_t negative_limit = 0xff800000000000;
    if (selected != 0) { result = selected == 1 ? positive_limit : negative_limit; }
  }
  const auto flags = value_flags({.value = result, .status = status}) |
                     (input.a + input.b >= accumulator_range ? carry : 0U) |
                     (overflowed ? overflow | limit : 0U);
  // Memory-to-accumulator does not alter S; L is sticky and also records V.
  constexpr unsigned overwritten = carry | overflow | zero | negative | unnormalized | extension;
  return {.value = result, .status = (status & ~overwritten) | flags};
}

std::uint64_t loaded(const unsigned word) {
  constexpr unsigned word_sign = 0x800000;
  constexpr std::uint64_t sign_extension = 0xff000000000000;
  return (static_cast<std::uint64_t>(word) << word_bits) |
         ((word & word_sign) != 0 ? sign_extension : 0U);
}

unsigned opcode(const Input input) {
  // FM13-118/122/Table12-14: W=1/EA=1,full A/B,distinct literal ADD destination.
  constexpr unsigned x_a = 0x56c000;
  constexpr unsigned x_b = 0x57c000;
  constexpr unsigned y_bit = 0x80000;
  constexpr unsigned add_a_b = 0x18;
  constexpr unsigned add_b_a = 0x10;
  return (input.b ? x_b : x_a) | (input.y ? y_bit : 0U) | (input.mode << 11U) | (input.reg << 8U) |
         (input.b ? add_b_a : add_a_b);
}

unsigned address(const Input input) {
  if (input.mode == 5) { return pointer + offset; }
  return input.mode == 7 ? pointer - 1 : pointer;
}

unsigned updated_pointer(const Input input) {
  switch (input.mode) {
    case 0: return pointer - offset;
    case 1: return pointer + offset;
    case 2:
    case 7: return pointer - 1;
    case 3: return pointer + 1;
    case 4:
    case 5: return pointer;
    default: throw Fault("Unqualified own ADD/load address mode");
  }
}

DspDebugState prepare(DspMachine &machine,
                      const Input input,
                      const std::span<const std::uint32_t> body,
                      const bool unknown) {
  DspProgram program;
  initialize(program, input);
  const auto entry = program.pc();
  for (const auto word : body) { program.word(word); }
  for (unsigned padding = 0; padding < tail; ++padding) { program.word(0); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  cells(machine, input, unknown);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.a == input.value.a && before.cpu.b == input.value.b &&
              before.cpu.sr == input.status && before.cpu.r.at(input.reg) == pointer &&
              before.cpu.n.at(input.reg) == offset && before.cpu.m.at(input.reg) == word_mask,
          "Genuine own setup establishes full old operands/status/linear AGU state");
  return before;
}

void sources(DspMachine &machine, const Input input) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto location : {address(input) - 1, address(input), address(input) + 1}) {
      const bool selected = (space == DspSpace::y) == input.y && location == address(input);
      require(machine.bus().read({.space = space, .address = location}) ==
                  (selected ? input.word : canary),
              "Read preserves selected/neighboring/opposite-space cells");
    }
  }
}
}
