// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_instruction_fixture.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace blofeld::test {
namespace {
constexpr unsigned word_bytes = 3;
constexpr unsigned initial_r = 10;
constexpr unsigned initial_n = 11;
constexpr unsigned initial_m = 12;
constexpr unsigned initial_status = 13;
constexpr unsigned status_mask = 0x2cff;
constexpr unsigned data_stride = 0x010101;
constexpr unsigned jump_absolute = 0x0af080;

void install_setup(DspBus &bus, const InstructionInput &input) {
  DspProgram program;
  constexpr std::array<const char *, 10> data_registers{
      "x0", "x1", "y0", "y1", "a0", "a1", "a2", "b0", "b1", "b2"};
  for (std::size_t index = 0; index < data_registers.size(); ++index) {
    const bool extension_register = index == 6 || index == 9;
    constexpr unsigned extension_mask = 0xff;
    program.immediate(data_registers.at(index),
                      input.registers.at(index) &
                          (extension_register ? extension_mask : word_mask));
  }
  for (unsigned index = 0; index < 8; ++index) {
    const auto suffix = std::to_string(index);
    program.immediate("r" + suffix,
                      index == 0 ? input.registers.at(initial_r) : instruction_data_begin);
    program.immediate("n" + suffix, index == 0 ? input.registers.at(initial_n) : 1);
    program.immediate("m" + suffix, index == 0 ? input.registers.at(initial_m) : word_mask);
  }
  program.immediate("sz", 0);
  program.immediate("ep", 0);
  program.immediate("sp", 0);
  for (unsigned index = 0; index < 15; ++index) {
    program.immediate("ssh", 0);
    program.immediate("ssl", 0);
  }
  program.immediate("sp", 0);
  program.immediate("sc", 0);
  program.immediate("la", word_mask);
  program.immediate("lc", 0);
  program.immediate("vba", 0);
  program.immediate("omr", 5);
  program.immediate("sr", masked_status | (input.registers.at(initial_status) & status_mask));
  program.word(jump_absolute);
  program.word(0);
  program.install(bus);
  for (std::size_t index = 0; index < input.program.size(); ++index) {
    bus.write({.address = static_cast<std::uint32_t>(index), .value = input.program.at(index)});
  }
  for (unsigned index = 0; index < instruction_data_words; ++index) {
    const auto offset = index * data_stride;
    bus.write({.space = DspSpace::x,
               .address = instruction_data_begin + index,
               .value = input.registers.at(0) ^ offset});
    bus.write({.space = DspSpace::y,
               .address = instruction_data_begin + index,
               .value = input.registers.at(2) ^ offset});
  }
}

void verify_fail_stop(DspCore &cpu) {
  try {
    cpu.step();
  } catch (const Fault &error) {
    require(std::string(error.what()).contains("requires reset after a rejected reference step"),
            "A native fuzz fault must preserve the core's fail-stop contract");
    return;
  }
  throw Fault("Native instruction fuzzing resumed a rejected CPU");
}

void snapshot_data(DspBus &bus, InstructionResult &result) {
  for (unsigned index = 0; index < instruction_data_words; ++index) {
    result.x.at(index) =
        bus.read({.space = DspSpace::x, .address = instruction_data_begin + index});
    result.y.at(index) =
        bus.read({.space = DspSpace::y, .address = instruction_data_begin + index});
  }
}
}

std::optional<InstructionInput>
decode_instruction_input(const std::span<const std::uint8_t> bytes) {
  constexpr std::size_t header_bytes =
      static_cast<std::size_t>(instruction_seed_words) * word_bytes;
  if (bytes.size() < header_bytes + word_bytes || bytes.size() > instruction_input_bytes ||
      bytes.size() % word_bytes != 0) {
    return std::nullopt;
  }
  InstructionInput result;
  for (std::size_t offset = 0; offset < bytes.size(); offset += word_bytes) {
    const auto value = (static_cast<std::uint32_t>(bytes[offset]) << 16U) |
                       (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U) | bytes[offset + 2];
    if (offset < header_bytes) {
      result.registers.at(offset / word_bytes) = value;
    } else {
      result.program.push_back(value);
    }
  }
  return result;
}

InstructionResult execute_instruction_input(const InstructionInput &input,
                                            const unsigned step_budget) {
  require(!input.program.empty() && input.program.size() <= instruction_program_words &&
              step_budget != 0 && step_budget <= instruction_step_limit,
          "Invalid own native instruction fixture bounds");
  DspBus bus;
  install_setup(bus, input);
  DspCore cpu(bus);
  unsigned setup_steps = 0;
  while (cpu.state().pc != 0) {
    constexpr unsigned setup_limit = 256;
    require(++setup_steps <= setup_limit, "Own native instruction setup exceeded its bound");
    cpu.step();
  }
  InstructionResult result;
  while (result.steps < step_budget) {
    try {
      cpu.step();
      ++result.steps;
    } catch (const Fault &error) {
      result.stop = InstructionStop::rejected;
      result.fault = error.what();
      verify_fail_stop(cpu);
      break;
    }
  }
  result.state = cpu.debug_state();
  snapshot_data(bus, result);
  return result;
}
}
