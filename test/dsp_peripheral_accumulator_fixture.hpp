// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_transfer_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::peripheral_accumulator {
struct Port {
  DspSpace space;
  unsigned address, bits;
  DspPort gpio = DspPort::c;
};

inline constexpr std::array ports{
    Port{.space = DspSpace::x, .address = 0xffffef, .bits = 24},
    Port{.space = DspSpace::x, .address = 0xffff89, .bits = 24},
    Port{.space = DspSpace::y, .address = 0xfffff5, .bits = 11, .gpio = DspPort::f},
    Port{.space = DspSpace::y, .address = 0xffff9d, .bits = 12, .gpio = DspPort::e}};
inline constexpr unsigned entry = 0x100;
inline constexpr unsigned source_data = 0x400;
inline constexpr unsigned canary = 0x654321;
inline constexpr unsigned ordinary_omr = 0x305;
inline constexpr unsigned saturation = 0x100000;
inline constexpr unsigned all_ccr = 0xff;
inline constexpr unsigned positive_edge = 0x7fffff;
inline constexpr unsigned negative_edge = 0x800000;
inline constexpr unsigned high_begin = 0xffffc0;
inline constexpr unsigned low_begin = 0xffff80;
inline constexpr unsigned bcr = 0xfffffb;
inline constexpr unsigned pin_bcr = 0x01ffff;
inline constexpr unsigned absolute_jump = 0x0af080;
inline constexpr std::uint64_t seed_a = 0x12a1b2c3d4e5f6;
inline constexpr std::uint64_t seed_b = 0xe93f4a5b6c7d8e;
inline constexpr std::array values{
    0U, 1U, 0x7fffU, 0x8000U, 0x7fffffU, 0x800000U, 0xffff80U, 0xffffffU};

struct Input {
  unsigned port = 0, value = 0, status = masked_status;
  bool b = false;
};

inline unsigned source_value(const Input input) {
  return input.value & ((1U << ports.at(input.port).bits) - 1);
}

inline std::uint64_t loaded(const Input input) {
  constexpr unsigned sign = 0x800000;
  constexpr std::uint64_t negative_extension = 0xff000000000000;
  const auto value = source_value(input);
  return (static_cast<std::uint64_t>(value) << word_bits) |
         ((value & sign) != 0 ? negative_extension : 0);
}

inline std::string reader(const Input input) {
  const auto port = ports.at(input.port);
  return "movep " + std::string(port.space == DspSpace::x ? "x" : "y") + ":<<" +
         number(port.address) + ',' + (input.b ? "b" : "a");
}

inline unsigned opcode(const Input input) {
  // FM13-136: independently pinned full destination fields14/15, distinct from raw A1/B1.
  constexpr unsigned high = 0x084000;
  constexpr unsigned low_x = 0x044080;
  constexpr unsigned low_y = 0x044020;
  constexpr unsigned y_bit = 0x10000;
  constexpr unsigned destination_shift = 8;
  constexpr unsigned a_field = 14;
  const auto port = ports.at(input.port);
  const bool y = port.space == DspSpace::y;
  const bool upper = port.address >= high_begin;
  const auto offset = port.address - (upper ? high_begin : low_begin);
  const auto high_base = high | (y ? y_bit : 0U);
  const auto low_base = y ? low_y : low_x;
  const auto base = upper ? high_base : low_base;
  constexpr unsigned split_bit = 0x20;
  constexpr unsigned low_mask = 0x1f;
  return base | ((a_field + static_cast<unsigned>(input.b)) << destination_shift) |
         (upper ? offset : ((offset & split_bit) << 1) | (offset & low_mask));
}

inline void seed_source(DspMachine &machine, const Input input) {
  const auto port = ports.at(input.port);
  if (port.space == DspSpace::x) {
    machine.bus().write(
        {.space = port.space, .address = port.address, .value = source_value(input)});
    return;
  }
  const auto mask = (1U << port.bits) - 1;
  auto &gpio = machine.bus().gpio(port.gpio);
  gpio.write({.reg = DspGpioRegister::control});
  gpio.write({.reg = DspGpioRegister::direction, .value = mask});
  gpio.write({.reg = DspGpioRegister::data, .value = source_value(input)});
}

inline DspDebugState prepare(DspMachine &machine, const Input input, const bool known = true) {
  DspProgram program;
  program.immediate("omr", ordinary_omr);
  program.emit("movep #>" + number(pin_bcr) + ",x:<<" + number(bcr));
  set_accumulator(program, "a", seed_a);
  set_accumulator(program, "b", seed_b);
  program.immediate("r4", source_data);
  program.immediate("x0", canary);
  program.immediate("sr", input.status);
  for (unsigned index = 0; index < 8; ++index) { program.word(0); }
  program.word(absolute_jump);
  program.word(entry);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, entry);
  for (unsigned index = 0; index < 32; ++index) { machine.bus().write({.address = entry + index}); }
  machine.bus().write({.address = entry, .value = opcode(input)});
  machine.bus().write({.space = DspSpace::y, .address = source_data, .value = canary});
  if (known) { seed_source(machine, input); }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.a == seed_a && before.cpu.b == seed_b && before.cpu.sr == input.status,
          "Actual native instructions establish all accumulator parts and status seeds");
  return before;
}

inline DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

inline void load_result(DspDebugState &state, const Input input) {
  (input.b ? state.cpu.b : state.cpu.a) = loaded(input);
}

inline void matrix(const auto &visit) {
  for (unsigned port = 0; port < ports.size(); ++port) {
    for (const bool b : {false, true}) {
      for (const auto value : values) {
        for (const unsigned scale : {0U, scale_down, scale_up}) {
          for (const unsigned flags : {0U, all_ccr}) {
            for (const unsigned sm : {0U, saturation}) {
              visit(Input{.port = port,
                          .value = value,
                          .status = masked_status | scale | flags | sm,
                          .b = b});
            }
          }
        }
      }
    }
  }
}

void native_checks();
void dependency_checks();
void guard_checks();
}
