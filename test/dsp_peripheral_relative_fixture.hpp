// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <string>

namespace blofeld::test::peripheral_relative {
struct Port {
  DspPort port;
  DspSpace space;
  unsigned address, bits;
};

inline constexpr std::array ports{
    Port{.port = DspPort::c, .space = DspSpace::x, .address = 0xffffbd, .bits = 12},
    Port{.port = DspPort::e, .space = DspSpace::y, .address = 0xffff9d, .bits = 12},
    Port{.port = DspPort::f, .space = DspSpace::y, .address = 0xfffff5, .bits = 11},
    Port{.port = DspPort::d, .space = DspSpace::x, .address = 0xffffd5, .bits = 2}};
inline constexpr unsigned ordinary_omr = 0x305;
inline constexpr unsigned status_all = 0xc003ff;
inline constexpr unsigned entry = 0x100;
inline constexpr unsigned timer_control = 0xffff8b;
inline constexpr unsigned timer_bit = 12;
inline constexpr unsigned low_begin = 0xffff80;
inline constexpr unsigned high_begin = 0xffffc0;
inline constexpr unsigned high_clear = 0x0cc000;
inline constexpr unsigned low_clear = 0x048000;
inline constexpr unsigned set_bit = 0x20;
inline constexpr unsigned space_bit = 0x40;
inline constexpr unsigned operand_shift = 8;
inline constexpr unsigned long_jump = 0x0af080;
inline constexpr unsigned bcr = 0xfffffb;
inline constexpr unsigned pin_bcr = 0x01ffff;
inline constexpr unsigned padding = 64;

struct Input {
  unsigned port = 0, bit = 0;
  bool set = false, level = false, flags = false, timer = false;
  unsigned start = entry, displacement = 32;
};

inline DspAddress operand(const Input input) {
  const auto port = ports.at(input.port);
  return input.timer ? DspAddress{.space = DspSpace::x, .address = timer_control}
                     : DspAddress{.space = port.space, .address = port.address};
}

// Independent FM13-27/29 templates, checked with48 Freescale encodings.
inline unsigned opcode(const Input input) {
  const auto address = operand(input);
  const bool high = address.address >= high_begin;
  return (high ? high_clear : low_clear) | (input.set ? set_bit : 0U) |
         (address.space == DspSpace::y ? space_bit : 0U) |
         ((address.address - (high ? high_begin : low_begin)) << operand_shift) | input.bit;
}

inline bool taken(const Input input) {
  return input.level == input.set;
}

inline unsigned target(const Input input) {
  return (input.start + input.displacement) & word_mask;
}

inline DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

inline void store_word(DspBus &bus, const DspStore word) {
  if (word.address >= boot) {
    const std::array data{word.value};
    bus.install_rom({.start = word.address, .words = data});
  } else {
    bus.write(word);
  }
}

inline DspDebugState prepare(DspMachine &machine, const Input input) {
  DspProgram code;
  code.immediate("omr", ordinary_omr);
  code.emit("movep #>" + number(pin_bcr) + ",x:<<" + number(bcr));
  code.immediate("sr", input.flags ? status_all : masked_status);
  for (unsigned index = 0; index < 8; ++index) { code.word(0); }
  code.word(long_jump);
  code.word(input.start);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::until(machine, input.start);
  if (input.timer) {
    machine.bus().timers().set_tio_drive(1, input.level);
  } else {
    const auto port = ports.at(input.port);
    const auto mask = (1U << port.bits) - 1;
    auto &gpio = machine.bus().gpio(port.port);
    gpio.write({.reg = DspGpioRegister::control});
    gpio.write({.reg = DspGpioRegister::direction, .value = mask});
    gpio.write(
        {.reg = DspGpioRegister::data, .value = input.level ? mask : mask & ~(1U << input.bit)});
  }
  if (input.start == entry) {
    constexpr unsigned before = entry - padding;
    for (unsigned index = 0; index < 3 * padding; ++index) {
      machine.bus().write({.address = before + index, .value = 0});
    }
  }
  if (input.start != entry) {
    for (const auto first : {(input.start + 2) & word_mask, target(input)}) {
      for (unsigned index = 0; index < 8; ++index) {
        store_word(machine.bus(), {.address = (first + index) & word_mask, .value = 0});
      }
    }
  }
  store_word(machine.bus(), {.address = input.start, .value = opcode(input)});
  store_word(machine.bus(),
             {.address = (input.start + 1) & word_mask, .value = input.displacement});
  return machine.cpu().debug_state();
}

inline DspDebugState branch_result(DspDebugState before, const Input input) {
  before.cpu.pc = taken(input) ? target(input) : ((input.start + 2) & word_mask);
  ++before.native_instructions;
  before.native_cycles += 5;
  return before;
}

void native_checks();
void boundary_checks();
void guard_checks();
}
