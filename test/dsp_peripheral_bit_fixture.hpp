// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::peripheral_bit {
struct Port {
  DspPort port;
  DspSpace space;
  unsigned address;
  unsigned bits;
};

inline constexpr std::array ports{
    Port{.port = DspPort::c, .space = DspSpace::x, .address = 0xffffbd, .bits = 12},
    Port{.port = DspPort::e, .space = DspSpace::y, .address = 0xffff9d, .bits = 12},
    Port{.port = DspPort::f, .space = DspSpace::y, .address = 0xfffff5, .bits = 11},
    Port{.port = DspPort::d, .space = DspSpace::x, .address = 0xffffd5, .bits = 2}};

struct Operation {
  DspBitOperation kind;
  unsigned high;
  unsigned low;
};

// Freescale-assembled templates, independent of the native decoder.
inline constexpr std::array operations{
    Operation{.kind = DspBitOperation::clear, .high = 0x0a8000, .low = 0x010000},
    Operation{.kind = DspBitOperation::set, .high = 0x0a8020, .low = 0x010020},
    Operation{.kind = DspBitOperation::change, .high = 0x0b8000, .low = 0x014000},
    Operation{.kind = DspBitOperation::test, .high = 0x0b8020, .low = 0x014020}};
inline constexpr unsigned seed_count = 5;
inline constexpr unsigned body = boot + (2 * seed_count);
inline constexpr unsigned status = masked_status | 0xfe;
inline constexpr unsigned peripheral_begin = 0xffff80;
inline constexpr unsigned high_begin = 0xffffc0;
inline constexpr unsigned bcr = 0xfffffb;
inline constexpr unsigned pin_bcr = 0x01ffff;

struct Input {
  unsigned port = 0;
  unsigned operation = 0;
  unsigned bit = 0;
  bool old = false;
  bool carry_in = false;
};

inline unsigned mask(const Input input) {
  return (1U << ports.at(input.port).bits) - 1;
}

inline unsigned seed(const Input input) {
  return input.old ? mask(input) : mask(input) & ~(1U << input.bit);
}

inline unsigned value(const Input input) {
  const auto selected = 1U << input.bit;
  switch (input.operation) {
    case 0: return seed(input) & ~selected;
    case 1: return seed(input) | selected;
    case 2: return seed(input) ^ selected;
    case 3: return seed(input);
    default: throw Fault("Invalid own bit operation");
  }
}

inline unsigned opcode(const Input input) {
  const auto port = ports.at(input.port);
  const bool high = port.address >= high_begin;
  const auto base = high ? operations.at(input.operation).high : operations.at(input.operation).low;
  return base | ((port.address - (high ? high_begin : peripheral_begin)) << 8U) |
         (port.space == DspSpace::y ? 0x40U : 0U) | input.bit;
}

inline DspDebugState prepare(DspMachine &machine, const Input input, const bool window) {
  const auto port = ports.at(input.port);
  const auto destination = [&](const unsigned address) {
    return std::string(port.space == DspSpace::x ? "x:<<" : "y:<<") + number(address);
  };
  DspProgram code;
  code.emit("movep #>" + number(pin_bcr) + ",x:<<" + number(bcr));
  code.emit("movep #>0," + destination(port.address + 2));
  code.emit("movep #>" + number(mask(input)) + ',' + destination(port.address + 1));
  code.emit("movep #>" + number(seed(input)) + ',' + destination(port.address));
  code.immediate("sr", status | static_cast<unsigned>(input.carry_in));
  if (window) { code.word(0); }
  code.word(opcode(input));
  for (unsigned index = 0; index < 32; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, seed_count);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body &&
              machine.bus().read({.space = port.space, .address = port.address}) == seed(input) &&
              before.cpu.sr == (status | static_cast<unsigned>(input.carry_in)),
          "Five actual native seeds establish the exact GPIO word and independent old carry");
  return before;
}

inline DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

void native_checks();
void pipeline_checks();
void guard_checks();
}
