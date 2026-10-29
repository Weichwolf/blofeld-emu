// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000;
constexpr unsigned caller = 0x100;
constexpr unsigned routine = 0x180;
constexpr unsigned tail = routine + 2;
constexpr unsigned status = 0xc00300;
constexpr unsigned ordinary_omr = 0x300;
// FM13-37/83/168: BSR displacement+$80, absolute JMP, RTS. NOP is all zero.
constexpr unsigned call_opcode = 0x050900;
constexpr unsigned jump_opcode = 0x0af080;
constexpr unsigned return_opcode = 0x00000c;
constexpr unsigned sr_immediate = 0x05f439;
constexpr unsigned omr_immediate = 0x05f43a;
constexpr unsigned all_ones = 0xffffff;
constexpr unsigned cycle_limit = 64;
constexpr unsigned expected_periods = 15;
constexpr unsigned expected_native_cost = 9;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 540000000;

struct Case {
  unsigned known_tail;
  unsigned value;
};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

DspDebugState prepare(DspMachine &machine, const Case sample) {
  const std::array code{omr_immediate, ordinary_omr, sr_immediate, status, jump_opcode, caller};
  machine.bus().install_rom({.start = boot, .words = code});
  machine.bus().write({.address = caller, .value = call_opcode});
  for (unsigned i = 1; i < 16; ++i) { machine.bus().write({.address = caller + i}); }
  machine.bus().write({.address = routine});
  machine.bus().write({.address = routine + 1, .value = return_opcode});
  for (unsigned i = 0; i < sample.known_tail; ++i) {
    machine.bus().write({.address = tail + i, .value = sample.value});
  }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  for (unsigned i = 0; i < 3; ++i) { machine.cpu().step(); }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == caller && before.cpu.sr == status && before.cpu.omr == ordinary_omr &&
              before.cpu.sp == 0 && before.sc == 0,
          "Genuine own setup reaches its ordinary masked caller with an empty stack");
  return before;
}

DspDebugState expected(DspDebugState before) {
  // Four actual instructions: BSR,NOP,RTS,NOP. Return padding is never executed.
  before.cpu.pc = caller + 2;
  before.stack_high.at(1) = caller + 1;
  before.stack_low.at(1) = status;
  before.native_instructions += 4;
  before.native_cycles += expected_native_cost;
  return before;
}

void native(const Case sample) {
  DspMachine machine;
  const auto after = expected(prepare(machine, sample));
  for (unsigned i = 0; i < 4; ++i) { machine.cpu().step(); }
  require(machine.cpu().debug_state() == after,
          "Primary RTS/BSR effects and four literal instruction costs ignore return fallthrough");
}

void pipeline(const Case sample) {
  const auto known_tail = sample.known_tail;
  const auto value = sample.value;
  DspMachine machine;
  const auto after = expected(prepare(machine, sample));
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run({.extal_hz = extal_hz,
                   .timebase_hz = timebase_hz,
                   .timing = DspFetchTiming::reference_cycle_order,
                   .execution = DspReferenceExecution::staged_register_boundaries,
                   .pipeline = DspFetchPipeline::automatic_memory},
                  machine);
  std::string error;
  try {
    while (machine.cpu().state().pc != caller + 2) {
      require(run.completed_cycles() < cycle_limit, "Own bounded caller exceeded its clock limit");
      static_cast<void>(
          run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available}));
    }
  } catch (const Fault &fault) { error = fault.what(); }
  require(error.empty() && machine.cpu().debug_state() == after &&
              run.completed_cycles() == expected_periods,
          "Unknown unused tail must match independent whole state and15periods: " + error);
  std::cout << "CONTROL tail-known=" << known_tail << " value=" << hex(value, 6)
            << " cycles=" << run.completed_cycles() << " unchanged=1\n";
}
}

int main() {
  try {
    for (const unsigned value : {0U, return_opcode, all_ones}) {
      for (const unsigned known : {0U, 1U, 2U, 4U}) {
        native({.known_tail = known, .value = value});
        pipeline({.known_tail = known, .value = value});
      }
    }
    std::cout << "PASS twelve own native controls and twelve known-or-unknown frontend windows\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
