// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
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
constexpr unsigned do_immediate = 0x060080; // FM13-56: DO #count, absolute last address.
constexpr unsigned do_cost = 5;             // FM A-1, immediate DO, independent native cost.
constexpr unsigned caller_return_cost = 8;  // BSR3 + RTS4 + resumed NOP1.
constexpr unsigned status = 0xc00300;
constexpr unsigned loop_flag = 0x8000;
constexpr unsigned retained_la = 0x321;
constexpr unsigned retained_lc = 7;
constexpr unsigned ordinary_omr = 0x300;
// FM13-37/83/168: BSR displacement+$80, absolute JMP, RTS. NOP is all zero.
constexpr unsigned call_opcode = 0x050900;
constexpr unsigned jump_opcode = 0x0af080;
constexpr unsigned return_opcode = 0x00000c;
constexpr unsigned sr_immediate = 0x05f439;
constexpr unsigned omr_immediate = 0x05f43a;
constexpr unsigned all_ones = 0xffffff;
constexpr unsigned cycle_limit = 64;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 540000000;

struct Case {
  unsigned count;
  unsigned size;
  unsigned known_tail;
  unsigned value;

  [[nodiscard]] static unsigned first() { return routine + 2; }

  [[nodiscard]] unsigned last() const { return first() + size - 1; }

  [[nodiscard]] unsigned ret() const { return last() + 1; }

  [[nodiscard]] unsigned tail() const { return ret() + 1; }

  [[nodiscard]] unsigned retired() const { return 4 + (count * size); }

  [[nodiscard]] unsigned native_cost() const {
    return caller_return_cost + do_cost + (count * size);
  }

  // Existing independently tested DO Decode spacing: five periods for a multiword
  // body, six for a single-word body. Caller BSR/RTS/NOP fixed contribution is14.
  [[nodiscard]] unsigned periods() const { return 14 + (size == 1 ? 6U : 5U) + (count * size); }
};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

DspDebugState prepare(DspMachine &machine, const Case sample) {
  const std::array code{omr_immediate, ordinary_omr, sr_immediate, status, jump_opcode, caller};
  machine.bus().install_rom({.start = boot, .words = code});
  machine.bus().write({.address = caller, .value = call_opcode});
  for (unsigned i = 1; i < 16; ++i) { machine.bus().write({.address = caller + i}); }
  machine.bus().write({.address = routine, .value = do_immediate | (sample.count << 8U)});
  machine.bus().write({.address = routine + 1, .value = sample.last()});
  for (unsigned i = 0; i < sample.size; ++i) {
    machine.bus().write({.address = Case::first() + i});
  }
  machine.bus().write({.address = sample.ret(), .value = return_opcode});
  for (unsigned i = 0; i < sample.known_tail; ++i) {
    machine.bus().write({.address = sample.tail() + i, .value = sample.value});
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

DspDebugState expected(DspDebugState before, const Case sample) {
  // Actual BSR, immediate DO, counted NOP body, RTS, resumed NOP. Tail never executes.
  before.cpu.pc = caller + 2;
  before.stack_high.at(1) = caller + 1;
  before.stack_low.at(1) = status;
  before.stack_high.at(2) = before.cpu.la;
  before.stack_low.at(2) = before.cpu.lc;
  before.stack_high.at(3) = Case::first();
  before.stack_low.at(3) = status;
  before.native_instructions += sample.retired();
  before.native_cycles += sample.native_cost();
  return before;
}

struct ReturnFrontier {
  DspLoopFetch fetch;
  DspProgramWord word;
  DspSubroutineControl control;
};

ReturnFrontier frontier(const bool active) {
  const DspLoopArchitecture initial{.pc = routine,
                                    .omr = ordinary_omr,
                                    .la = retained_la,
                                    .lc = retained_lc,
                                    .sp = 1,
                                    .sc = 1,
                                    .status = status};
  DspLoopFetch fetch(initial);
  const unsigned count = active ? 2U : 1U;
  const DspProgramWord opcode{.address = routine, .value = do_immediate | (count << 8U)};
  const DspProgramWord extension{.address = routine + 1, .value = Case::first()};
  static_cast<void>(fetch.fetched(opcode));
  static_cast<void>(fetch.fetched(extension));
  auto opened = initial;
  opened.pc = Case::first();
  opened.la = Case::first();
  opened.lc = count;
  opened.sp = 3;
  opened.sc = 3;
  opened.status |= loop_flag;
  fetch.opened({.word = opcode, .extension = extension, .before = initial, .after = opened});
  const DspProgramWord last{.address = Case::first(), .value = active ? return_opcode : 0U};
  require(fetch.fetched(last).has_value(), "Own final DO word retains its actual boundary");
  const DspProgramWord word =
      active ? last : DspProgramWord{.address = Case::first() + 1, .value = return_opcode};
  if (!active) { static_cast<void>(fetch.fetched(word)); }
  auto before = fetch.prediction().control();
  before.pc = word.address;
  auto after = before;
  after.pc = caller + 1;
  --after.sp;
  --after.sc;
  fetch.discard_after(fetch.prediction(), after.pc);
  return {.fetch = fetch, .word = word, .control = {.before = before, .after = after}};
}

void refused_return(const unsigned variant) {
  auto sample = frontier(variant == 0);
  auto &control = sample.control;
  switch (variant) {
    case 0: break; // Return without a recorded child call while the DO remains active.
    case 1: control.before.sp = 0; break;
    case 2: ++control.after.sp; break;
    case 3: ++control.after.sc; break;
    case 4: control.after.status ^= loop_flag; break;
    case 5: ++control.after.pc; break;
    case 6: control.after.omr ^= 4U; break;
    case 7: ++control.before.lc; break;
    default: throw Fault("Unknown postloop return refusal vector");
  }
  const auto before = sample.fetch.prediction().control();
  const auto depth = sample.fetch.depth();
  const auto pending = sample.fetch.pending();
  try {
    sample.fetch.predict_subroutine(sample.word, control, false);
  } catch (const Fault &error) {
    require(std::string(error.what()).contains("Unqualified subroutine/active DO read P:") &&
                sample.fetch.prediction().control() == before && sample.fetch.depth() == depth &&
                sample.fetch.pending() == pending,
            "Invalid parent return preserves forecast, active depth and pending native boundaries");
    return;
  }
  throw Fault("Invalid postloop return was accepted");
}

void native(const Case sample) {
  DspMachine machine;
  const auto after = expected(prepare(machine, sample), sample);
  for (unsigned i = 0; i < sample.retired(); ++i) { machine.cpu().step(); }
  require(machine.cpu().debug_state() == after,
          "Primary BSR/DO/RTS effects, retained stack and literal costs ignore return fallthrough");
}

void pipeline(const Case sample) {
  DspMachine machine;
  const auto after = expected(prepare(machine, sample), sample);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run({.extal_hz = extal_hz,
                   .timebase_hz = timebase_hz,
                   .timing = DspFetchTiming::reference_cycle_order,
                   .execution = DspReferenceExecution::staged_register_boundaries,
                   .pipeline = DspFetchPipeline::automatic_memory},
                  machine);
  while (machine.cpu().state().pc != caller + 2) {
    require(run.completed_cycles() < cycle_limit, "Own bounded caller exceeded its clock limit");
    static_cast<void>(
        run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available}));
  }
  require(machine.cpu().debug_state() == after && run.completed_cycles() == sample.periods(),
          "Post-DO RTS matches full native state, all stack slots and literal elapsed periods");
}
}

int main() {
  try {
    for (const unsigned count : {1U, 2U, 3U}) {
      for (const unsigned size : {1U, 2U, 4U}) {
        for (const unsigned value : {0U, return_opcode, all_ones}) {
          for (const unsigned known : {2U, 4U}) {
            const Case sample{.count = count, .size = size, .known_tail = known, .value = value};
            native(sample);
            pipeline(sample);
          }
        }
      }
    }
    for (unsigned variant = 0; variant < 8; ++variant) { refused_return(variant); }
    std::cout << "PASS54 own native DO-return controls,54 frontend windows,8 refusals\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
