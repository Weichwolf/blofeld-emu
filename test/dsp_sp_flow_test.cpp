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
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000, caller = 0x100, callee = 0x180;
constexpr unsigned do_immediate = 0x060080, sp_short = 0x0500bb, sp_long = 0x05f43b;
constexpr unsigned sr_short = 0x0500b9, call_short = 0x0d0000, rts = 0x00000c;
constexpr unsigned call_long = 0x0bf080;
constexpr unsigned status = 0x300, retained_la = 0x321, retained_lc = 7;
constexpr unsigned first_high = 0x111111, first_low = 0x222222;
constexpr unsigned second_high = 0x333333, second_low = 0x444444;
constexpr unsigned body = caller + 2, last = body + 1, writer = last + 1;
constexpr unsigned padding = 32, budget = 160;
constexpr unsigned extal = 12000000, timebase = 540000000;

void require(const bool value, const std::string &message) {
  if (!value) { throw Fault(message); }
}

struct Case {
  unsigned count;
  bool extended;
  bool adjacent = false;
  bool long_call = false;

  [[nodiscard]] unsigned words() const { return extended ? 2U : 1U; }

  [[nodiscard]] unsigned call() const { return writer + words() + (adjacent ? 1U : 5U); }

  [[nodiscard]] unsigned tail() const { return call() + (long_call ? 2U : 1U); }
};

DspDebugState prepare(DspMachine &machine, const Case sample) {
  // Own FM literal MOVEC OMR/SR, two SSH pushes and SSL stores, LA/LC and JMP.
  constexpr std::array seed{0x05f43aU,
                            0x300U,
                            0x05f439U,
                            status,
                            0x05f43cU,
                            first_high,
                            0x05f43dU,
                            first_low,
                            0x05f43cU,
                            second_high,
                            0x05f43dU,
                            second_low,
                            0x05f43eU,
                            retained_la,
                            0x05f43fU,
                            retained_lc,
                            0x0af080U,
                            caller};
  machine.bus().install_rom({.start = boot, .words = seed});
  for (unsigned address = caller; address < callee + padding; ++address) {
    machine.bus().write({.address = address});
  }
  const auto put = [&](const unsigned address, const unsigned value) {
    machine.bus().write({.address = address, .value = value});
  };
  put(caller, do_immediate | (sample.count << 8U));
  put(caller + 1, last);
  put(writer, sample.extended ? sp_long : sp_short);
  put(writer + sample.words(), sr_short);
  put(sample.call(), sample.long_call ? call_long : call_short | callee);
  if (sample.long_call) { put(sample.call() + 1, callee); }
  put(callee + 1, rts);
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  constexpr unsigned seed_instructions = 9;
  for (unsigned i = 0; i < seed_instructions; ++i) { machine.cpu().step(); }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == caller && before.cpu.sp == 2 && before.sc == 2 &&
              before.cpu.la == retained_la && before.cpu.lc == retained_lc &&
              before.cpu.sr == status && before.stack_high[1] == first_high &&
              before.stack_low[2] == second_low,
          "Actual own native setup establishes the independent loop and retained stack");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

DspFetchRunInputs contract() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory};
}

DspDebugState expected(DspDebugState before, const Case sample) {
  before.cpu.pc = sample.tail() + 1;
  before.cpu.sp = 0;
  before.cpu.sr = 0;
  before.cpu.stack_pc = before.stack_high[0];
  before.cpu.stack_sr = before.stack_low[0];
  // FM DO pushes previous LA/LC, then body PC/SR. JSR later overwrites cell1.
  before.stack_high[3] = retained_la;
  before.stack_low[3] = retained_lc;
  before.stack_high[4] = body;
  before.stack_low[4] = status;
  before.stack_high[1] = sample.tail();
  before.stack_low[1] = 0;
  const unsigned fixed_instructions = sample.adjacent ? 7U : 11U;
  // DO5, short JSR3/long JSR4, RTS3; remaining own instructions1 each.
  const unsigned fixed_native_cost = (sample.adjacent ? 15U : 19U) + (sample.long_call ? 1U : 0U);
  before.native_instructions += fixed_instructions + (2 * sample.count);
  before.native_cycles += fixed_native_cost + (2 * sample.count) + (sample.extended ? 1U : 0U);
  return before;
}

struct Seen {
  unsigned ends = 0;
  bool e1 = false, sp = false, call = false, ret = false, done = false;
};

void advance(DspMachine &machine, DspFetchRun &run, const Case sample, Seen &seen) {
  const auto cycle =
      run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
  if (!cycle.pipeline) { throw Fault("Missing SP/DO/call pipeline cycle"); }
  const auto &phase = *cycle.pipeline;
  if (phase.execute1 && phase.execute1->address == writer) {
    const bool ending =
        phase.retirement_published && phase.execute2 && phase.execute2->address == last;
    require(seen.ends + (ending ? 1U : 0U) == sample.count && machine.cpu().state().sp == 2 &&
                machine.cpu().debug_state().sc == 2 && !phase.transfer,
            "DO completes all native ends before SP E1;actual SP stays held");
    seen.e1 = true;
  }
  if (!phase.retirement_published) { return; }
  if (!phase.execute2) { throw Fault("Missing SP/DO/call E2 identity"); }
  const auto pc = phase.execute2->address;
  const auto state = machine.cpu().debug_state();
  if (pc == last) {
    ++seen.ends;
  } else if (pc == writer) {
    require(seen.e1 && state.cpu.sp == 0 && state.sc == 2 && state.stack_high[2] == second_high &&
                state.stack_high[3] == retained_la && state.stack_low[4] == status,
            "SP E2 retains SC and both old ordinary and completed-loop stack cells");
    seen.sp = true;
  } else if (pc == sample.call()) {
    require(seen.sp && state.cpu.sp == 1 && state.sc == 3 && state.stack_high[1] == sample.tail() &&
                state.stack_low[1] == 0,
            "Actual JSR uses the cleared SP and independently retained SC");
    seen.call = true;
  } else if (pc == callee + 1) {
    require(seen.call && state.cpu.sp == 0 && state.sc == 2 && state.cpu.pc == sample.tail(),
            "Actual RTS restores caller PC while retaining the independent counter baseline");
    seen.ret = true;
  } else if (pc == sample.tail()) {
    require(seen.ret, "Caller NOP follows the genuine native return");
    seen.done = true;
  }
}

void finish(DspMachine &machine, DspFetchRun &run, const Case sample, Seen &seen) {
  for (unsigned i = 0; i < budget && !seen.done; ++i) { advance(machine, run, sample, seen); }
  require(seen.done, "Bounded native DO/SP/SR/JSR/RTS sequence completes");
}

void one_case(const Case sample) {
  DspMachine machine;
  const auto after = expected(prepare(machine, sample), sample);
  DspFetchRun run(contract(), machine);
  Seen seen;
  for (unsigned i = 0; i < budget && !seen.e1; ++i) { advance(machine, run, sample, seen); }
  require(seen.e1 && !seen.sp, "Actual after-loop SP E1 has a nonempty held suffix");
  const auto saved = run.checkpoint(machine);
  const auto prefix = seen;
  finish(machine, run, sample, seen);
  require(machine.cpu().debug_state() == after, "Literal full CPU/stack/native cost endpoint");
  const auto ticks = run.elapsed_ticks();
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    auto suffix = prefix;
    finish(machine, run, sample, suffix);
    require(machine.cpu().debug_state() == after && run.elapsed_ticks() == ticks,
            "Restored DO/SP/call suffix retains the independent endpoint and clock");
  }
  DspMachine target;
  DspFetchRun clone(saved, target);
  auto suffix = prefix;
  finish(target, clone, sample, suffix);
  require(target.cpu().debug_state() == after && clone.elapsed_ticks() == ticks,
          "Fresh owner retains completed DO history and held SP before real calls");
}
}

int main(const int argc, const char *const argv[]) {
  try {
    const bool adjacent = argc == 2 && std::string_view(argv[1]) == "--adjacent";
    require(argc == 1 || adjacent, "Select default SP flow or --adjacent");
    for (const unsigned count : {1U, 2U, 3U, 17U}) {
      if (adjacent) {
        for (const bool extended : {false, true}) {
          for (const bool long_call : {false, true}) {
            one_case(
                {.count = count, .extended = extended, .adjacent = true, .long_call = long_call});
          }
        }
      } else {
        for (const bool extended : {false, true}) {
          one_case({.count = count, .extended = extended});
        }
      }
    }
    std::cout
        << (adjacent
                ? "SP adjacency=16 DO exits;short/long SP0;SR0;short/long JSR;32suffixes/16clones\n"
                : "SP flow=8 DO exits;native SP/SR/JSR/RTS;16suffixes/8clones\n");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
