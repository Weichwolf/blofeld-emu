// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_peripheral_read_form.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_relative_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string_view>

namespace blofeld::test::peripheral_relative {
namespace {
struct Oracle {
  DspDebugState expected;
  unsigned captures = 0, retired = 0;
};

unsigned target_retirement(const Input input) {
  return taken(input) ? 12U : 13U;
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto clock = cycle.completed_cycles;
  const auto first = target_retirement(input);
  if (cycle.bus.peripheral_read) {
    ++oracle.captures;
    require(clock == 4, "Relative branch captures its actual selected bit in the declared AG1");
  }
  require(!phase.memory && !phase.memory_source_stall && phase.agu_dependencies == 0,
          "A fixed relative bit branch has no invented memory writeback or AGU source hold");
  const bool retires = clock == 7 || (clock >= first && clock < first + 6);
  require(phase.retirement_published == retires,
          "Independent5/6-period branch intervals place every actual target NOP retirement");
  if (retires) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "Only the genuine own branch or selected sequential target retires");
    if (clock == 7) {
      oracle.expected = branch_result(oracle.expected, input);
    } else {
      ++oracle.expected.cpu.pc;
      ++oracle.expected.native_instructions;
      ++oracle.expected.native_cycles;
    }
    ++oracle.retired;
  }
  require(machine.cpu().debug_state() == oracle.expected,
          "Every period agrees with independent PC/CCR/CPU/stack/native-counter expectations");
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, Oracle &oracle) {
  const auto end = target_retirement(input) + 5;
  while (run.completed_cycles() < end) { step(run, machine, input, oracle); }
  require(oracle.captures == 1 && oracle.retired == 7 &&
              run.elapsed_ticks() == end * pipeline::period,
          "One true sampled read and native branch precede six independently timed NOPs");
}

struct Window {
  Input input;
  unsigned cut = 0;
  bool flip = false;
};

void window(const Window sample) {
  const auto input = sample.input;
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, input)};
  if (input.start != entry) {
    constexpr unsigned prefetch_words = 16;
    for (const auto first : {(input.start + 2) & word_mask, target(input)}) {
      for (unsigned index = 0; index < prefetch_words; ++index) {
        store_word(machine.bus(), {.address = (first + index) & word_mask, .value = 0});
      }
    }
  }
  const auto form = decode_peripheral_read(opcode(input));
  require(form && form->relative && form->bit == input.bit && form->words == 2 &&
              form->set == input.set,
          "The real opcode decodes a relative two-word bit operand without a CCR condition");
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  if (sample.flip) {
    while (run.completed_cycles() < 4) { step(run, machine, input, oracle); }
    machine.bus().timers().set_tio_drive(1, !input.level);
  }
  if (sample.cut == 0) {
    finish(run, machine, input, oracle);
    return;
  }
  while (run.completed_cycles() < sample.cut) { step(run, machine, input, oracle); }
  const auto saved = run.checkpoint(machine);
  const auto prefix = oracle;
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    run.restore(saved, machine);
    oracle = prefix;
    finish(run, machine, input, oracle);
  }
}

void window_set(const Input input) {
  window({.input = input});
  if (!input.flags) { return; }
  for (unsigned cut = 3; cut <= 10; ++cut) { window({.input = input, .cut = cut}); }
}

void timer_windows() {
  for (const bool set : {false, true}) {
    for (const bool level : {false, true}) {
      const Input input{.bit = timer_bit, .set = set, .level = level, .flags = true, .timer = true};
      window({.input = input});
      for (unsigned cut = 4; cut <= 8; ++cut) {
        window({.input = input, .cut = cut, .flip = true});
      }
    }
  }
}

void boundary_windows() {
  constexpr unsigned positive = 17;
  constexpr unsigned negative = word_mask - 31;
  for (const bool set : {false, true}) {
    for (const bool level : {false, true}) {
      window_set({.set = set,
                  .level = level,
                  .flags = true,
                  .start = word_mask,
                  .displacement = positive});
      window_set({.set = set, .level = level, .flags = true, .start = 1, .displacement = negative});
    }
  }
}

void pipeline_checks() {
  for (unsigned port = 0; port < ports.size(); ++port) {
    for (const auto bit : {0U, ports.at(port).bits - 1}) {
      for (const bool set : {false, true}) {
        for (const bool level : {false, true}) {
          for (const bool flags : {false, true}) {
            for (const auto displacement : {32U, word_mask - 31}) {
              window_set({.port = port,
                          .bit = bit,
                          .set = set,
                          .level = level,
                          .flags = flags,
                          .displacement = displacement});
            }
          }
        }
      }
    }
  }
  timer_windows();
  boundary_windows();
  std::cout << "128 GPIO windows/512 cuts1024 suffixes,4timer windows20flipped "
               "cuts40suffixes;8wrap windows64cuts128suffixes passed\n";
}
}
}

int main(const int count, char **arguments) {
  try {
    using namespace blofeld::test::peripheral_relative;
    const std::string_view mode = count == 2 ? arguments[1] : "";
    if (mode == "native") {
      native_checks();
    } else if (mode == "pipeline") {
      pipeline_checks();
    } else if (mode == "boundary") {
      boundary_checks();
    } else if (mode == "guards") {
      guard_checks();
    } else {
      throw blofeld::Fault("Expected native,pipeline,boundary or guards");
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
