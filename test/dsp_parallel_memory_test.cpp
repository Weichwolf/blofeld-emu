// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_address_stage.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_parallel_memory_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace blofeld::test::parallel_memory {
namespace {
struct Mode {
  const char *suffix;
  unsigned result;
  bool updated;
};

constexpr std::array modes{Mode{.suffix = "", .result = 7, .updated = false},
                           Mode{.suffix = "+n", .result = 2, .updated = true},
                           Mode{.suffix = "-", .result = 6, .updated = true},
                           Mode{.suffix = "+", .result = 0, .updated = true}};

std::string operand(const unsigned index, const Mode mode) {
  auto text = "(r" + std::to_string(index) + ')';
  text.append(mode.suffix);
  if (mode.suffix == std::string_view("+n")) { text += std::to_string(index); }
  return text;
}

std::string move(const unsigned index, const Mode mode, const bool load, const bool y) {
  const auto memory = std::string(y ? "y:" : "x:") + operand(index, mode);
  const std::string reg = y ? "y0" : "x0";
  auto text = load ? memory : reg;
  text.push_back(',');
  text.append(load ? reg : memory);
  return text;
}

struct Case {
  unsigned x = 0;
  unsigned y = 4;
  Mode mode_x;
  Mode mode_y;
  bool load_x = false;
  bool load_y = false;
};

void one_dual(const Case sample) {
  DspMachine machine;
  auto instruction = std::string("move ");
  instruction += move(sample.x, sample.mode_x, sample.load_x, false);
  instruction.push_back(' ');
  instruction += move(sample.y, sample.mode_y, sample.load_y, true);
  const std::array code{instruction};
  prepare(machine, code, {.primary = {.index = sample.x}, .secondary = sample.y});
  machine.bus().write({.space = DspSpace::x, .address = initial, .value = word_x});
  machine.bus().write({.space = DspSpace::y, .address = initial, .value = word_y});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 3);
  const auto ag1 = stages(run.advance_cycle(machine, fetch_clock::input())).ag1_address;
  require(ag1 && ag1->secondary && ag1->update.index == sample.x && ag1->update.before == initial &&
              ag1->update.value == sample.mode_x.result &&
              ag1->pointer_update == sample.mode_x.updated &&
              ag1->secondary->update.index == sample.y &&
              ag1->secondary->update.before == initial &&
              ag1->secondary->update.value == sample.mode_y.result &&
              ag1->secondary->pointer_update == sample.mode_y.updated &&
              ag1->data_access_count == 2 && machine.cpu().debug_state() == before,
          "Native dual AG1 retains both literal EA/R results without visible mutation");
  static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
  auto expected = before;
  expected.cpu.r.at(sample.x) = sample.mode_x.result;
  expected.cpu.r.at(sample.y) = sample.mode_y.result;
  require(machine.cpu().debug_state() == expected, "AG2 publishes both real pointer results once");
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto result = stages(cycle).memory;
  if (sample.load_x) { expected.cpu.x = pipeline::unit + word_x; }
  if (sample.load_y) { expected.cpu.y = word_y; }
  require(result && result->additional_data && cycle.bus.core_access_count == 3 &&
              machine.cpu().debug_state() == expected &&
              machine.bus().read({.space = DspSpace::x, .address = initial}) ==
                  (sample.load_x ? word_x : memory_pipeline::first) &&
              machine.bus().read({.space = DspSpace::y, .address = initial}) ==
                  (sample.load_y ? word_y : memory_pipeline::half),
          "One actual F2 plus native X/Y data effects commits the correct loads/stores in E1");
  const auto first_space = sample.load_x && !sample.load_y ? DspSpace::y : DspSpace::x;
  require(result && result->additional_data && result->data.target.space == first_space &&
              result->additional_data->target.space != first_space,
          "Native mixed transfer order performs both stores before loads");
  static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(machine.cpu().debug_state() == expected,
          "E2 retires the actual dual native instruction without losing either accepted pointer");
}
}

void address_checks() {
  for (unsigned index = 0; index < 8; ++index) {
    const auto second = index < 4 ? index + 4 : index - 4;
    for (const auto x : modes) {
      for (const auto y : modes) {
        for (const bool load_x : {false, true}) {
          for (const bool load_y : {false, true}) {
            one_dual({.x = index,
                      .y = second,
                      .mode_x = x,
                      .mode_y = y,
                      .load_x = load_x,
                      .load_y = load_y});
          }
        }
      }
    }
  }

  struct Selector {
    unsigned x;
    unsigned y;
  };

  constexpr std::array selectors{Selector{.x = 0, .y = 5},
                                 Selector{.x = 1, .y = 6},
                                 Selector{.x = 2, .y = 7},
                                 Selector{.x = 3, .y = 4},
                                 Selector{.x = 4, .y = 1},
                                 Selector{.x = 5, .y = 2},
                                 Selector{.x = 6, .y = 3},
                                 Selector{.x = 7, .y = 0}};
  for (const auto selector : selectors) {
    for (const bool load_x : {false, true}) {
      for (const bool load_y : {false, true}) {
        one_dual({.x = selector.x,
                  .y = selector.y,
                  .mode_x = modes[3],
                  .mode_y = modes[1],
                  .load_x = load_x,
                  .load_y = load_y});
      }
    }
  }
}
}

int main(const int argc, char **argv) {
  try {
    using namespace blofeld::test::parallel_memory;
    if (argc != 2) { throw blofeld::Fault("Expected address, execution or state group"); }
    const auto group = std::string_view(argv[1]);
    if (group == "address") {
      address_checks();
    } else if (group == "execution") {
      execution_checks();
    } else if (group == "state") {
      state_checks();
    } else {
      throw blofeld::Fault("Unknown parallel memory group");
    }
    std::cout << "Native parallel memory " << group << " checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
