// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_address_stage.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::memory_pipeline;

struct Mode {
  const char *pattern;
  unsigned result;
  bool updates;
};

constexpr std::array modes{Mode{.pattern = "(r)-n", .result = 4, .updates = true},
                           Mode{.pattern = "(r)+n", .result = 2, .updates = true},
                           Mode{.pattern = "(r)-", .result = 6, .updates = true},
                           Mode{.pattern = "(r)+", .result = 0, .updates = true},
                           Mode{.pattern = "(r)", .result = 7, .updates = false}};

std::string operand(const Mode mode, const unsigned index) {
  auto result = std::string(mode.pattern);
  const auto reg = std::to_string(index);
  result.replace(result.find('r'), 1, "r" + reg);
  if (const auto offset = result.find('n'); offset != std::string::npos) {
    result.replace(offset, 1, "n" + reg);
  }
  return result;
}

void one_form(const unsigned index, const Mode mode, const DspSpace space, const bool store) {
  DspMachine machine;
  const std::string name = space == DspSpace::x ? "x" : "y";
  const auto memory = name + ':' + operand(mode, index);
  const std::array code{"move " + (store ? std::string("x0,") + memory : memory + ",x0")};
  prepare(machine, code, {.index = index});
  machine.bus().write({.space = space, .address = 7, .value = second});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 3);
  const auto ag1 = run.advance_cycle(machine, fetch_clock::input());
  const auto captured = stages(ag1).ag1_address;
  require(captured && !captured->published && captured->pointer_update == mode.updates &&
              captured->update.before == 7 && captured->update.value == mode.result &&
              captured->update.address == 7 && captured->update.index == index &&
              captured->update.space == space && captured->word.address == body &&
              machine.cpu().debug_state() == before,
          memory + ": actual native AG1 calculates EA/modulo result without publishing R or data");
  const auto ag2 = run.advance_cycle(machine, fetch_clock::input());
  auto visible = before;
  visible.cpu.r.at(index) = mode.result;
  const auto updated = stages(ag2).ag2_address;
  require(updated && updated->published && machine.cpu().debug_state() == visible &&
              machine.bus().read({.space = space, .address = 7}) == second,
          memory + ": AG2 publishes only the real R result, with no data side effect");
  const auto e1 = run.advance_cycle(machine, fetch_clock::input());
  const auto effect = stages(e1).memory;
  if (!store) { visible.cpu.x = pipeline::unit + second; }
  require(
      effect && effect->data.target.address == 7 && effect->data.target.space == space &&
          effect->data.kind == (store ? DspNativeAccessKind::write : DspNativeAccessKind::read) &&
          effect->data.value == (store ? first : second) && effect->move.has_value() == !store &&
          machine.cpu().debug_state() == visible && e1.bus.core_access_count == 2 &&
          machine.bus().read({.space = space, .address = 7}) == (store ? first : second),
      memory + ": E1 commits the actual native data/load field once before retirement");
  const auto e2 = run.advance_cycle(machine, fetch_clock::input());
  visible.cpu.pc = body + 1;
  ++visible.native_instructions;
  ++visible.native_cycles;
  require(stages(e2).retirement_published && machine.cpu().debug_state() == visible &&
              run.completed_cycles() == 7 && run.elapsed_ticks() == 7 * pipeline::period,
          memory + ": E2 retires the true native result and retains the already published pointer");
}

void zero_offset() {
  DspMachine machine;
  const std::array<std::string, 1> code{"move x:(r0)+n0,x0"};
  prepare(machine, code, {.offset = 0});
  machine.bus().write({.space = DspSpace::x, .address = 7, .value = second});
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 3);
  const auto phase = stages(run.advance_cycle(machine, fetch_clock::input()));
  require(phase.ag1_address && phase.ag1_address->pointer_update &&
              phase.ag1_address->update.before == 7 && phase.ag1_address->update.value == 7,
          "An actual zero-offset native post-update still occupies AG2");
  clocks(run, machine, 3);
  require(machine.cpu().state().x == pipeline::unit + second && machine.cpu().state().r[0] == 7 &&
              machine.cpu().state().pc == body + 1,
          "A zero-offset event reaches actual load and retirement once");
}
}

int main() {
  try {
    for (unsigned index = 0; index < 8; ++index) {
      for (const auto mode : modes) {
        for (const auto space : {DspSpace::x, DspSpace::y}) {
          for (const bool store : {false, true}) { one_form(index, mode, space, store); }
        }
      }
    }
    zero_offset();
    stream_checks();
    guard_checks();
    std::cout
        << "Automatic native memory pipeline:160 modulo forms and overlap/restore guards passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
