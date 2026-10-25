// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_interlock_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <charconv>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned x_cookie = 0x13579b;
constexpr unsigned y_cookie = 0x2468ac;

struct Program {
  DspProgram code;
  std::vector<std::uint32_t> words;
  unsigned instructions = 0;
};

Program read_program(const char *path) {
  std::ifstream input(path);
  require(input.is_open(), "Cannot open own interlock program");
  Program result;
  for (std::string source; std::getline(input, source);) {
    require(!source.empty() && result.instructions < 4096, "Bounded nonempty own instructions");
    // NXP canonicalizes this commutative signed product to QQQ=y0,x0.
    // The SDK otherwise selects its different nonparallel MPYsu encoding.
    const auto *const canonical = source == "mpy x0,y0,a" ? "mpy y0,x0,a" : source.c_str();
    const auto words = checkpoint::assemble(canonical);
    for (const auto word : words) {
      result.words.push_back(word);
      result.code.word(word);
    }
    ++result.instructions;
  }
  require(input.eof() && result.instructions != 0, "Read the complete own interlock program");
  return result;
}

unsigned argument(const char *text) {
  const std::string_view value(text);
  unsigned result = 0;
  const auto converted = std::from_chars(value.data(), value.data() + value.size(), result);
  require(converted.ec == std::errc{} && converted.ptr == value.data() + value.size(),
          "Invalid own interlock window index");
  return result;
}

int run(const Program &program, const bool dependencies, const unsigned low, const unsigned high) {
  require(low < high && high < program.instructions, "Own interlock observation window bound");
  DspMachine machine(interlocks::contract(dependencies));
  interlocks::initialize(machine, program.code);
  machine.bus().write({.space = DspSpace::x, .address = 0x100, .value = x_cookie});
  machine.bus().write({.space = DspSpace::y, .address = 0x100, .value = y_cookie});
  std::uint64_t start_ticks = 0;
  std::uint64_t window = 0;
  for (unsigned index = 0; index < program.instructions; ++index) {
    const auto before = machine.cpu().debug_state();
    try {
      const auto native = machine.run_clock().native_cycle_units();
      const auto stalled = machine.run_clock().interlock_cycle_units();
      const auto ticks = machine.run_clock().elapsed_ticks();
      const auto plan = machine.plan_step();
      require(plan == machine.plan_step() && machine.cpu().debug_state() == before,
              "Dependency timing previews are pure and repeatable");
      machine.cpu().step();
      const auto &clock = machine.run_clock();
      require(clock.native_cycle_units() == native + plan.native_cycle_units &&
                  clock.interlock_cycle_units() == stalled + plan.interlock_cycle_units &&
                  clock.elapsed_ticks() == ticks + plan.duration_ticks &&
                  clock.timer_clk2_edges() ==
                      (clock.native_cycle_units() + clock.interlock_cycle_units()) / 2,
              "Actual completion commits native units, dependency clocks, ticks and CLK2 together");
      if (index == low) { start_ticks = clock.elapsed_ticks(); }
      if (index == high) { window = clock.elapsed_ticks() - start_ticks; }
    } catch (const Fault &error) {
      if (machine.cpu().debug_state().native_instructions != before.native_instructions) { throw; }
      // A pure plan refusal has no effects; explicitly attempting the same
      // instruction must visibly quarantine the owner before operand accesses.
      if (!machine.timing_faulted()) {
        try {
          machine.cpu().step();
        } catch (const Fault &) {
          require(machine.timing_faulted(),
                  "An attempted unsupported instruction quarantines timing");
        }
      }
      require(machine.timing_faulted() &&
                  machine.cpu().debug_state().native_instructions == before.native_instructions,
              "Unsupported dependency timing rejects before native retirement");
      std::cout << "fault index=" << index << " pc=" << before.cpu.pc
                << " retired=" << before.native_instructions << '\n';
      std::cerr << error.what() << '\n';
      return 2;
    }
  }
  const auto state = machine.cpu().state();
  std::cout << "done window=" << window << " native=" << machine.run_clock().native_cycle_units()
            << " stalls=" << machine.run_clock().interlock_cycle_units()
            << " instructions=" << machine.cpu().debug_state().native_instructions
            << " pc=" << state.pc << " sr=" << state.sr << " x1=" << (state.x >> word_bits)
            << " y1=" << (state.y >> word_bits) << " x0=" << (state.x & word_mask) << '\n';
  return 0;
}
}

int main(const int count, char **arguments) {
  try {
    if (count == 1) {
      blofeld::test::interlocks::state_checks();
      std::cout << "Native dependency clocks, scheduling, reset and checkpoint checks passed.\n";
      return 0;
    }
    require(count == 3 || count == 5, "Own interlock fixture arguments");
    const std::string_view mode(arguments[1]);
    const auto program = read_program(arguments[2]);
    if (mode == "dump") {
      require(count == 3, "Own word-dump arguments");
      for (const auto word : program.words) {
        std::cout << std::hex << std::setfill('0') << std::setw(6) << word;
      }
      std::cout << '\n';
      return 0;
    }
    require(count == 5 && (mode == "nominal" || mode == "interlocks"), "Own clock profile");
    return run(program, mode == "interlocks", argument(arguments[3]), argument(arguments[4]));
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
