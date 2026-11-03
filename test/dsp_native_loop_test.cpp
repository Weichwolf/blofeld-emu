// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp_fetch_native_fixture.hpp"
#include "dsp_native_loop_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test::native_loop;
using fixture::require;
constexpr unsigned immediate_x0 = 0x44f400, payload = 0x123456;
constexpr std::uint64_t low_word = 0xffffff;
constexpr unsigned do_immediate = 0x060080;
constexpr unsigned dor_immediate = 0x060090;

struct Case {
  unsigned count;
  unsigned width;
  unsigned prefill;
  bool relative;
  bool move = false;
};

dsp56300::Registers
expected(const dsp56300::Registers &initial, const Case test, const unsigned completed) {
  const auto per_iteration = test.width - (test.move ? 1U : 0U);
  auto result = opened(initial, {.count = test.count, .width = test.width});
  if (completed == test.count * per_iteration) {
    result = finished(initial, {.count = test.count, .width = test.width});
  } else {
    const auto instruction = completed % per_iteration;
    result.pc = body + instruction + (test.move && instruction != 0 ? 1U : 0U);
    result.lc = test.count - completed / per_iteration;
  }
  if (test.move && completed != 0) { result.x = (result.x & ~low_word) | payload; }
  return result;
}

void cadence(const Case test) {
  std::array<std::uint32_t, padding_words> code{};
  code[0] = (test.relative ? dor_immediate : do_immediate) | (test.count << 8);
  code[1] = test.relative ? test.width + 1 : body + test.width - 1;
  if (test.move) {
    code[2] = immediate_x0;
    code[3] = payload;
  }
  fixture::Fixture f(code);
  attach(f, test.prefill);
  const auto initial = f.machine.cpu().registers();
  const auto progress = f.machine.cpu().progress();
  const auto ticks = f.machine.clock().elapsed_ticks();
  const auto total_periods = setup_periods + (test.count * test.width);
  const auto body_instructions = test.count * (test.width - (test.move ? 1U : 0U));
  for (unsigned elapsed = 1; elapsed <= total_periods; ++elapsed) {
    const auto result = advance(f);
    require(f.machine.clock().elapsed_ticks() == ticks + elapsed * period,
            "Each authored loop observation advances exactly one native clock period");
    if (elapsed == 1) {
      require(f.machine.cpu().registers() ==
                      opened(initial, {.count = test.count, .width = test.width}) &&
                  f.machine.cpu().progress() == progress,
              "All real PCU groups publish literal DO/stack state before held retirement");
    }
    if (!result.native || !result.native->retired) { continue; }
    const auto count = f.machine.cpu().progress().instructions - progress.instructions;
    require(count != 0 && count <= body_instructions + 1,
            "Actual retirements stay within the independent DO/body instruction count");
    const auto completed = static_cast<unsigned>(count - 1);
    const auto moves = test.move ? (completed + test.width - 2) / (test.width - 1) : 0;
    const auto expected_periods = setup_periods + completed + moves;
    const auto current = f.machine.cpu().progress();
    require(f.machine.cpu().registers() == expected(initial, test, completed) &&
                elapsed == expected_periods &&
                current.estimated_cycles == progress.estimated_cycles + expected_periods,
            "Full native state and zero-overhead body cadence match literal instruction semantics");
  }
  require(f.machine.cpu().progress().instructions ==
                  progress.instructions + body_instructions + 1 &&
              f.machine.cpu().registers() == expected(initial, test, body_instructions),
          "The complete loop reaches its literal restored outer state within the native budget");
}
} // namespace

int main(const int argc, char **argv) {
  try {
    if (argc == 2 && std::string_view{argv[1]} == "--control") {
      blofeld::test::native_loop::control_checks();
      return 0;
    }
    blofeld::test::esai::require(argc == 1, "Expected no arguments or --control");
    for (const auto count : {0U, 1U, 3U}) {
      for (const auto width : {1U, 2U, 4U, 12U}) {
        for (const auto prefill : {2U, 8U}) {
          for (const bool relative : {false, true}) {
            cadence({.count = count, .width = width, .prefill = prefill, .relative = relative});
            if (width != 1) {
              cadence({.count = count,
                       .width = width,
                       .prefill = prefill,
                       .relative = relative,
                       .move = true});
            }
          }
        }
      }
    }
    std::cout << "84 authored DO/DOR cadence cases retain full native state and real F2 identity\n";
    blofeld::test::native_loop::state_checks();
    blofeld::test::native_loop::event_checks();
    blofeld::test::native_loop::control_checks();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
