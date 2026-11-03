// SPDX-License-Identifier: GPL-3.0-only
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_native_fixture.hpp"
#include "dsp_native_loop_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <iostream>

namespace blofeld::test::native_loop {
namespace {
using fixture::require;
constexpr unsigned long_call = 0x0bf080, return_subroutine = 0x00000c;
constexpr unsigned handler = 0x180, vector = 0x10, priority = 0xffffff, edge_priority = 6;
constexpr unsigned immediate_y1 = 0x47f400, immediate_x0 = 0x44f400;
constexpr unsigned dor_y1 = 0x06c710, do_x0 = 0x06c400;
constexpr std::uint64_t low_word = 0xffffff;

struct RegisterCase {
  unsigned count;
  bool relative;
};

void finish(fixture::Fixture &f, const std::uint64_t target) {
  constexpr unsigned bound = 128;
  for (unsigned index = 0; f.machine.cpu().progress().instructions < target && index < bound;
       ++index) {
    static_cast<void>(advance(f));
  }
  require(f.machine.cpu().progress().instructions == target,
          "Native control returns to the exact authored main instruction count");
}

void register_count(const RegisterCase test) {
  constexpr unsigned loop_body = fixture::program + 4;
  constexpr unsigned width = 4;
  std::array<std::uint32_t, padding_words> code{};
  code[0] = test.relative ? immediate_y1 : immediate_x0;
  code[1] = test.count;
  code[2] = test.relative ? dor_y1 : do_x0;
  code[3] = test.relative ? width + 1 : loop_body + width - 1;
  fixture::Fixture f(code);
  auto initial = f.machine.cpu().registers();
  initial.pc = fixture::program + 2;
  if (test.relative) {
    initial.y = (initial.y & low_word) | (std::uint64_t{test.count} << word_bits);
  } else {
    initial.x = (initial.x & ~low_word) | test.count;
  }
  dma_test::step_native(f.machine);
  require(f.machine.cpu().registers() == initial,
          "The actual authored MOVE establishes the literal register count before attachment");
  attach(f);
  const auto progress = f.machine.cpu().progress();
  finish(f, progress.instructions + 1);
  auto frame = initial;
  frame.stack[1] = (std::uint64_t{initial.la} << word_bits) | initial.lc;
  frame.pc = loop_body + width;
  if (test.count != 0) {
    frame.pc = loop_body;
    frame.la = loop_body + width - 1;
    frame.lc = test.count;
    frame.sp = 2;
    frame.sc = 2;
    frame.sr = (initial.sr & ~forever_flag) | loop_flag;
    frame.stack[2] = (std::uint64_t{loop_body} << word_bits) | initial.sr;
  }
  require(f.machine.cpu().registers() == frame,
          "All native register-source PCU groups establish the independently expected loop frame");
  const auto start = f.machine.clock().elapsed_ticks();
  const auto setup_cycles = f.machine.cpu().progress().estimated_cycles;
  const auto total = test.count * width;
  for (unsigned index = 0; index < total; ++index) {
    const auto result = advance(f);
    auto expected = frame;
    const auto completed = index + 1;
    expected.pc = loop_body + completed % width;
    expected.lc = test.count - completed / width;
    if (completed == total) {
      expected = initial;
      expected.stack = frame.stack;
      expected.pc = loop_body + width;
    }
    require(
        result.native && result.native->retired && f.machine.cpu().registers() == expected &&
            f.machine.clock().elapsed_ticks() == start + completed * period &&
            f.machine.cpu().progress().estimated_cycles == setup_cycles + completed,
        "Every register-count body NOP has its literal full state and one actual native period");
  }
}

void call_return() {
  std::array<std::uint32_t, padding_words> code{};
  code[0] = dor_two; // DOR #2; JSR to a NOP/RTS; two tail NOPs.
  code[1] = 5;
  code[2] = long_call;
  code[3] = handler;
  fixture::Fixture f(code);
  f.machine.bus().write({.address = handler});
  f.machine.bus().write({.address = handler + 1, .value = return_subroutine});
  attach(f);
  const auto initial = f.machine.cpu().registers();
  const auto progress = f.machine.cpu().progress();
  constexpr unsigned instructions = 11;
  finish(f, progress.instructions + instructions);
  auto expected = finished(initial, {.count = 2, .width = 4});
  expected.stack[3] = (std::uint64_t{body + 2} << word_bits) | initial.sr | loop_flag;
  require(f.machine.cpu().registers() == expected,
          "Native JSR/RTS retains its exact return frame, outer loop and every unrelated register");
}

void interrupt() {
  std::array<std::uint32_t, padding_words> code{};
  constexpr unsigned move_sr = 0x05f439;
  constexpr unsigned loop_body = fixture::program + 4;
  code[0] = move_sr; // Actual MOVEC #0,SR; DOR #3; four NOPs.
  code[2] = dor_three;
  code[3] = 5;
  fixture::Fixture f(code);
  f.machine.bus().write({.address = vector});
  f.machine.bus().write({.address = vector + 1});
  attach(f);
  auto initial = f.machine.cpu().registers();
  const auto before = f.machine.cpu().progress();
  static_cast<void>(advance(f));
  static_cast<void>(advance(f));
  initial.sr = 0;
  initial.pc = fixture::program + 2;
  require(f.machine.cpu().registers() == initial &&
              f.machine.cpu().progress().instructions == before.instructions + 1,
          "The actual authored MOVEC unmasks IRQs without changing unrelated native registers");
  // MOVEC's two-word successor still needs its actual F2 extension before
  // DO admission. Observe its real retirement rather than fitting a delay.
  finish(f, before.instructions + 2);
  static_cast<void>(advance(f));
  const auto foreground = f.machine.cpu().registers();
  const auto count = f.machine.cpu().progress().instructions;
  require(foreground.pc == loop_body + 1 && foreground.lc == 3 && foreground.sp == 2,
          "IRQA is asserted at an actual active-loop foreground boundary");
  f.machine.bus().write({.space = DspSpace::x, .address = priority, .value = edge_priority});
  f.machine.bus().observe_irqa(true);
  f.machine.bus().observe_irqa(false);
  const auto first = advance(f);
  require(first.native && first.native->pending &&
              first.native->pending->effect.kind == dsp56300::EffectKind::fetch_extension &&
              first.native->pending->effect.address == vector + 1 &&
              f.machine.interrupt_pending() && f.machine.cpu().registers() == foreground,
          "The native vector owns both reads before changing the actual loop foreground");
  const auto second = advance(f);
  const auto tail = advance(f);
  require(second.native && second.native->retired && tail.native && tail.native->retired &&
              f.machine.cpu().registers() == foreground && !f.machine.interrupt_pending() &&
              !f.machine.bus().interrupt_request(0) &&
              f.machine.cpu().progress().instructions == count + 2,
          "Only both genuine NOP vector slots acknowledge and resume the exact outer loop state");
  constexpr unsigned instructions = 16; // MOVEC +DO +12body +2vector NOPs.
  finish(f, before.instructions + instructions);
  auto expected = initial;
  expected.pc = loop_body + 4;
  expected.stack[1] = (std::uint64_t{initial.la} << word_bits) | initial.lc;
  expected.stack[2] = (std::uint64_t{loop_body} << word_bits) | initial.sr;
  require(f.machine.cpu().registers() == expected,
          "The interrupted loop eventually restores every native register and physical stack cell");
}
} // namespace

void control_checks() {
  for (const unsigned count : {0U, 1U, 3U, 16U}) {
    for (const bool relative : {false, true}) {
      register_count({.count = count, .relative = relative});
    }
  }
  call_return();
  interrupt();
  std::cout << "Native loop JSR/RTS and real IRQA foreground checks passed\n";
}
} // namespace blofeld::test::native_loop
