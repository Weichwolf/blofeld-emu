// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_transfer_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace blofeld::test {
namespace {
constexpr unsigned count_address = 0x20;
constexpr unsigned unread_address = 0x200;
constexpr unsigned saved_count = 0x33;
constexpr unsigned data_cookie = 0x234567;
constexpr unsigned stack_cookie = 0x112233;
constexpr unsigned timer_control = 0xffff8f;
constexpr unsigned timer_vector = 0x54;
constexpr unsigned priority_register = 0xfffffe;
constexpr unsigned irq_status = 0xc00000;
constexpr unsigned condition_mask = 0xff;
constexpr unsigned zero_rep_immediate = 0x0600a0;
constexpr unsigned pflush = 3;
// DSP56300FM13-58/59: immediate, X0, absolute X, postincrement X, and LC counts.
constexpr unsigned zero_do_immediate = 0x060080;
constexpr unsigned zero_do_x0 = 0x06c400;
constexpr unsigned zero_do_absolute = 0x062000;
constexpr unsigned zero_do_indirect = 0x064800;
constexpr unsigned zero_do_lc = 0x06ff00;

struct Case {
  std::string source;
  unsigned status = masked_status;
};

unsigned setup(DspProgram &program, const Case &input) {
  program.immediate("lc", input.source == "lc" ? 0 : saved_count);
  program.immediate("r0", count_address);
  program.immediate("n0", 3);
  program.immediate("m0", word_mask);
  program.immediate("r1", unread_address);
  program.immediate("m1", word_mask);
  program.immediate("x0", 0);
  program.immediate("x1", data_cookie);
  program.immediate("y0", data_cookie);
  set_accumulator(program, "a", 0);
  set_accumulator(program, "b", 0x400000000000);
  program.emit("move x0,x:(r0)");
  if (input.source == "ssh") {
    program.immediate("ssh", 0);
    program.immediate("ssl", stack_cookie);
  }
  program.immediate("sr", input.status);
  for (unsigned index = 0; index < 3; ++index) { program.emit("nop"); }
  const auto prefix = program.pc();
  if (input.source == "#0") {
    // FM13-161 immediate encoding, independent of the SDK assembler.
    program.word(zero_rep_immediate);
  } else if (input.source == "x:$20") {
    // FM13-161: absolute short X count and postincrement-by-N X EA count.
    constexpr unsigned absolute_x = 0x062020;
    program.word(absolute_x);
  } else if (input.source == "x:(r0)+n0") {
    constexpr unsigned indirect_x = 0x064820;
    program.word(indirect_x);
  } else {
    program.emit("rep " + input.source);
  }
  return prefix;
}

void test_source(const Case &input) {
  DspProgram program;
  const auto prefix = setup(program, input);
  const auto body = program.pc();
  program.emit("move x:(r1)+,x1"); // Unknown RAM must not be read by an annulled target.
  const auto next = program.pc();
  program.emit("move #>$77,y0");
  DspBus bus;
  program.install(bus);
  DspCore cpu(bus);
  run_to(cpu, prefix);
  const auto before = cpu.state();
  cpu.step();
  const auto after = cpu.state();
  auto addresses = before.r;
  if (input.source == "x:(r0)+n0") { addresses[0] += 3; }
  const auto stack = input.source == "ssh" ? 0 : before.sp;
  require(
      next == body + 1 && after.pc == next && after.lc == before.lc && after.la == before.la &&
          after.sr == before.sr && after.omr == before.omr && after.a == before.a &&
          after.b == before.b && after.x == before.x && after.y == before.y &&
          after.r == addresses && after.n == before.n && after.m == before.m && after.sp == stack,
      "Normal zero REP must annul only its target and preserve LC/flags/state: " + input.source);
  cpu.step();
  constexpr unsigned following_value = 0x77;
  require(cpu.state().y == following_value && cpu.state().pc == next + 2,
          "Instruction following zero REP must execute normally");
}

void test_sources() {
  unsigned cases = 0;
  for (const auto *source : {"#0", "x0", "x:$20", "x:(r0)+n0", "lc", "ssh", "a"}) {
    for (const unsigned flags : {0U, carry | zero, condition_mask}) {
      test_source({.source = source, .status = masked_status | flags});
      ++cases;
    }
  }
  std::cout << cases << " zero-REP count-source/flag vectors passed\n";
}

void test_unimplemented_target_is_not_executed() {
  DspProgram program;
  const auto prefix = setup(program, {.source = "x0"});
  program.word(pflush);
  const auto next = program.pc();
  program.emit("nop");
  DspBus bus;
  program.install(bus);
  DspCore cpu(bus);
  run_to(cpu, prefix);
  cpu.step();
  require(cpu.state().pc == next && cpu.state().lc == saved_count,
          "An annulled valid one-word cache instruction must have no implementation side effects");
}

void expect_fault(DspCore &cpu, const std::string &reason) {
  try {
    cpu.step();
  } catch (const Fault &fault) {
    const std::string message = fault.what();
    require(message.contains(reason) && message.contains("pc=") && message.contains("omr="),
            "Zero-REP rejection must retain its original reason and CPU context: " + message);
    return;
  }
  throw Fault("Unqualified zero REP unexpectedly succeeded");
}

void test_strict_boundaries() {
  for (const bool extended : {false, true}) {
    DspProgram program;
    const auto prefix = setup(program, {.source = "x0"});
    if (extended) { program.emit("move x0,x:>$300"); }
    DspBus bus;
    program.install(bus);
    DspCore cpu(bus);
    run_to(cpu, prefix);
    expect_fault(cpu, extended ? "zero REP target" : "Unknown DSP memory read at P:");
  }
  DspProgram program;
  const auto prefix = setup(program, {.source = "x0", .status = masked_status | compatibility});
  program.emit("nop");
  DspBus bus;
  program.install(bus);
  DspCore cpu(bus);
  run_to(cpu, prefix);
  expect_fault(cpu, "compatibility-zero");
}

void test_forbidden_loop_end_positions() {
  for (unsigned distance = 0; distance < 3; ++distance) {
    DspProgram program;
    program.immediate("x0", 0);
    const auto loop = program.pc() + 2;
    const auto last = loop + 4;
    program.emit("do #2," + number(last + 1));
    const auto prefix = last - distance;
    while (program.pc() != prefix) { program.emit("nop"); }
    program.emit("rep x0");
    program.emit("nop");
    DspBus bus;
    program.install(bus);
    DspCore cpu(bus);
    run_to(cpu, prefix);
    expect_fault(cpu, "Forbidden zero REP near DO loop end");
  }
}

void test_annulment_at_program_address_wrap() {
  DspProgram program;
  constexpr unsigned prefix = word_mask - 1;
  constexpr unsigned absolute_jump = 0x0af080;
  program.word(absolute_jump);
  program.word(prefix);
  DspBus bus;
  program.install(bus);
  const std::array<std::uint32_t, 2> words{zero_rep_immediate, 0};
  bus.install_rom({.start = prefix, .words = words});
  DspCore cpu(bus);
  run_to(cpu, prefix);
  const auto before = cpu.state();
  cpu.step();
  require(cpu.state().pc == 0 && cpu.state().sr == before.sr && cpu.state().lc == before.lc,
          "Annulled target at P:ffffff must wrap the 24-bit PC without operand execution");
  expect_fault(cpu, "Unknown DSP memory read at P:0x000000");
}

void test_zero_repeat_in_legal_do_body() {
  DspProgram program;
  program.immediate("lc", saved_count);
  program.immediate("r0", 0);
  program.immediate("m0", word_mask);
  program.immediate("r1", unread_address);
  program.immediate("m1", word_mask);
  const auto prefix = program.pc();
  // Two-word DO; REP is LA-4, outside the A.3.1 forbidden final three positions.
  program.emit("do #2," + number(prefix + 7));
  program.word(zero_rep_immediate);
  program.emit("move x:(r1)+,x0");
  program.emit("move (r0)+");
  program.emit("nop");
  program.emit("nop");
  const auto end = program.pc();
  DspBus bus;
  program.install(bus);
  DspCore cpu(bus);
  run_to(cpu, prefix);
  constexpr unsigned step_limit = 20;
  for (unsigned count = 0; cpu.state().pc != end; ++count) {
    require(count < step_limit,
            "Native DO with annulled REP must retire within its independent bound");
    cpu.step();
  }
  require(cpu.state().r[0] == 2 && cpu.state().r[1] == unread_address &&
              cpu.state().lc == saved_count && cpu.state().sp == 0 &&
              (cpu.state().sr & 0x8000U) == 0,
          "Legal zero REP inside DO must preserve its outer counter and normal loop end");
}

void check_zero_do(const DspDebugState &before,
                   const DspDebugState &after,
                   const unsigned body,
                   const unsigned end,
                   const bool count_ea) {
  const auto &old = before.cpu;
  const auto &state = after.cpu;
  auto addresses = old.r;
  if (count_ea) { addresses[0] += 3; }
  require(state.pc == end && state.lc == old.lc && state.la == old.la && state.sp == old.sp &&
              state.sr == old.sr && state.omr == old.omr && state.a == old.a && state.b == old.b &&
              state.x == old.x && state.y == old.y && state.r == addresses && state.n == old.n &&
              state.m == old.m && after.sc == before.sc && after.ep == before.ep &&
              after.sz == before.sz,
          "Annulled DO must restore live control/counter state without executing its body");
  // FM13-56/57: LA/LC and body-PC/SR are pushed before the loop is annulled.
  // MCPX b2 independently records retained words even when live SP returns to zero.
  for (unsigned slot = 0; slot < before.stack_high.size(); ++slot) {
    auto high = before.stack_high[slot];
    auto low = before.stack_low[slot];
    if (slot == old.sp + 1) {
      high = old.la;
      low = old.lc;
    }
    if (slot == old.sp + 2) {
      high = body;
      low = old.sr;
    }
    require(after.stack_high[slot] == high && after.stack_low[slot] == low,
            "Annulled DO must leave its two frame writes in physical stack slots only");
  }
}

void test_zero_do_sources() {
  unsigned cases = 0;
  for (const auto *source : {"#0", "x0", "x:$20", "x:(r0)+n0", "lc"}) {
    for (const unsigned flags : {0U, carry | zero, condition_mask}) {
      DspProgram program;
      for (unsigned index = 1; index <= 2; ++index) {
        program.immediate("ssh", address_seed + index);
        program.immediate("ssl", offset_seed + index);
      }
      program.emit("move ssh,x1");
      program.emit("move ssh,x1");
      constexpr unsigned saved_address = 0x345;
      program.immediate("la", saved_address);
      program.immediate("lc", std::string(source) == "lc" ? 0 : saved_count);
      program.immediate("r0", count_address);
      program.immediate("n0", 3);
      program.immediate("m0", word_mask);
      program.immediate("r1", unread_address);
      program.immediate("x0", 0);
      program.immediate("x1", data_cookie);
      program.emit("move x0,x:(r0)");
      program.immediate("sr", masked_status | flags);
      const auto prefix = program.pc();
      unsigned instruction = zero_do_immediate;
      const std::string name = source;
      if (name == "x0") { instruction = zero_do_x0; }
      if (name == "x:$20") { instruction = zero_do_absolute; }
      if (name == "x:(r0)+n0") { instruction = zero_do_indirect; }
      if (name == "lc") { instruction = zero_do_lc; }
      program.word(instruction);
      const auto body = prefix + 2;
      const auto end = body + 2;
      program.word(end - 1);
      program.emit("move x:(r1)+,x1");
      program.emit("nop");
      program.emit("nop");
      DspBus bus;
      program.install(bus);
      DspCore cpu(bus);
      run_to(cpu, prefix);
      const auto before = cpu.debug_state();
      cpu.step();
      check_zero_do(before, cpu.debug_state(), body, end, name == "x:(r0)+n0");
      ++cases;
    }
  }
  std::cout << cases << " zero-DO count-source/flag/retained-stack vectors passed\n";
}

void test_zero_do_stack_overflow_rejection() {
  DspProgram program;
  program.immediate("sp", 14);
  for (unsigned count = 0; count < 4; ++count) { program.emit("nop"); }
  const auto prefix = program.pc();
  program.word(zero_do_immediate);
  program.word(prefix + 2);
  program.emit("nop");
  DspBus bus;
  program.install(bus);
  DspCore cpu(bus);
  run_to(cpu, prefix);
  expect_fault(cpu, "zero DO stack overflow at SP=14");
}

void test_nested_annulled_do_preserves_outer_frame() {
  DspProgram program;
  program.immediate("lc", saved_count);
  program.immediate("r0", 0);
  program.immediate("m0", word_mask);
  program.immediate("r1", unread_address);
  const auto outer = program.pc();
  const auto inner = outer + 2;
  const auto inner_body = inner + 2;
  const auto inner_end = inner + 4;
  const auto outer_end = inner + 9;
  program.emit("do #2," + number(outer_end));
  program.word(zero_do_immediate);
  program.word(inner_end - 1);
  program.emit("move x:(r1)+,x0");
  program.emit("nop");
  program.emit("move (r0)+");
  for (unsigned count = 0; count < 4; ++count) { program.emit("nop"); }
  require(program.pc() == outer_end, "Nested zero-DO diagnostic uses its literal end addresses");
  DspBus bus;
  program.install(bus);
  DspCore cpu(bus);
  run_to(cpu, outer);
  unsigned annulments = 0;
  constexpr unsigned step_limit = 30;
  for (unsigned count = 0; cpu.state().pc != outer_end; ++count) {
    require(count < step_limit, "Nested annulled DO must retire within its independent bound");
    const auto before = cpu.debug_state();
    cpu.step();
    if (before.cpu.pc == inner) {
      check_zero_do(before, cpu.debug_state(), inner_body, inner_end, false);
      ++annulments;
    }
  }
  require(annulments == 2 && cpu.state().r[0] == 2 && cpu.state().r[1] == unread_address &&
              cpu.state().sp == 0 && cpu.state().lc == saved_count,
          "Each inner zero DO must preserve the live outer loop until normal completion");
}

void test_pending_irq_across_zero_repeat_chain() {
  DspProgram program;
  program.immediate("sr", irq_status);
  program.emit("movep #>" + number(0x100) + ",x:<<" + number(priority_register));
  program.emit("movep #>5,x:<<" + number(timer_control - 1));
  program.emit("movep #>5,x:<<" + number(timer_control - 2));
  program.emit("movep #>7,x:<<" + number(timer_control));
  program.emit("rep #1");
  const auto first_body = program.pc();
  program.emit("nop");
  program.word(zero_rep_immediate);
  program.emit("nop");
  const auto next_prefix = program.pc();
  program.emit("rep #1");
  program.emit("nop");
  const auto foreground = program.pc();
  program.emit("nop");
  DspBus bus;
  program.install(bus);
  for (unsigned offset = 0; offset < 2; ++offset) {
    bus.write({.address = timer_vector + offset, .value = 0});
  }
  DspCore cpu(bus);
  run_to(cpu, first_body);
  bus.timers().advance_clk2(1);
  cpu.step();
  cpu.step();
  require(cpu.state().pc == next_prefix && bus.interrupt_request(0).has_value(),
          "Pending IRQ must remain deferred across the consecutive zero REP prefix");
  cpu.step();
  cpu.step();
  require(cpu.state().pc == foreground && bus.interrupt_request(0).has_value(),
          "Consecutive positive REP after annulment must also defer the pending IRQ");
  cpu.step();
  require(cpu.state().pc == foreground && cpu.state().sp == 0,
          "Fast vector must execute before the next ordinary foreground instruction");
}
}
}

int main() {
  try {
    blofeld::test::test_zero_do_sources();
    blofeld::test::test_zero_do_stack_overflow_rejection();
    blofeld::test::test_nested_annulled_do_preserves_outer_frame();
    blofeld::test::test_sources();
    blofeld::test::test_unimplemented_target_is_not_executed();
    blofeld::test::test_strict_boundaries();
    blofeld::test::test_forbidden_loop_end_positions();
    blofeld::test::test_annulment_at_program_address_wrap();
    blofeld::test::test_zero_repeat_in_legal_do_body();
    blofeld::test::test_pending_irq_across_zero_repeat_chain();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
