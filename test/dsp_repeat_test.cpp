// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace blofeld::test {
namespace {
constexpr unsigned source = 0x10;
constexpr unsigned sentinel = 0x654321;
constexpr unsigned literal = 0x123456;
// FM13-161: REP #1 and #3; MOVE long immediate has its own mandatory second word.
constexpr unsigned repeat_once = 0x0601a0;
constexpr unsigned repeat_three = 0x0603a0;
constexpr unsigned immediate_x0 = 0x44f400;

void extended_target(const bool known_extension) {
  DspMachine machine;
  DspProgram code;
  code.word(repeat_three);
  const auto body = code.pc();
  code.word(immediate_x0);
  if (known_extension) { code.word(literal); }
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  machine.cpu().step();
  const auto ready = machine.checkpoint();
  const auto before = machine.cpu().debug_state();
  native_access::Recorder capture;
  const auto error = native_access::fault([&] { capture.step(machine); });
  require(error.contains("Unsupported reference REP target") && error.contains("pc=0xff0001") &&
              capture.rows.size() == 2 && machine.cpu().state().x == before.cpu.x &&
              machine.cpu().debug_state().native_instructions == before.native_instructions,
          "FM13-160/A.3.8 forbids a two-word REP body before its extension or result is used");
  native_access::pair(capture.rows,
                      {.kind = native_access::Kind::opcode,
                       .target = {.address = body},
                       .value = immediate_x0,
                       .pc = body});
  static_cast<void>(native_access::fault([&] { machine.cpu().step(); }));
  machine.restore(ready);
  require(machine.cpu().debug_state() == before,
          "A trusted unloaded-REP checkpoint recovers the exact pre-rejection CPU state");
}

void forbidden_flow(const unsigned opcode) {
  DspMachine machine;
  DspProgram code;
  code.immediate("sr", masked_status | zero);
  code.word(repeat_once);
  const auto body = code.pc();
  code.word(opcode);
  code.emit("nop");
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  machine.cpu().step();
  machine.cpu().step();
  const auto before = machine.cpu().debug_state();
  native_access::Recorder capture;
  const auto error = native_access::fault([&] { capture.step(machine); });
  require(error.contains("Unsupported reference REP target") && capture.rows.size() == 2 &&
              machine.cpu().state().sp == before.cpu.sp &&
              machine.cpu().debug_state().native_instructions == before.native_instructions &&
              capture.rows.front().target.address == body,
          "Forbidden REP flow is rejected before execution, including an untaken conditional");
}

void valid_latched_word() {
  DspMachine machine;
  DspProgram code;
  code.emit("jmp $0");
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  machine.cpu().step();
  machine.bus().write({.address = 0, .value = repeat_three});
  // FM13-118: X0 destination, W=1, aa=$10, MOVE low byte=0; one instruction word.
  constexpr unsigned load = 0x449000;
  machine.bus().write({.address = 1, .value = load});
  machine.bus().write({.address = 2, .value = 0});
  machine.bus().write({.space = DspSpace::x, .address = source, .value = literal});
  machine.cpu().step();
  native_access::Recorder capture;
  capture.step(machine);
  require(machine.cpu().state().x == literal && machine.cpu().state().lc == 2,
          "The independently encoded single-word REP target executes its real operand read");
  machine.bus().write({.address = 1, .value = immediate_x0});
  machine.bus().write({.space = DspSpace::x, .address = source, .value = sentinel});
  const auto held = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(held);
    capture.step(machine);
    require(capture.rows.size() == 2 && capture.rows.front().kind == native_access::Kind::read &&
                machine.cpu().state().x == sentinel && machine.cpu().state().lc == 1,
            "Loaded legal REP retains its opcode and rereads data after program/data mutation");
    capture.step(machine);
    require(machine.cpu().state().x == sentinel && machine.cpu().state().pc == 2 &&
                machine.cpu().state().lc == 0,
            "Both checkpoint replays finish three actual iterations of the legal latched word");
  }
}

void invalid_preview() {
  constexpr unsigned extal_hz = 12000000;
  constexpr unsigned timebase_hz = 540000000;
  DspMachine machine(
      DspRunContract{.clock = {.extal_hz = extal_hz,
                               .timebase_hz = timebase_hz,
                               .switch_timing = DspClockSwitch::reference_instruction_boundary},
                     .timing = DspRunTiming::native_internal_estimates});
  DspProgram code;
  code.word(repeat_once);
  code.word(immediate_x0);
  code.word(literal);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.cpu().step();
  const auto before = machine.cpu().debug_state();
  const auto ticks = machine.run_clock().elapsed_ticks();
  const auto error = native_access::fault([&] { static_cast<void>(machine.plan_step()); });
  require(error.contains("Unsupported reference REP target") &&
              machine.cpu().debug_state() == before &&
              machine.run_clock().elapsed_ticks() == ticks && !machine.timing_faulted(),
          "A pure nominal preview rejects the invalid REP body without executing or faulting");
  DspProgram repaired;
  repaired.word(repeat_once);
  repaired.word(0);
  repaired.install(machine.bus());
  require(machine.plan_step().native_cycle_units == 1,
          "The still-unloaded target can be repaired before any body execution");
  machine.cpu().step();
  require(machine.cpu().state().pc == boot + 2 && machine.cpu().state().lc == 0,
          "The same healthy pending REP then executes the actual legal NOP body");
}
}
}

int main() {
  using namespace blofeld::test;
  try {
    extended_target(true);
    extended_target(false);
    // FM Jcc/JSR/RTS/WAIT/STOP/REP formats; NE=2 (table12-17), Z=1 makes JNE untaken.
    constexpr std::array forbidden{0x0e2001U, 0x0d0001U, 0x0cU, 0x86U, 0x87U, repeat_once};
    for (const auto opcode : forbidden) { forbidden_flow(opcode); }
    valid_latched_word();
    invalid_preview();
    std::cout << "Single-word REP qualification, pre-effect rejection and latched replay passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
