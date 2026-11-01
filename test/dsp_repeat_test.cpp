// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"
#include "dsp_rom_program.hpp"
#include "esai_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::DspRomProgram;
using blofeld::test::DspRomRegister;
using blofeld::test::esai::rejects;
using blofeld::test::esai::require;
constexpr unsigned source = 0x10;
constexpr unsigned sentinel = 0x654321;
constexpr unsigned literal = 0x123456;
constexpr unsigned repeat_once = 0x0601a0;
constexpr unsigned repeat_three = 0x0603a0;
constexpr unsigned immediate_x0 = 0x44f400;
constexpr unsigned boot = DspRomProgram::boot;
constexpr unsigned masked_status = 0xc00300;
constexpr unsigned zero = 4;
constexpr unsigned maximum_effects = 32;
constexpr std::uint64_t hold = 90;
constexpr std::uint64_t recovered = 180;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .switch_timing = DspClockSwitch::reference_cycle_boundary};

void reset(OwnDspMachine &machine, const DspRomProgram &program) {
  machine.bus().install_rom({.start = boot, .words = program.words()});
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  require(machine.advance_hardware_reset_ticks(hold) == hold, "Actual native reset hold completes");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(hold) == hold &&
              machine.clock().elapsed_ticks() == recovered,
          "Actual native reset recovery completes before any REP instruction");
}

struct Trace {
  std::vector<dsp56300::InstructionEffect> effects;
  dsp56300::InstructionProgress result;
};

Trace execute(OwnDspMachine &machine) {
  Trace trace;
  trace.result = machine.begin_fetch();
  const auto ticks = machine.clock().elapsed_ticks();
  for (unsigned count = 0;
       count < maximum_effects && trace.result.pending &&
       trace.result.fault.kind == dsp56300::FaultKind::none && !trace.result.retired;
       ++count) {
    const auto pending = trace.result.pending.value();
    trace.effects.push_back(pending.effect);
    trace.result = machine.advance(pending.ticket);
  }
  require(machine.clock().elapsed_ticks() == ticks,
          "Native effect observation cannot manufacture instruction clocks");
  require(trace.result.retired || trace.result.fault.kind != dsp56300::FaultKind::none,
          "The actual native instruction retires or fails within its effect bound");
  return trace;
}

Trace step(OwnDspMachine &machine) {
  auto trace = execute(machine);
  if (trace.result.fault.kind != dsp56300::FaultKind::none) {
    const auto fault = trace.result.fault;
    constexpr std::array spaces{"P:", "X:", "Y:"};
    const bool read = fault.kind == dsp56300::FaultKind::memory_read ||
                      fault.kind == dsp56300::FaultKind::io_read;
    throw Fault("REP native " + std::string(read ? "read" : "execute") + " " +
                spaces.at(static_cast<unsigned>(fault.space)) + hex(fault.address, 6) +
                " pc=" + hex(fault.pc, 6) + " omr=" + hex(machine.cpu().registers().omr, 6) +
                " value=" + hex(fault.value, 6));
  }
  require(trace.result.retired, "The actual authored native instruction retires");
  return trace;
}

bool is_fetch(const dsp56300::InstructionEffect effect) {
  return effect.kind == dsp56300::EffectKind::fetch_opcode ||
         effect.kind == dsp56300::EffectKind::fetch_extension;
}

void invalid_body(OwnDspMachine &machine, const unsigned body, const unsigned opcode) {
  const auto ready = machine.checkpoint();
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto processing = machine.cpu().processing_state();
  const auto trace = execute(machine);
  const dsp56300::Fault expected{.kind = dsp56300::FaultKind::invalid_instruction,
                                 .space = dsp56300::Space::program,
                                 .address = body,
                                 .value = opcode,
                                 .pc = body};
  require(trace.result.fault == expected && !trace.result.retired && trace.effects.size() == 1 &&
              trace.effects.front().kind == dsp56300::EffectKind::fetch_opcode &&
              trace.effects.front().space == dsp56300::Space::program &&
              trace.effects.front().address == body && trace.effects.front().pc == body &&
              machine.cpu().registers() == before && machine.cpu().progress() == progress &&
              machine.cpu().processing_state() == processing,
          "FM13-160/A.3.8 rejects the real opcode before extension, operand, PCU or retirement "
          "effects; observed=" +
              std::to_string(trace.effects.size()) +
              " fault=" + std::to_string(static_cast<unsigned>(trace.result.fault.kind)));
  const auto error = rejects([&] { static_cast<void>(machine.begin_fetch()); });
  require(!error.empty() && machine.cpu().registers() == before &&
              machine.cpu().progress() == progress && machine.cpu().fault() == expected &&
              machine.cpu().processing_state() == processing,
          "Faulted native REP cannot execute another body or change architectural state");
  machine.restore(ready);
  require(machine.cpu().registers() == before && machine.cpu().progress() == progress &&
              machine.cpu().fault().kind == dsp56300::FaultKind::none &&
              machine.cpu().processing_state() == processing,
          "Trusted unloaded-REP restore recovers the exact healthy pre-rejection CPU");
}

void extended_target(const bool known_extension) {
  OwnDspMachine machine(clocks);
  DspRomProgram code;
  code.word(repeat_three);
  const auto body = code.pc();
  code.word(immediate_x0);
  if (known_extension) { code.word(literal); }
  reset(machine, code);
  static_cast<void>(step(machine));
  invalid_body(machine, body, immediate_x0);
}

void forbidden_flow(const unsigned opcode) {
  OwnDspMachine machine(clocks);
  DspRomProgram code;
  code.immediate(DspRomRegister::sr, masked_status | zero);
  code.word(repeat_once);
  const auto body = code.pc();
  code.word(opcode);
  code.word(0);
  reset(machine, code);
  static_cast<void>(step(machine));
  static_cast<void>(step(machine));
  require((machine.cpu().registers().sr & zero) != 0,
          "The untaken NE conditional control has its independently required Z flag");
  invalid_body(machine, body, opcode);
}

void latched_read(const Trace &trace) {
  require(std::ranges::none_of(trace.effects, is_fetch) &&
              std::ranges::count_if(trace.effects,
                                    [](const auto effect) {
                                      return effect.kind == dsp56300::EffectKind::read &&
                                             effect.space == dsp56300::Space::x &&
                                             effect.address == source;
                                    }) == 1,
          "A loaded REP retains its opcode and performs exactly one new real data read");
}

void valid_latched_word() {
  OwnDspMachine machine(clocks);
  DspRomProgram code;
  constexpr unsigned jump = 0x0af080;
  code.append(std::array{jump, 0U});
  reset(machine, code);
  static_cast<void>(step(machine));
  constexpr unsigned load = 0x449000; // FM MOVE X:aa,X0, aa=$10, one word.
  machine.bus().write({.address = 0, .value = repeat_three});
  machine.bus().write({.address = 1, .value = load});
  machine.bus().write({.address = 2, .value = 0});
  machine.bus().write({.space = DspSpace::x, .address = source, .value = literal});
  static_cast<void>(step(machine));
  const auto first = step(machine);
  require(machine.cpu().registers().x == literal && machine.cpu().registers().lc == 2 &&
              std::ranges::count_if(first.effects, is_fetch) == 1,
          "The first legal single-word REP iteration fetches its opcode and reads actual RAM");
  machine.bus().write({.address = 1, .value = immediate_x0});
  machine.bus().write({.space = DspSpace::x, .address = source, .value = sentinel});
  const auto held = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(held);
    latched_read(step(machine));
    require(machine.cpu().registers().x == sentinel && machine.cpu().registers().lc == 1,
            "The loaded legal opcode survives P/data mutation and a trusted checkpoint");
    latched_read(step(machine));
    require(machine.cpu().registers().x == sentinel && machine.cpu().registers().pc == 2 &&
                machine.cpu().registers().lc == 0 && machine.cpu().progress().instructions == 5 &&
                !machine.cpu().instruction_fetch().latched_opcode,
            "Both native suffixes finish three real iterations and release the opcode latch");
    constexpr unsigned back_to_body = 0x0c0001;
    machine.bus().write({.address = 2, .value = back_to_body});
    static_cast<void>(step(machine));
    const auto ordinary = step(machine);
    require(machine.cpu().registers().x == back_to_body && machine.cpu().registers().pc == 3 &&
                machine.cpu().progress().instructions == 7 &&
                std::ranges::count_if(ordinary.effects, is_fetch) == 2,
            "Ordinary execution after REP reads the changed opcode and its real extension");
  }
}

void invalid_preview() {
  OwnDspMachine machine(clocks);
  DspRomProgram code;
  code.append(std::array{repeat_once, immediate_x0, literal});
  reset(machine, code);
  static_cast<void>(step(machine));
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto ticks = machine.clock().elapsed_ticks();
  const auto opcode = machine.bus().sample_program(before.pc);
  if (!opcode) { throw Fault("The pure REP preview requires an actually known authored opcode"); }
  const auto invalid = dsp56300::inspect_instruction(*opcode);
  require(
      invalid && invalid->words == 2 && !invalid->repeatable &&
          machine.cpu().registers() == before && machine.cpu().progress() == progress &&
          machine.clock().elapsed_ticks() == ticks && !machine.clock().faulted() &&
          machine.cpu().fault().kind == dsp56300::FaultKind::none,
      "Pure native shape inspection refuses an invalid REP body without CPU, I/O or clock effects");
  DspRomProgram repaired;
  repaired.append(std::array{repeat_once, 0U});
  machine.bus().install_rom({.start = boot, .words = repaired.words()});
  const auto legal = machine.bus().sample_program(before.pc);
  if (!legal) { throw Fault("The repaired authored REP opcode is not known"); }
  const auto shape = dsp56300::inspect_instruction(*legal);
  require(shape && shape->words == 1 && shape->repeatable && machine.cpu().registers() == before &&
              machine.cpu().progress() == progress && machine.clock().elapsed_ticks() == ticks,
          "The still-unloaded target can change before actual execution without spending time");
  static_cast<void>(step(machine));
  require(machine.cpu().registers().pc == boot + 2 && machine.cpu().registers().lc == 0,
          "The same healthy pending REP executes the actual repaired NOP body");
}
}

int main() {
  try {
    extended_target(true);
    extended_target(false);
    constexpr std::array forbidden{0x0e2001U, 0x0d0001U, 0x0cU, 0x86U, 0x87U, repeat_once};
    for (const auto opcode : forbidden) { forbidden_flow(opcode); }
    valid_latched_word();
    invalid_preview();
    std::cout
        << "Native single-word REP rejection, pure inspection and actual latched replay passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
