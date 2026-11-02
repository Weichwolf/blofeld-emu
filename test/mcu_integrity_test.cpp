// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <string>

namespace {
using namespace blofeld;
using test::require;
constexpr unsigned oscillator = 12000000, common_hz = 540000000;
constexpr unsigned operand = 0x1234, other_operand = 0x2345, expected_flags = 0xd0;
constexpr std::uint64_t initial_ticks = 450;

McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspRuntimeInputs settings() {
  return {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          .maximum_cycles_per_period = 16,
          .writes = McuWriteOrder::before_mcu,
          .external_writes = McuWriteOrder::before_mcu,
          .wait_dsp_first = true,
          .reset_dsp_first = true,
          .initial_reset = OwnDspResetInputs{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
}

std::array<unsigned, 6> registers(const CpuState state) {
  return {state.pc, state.d, state.x, state.y, state.sp, state.ccr};
}

void advanced(const CpuInspectionStamp before, const Hc12 &cpu) {
  const auto after = cpu.inspection_stamp();
  require(before.owner != 0 && after.owner == before.owner && after.revision > before.revision,
          "A native or explicit CPU mutation invalidates the old integrity stamp");
}

void debug_and_restore() {
  McuMachine mcu(test::checkpoint_image(), BLOFELD_DEFAULT_SLA, contract());
  test::drive_interrupts(mcu.bus());
  const auto initial = mcu.cpu().state();
  const auto original = mcu.checkpoint();
  const auto stamp = mcu.cpu().inspection_stamp();
  static_cast<void>(mcu.cpu().state());
  static_cast<void>(mcu.cpu().disassemble());
  static_cast<void>(mcu.cpu().pending_interrupt());
  static_cast<void>(mcu.checkpoint());
  require(mcu.cpu().inspection_stamp() == stamp, "Read-only inspection cannot mutate the stamp");
  const std::array<CpuState, 6> replacements{{{.pc = test::entry + 1},
                                              {.pc = test::entry, .d = operand},
                                              {.pc = test::entry, .x = operand},
                                              {.pc = test::entry, .y = operand},
                                              {.pc = test::entry, .sp = test::stack},
                                              {.pc = test::entry, .ccr = expected_flags}}};
  for (const auto value : replacements) {
    auto before = mcu.cpu().inspection_stamp();
    mcu.cpu().set_state(value);
    advanced(before, mcu.cpu());
    require(registers(mcu.cpu().state()) == registers(value), "Literal debug register replacement");
    before = mcu.cpu().inspection_stamp();
    mcu.cpu().set_state(value);
    advanced(before, mcu.cpu());
    mcu.cpu().set_state(initial);
    require(registers(mcu.cpu().state()) == registers(initial), "Literal register restoration");
  }
  auto before = mcu.cpu().inspection_stamp();
  mcu.cpu().reset();
  advanced(before, mcu.cpu());
  std::set<std::uint64_t> owners{before.owner};
  for (unsigned restore = 0; restore < 4; ++restore) {
    before = mcu.cpu().inspection_stamp();
    mcu.restore(original);
    const auto after = mcu.cpu().inspection_stamp();
    require(after.owner != before.owner && owners.insert(after.owner).second &&
                registers(mcu.cpu().state()) == registers(initial),
            "Fresh checkpoint owners cannot reuse a former token even when all registers agree");
  }
}

void native_and_wait() {
  // Independently authored HC12 LDD #1234; LDX #2345; NOP.
  constexpr std::array<std::uint8_t, 7> program{0xcc, 0x12, 0x34, 0xce, 0x23, 0x45, 0xa7};
  McuMachine mcu(test::checkpoint_image(program), BLOFELD_DEFAULT_SLA, contract());
  test::drive_interrupts(mcu.bus());
  auto before = mcu.cpu().inspection_stamp();
  require(mcu.cpu().step().instruction_executed && mcu.cpu().state().pc == test::entry + 3 &&
              mcu.cpu().state().d == operand,
          "The actual native LDD retains its literal result and instruction length");
  advanced(before, mcu.cpu());
  before = mcu.cpu().inspection_stamp();
  require(mcu.cpu().step().instruction_executed && mcu.cpu().state().pc == test::entry + 6 &&
              mcu.cpu().state().x == other_operand,
          "The actual native LDX retains its literal result and instruction length");
  advanced(before, mcu.cpu());
  constexpr std::array<std::uint8_t, 1> wait{0x3e};
  McuMachine waiting(test::checkpoint_image(wait), BLOFELD_DEFAULT_SLA, contract());
  test::drive_interrupts(waiting.bus());
  waiting.cpu().set_state({.pc = test::entry, .sp = test::stack, .ccr = expected_flags});
  before = waiting.cpu().inspection_stamp();
  require(waiting.cpu().step().instruction_executed && waiting.cpu().waiting(),
          "The authored native WAI genuinely stacks and enters idle state");
  advanced(before, waiting.cpu());
  const auto held = registers(waiting.cpu().state());
  before = waiting.cpu().inspection_stamp();
  const auto idle = waiting.idle_bus_cycles();
  require(waiting.advance_wait(2) == 2 && waiting.idle_bus_cycles() == idle + 2 &&
              waiting.cpu().waiting() && registers(waiting.cpu().state()) == held,
          "Actual committed WAI clocks preserve the held architectural registers");
  advanced(before, waiting.cpu());
}

void failed_native() {
  constexpr std::array<std::uint8_t, 3> unknown_read{0xb6, 0x30, 0};
  McuMachine mcu(test::checkpoint_image(unknown_read), BLOFELD_DEFAULT_SLA, contract());
  test::drive_interrupts(mcu.bus());
  const auto before = mcu.cpu().inspection_stamp();
  bool rejected = false;
  try {
    static_cast<void>(mcu.cpu().step());
  } catch (const Fault &error) { rejected = std::string(error.what()).contains("3000"); }
  require(rejected, "The native unmapped read remains a visible address-specific failure");
  advanced(before, mcu.cpu());
}

enum class Mutation : std::uint8_t {
  read,
  same,
  pc,
  d,
  x,
  y,
  sp,
  ccr,
  revert,
  reset,
  step,
  restore
};
enum class Lane : std::uint8_t { query, observe, peripheral_query, peripheral_observe };

void mutate(McuMachine &mcu, const McuCheckpoint &snapshot, const Mutation kind) {
  const auto initial = mcu.cpu().state();
  auto next = initial;
  switch (kind) {
    case Mutation::read:
      static_cast<void>(mcu.cpu().disassemble());
      static_cast<void>(mcu.cpu().pending_interrupt());
      static_cast<void>(mcu.checkpoint());
      return;
    case Mutation::same: break;
    case Mutation::pc: ++next.pc; break;
    case Mutation::d: next.d = operand; break;
    case Mutation::x: next.x = operand; break;
    case Mutation::y: next.y = operand; break;
    case Mutation::sp: next.sp = test::stack; break;
    case Mutation::ccr: next.ccr = 0; break;
    case Mutation::revert: next.d = operand; break;
    case Mutation::reset: mcu.cpu().reset(); return;
    case Mutation::step: static_cast<void>(mcu.cpu().step()); return;
    case Mutation::restore: mcu.restore(snapshot); return;
    default: throw Fault("Invalid integrity mutation case");
  }
  mcu.cpu().set_state(next);
  if (kind == Mutation::revert) { mcu.cpu().set_state(initial); }
}

void callback(const Mutation mutation, const Lane lane) {
  McuMachine mcu(test::checkpoint_image(), BLOFELD_DEFAULT_SLA, contract());
  test::drive_interrupts(mcu.bus());
  const auto snapshot = mcu.checkpoint();
  OwnDspMachine dsp({.extal_hz = oscillator,
                     .timebase_hz = common_hz,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary});
  const bool observing = lane == Lane::observe || lane == Lane::peripheral_observe;
  const bool peripheral = lane == Lane::peripheral_query || lane == Lane::peripheral_observe;
  bool pending = observing;
  unsigned actions = 0;
  const auto action = [&] {
    ++actions;
    mutate(mcu, snapshot, mutation);
  };
  const auto query = [&]() -> std::optional<std::uint64_t> {
    if (!observing) { action(); }
    return pending ? std::optional{initial_ticks} : std::nullopt;
  };
  const auto observe = [&](std::uint64_t) {
    pending = false;
    action();
  };
  DspRuntimeEvents external;
  DspRuntimePeripheralEvents peer;
  if (peripheral) {
    peer = {.identity = std::make_shared<const PeripheralEventIdentity>(),
            .next_ticks = query,
            .observe = observe,
            .order = DspPeerOrder::peer_first,
            .maximum_events_per_period = 4};
  } else {
    external = {.identity = std::make_shared<const bool>(true),
                .next_ticks = query,
                .observe = observe,
                .order = DspPeerOrder::peer_first,
                .maximum_events_per_period = 4};
  }
  bool rejected = false;
  try {
    const McuDspRuntime runtime(settings(), mcu, dsp, external, peer);
  } catch (const Fault &error) {
    rejected = std::string(error.what()).contains("changed CPU state or time");
  }
  require(actions > 0 && rejected == (mutation != Mutation::read),
          "All four callback lanes permit reads and reject every CPU mutation/restore/revert");
}
}

int main() {
  try {
    debug_and_restore();
    native_and_wait();
    failed_native();
    constexpr std::array mutations{Mutation::read,
                                   Mutation::same,
                                   Mutation::pc,
                                   Mutation::d,
                                   Mutation::x,
                                   Mutation::y,
                                   Mutation::sp,
                                   Mutation::ccr,
                                   Mutation::revert,
                                   Mutation::reset,
                                   Mutation::step,
                                   Mutation::restore};
    for (const auto lane :
         {Lane::query, Lane::observe, Lane::peripheral_query, Lane::peripheral_observe}) {
      for (const auto mutation : mutations) { callback(mutation, lane); }
    }
    std::cout << "CPU integrity: literal native/debug states, WAI clocks, failure, fresh owners; "
                 "44 mutation refusals and four read-only callback lanes passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
