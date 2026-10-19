// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_clocked_midi.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_midi.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "blofeld/mcu_sci.hpp"
#include "lcd_fixture.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "original_owned_clock_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <ostream>
#include <string>

namespace blofeld::test::owned_clock {
namespace {
constexpr unsigned bus_hz = 30000000;
constexpr unsigned divisor = 60;
constexpr unsigned bit_periods = 960;
constexpr unsigned frame_periods = 9600;
constexpr unsigned maximum_delay = 360;
constexpr unsigned sci_vector = 0xffd6;
constexpr unsigned sci_handler = 0xb5c1;
constexpr unsigned mailbox = 0x19b6;
constexpr unsigned activity_flags = 0x1812;
constexpr unsigned activity_timeout = 0x1830;
constexpr unsigned realtime_begin = 0xf8;
constexpr unsigned activity_duration = 20;
constexpr unsigned idle_bound = 1000;
constexpr unsigned step_limit = 100000;
constexpr unsigned ordinary_isr_instructions = 16;
constexpr unsigned realtime_isr_instructions = 12;
constexpr unsigned prefix_idle = 2282;
constexpr unsigned prefix_retired = 1358;
constexpr std::uint64_t replay_cycles = 141816115;
constexpr std::uint64_t first_decision = main_cycles + 10191;
constexpr std::array<std::uint8_t, 8> midi{0xf8, 0x90, 60, 100, 0xf8, 0x80, 60, 0};
constexpr std::uint64_t wire_begin = main_cycles + bit_periods;
constexpr std::uint64_t wire_end = wire_begin + (midi.size() * frame_periods);

ExecutionContext context(const McuMachine &machine) {
  const auto state = machine.cpu().state();
  return {.pc = state.pc, .ccr = state.ccr};
}

struct Activity {
  unsigned flags = 0;
  std::uint64_t timeout = 0;
};

Activity activity(McuBus &bus) {
  return {.flags = bus.read(activity_flags),
          .timeout = bus.read_memory({.address = activity_timeout, .size = 2})};
}

void check_native_frame(McuBus &bus, const CpuState foreground) {
  const std::array<unsigned, 9> expected{foreground.ccr,
                                         foreground.d & byte_mask,
                                         static_cast<unsigned>(foreground.d) >> 8U,
                                         static_cast<unsigned>(foreground.x) >> 8U,
                                         foreground.x & byte_mask,
                                         static_cast<unsigned>(foreground.y) >> 8U,
                                         foreground.y & byte_mask,
                                         static_cast<unsigned>(foreground.pc) >> 8U,
                                         foreground.pc & byte_mask};
  for (unsigned index = 0; index < expected.size(); ++index) {
    const auto address = static_cast<std::uint16_t>(foreground.sp - 9U + index);
    require(bus.read(address) == expected[index],
            "Each actual interrupt retains the NXP frame layout");
  }
}

struct StepObservation {
  CpuState before;
  CpuStep result;
  unsigned opcode = 0;
  bool waiting = false;
  std::uint64_t cycles = 0;
};

struct MidiInterrupts {
  struct Active {
    CpuState foreground;
    unsigned vector = 0;
    unsigned instructions = 0;
    Activity previous;
  };

  void finish(McuBus &bus) {
    if (!active) { throw Fault("Missing native MIDI interrupt state"); }
    if (active->vector != sci_vector) { return; }
    require(serial < midi.size(), "Original SCI must not fabricate extra received frames");
    const auto value = midi.at(serial);
    const bool realtime = value >= realtime_begin;
    const auto expected_flags = realtime ? active->previous.flags : active->previous.flags | 4U;
    const auto expected_timeout = realtime ? active->previous.timeout : activity_duration;
    const auto current = activity(bus);
    require(
        bus.read(mailbox) == value && current.flags == expected_flags &&
            current.timeout == expected_timeout &&
            active->instructions ==
                (realtime ? realtime_isr_instructions : ordinary_isr_instructions),
        "Original SCI stores each exact wire byte and preserves its independent ISR/activity path");
    ++serial;
  }

  void observe(McuMachine &machine, const StepObservation step) {
    const auto after = machine.cpu().state();
    std::optional<CpuState> chained;
    if (active && step.result.instruction_executed) { ++active->instructions; }
    if (active && step.result.instruction_executed && step.opcode == return_interrupt) {
      finish(machine.bus());
      if (step.result.interrupt_vector) {
        chained = active->foreground;
      } else {
        require(same_cpu(active->foreground, after),
                "Every native SCI/RTI return restores all interrupted foreground registers");
      }
      active.reset();
      ++returns;
    }
    if (!step.result.interrupt_vector) { return; }
    const auto vector = *step.result.interrupt_vector;
    require(!active && (vector == sci_vector || vector == realtime_vector),
            "Only actual non-nested original SCI and RTI sources may enter");
    auto foreground = chained.value_or(step.before);
    if (step.waiting && !chained) {
      foreground.sp = static_cast<std::uint16_t>(foreground.sp + 9U);
    }
    require(step.result.stack_reused == (step.waiting || chained.has_value()) &&
                after.sp == foreground.sp - 9U &&
                after.pc == (vector == sci_vector ? sci_handler : realtime_handler),
            "Native WAI/RTI chaining reuses frames; ordinary native IRQ entry pushes exactly once");
    check_native_frame(machine.bus(), foreground);
    active =
        Active{.foreground = foreground, .vector = vector, .previous = activity(machine.bus())};
    ++entries;
    if (step.result.stack_reused) { ++reused; }
    if (vector == sci_vector && first_serial == 0) { first_serial = step.cycles; }
  }

  std::optional<Active> active;
  unsigned entries = 0;
  unsigned reused = 0;
  unsigned returns = 0;
  unsigned serial = 0;
  std::uint64_t first_serial = 0;
};

struct MidiRun {
  McuMachine &machine;
  LcdFixture &display;
  McuClockedMidiInput &input;
  Census census{};
  FlashPhases phases{};
  MidiInterrupts interrupts{};
  Inputs inputs{.phy = true, .capture = true};

  void attach() {
    machine.set_cycle_observer([this](const auto cycle) {
      input.observe(machine.bus(), machine.run_clock(), cycle.context);
      phases.observe(cycle, machine.run_clock().bus_cycles());
    });
    machine.set_wait_observer([this](const auto cycle) {
      input.observe(machine.bus(), machine.run_clock(), cycle.context);
    });
    machine.bus().set_observer([this](const auto &event) { display.observe(event); });
  }

  void advance() {
    inputs.observe(machine);
    const auto before = machine.cpu().state();
    const bool waiting = machine.cpu().waiting();
    const auto cycles = machine.run_clock().bus_cycles();
    if (waiting && !machine.cpu().pending_interrupt()) {
      const auto remaining = cycles < wire_end ? wire_end - cycles : 1;
      machine.advance_wait(static_cast<unsigned>(std::min<std::uint64_t>(idle_bound, remaining)));
    } else {
      census.before(machine);
      const auto opcode = machine.bus().peek(before.pc);
      const auto result = machine.cpu().step();
      census.retire(before, result);
      interrupts.observe(machine,
                         {.before = before,
                          .result = result,
                          .opcode = opcode,
                          .waiting = waiting,
                          .cycles = cycles});
    }
    input.synchronize(machine.run_clock(), context(machine));
  }

  [[nodiscard]] bool waiting() const {
    return machine.cpu().waiting() && !machine.cpu().pending_interrupt();
  }
};

void reach_partial(MidiRun &run) {
  run.attach();
  require(run.machine.advance_wait(bit_periods) == bit_periods,
          "A source-observed high mark advances actual owned WAI time before input enqueue");
  run.input.synchronize(run.machine.run_clock(), context(run.machine));
  run.input.enqueue(midi);
  for (unsigned step = 0; step < step_limit; ++step) {
    run.advance();
    if (run.waiting() && run.interrupts.returns == 1) {
      require(
          run.machine.run_clock().bus_cycles() == replay_cycles &&
              run.machine.idle_bus_cycles() == prefix_idle &&
              run.census.retired == prefix_retired && run.interrupts.serial == 0 &&
              run.input.queued_bytes() == midi.size(),
          "The native first RTI/foreground returns mid-byte at its independently pinned boundary");
      return;
    }
  }
  throw Fault("Original MIDI prefix did not reach its bounded partial-frame checkpoint");
}

struct ReplayCase {
  unsigned level = 0;
  unsigned delay = 0;
  unsigned attempt = 0;
};

void finish_midi(MidiRun &run,
                 const std::filesystem::path &path,
                 std::ostream &trace,
                 const ReplayCase selected) {
  run.attach();
  try {
    unsigned step = 0;
    for (; step < step_limit; ++step) {
      if (run.waiting() && run.input.queued_bytes() == 0 && run.interrupts.serial == midi.size()) {
        break;
      }
      run.advance();
    }
    require(step < step_limit && run.interrupts.entries == run.interrupts.returns &&
                !run.interrupts.active && run.machine.cpu().state().pc == main_resume &&
                run.machine.cpu().state().sp == wait_stack &&
                run.interrupts.first_serial == first_decision + selected.delay &&
                run.machine.run_clock().bus_cycles() >= wire_end && !run.input.faulted() &&
                !run.phases.pending && run.phases.high == run.phases.low,
            "Every clocked original MIDI byte returns naturally to WAI with its native first IRQ "
            "phase");
  } catch (...) {
    run.census.write(artifact(path, ".census.jsonl"));
    run.phases.write(artifact(path, ".phases.json"));
    throw;
  }
  run.census.write(artifact(path, ".census.jsonl"));
  run.phases.write(artifact(path, ".phases.json"));
  trace << "midi input=" << selected.level << " delay=" << selected.delay
        << " replay=" << selected.attempt << " retired=" << run.census.retired
        << " entries=" << run.interrupts.entries << " reused=" << run.interrupts.reused
        << " returns=" << run.interrupts.returns << " serial=" << run.interrupts.serial
        << " first-sci=" << run.interrupts.first_serial
        << " cycles=" << run.machine.run_clock().bus_cycles()
        << " idle=" << run.machine.idle_bus_cycles()
        << " ticks=" << run.machine.run_clock().elapsed_ticks()
        << " OSC=" << run.machine.run_clock().oscillator_cycles()
        << " extra-flash=" << run.phases.low << '\n';
  trace.flush();
}

void check_strict(McuMachine &machine,
                  LcdFixture &display,
                  const unsigned level,
                  std::ostream &trace) {
  McuClockedMidiInput input(bus_hz, SciReceivePin::pr2, machine.run_clock());
  MidiRun run{.machine = machine, .display = display, .input = input};
  reach_partial(run);
  try {
    for (unsigned step = 0; step < step_limit; ++step) { run.advance(); }
  } catch (const Fault &fault) {
    require(
        machine.timing_faulted() && !machine.sci_receive_reference() &&
            machine.run_clock().bus_cycles() == first_decision &&
            std::string(fault.what()).contains("Unverified SCI stop-to-data flag transfer"),
        "Original continuously sampled strict input must expose its exact unqualified SCI edge");
    trace << "midi-strict input=" << level << " cycles=" << machine.run_clock().bus_cycles()
          << " idle=" << machine.idle_bus_cycles() << '\n';
    detach(machine);
    return;
  }
  throw Fault(
      "Original unqualified SCI input unexpectedly completed without its visible timing fault");
}
}

void verify_midi_reference_replays(McuMachine &machine,
                                   const LcdFixture &display,
                                   const std::filesystem::path &path,
                                   std::ostream &trace,
                                   const unsigned level) {
  const auto baseline = machine.checkpoint();
  const LcdFixture baseline_display(machine.bus(), display);
  const auto initial = machine.bus().power_off_flash_image();
  LcdFixture strict_display(machine.bus(), baseline_display);
  check_strict(machine, strict_display, level, trace);
  for (const unsigned delay :
       {0U, 1U, divisor - 1, divisor, divisor + 1, maximum_delay - 1, maximum_delay}) {
    machine.restore(baseline);
    LcdFixture current_display(machine.bus(), baseline_display);
    machine.set_sci_receive_reference(
        SciReceiveReference{.baud_divisor = divisor, .transfer_delay_bus_cycles = delay});
    McuClockedMidiInput input(bus_hz, SciReceivePin::pr2, machine.run_clock());
    MidiRun prefix{.machine = machine, .display = current_display, .input = input};
    reach_partial(prefix);
    const auto frozen = machine.checkpoint();
    const LcdFixture frozen_display(machine.bus(), current_display);
    const auto frozen_input = input;
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
      machine.restore(frozen);
      LcdFixture replay_display(machine.bus(), frozen_display);
      auto replay_input = frozen_input;
      MidiRun replay{.machine = machine, .display = replay_display, .input = replay_input};
      const auto suffix = ".midi" + std::to_string(delay) + ".r" + std::to_string(attempt);
      finish_midi(replay,
                  artifact(path, suffix),
                  trace,
                  {.level = level, .delay = delay, .attempt = attempt});
      check_nvm(machine, initial);
      detach(machine);
    }
  }
  machine.restore(baseline);
}
}
