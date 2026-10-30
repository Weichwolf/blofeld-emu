// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp56kEmu/assembler.h"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000, extal = 12000000, timebase = 1080000000;
constexpr unsigned pin = 0x200, padding = 16, maximum_periods = 96;
constexpr std::uint64_t reset_ticks = 180;
// FM13-24/36: BCLR #9,X:$ffffbd; BSET #0,X:$ffff8f, short I/O.
constexpr unsigned gpio_clear = 0x013d09, timer_set = 0x010f20;
// DSP56371UM TCSR: overflow/compare are write-one-to-clear, TE is bit0.
constexpr unsigned timer_control = 0xffff8f, flags = 0x300000, compare_flag = 0x200000;
constexpr unsigned wrap_load = 0xfffffe, word_mask = 0xffffff;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

DspFetchRunInputs settings() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory,
          .resets = DspFetchResetTiming::reference_cycle_boundary};
}

DspFetchCycleInputs inputs() {
  return {.known_requests = 15, .grant = DspDmaGrant::available};
}

DspHardwareResetInputs reset_pins() {
  return {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2};
}

class Program {
public:
  void emit(const char *source) {
    const auto result = assembler_.assemble(source);
    require(result.success(), "Own lifecycle program must assemble");
    for (unsigned i = 0; i < result.wordCount; ++i) { words_.push_back(result.word[i]); }
  }

  unsigned install(DspBus &bus, const unsigned word) {
    if (word == gpio_clear) {
      emit("movep #$000000,x:<<$ffffbf");
      emit("movep #$000200,x:<<$ffffbe");
      emit("movep #$000200,x:<<$ffffbd");
    }
    for (unsigned i = 0; i < padding; ++i) { emit("nop"); }
    const auto body = boot + static_cast<unsigned>(words_.size());
    words_.push_back(word);
    for (unsigned i = 0; i < padding; ++i) { emit("nop"); }
    bus.install_rom({.start = boot, .words = words_});
    return body;
  }

private:
  dsp56k::Assembler assembler_;
  std::vector<std::uint32_t> words_;
};

void recover_reset(DspMachine &machine, DspFetchRun &run) {
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Actual reset is held for two EXTAL periods");
  run.release_hardware_reset(machine);
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Actual reset recovers for two EXTAL periods");
}

struct Cut {
  DspFetchCheckpoint saved;
  DspDebugState before;
  DspReferenceTicket ticket;
};

Cut until_read(DspMachine &machine, DspFetchRun &run, const unsigned body) {
  for (unsigned i = 0; i < maximum_periods; ++i) {
    const auto cycle = run.advance_cycle(machine, inputs());
    if (!cycle.bus.memory || !cycle.bus.memory->bit_read) { continue; }
    const auto pending = machine.reference_pending();
    require(pending.has_value() && machine.cpu().state().pc == body,
            "E1 peripheral read precedes CPU/CCR retirement at the own opcode");
    if (!pending) { throw Fault("Actual pending native bit ticket is missing"); }
    return {.saved = run.checkpoint(machine),
            .before = machine.cpu().debug_state(),
            .ticket = pending->ticket};
  }
  throw Fault("Own lifecycle program did not reach its real RMW read");
}

void finish(DspMachine &machine, DspFetchRun &run, const Cut &cut) {
  const auto cycle = run.advance_cycle(machine, inputs());
  auto expected = cut.before;
  ++expected.cpu.pc;
  expected.cpu.sr |= 1;
  ++expected.native_instructions;
  expected.native_cycles += 2;
  require(cycle.bus.bit_write && cycle.bus.bit_write->value == 1 && cycle.pipeline &&
              cycle.pipeline->retirement_published && machine.cpu().debug_state() == expected,
          "E2 commits the real captured old bit and exact native CPU/stack/CCR retirement");
}

void timer(const bool asynchronous) {
  DspMachine machine;
  Program program;
  const auto body = program.install(machine.bus(), timer_set);
  DspFetchRun run(settings(), machine, reset_pins());
  recover_reset(machine, run);
  auto &initial_timers = machine.bus().timers();
  initial_timers.write({.address = timer_control - 1, .value = asynchronous ? 0U : wrap_load});
  initial_timers.write({.address = timer_control - 2, .value = word_mask});
  initial_timers.write({.address = timer_control, .value = 1});
  if (!asynchronous) { initial_timers.advance_clk2(3); }
  const auto cut = until_read(machine, run, body);
  const auto sampled_flags = asynchronous ? 0U : flags;
  require(initial_timers.read(timer_control, flags) == sampled_flags,
          "E1 read does not acknowledge actual clock-generated W1C flags");
  for (unsigned replay = 0; replay < 3; ++replay) {
    run.restore(cut.saved, machine);
    auto &timers = machine.bus().timers();
    if (asynchronous) {
      const auto count = timers.read(timer_control - 3);
      timers.write({.address = timer_control - 2, .value = count + 1});
      timers.advance_clk2(1);
      require(timers.read(timer_control, flags) == compare_flag,
              "A real counter edge creates a new compare flag strictly after the E1 read");
    }
    finish(machine, run, cut);
    require(timers.read(timer_control, flags) == (asynchronous ? compare_flag : 0U) &&
                timers.read(timer_control, 1) == 1,
            "E2 writes captured full-word W1C bits without acknowledging a newer status flag");
  }
  DspMachine target;
  DspFetchRun cloned(cut.saved, target);
  finish(target, cloned, cut);
  require(target.bus().timers().read(timer_control, flags) == 0,
          "Distinct native owner restores captured full-word timer writeback");
  std::cout << "PASS timer old-flags=" << sampled_flags
            << " real-post-read-compare=" << asynchronous << " restores=3 clone=1\n";
}

void reset() {
  DspMachine machine;
  Program program;
  const auto body = program.install(machine.bus(), gpio_clear);
  DspFetchRun run(settings(), machine, reset_pins());
  recover_reset(machine, run);
  const auto cut = until_read(machine, run, body);
  const auto old_epoch = run.native_epoch();
  auto &port = machine.bus().gpio(DspPort::c);
  require((port.outputs().levels & pin) == pin, "Actual output remains high after E1 read");
  run.begin_hardware_reset(machine, reset_pins());
  require(run.native_epoch() == old_epoch + 1 && !machine.reference_pending() &&
              machine.cpu().state().pc == boot && (port.outputs().mask & pin) == 0,
          "Real reset clears pending RMW, gates output and returns the CPU to its boot vector");
  recover_reset(machine, run);
  const auto next = until_read(machine, run, body);
  require(next.ticket != cut.ticket && (port.outputs().levels & pin) == pin,
          "Reset does not leak the canceled write into the next native program instance");
  finish(machine, run, next);
  require((port.outputs().levels & pin) == 0,
          "The first post-reset RMW commits only its new actual sampled value");
  std::cout << "PASS reset cancels-read=1 fresh-epoch=1 new-native-RMW=1\n";
}

void fault() {
  DspMachine machine;
  Program program;
  const auto body = program.install(machine.bus(), gpio_clear);
  DspFetchRun run(settings(), machine, reset_pins());
  recover_reset(machine, run);
  const auto cut = until_read(machine, run, body);
  auto &port = machine.bus().gpio(DspPort::c);
  // The real peer agrees with the present high output, but opposes the E2 low write.
  port.set_drive({.mask = pin, .levels = pin});
  const auto cycles = run.completed_cycles();
  const auto ticks = run.elapsed_ticks();
  std::string diagnostic;
  try {
    static_cast<void>(run.advance_cycle(machine, inputs()));
  } catch (const Fault &error) { diagnostic = error.what(); }
  require(diagnostic.contains("pad contention") && diagnostic.contains("X:0xffffbd") &&
              diagnostic.contains("pc=") && diagnostic.contains("omr=") && run.faulted() &&
              machine.cpu().debug_state() == cut.before && run.completed_cycles() == cycles &&
              run.elapsed_ticks() == ticks && (port.outputs().levels & pin) == pin,
          "Failed real E2 write preserves CPU, time and output; reports address/PC/OMR and "
          "quarantines");
  bool refused = false;
  try {
    static_cast<void>(run.advance_cycle(machine, inputs()));
  } catch (const Fault &error) {
    refused = std::string_view(error.what()).contains("healthy inactive bound owner");
  }
  require(refused, "Quarantined scheduler refuses a second write attempt");
  run.restore(cut.saved, machine);
  require(!run.faulted() && machine.cpu().debug_state() == cut.before,
          "A trusted pre-fault checkpoint recovers the pending E1 read");
  finish(machine, run, cut);
  require((machine.bus().gpio(DspPort::c).outputs().levels & pin) == 0,
          "Trusted restore removes the later hostile peer and commits the sampled write once");
  std::cout << "PASS fault E2-contention=1 no-retirement=1 quarantine=1 trusted-recovery=1\n";
}
}

int main(const int argc, const char *const *argv) {
  try {
    require(argc == 2, "Choose timer-old, timer-async, reset or fault");
    const std::string_view requested(argv[1]);
    if (requested == "timer-old") {
      timer(false);
    } else if (requested == "timer-async") {
      timer(true);
    } else if (requested == "reset") {
      reset();
    } else if (requested == "fault") {
      fault();
    } else {
      throw Fault("Unknown RMW lifecycle case");
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL RMW lifecycle: " << error.what() << '\n';
    return 1;
  }
}
