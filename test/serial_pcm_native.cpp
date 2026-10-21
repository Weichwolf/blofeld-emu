// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_esai.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/serial_pcm.hpp"
#include "dsp_norm_fixture.hpp"
#include "serial_pcm_fixture.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::pcm {
namespace {
constexpr std::uint64_t start = 2000;
constexpr std::uint64_t half_period = 23;
constexpr std::uint64_t observations = 516;
constexpr std::uint64_t end = start + ((observations - 1) * half_period);
constexpr std::uint64_t baseline_tick = 600;
constexpr std::uint64_t capture = 6000;
constexpr auto timing = DspEsaiClockTiming::reference_reset_edges;

DspProgram program(const bool second) {
  constexpr unsigned primary = 0xffffa0;
  constexpr unsigned secondary = 0xffff80;
  const auto base = second ? secondary : primary;
  const std::string space = second ? "y:<<" : "x:<<";
  DspProgram result;
  const auto put = [&](const unsigned offset, const unsigned value) {
    result.emit("movep #>" + number(value) + ',' + space + number(base + offset));
  };
  constexpr unsigned control = 21;
  constexpr unsigned clock = 22;
  constexpr unsigned port = 31;
  constexpr unsigned direction = 30;
  constexpr unsigned mux = 0x838;
  constexpr unsigned word32 = 0x7d00;
  constexpr unsigned hckt_clock = 0x640301;
  constexpr unsigned first = 0xa5c39f;
  constexpr unsigned next = 0x123456;
  put(control, word32);
  put(clock, hckt_clock);
  put(port, mux);
  put(direction, mux);
  put(0, first);
  put(control, word32 | 1U);
  constexpr unsigned delay = 64;
  for (unsigned i = 0; i < delay; ++i) { result.emit("nop"); }
  put(0, next);
  const auto loop = result.pc();
  result.emit("nop");
  constexpr unsigned jump = 0x0af080;
  result.word(jump);
  result.word(loop);
  return result;
}

struct NativeWire {
  SerialPcmReceiver receiver{{}};
  std::uint64_t cursor = 0;
  std::vector<SerialPcmFrame> frames;

  DspExternalEvents source(DspMachine &machine, const DspEsaiInstance instance) {
    return {.next_ticks = [&]() -> std::optional<std::uint64_t> {
              if (cursor == 0) { return baseline_tick; }
              if (cursor == observations + 1) { return std::nullopt; }
              return start + ((cursor - 1) * half_period);
            },
            .observe =
                [&, instance](const std::uint64_t at) {
                  const auto before = machine.cpu().debug_state();
                  if (cursor != 0) {
                    machine.bus().observe_esai_transmit(
                        instance, {.high_clock = cursor % 2 != 0, .timing = timing});
                  }
                  const auto clock = machine.bus().esai_clock_outputs(instance);
                  const auto data = machine.bus().esai_transmit_outputs(instance);
                  require(clock.driven == 3 && clock.known == 3,
                          "Native MOVEP setup must actually drive both known output clocks");
                  const auto bit = (data.driven & data.known & 1U) == 0
                                       ? std::nullopt
                                       : std::optional{(data.levels & 1U) != 0};
                  const auto frame = receiver.observe(at,
                                                      {.clock = (clock.levels & 1U) != 0,
                                                       .frame_sync = (clock.levels & 2U) != 0,
                                                       .data = bit});
                  if (frame) { frames.push_back(*frame); }
                  require(machine.cpu().debug_state() == before,
                          "Observing actual serial outputs must not execute or change the CPU");
                  ++cursor;
                },
            .order = DspExternalOrder::external_before_dsp,
            .maximum_events = 1024};
  }
};

void native_case(const bool second) {
  constexpr unsigned extal = 12000000;
  constexpr std::uint64_t common = 540000000;
  constexpr unsigned budget = 1024;
  constexpr auto through = DspScheduleBoundary::through;
  const auto instance = second ? DspEsaiInstance::secondary : DspEsaiInstance::primary;
  DspMachine machine(
      DspRunContract{.clock = {.extal_hz = extal,
                               .timebase_hz = common,
                               .switch_timing = DspClockSwitch::reference_instruction_boundary},
                     .timing = DspRunTiming::native_internal_estimates});
  program(second).install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.bus().observe_esai_transmit(instance, {.high_clock = false, .timing = timing});
  machine.cpu().reset_spi_boot();
  DspScheduledRun schedule({.observation = DspStepObservation::reference_completion_boundary},
                           machine);
  NativeWire wire;
  const auto source = wire.source(machine, instance);
  schedule.advance_to(machine, capture, through, budget, {}, {}, source);
  require(wire.frames.empty() && wire.receiver.locked() && wire.cursor > 1,
          "The joint snapshot is inside an actually transmitted native word");
  const auto cpu = machine.checkpoint();
  const auto saved_schedule = schedule.checkpoint(machine);
  const auto saved_wire = wire;
  std::optional<DspDebugState> completed;
  std::vector<SerialPcmFrame> expected;
  for (unsigned replay = 0; replay < 3; ++replay) {
    machine.restore(cpu);
    schedule.restore(saved_schedule, machine);
    wire = saved_wire;
    schedule.advance_to(machine, end, through, budget, {}, {}, source);
    // Twelve setup units, 64 NOPs, two refill units, then a 1+4-unit loop.
    // At 13845 ticks the last completed prefix is 304*45; JMP ends at 308*45.
    constexpr unsigned completed_units = 304;
    constexpr std::uint64_t completed_ticks = 13680;
    constexpr std::uint64_t next_completion = 13860;
    require(machine.run_clock().native_cycle_units() == completed_units &&
                machine.run_clock().elapsed_ticks() == completed_ticks &&
                schedule.next_completion_ticks(machine) == next_completion,
            "The receiver cannot charge native cycles or finish the pending JMP");
    constexpr std::int32_t signed_first = -5913697;
    constexpr std::int32_t signed_next = 1193046;
    require(wire.cursor == observations + 1 && wire.frames.size() == 1 &&
                wire.receiver.completed_frames() == 1,
            "Exactly one frame comes from the actual finite native ESAI clock source");
    const auto &frame = wire.frames.front();
    constexpr std::uint64_t first_sample = 2230;
    constexpr std::uint64_t last_sample = 13822;
    require(frame.channels == 2 && frame.samples[0] == signed_first &&
                frame.samples[1] == signed_next && frame.first_tick == first_sample &&
                frame.last_tick == last_sample,
            "The actual native shifter retains its first word and refills the next slot");
    if (!completed) {
      completed = machine.cpu().debug_state();
      expected = wire.frames;
    }
    require(machine.cpu().debug_state() == *completed && wire.frames == expected,
            "Joint native CPU/divider/source/receiver copies reproduce exact PCM and time");
  }
}
}

void native_serial_pcm() {
  native_case(false);
  native_case(true);
}
}
