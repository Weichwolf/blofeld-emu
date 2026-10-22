// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_esai.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/serial_pcm.hpp"
#include "dsp_norm_fixture.hpp"
#include "host_audio_fixture.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test {
namespace {
constexpr std::uint64_t common = 3072000000;
constexpr std::uint64_t start = 20000;
constexpr std::uint64_t half_period = 125;
constexpr std::uint64_t observations = 516;
constexpr std::uint64_t end = start + ((observations - 1) * half_period);

DspProgram program(const bool secondary) {
  constexpr unsigned primary_base = 0xffffa0;
  constexpr unsigned secondary_base = 0xffff80;
  const auto base = secondary ? secondary_base : primary_base;
  const std::string space = secondary ? "y:<<" : "x:<<";
  DspProgram result;
  const auto put = [&](const unsigned offset, const unsigned value) {
    result.emit("movep #>" + number(value) + ',' + space + number(base + offset));
  };
  constexpr unsigned transmit_control = 21;
  constexpr unsigned clock_control = 22;
  constexpr unsigned mux_control = 31;
  constexpr unsigned direction = 30;
  constexpr unsigned serial_pads = 0x838;
  constexpr unsigned word32 = 0x7d00;
  constexpr unsigned external_clock = 0x640301;
  constexpr unsigned first_word = 0xa5c39f;
  constexpr unsigned second_word = 0x123456;
  put(transmit_control, word32);
  put(clock_control, external_clock);
  put(mux_control, serial_pads);
  put(direction, serial_pads);
  put(0, first_word);
  put(transmit_control, word32 | 1U);
  // Refill after the first slot has loaded, before the second slot begins.
  constexpr unsigned refill_delay = 96;
  for (unsigned i = 0; i < refill_delay; ++i) { result.emit("nop"); }
  put(0, second_word);
  const auto loop = result.pc();
  result.emit("nop");
  constexpr unsigned jump = 0x0af080;
  result.word(jump);
  result.word(loop);
  return result;
}
}

std::vector<std::uint32_t> host_audio_native_program(const bool secondary) {
  DspBus bus;
  const auto code = program(secondary);
  code.install(bus);
  std::vector<std::uint32_t> words;
  for (unsigned address = boot; address < code.pc(); ++address) {
    words.push_back(bus.read({.space = DspSpace::p, .address = address}));
  }
  return words;
}

SerialPcmFrame host_audio_native_frame(const bool secondary) {
  constexpr unsigned extal = 12000000;
  DspMachine machine(
      DspRunContract{.clock = {.extal_hz = extal,
                               .timebase_hz = common,
                               .switch_timing = DspClockSwitch::reference_instruction_boundary},
                     .timing = DspRunTiming::native_internal_estimates});
  program(secondary).install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  const auto instance = secondary ? DspEsaiInstance::secondary : DspEsaiInstance::primary;
  constexpr auto timing = DspEsaiClockTiming::reference_reset_edges;
  machine.bus().observe_esai_transmit(instance, {.high_clock = false, .timing = timing});
  machine.cpu().reset_spi_boot();
  DspScheduledRun schedule({.observation = DspStepObservation::reference_completion_boundary},
                           machine);
  SerialPcmReceiver receiver({});
  std::optional<SerialPcmFrame> completed;
  std::uint64_t cursor = 0;
  constexpr unsigned baseline = 10000;
  const DspExternalEvents events{
      .next_ticks = [&]() -> std::optional<std::uint64_t> {
        if (cursor == 0) { return baseline; }
        if (cursor > observations) { return std::nullopt; }
        return start + ((cursor - 1) * half_period);
      },
      .observe =
          [&](const std::uint64_t at) {
            if (cursor != 0) {
              machine.bus().observe_esai_transmit(
                  instance, {.high_clock = cursor % 2 != 0, .timing = timing});
            }
            const auto clocks = machine.bus().esai_clock_outputs(instance);
            const auto data = machine.bus().esai_transmit_outputs(instance);
            if (clocks.driven != 3 || clocks.known != 3) {
              throw Fault("Native DSP must drive both actual ESAI clock outputs");
            }
            const auto frame =
                receiver.observe(at,
                                 {.clock = (clocks.levels & 1U) != 0,
                                  .frame_sync = (clocks.levels & 2U) != 0,
                                  .data = (data.driven & data.known & 1U) == 0
                                              ? std::nullopt
                                              : std::optional{(data.levels & 1U) != 0}});
            if (frame) { completed = frame; }
            ++cursor;
          },
      .order = DspExternalOrder::external_before_dsp,
      .maximum_events = 1024};
  constexpr unsigned instruction_budget = 1024;
  schedule.advance_to(
      machine, end, DspScheduleBoundary::through, instruction_budget, {}, {}, events);
  constexpr std::int32_t left = -5913697;
  constexpr std::int32_t right = 1193046;
  constexpr unsigned bit_ticks = 1000;
  if (!completed || receiver.completed_frames() != 1 || completed->channels != 2 ||
      completed->samples[0] != left || completed->samples[1] != right ||
      completed->bit_ticks != bit_ticks || !completed->uniform_clock) {
    throw Fault("Native MOVEP/ESAI/serial receiver did not produce the independent stereo pair");
  }
  return *completed;
}
}
