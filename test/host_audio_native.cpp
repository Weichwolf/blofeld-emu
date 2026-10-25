// SPDX-License-Identifier: GPL-3.0-only
#include "audio/serial_pcm.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_norm_fixture.hpp"
#include "host_audio_fixture.hpp"
#include "serial_pcm_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::audio {
namespace {
DspProgram producer(const bool secondary) {
  constexpr unsigned primary_base = 0xffffa0;
  constexpr unsigned secondary_base = 0xffff80;
  const auto base = secondary ? secondary_base : primary_base;
  const std::string space = secondary ? "y:<<" : "x:<<";
  DspProgram result;
  const auto put = [&](const unsigned offset, const unsigned value) {
    result.emit("movep #>" + number(value) + ',' + space + number(base + offset));
  };
  constexpr unsigned control = 21;
  constexpr unsigned clock = 22;
  constexpr unsigned mux = 0x838;
  constexpr unsigned word32 = 0x7d00;
  constexpr unsigned hckt_clock = 0x640301;
  constexpr unsigned first = 0xa5c39f;
  constexpr unsigned next = 0x123456;
  put(control, word32);
  put(clock, hckt_clock);
  constexpr unsigned port = 31;
  constexpr unsigned direction = 30;
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
}

std::vector<SerialPcmFrame> native_frames(const bool secondary) {
  constexpr unsigned extal = 12000000;
  constexpr std::uint64_t start = 5000;
  constexpr std::uint64_t baseline = 3500;
  constexpr std::uint64_t half_period = 125;
  constexpr std::uint64_t observations = (frame_count * 512) + 4;
  constexpr auto timing = DspEsaiClockTiming::reference_reset_edges;
  const auto instance = secondary ? DspEsaiInstance::secondary : DspEsaiInstance::primary;
  DspMachine machine(
      DspRunContract{.clock = {.extal_hz = extal,
                               .timebase_hz = timebase,
                               .switch_timing = DspClockSwitch::reference_instruction_boundary},
                     .timing = DspRunTiming::native_internal_estimates});
  producer(secondary).install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.bus().observe_esai_transmit(instance, {.high_clock = false, .timing = timing});
  machine.cpu().reset_spi_boot();
  DspScheduledRun schedule({.observation = DspStepObservation::reference_completion_boundary},
                           machine);
  SerialPcmReceiver receiver({});
  std::vector<SerialPcmFrame> frames;
  frames.reserve(frame_count);
  std::uint64_t cursor = 0;
  const DspExternalEvents source{
      .next_ticks = [&]() -> std::optional<std::uint64_t> {
        if (cursor == 0) { return baseline; }
        if (cursor == observations + 1) { return std::nullopt; }
        return start + ((cursor - 1) * half_period);
      },
      .observe =
          [&](const std::uint64_t at) {
            if (cursor != 0) {
              machine.bus().observe_esai_transmit(
                  instance, {.high_clock = cursor % 2 != 0, .timing = timing});
            }
            const auto clock = machine.bus().esai_clock_outputs(instance);
            const auto data = machine.bus().esai_transmit_outputs(instance);
            pcm::require(clock.driven == 3 && clock.known == 3,
                         "Native program must drive known ESAI clock and data pads");
            const auto frame =
                receiver.observe(at,
                                 {.clock = (clock.levels & 1U) != 0,
                                  .frame_sync = (clock.levels & 2U) != 0,
                                  .data = (data.driven & data.known & 1U) == 0
                                              ? std::nullopt
                                              : std::optional{(data.levels & 1U) != 0}});
            if (frame) { frames.push_back(*frame); }
            ++cursor;
          },
      .order = DspExternalOrder::external_before_dsp,
      .maximum_events = observations + 1};
  constexpr unsigned budget = 100000;
  const auto end = start + ((observations - 1) * half_period);
  schedule.advance_to(machine, end, DspScheduleBoundary::through, budget, {}, {}, source);
  pcm::require(frames.size() == frame_count, "Native ESAI must transmit every bounded test frame");
  for (std::size_t index = 0; index < frames.size(); ++index) {
    pcm::require(frames[index].samples[0] == (index == 0 ? first_word : next_word) &&
                     frames[index].samples[1] == next_word,
                 "Native serial words must match independent literal expectations");
  }
  return frames;
}
}
