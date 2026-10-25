// SPDX-License-Identifier: GPL-3.0-only
#include "audio/serial_pcm.hpp"
#include "dax_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp_audio_dma_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>

namespace blofeld::test::audio_dma {
void Experiment::pcm_frame(const SerialPcmFrame &frame) {
  const auto index = driver.decoded++;
  require(index < frames && frame.channels == 2 && frame.uniform_clock &&
              frame.bit_ticks == bit_ticks &&
              frame.first_tick == first_sample + (index * frame_ticks) &&
              frame.last_tick ==
                  first_sample + (index * frame_ticks) + ((frame_bits - 1) * bit_ticks),
          "Every complete PCM frame has independent literal phase/slot/bit timing");
  const auto left = index == 0 ? 0U : expected_word(0, (index * 2) - 1, driver.changed);
  const auto right = expected_word(0, index * 2, driver.changed);
  require(frame.samples[0] == signed_word(left) && frame.samples[1] == signed_word(right),
          "Actual ESAI bits retain the priming word and the independently predicted ring order");
  driver.trace += "PCM " + std::to_string(index) + ' ' + std::to_string(frame.samples[0]) + ' ' +
                  std::to_string(frame.samples[1]) + '\n';
}

void Experiment::step() {
  require(driver.tick < final_tick, "Explicit dual-output reference edge bound exceeded");
  ++driver.tick;
  const bool high = (driver.tick & 1U) != 0;
  machine.bus().observe_esai_transmit(
      DspEsaiInstance::secondary,
      {.high_clock = high, .timing = DspEsaiClockTiming::reference_reset_edges});
  machine.observe_dax({.core_clock = false,
                       .aci = high,
                       .rates = {.core_hz = reference_core_hz, .aci_hz = reference_aci_hz},
                       .timing = DspDaxTiming::reference_rising_edges});
  const auto clocks = machine.bus().esai_clock_outputs(DspEsaiInstance::secondary);
  const auto data = machine.bus().esai_transmit_outputs(DspEsaiInstance::secondary);
  require(clocks.driven == 3 && clocks.known == 3,
          "Both actual generated ESAI clocks remain known");
  const auto frame = driver.pcm.observe(driver.tick,
                                        {.clock = (clocks.levels & 1U) != 0,
                                         .frame_sync = (clocks.levels & 2U) != 0,
                                         .data = (data.driven & data.known & 1U) != 0
                                                     ? std::optional{(data.levels & 1U) != 0}
                                                     : std::nullopt});
  if (frame) { pcm_frame(*frame); }
  if (driver.tick % 4 == 3) {
    const auto output = machine.bus().dax_output();
    require(output.driven, "The actual DAX ADO pin must drive every predicted biphase unit");
    driver.bmc += output.level ? '1' : '0';
  }
  grant();
}

void Experiment::dax_frames() const {
  constexpr unsigned units_per_frame = 128;
  constexpr unsigned units_per_subframe = 64;
  require(driver.bmc.size() == frames * units_per_frame + 1,
          "DAX has66 complete frames and one explicitly retained next-frame preamble unit");
  const std::string_view bits(driver.bmc);
  for (unsigned frame = 0; frame < frames; ++frame) {
    const auto offset = static_cast<std::size_t>(frame) * units_per_frame;
    const auto left =
        dax_test::decode(bits.substr(offset, units_per_subframe), frame == 0 ? 'Z' : 'X');
    const auto right =
        dax_test::decode(bits.substr(offset + units_per_subframe, units_per_subframe), 'Y');
    const auto a = frame == 0 ? 0U : expected_word(3, (frame * 2) - 2, driver.changed);
    const auto b = frame == 0 ? 0U : expected_word(3, (frame * 2) - 1, driver.changed);
    require(left.audio == a && right.audio == b && left.flags == 0 && right.flags == 0,
            "Independent BMC preamble/boundary/parity decoder retains actual DAX priming and ring");
  }
}

void Experiment::verify() {
  require(driver.tick == final_tick && driver.cycle == final_tick + 1 &&
              driver.words == std::array{word_count, word_count} && driver.decoded == frames &&
              machine.cpu().debug_state() == paused_,
          "Both repeating channels drain while the native CPU and its real launch frame pause");
  for (const unsigned first : {source0, source3}) {
    require(machine.bus().read({.space = DspSpace::x, .address = first}) ==
                    ring_start + (word_count % ring_size) &&
                machine.bus().read({.space = DspSpace::x, .address = first - 2}) ==
                    (ring_size - 1 - word_count % ring_size) * ring_size &&
                machine.bus().read({.space = DspSpace::x, .address = first - 3}) ==
                    (first == source0 ? control0 : control3),
            "Original DTM5/D3D counters reload and source rings wrap twice without clearing DE");
  }
  for (unsigned i = 0; i < ring_size; ++i) {
    require(machine.bus().read({.space = DspSpace::x, .address = ring_start + i}) ==
                (driver.changed && i == 0 ? replacement : sample(i)),
            "DMA cannot rewrite its source ring");
  }
  for (const auto address : {ring_start - 1, ring_start + ring_size}) {
    require(machine.bus().read({.space = DspSpace::x, .address = address}) == guard,
            "Both source-neighbor guards survive every original ring reload");
  }
  require((machine.bus().read({.space = DspSpace::y, .address = esai_status}) & esai_underrun) == 0,
          "Every actual ESAI refill precedes its next slot; underruns cannot be hidden");
  require(dax_test::rejects([&] {
            static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = dma_status}));
          }).contains("repeating DTD lifetime"),
          "Unqualified repeating DTD status remains a visible failure");
  dax_frames();
  intact();
}

void Experiment::finish() {
  while (driver.tick < final_tick) { step(); }
  verify();
}

void Experiment::emit(std::ostream &output, const std::string &name) const {
  output << "CASE " << name << '\n' << driver.trace << "BMC " << driver.bmc << '\n';
  output << "RESULT " << driver.words[0] << ' ' << driver.words[1] << ' ' << driver.decoded << ' '
         << driver.bmc.size() << ' ' << driver.tick << '\n';
}
} // namespace blofeld::test::audio_dma

int main(const int count, char **arguments) {
  try {
    using namespace blofeld::test::audio_dma;
    blofeld::test::require(count == 3, "Expected original firmware archive and private trace");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    Experiment run(firmware);
    const auto initial = run.machine.checkpoint();
    const auto initial_driver = run.driver;
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    run.finish();
    run.emit(trace, "baseline");
    run.machine.restore(initial);
    run.driver = initial_driver;
    while (run.driver.tick < captured_tick) { run.step(); }
    const auto held = run.machine.checkpoint();
    const auto held_driver = run.driver;
    std::optional<Driver> expected;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.machine.restore(held);
      run.driver = held_driver;
      run.machine.bus().write(
          {.space = blofeld::DspSpace::x, .address = ring_start, .value = replacement});
      run.driver.changed = true;
      run.finish();
      if (expected) {
        blofeld::test::require(run.driver == *expected, "Trusted held-word suffix replay differs");
      }
      expected = run.driver;
      run.emit(trace, "changed" + std::to_string(replay));
    }
    std::cout << "Original dual repeating DMA controls, real ESAI/DAX streams and held-word replay "
                 "verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
