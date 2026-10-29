// SPDX-License-Identifier: GPL-3.0-only
#include "board/reference_square_clock.hpp"
#include "dax_fixture.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_esai.hpp"
#include "esai_fixture.hpp"
#include "reference_audio_clock_fixture.hpp"

#include <array>
#include <cstddef>
#include <iostream>
#include <string>
#include <string_view>

namespace blofeld::test::square_clock {
using esai::require;

AudioRig::AudioRig(const reference::SquareClockConfig config) : source(config) {
  initialize_endpoints();
}

void AudioRig::initialize_endpoints() {
  observe(source.current().level); // Establish both input levels without an extra edge.
  esai.write(esai::clock, transmit_clock);
  esai.write(esai::control, transmit_format);
  esai.write(0, dax_test::first_sample);
  esai.write(esai::control, transmit_format | 1U);
  esai.set_active(true);
  dax.write(dax_test::control, external_256);
  dax.write(dax_test::non_audio, 0);
  dax.write(dax_test::audio_a, dax_test::first_sample);
  dax.write(dax_test::audio_a, dax_test::other_sample);
  dax.set_active(true);
  last_sck = false;
}

void AudioRig::observe(const bool level) {
  esai.observe({.high_clock = level, .timing = DspEsaiClockTiming::reference_reset_edges});
  dax.observe({.aci = level,
               .rates = {.core_hz = core_hz, .aci_hz = source.config().frequency_hz},
               .timing = DspDaxTiming::reference_rising_edges});
}

void AudioRig::tick() {
  const auto edge = source.next();
  const auto units = dax.half_bits();
  observe(edge.level);
  source.consume(edge.ticks);
  if (dax.half_bits() != units) { bmc.push_back(dax.output().level ? '1' : '0'); }
  const auto clocks = esai.clock_outputs();
  const bool sck = (clocks.levels & 1U) != 0;
  if (sck != last_sck && !sck) { // Original TCKP=1 launches on falling SCKT.
    pcm.push_back(esai::sample(esai.outputs(), 0));
    constexpr unsigned slot_bits = 32;
    if (pcm.size() % slot_bits == 1) {
      constexpr unsigned frame_bits = 64;
      esai.write(0, pcm.size() % frame_bits == 1 ? dax_test::other_sample : dax_test::first_sample);
    }
  }
  last_sck = sck;
}

void AudioRig::reset_endpoints() {
  esai.reset();
  dax.reset();
  pcm.clear();
  bmc.clear();
  initialize_endpoints();
}

namespace {
constexpr std::array cases{
    reference::SquareClockConfig{.timebase_hz = 211680000000, .frequency_hz = 11289600},
    reference::SquareClockConfig{.timebase_hz = 138240000000, .frequency_hz = 12288000}};
constexpr unsigned edges_per_frame = 512;
constexpr std::string_view first_slot = "10000000000000000000000111111111";
constexpr std::string_view second_slot = "00010010001101000101011000000000";

void frame_payloads(const AudioRig &rig) {
  const auto frame = std::string(first_slot) + std::string(second_slot);
  require(rig.pcm == frame + frame, "Independent literal ESAI 24-bit words and eight padding bits");
  constexpr std::size_t subframe = 64;
  require(rig.bmc.size() == subframe * 4, "Two frames emit exactly 256 real DAX BMC units");
  constexpr std::array preambles{'Z', 'Y', 'X', 'Y'};
  for (std::size_t i = 0; i < preambles.size(); ++i) {
    const char preamble = preambles.at(i);
    require(
        dax_test::decode(std::string_view(rig.bmc).substr(i * subframe, subframe), preamble) ==
            dax_test::Decoded{.audio = i % 2 == 0 ? dax_test::first_sample : dax_test::other_sample,
                              .flags = 0},
        "Independent BMC receiver sees both actual primed samples, parity and block preambles");
  }
}
}

void serial_checks() {
  for (const auto config : cases) {
    AudioRig rig(config);
    for (unsigned edge = 1; edge <= edges_per_frame * 2; ++edge) {
      rig.tick();
      // First selected rising HCKT is edge1; ESAI launches every fourth rising edge.
      require(rig.pcm.size() == (edge + 1) / 8 && rig.bmc.size() == (edge + 1) / 4,
              "Independent absolute edge counts pin both different divider paths");
    }
    frame_payloads(rig);
    require(rig.source.current().ticks ==
                    config.timebase_hz / config.frequency_hz * edges_per_frame &&
                rig.esai.frames_started() == 2 && rig.esai.slots_started() == 4,
            "Two frames preserve the exact source time and real peripheral frame counters");
  }
  std::cout << "2 conditional rates;2048 shared source edges;256 ESAI bits;512 DAX units\n";
}

void snapshot_checks() {
  constexpr unsigned cuts = edges_per_frame;
  constexpr unsigned suffix = edges_per_frame * 2;
  for (const auto config : cases) {
    AudioRig prefix(config);
    for (unsigned cut = 0; cut < cuts; ++cut) {
      const auto saved = prefix;
      auto first = saved;
      auto second = saved;
      for (unsigned edge = 0; edge < suffix; ++edge) {
        first.tick();
        second.tick();
        const auto absolute_edge = cut + edge + 1;
        require(first.source.current().ticks == absolute_edge * first.source.half_period_ticks() &&
                    first.source.current().level == (absolute_edge % 2 != 0) &&
                    first.pcm.size() == (absolute_edge + 1) / 8 &&
                    first.bmc.size() == (absolute_edge + 1) / 4,
                "Every restored edge also matches independent absolute time and divider counts");
        require(first.source == second.source && first.pcm == second.pcm &&
                    first.bmc == second.bmc && first.esai.outputs() == second.esai.outputs() &&
                    first.esai.clock_outputs() == second.esai.clock_outputs() &&
                    first.dax.output() == second.dax.output() &&
                    first.dax.half_bits() == second.dax.half_bits(),
                "Copied source and both peripheral phases reproduce every suffix edge");
      }
      auto reset = saved;
      const auto before = reset.source;
      reset.reset_endpoints();
      require(reset.source == before && reset.source.next() == prefix.source.next(),
              "Guest endpoint resets never reset or rephase the external oscillator");
      for (unsigned edge = 0; edge < suffix; ++edge) { reset.tick(); }
      require(reset.pcm.size() == suffix / 8 && reset.bmc.size() == suffix / 4,
              "Reprimed endpoints use only subsequent real edges at every initial phase");
      frame_payloads(reset);
      prefix.tick();
    }
  }
  std::cout << "1024 joint cuts;2048 copied suffixes;1024 endpoint resets;3145728 suffix edges\n";
}
}
