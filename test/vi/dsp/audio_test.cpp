// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/audio.hpp"
#include "dsp/dax.hpp"
#include "dsp/dma.hpp"
#include "dsp/esai.hpp"
#include "io/pcm_output.hpp"

#include <array>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <stop_token>
#include <vector>

namespace {
using namespace blofeld::vi;
using namespace blofeld::vi::dsp;
using dsp56300::Word;
constexpr Word tx0 = 0xffff80, control = 0xffff95, clock_control = 0xffff96;
constexpr Word format = 0x7d01, clock_format = 0x640301, first_word = 0x123456;
constexpr unsigned nominal_pll = 18;
constexpr std::uint64_t configure_time = 100;
constexpr unsigned nominal_slot = 2304, rate_reference = 11289600, frame_ticks = 256;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

struct Consumer {
  std::vector<PcmOutput::Frame> frames;
  std::vector<PcmOutput::Block> metadata;
  bool accepted = true;

  PcmOutput output() {
    return {.owner = this,
            .write = [](void *owner, const PcmOutput::Block block, const std::stop_token &) {
              auto &self = *static_cast<Consumer *>(owner);
              if (!self.accepted) { return false; }
              self.frames.insert(self.frames.end(), block.frames.begin(), block.frames.end());
              auto record = block;
              record.frames = {}; // Borrowed spans never survive synchronous submission.
              self.metadata.push_back(record);
              return true;
            }};
  }
};

void configure(Esai &esai) {
  require(esai.write({.address = clock_control, .value = clock_format}) &&
              esai.write({.address = control, .value = format}),
          "Configure stereo ESAI words");
  esai.connected(true);
}

void memory_to_pcm() {
  Esai esai;
  Dax dax;
  Dma dma;
  Consumer consumer;
  const std::stop_token stop;
  Audio audio(esai, dax, consumer.output(), stop);
  configure(esai);
  audio.reset(7);
  require(audio.configure(0, {.numerator = nominal_pll}),
          "Bind external audio to the selected PLL ratio");
  constexpr Word base = 0x660;
  std::array<Word, 64> words{};
  constexpr Word negative = 0x800000;
  for (unsigned index = 0; index < words.size(); ++index) {
    words[index] = ((index + 1) << 16U) | ((index & 1) != 0 ? negative : 0);
  }
  constexpr std::array configuration{Dma::Write{.address = 0xfffff3, .value = 1},
                                     Dma::Write{.address = 0xfffff2, .value = 0xffffc1},
                                     Dma::Write{.address = 0xffffef, .value = base},
                                     Dma::Write{.address = 0xffffee, .value = tx0},
                                     Dma::Write{.address = 0xffffed, .value = 0xfc0},
                                     Dma::Write{.address = 0xffffec, .value = 0xaab604}};
  for (const auto item : configuration) { require(dma.write(item), "Configure actual DMA state"); }
  require(esai.write({.address = tx0, .value = words.back()}),
          "Prime the word preceding the DMA ring");
  constexpr unsigned request = 22;
  for (unsigned index = 0; index < words.size(); ++index) {
    require(audio.deadline() == (std::uint64_t{index + 1} * nominal_slot) && audio.clock() &&
                esai.request() && esai.request() && dma.trigger(request),
            "Each real slot retains its empty request until a DMA word is accepted");
    require(dma.service(
                0,
                [&](const Dma::Address source, Word &value) {
                  if (source.space != dsp56300::Space::x || source.address < base ||
                      source.address - base >= words.size()) {
                    return false;
                  }
                  value = words[source.address - base];
                  return true;
                },
                [&](const Dma::Address target, const Word value) {
                  return target.space == dsp56300::Space::y && target.address == tx0 &&
                         esai.write({.address = tx0, .value = value});
                }) &&
                !esai.request() && audio.submit(),
            "Real DMA words refill the register before the next event and complete PCM blocks");
  }
  constexpr Word source_register = 0xffffef;
  Word source = 0;
  require(dma.read(source_register, source) && source == base,
          "A complete ring traversal restores the exact source address");
  require(consumer.frames.size() == 32 && consumer.metadata.size() == 2,
          "Sixty-four transferred words publish exactly thirty-two stereo frames");
  constexpr std::int32_t magnitude = 65536;
  constexpr std::int32_t sign = 8388608;
  for (unsigned index = 0; index < consumer.frames.size(); ++index) {
    const auto left_index = index == 0 ? 64 : index * 2;
    const auto left = (static_cast<std::int32_t>(left_index) * magnitude) - sign;
    const auto right = static_cast<std::int32_t>((index * 2) + 1) * magnitude;
    require(consumer.frames[index] == PcmOutput::Frame{left, right},
            "Each signed PCM channel equals its independently indexed source word");
  }
  require(consumer.metadata[0].epoch == 7 && consumer.metadata[0].first_frame == 0 &&
              consumer.metadata[1].first_frame == 16 && consumer.metadata[0].underruns == 0 &&
              consumer.metadata[0].reference_hz == rate_reference &&
              consumer.metadata[0].frame_ticks == frame_ticks,
          "The consumer sees real positions, no underruns and the exact nominal 44100 Hz ratio");
}

void fractional_clock_and_underrun() {
  Esai esai;
  Dax dax;
  Consumer consumer;
  const std::stop_token stop;
  Audio audio(esai, dax, consumer.output(), stop);
  configure(esai);
  require(esai.write({.address = tx0, .value = first_word}) &&
              audio.configure(0, {.numerator = 7, .denominator = 3}),
          "Configure a fractional clock");
  constexpr std::array<std::uint64_t, 4> deadlines{299, 598, 896, 1195};
  for (const auto deadline : deadlines) {
    require(audio.deadline() == deadline && audio.clock() && audio.submit(),
            "Fractional slot periods retain oscillator phase without per-slot rounding drift");
  }
  require(consumer.frames.empty() && audio.submit(true) && consumer.frames.size() == 2 &&
              consumer.frames[0] == PcmOutput::Frame{first_word, first_word} &&
              consumer.frames[1] == consumer.frames[0] && consumer.metadata[0].underruns == 3,
          "Only complete frames flush, with actual retained-data underruns marked explicitly");
  consumer.accepted = false;
  require(audio.clock() && audio.clock() && !audio.submit(true) && audio.error() != nullptr &&
              consumer.frames.size() == 2,
          "A rejected output cannot become accepted or counted PCM");
}

void digital_clock_and_unknown_data() {
  Esai esai;
  Dax dax;
  Consumer consumer;
  const std::stop_token stop;
  Audio audio(esai, dax, consumer.output(), stop);
  dax.connected(true);
  constexpr Word dax_audio = 0xffffd2;
  require(dax.write({.address = dax_audio, .value = first_word}) &&
              dax.write({.address = dax_audio, .value = first_word}) &&
              audio.configure(configure_time, {.numerator = nominal_pll}) &&
              audio.deadline() == 512 && audio.clock() && audio.deadline() == 1024 && dax.request(),
          "DAX internal clock uses Fosc/1024 independently of the external audio clock");
  require(audio.submit(true) && consumer.frames.empty(),
          "DAX never duplicates the host PCM stream");
  configure(esai);
  constexpr std::array<std::uint64_t, 3> digital_deadlines{1024, 1536, 2048};
  require(audio.configure(1024, {.numerator = nominal_pll}),
          "Enable the unprimed ESAI transmitter");
  for (const auto deadline : digital_deadlines) {
    require(audio.deadline() == deadline && audio.clock(),
            "Only due DAX events advance before ESAI");
  }
  require(audio.deadline() == nominal_slot && !audio.clock() && audio.error() != nullptr &&
              consumer.frames.empty(),
          "Unprimed ESAI words fail when their real transmit deadline is reached");
}
}

int main() try {
  memory_to_pcm();
  fractional_clock_and_underrun();
  digital_clock_and_unknown_data();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
