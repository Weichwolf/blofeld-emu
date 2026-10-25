// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "samples/sample_library.hpp"
#include "samples/sample_wave.hpp"
#include "samples/sample_wave_detail.hpp"

#include <cstddef>
#include <cstdint>

namespace blofeld::samples::wave_detail {
namespace {
int signed_byte(const std::uint8_t byte) {
  constexpr unsigned sign = 0x80;
  return byte < sign ? byte : static_cast<int>(byte) - 256;
}

WaveInstrument instrument_metadata(const Bytes bytes) {
  if (bytes.size() != 7) { throw Fault("Invalid WAV inst chunk length"); }
  const WaveInstrument result{.note = bytes[0],
                              .correction_cents = signed_byte(bytes[1]),
                              .gain_db = signed_byte(bytes[2]),
                              .lowest_key = bytes[3],
                              .highest_key = bytes[4],
                              .lowest_velocity = bytes[5],
                              .highest_velocity = bytes[6]};
  constexpr int maximum_correction = 50;
  if (result.note > last_midi_key || result.correction_cents < -maximum_correction ||
      result.correction_cents > maximum_correction || result.lowest_key > result.highest_key ||
      result.highest_key > last_midi_key || result.lowest_velocity > result.highest_velocity ||
      result.highest_velocity > last_midi_key) {
    throw Fault("Invalid WAV instrument pitch/key/velocity metadata");
  }
  return result;
}

void sampler_metadata(SampleWave &wave, const Bytes bytes) {
  constexpr unsigned header_bytes = 36;
  constexpr unsigned loop_bytes = 24;
  constexpr unsigned loop_count_offset = 28;
  constexpr unsigned extra_bytes_offset = 32;
  constexpr unsigned maximum_loops = 1024;
  if (bytes.size() < header_bytes) { throw Fault("Truncated WAV smpl header"); }
  const auto count = little(bytes, loop_count_offset, 4);
  const auto extra = little(bytes, extra_bytes_offset, 4);
  if (count > maximum_loops || header_bytes + count * loop_bytes + extra != bytes.size()) {
    throw Fault("Invalid or excessive WAV sampler loop/data extent");
  }
  const auto note = little(bytes, 12, 4);
  if (note > last_midi_key) { throw Fault("Invalid WAV sampler root key"); }
  constexpr double fraction_scale = 4294967296.0;
  wave.sampler_root =
      static_cast<double>(note) + (static_cast<double>(little(bytes, 16, 4)) / fraction_scale);
  for (std::size_t i = 0; i < count; ++i) {
    const auto record = bytes.subspan(header_bytes + (i * loop_bytes), loop_bytes);
    const WaveLoop loop{.identifier = static_cast<std::uint32_t>(little(record, 0, 4)),
                        .type = static_cast<std::uint32_t>(little(record, 4, 4)),
                        .start = static_cast<std::uint32_t>(little(record, 8, 4)),
                        .end_inclusive = static_cast<std::uint32_t>(little(record, 12, 4)),
                        .fraction = static_cast<std::uint32_t>(little(record, 16, 4)),
                        .play_count = static_cast<std::uint32_t>(little(record, 20, 4))};
    if (loop.type > 2 || loop.start > loop.end_inclusive ||
        loop.end_inclusive >= wave.mono.size()) {
      throw Fault("Unsupported WAV loop type or out-of-range inclusive loop endpoint");
    }
    wave.loops.push_back(loop);
  }
}
}

void metadata(SampleWave &wave, const Bytes sampler, const Bytes instrument) {
  if (!sampler.empty()) { sampler_metadata(wave, sampler); }
  if (!instrument.empty()) { wave.instrument = instrument_metadata(instrument); }
}
}
