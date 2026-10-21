// SPDX-License-Identifier: GPL-3.0-only
#include "sample_wave_client.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/sample_library.hpp"
#include "blofeld/sample_wave.hpp"
#include "sample_xml.hpp"

#include <filesystem>
#include <iomanip>
#include <limits>
#include <optional>
#include <ostream>
#include <string_view>
#include <utility>

namespace blofeld::samples {
namespace {
double selected_root(const SampleWave &wave, const std::string_view mode) {
  if (mode == "auto") {
    const auto detected = detect_sample_pitch(wave);
    if (!detected) { throw Fault("No reliable WAV root detected; supply a numeric root key"); }
    return detected->midi_note;
  }
  if (mode == "metadata") {
    const auto root = metadata_root(wave);
    if (!root) { throw Fault("No WAV root metadata; choose auto or supply a numeric root key"); }
    return *root;
  }
  return xml::number(mode);
}

void instrument_information(const std::optional<WaveInstrument> &instrument, std::ostream &out) {
  if (!instrument) {
    out << "null";
    return;
  }
  const auto &v = *instrument;
  out << "{\"note\":" << v.note << ",\"correction_cents\":" << v.correction_cents
      << ",\"gain_db\":" << v.gain_db << ",\"lowest_key\":" << v.lowest_key
      << ",\"highest_key\":" << v.highest_key << ",\"lowest_velocity\":" << v.lowest_velocity
      << ",\"highest_velocity\":" << v.highest_velocity << '}';
}
}

void wave_information(const std::filesystem::path &path, std::ostream &output) {
  const auto wave = load_sample_wave(path);
  const auto pitch = detect_sample_pitch(wave);
  const auto root = metadata_root(wave);
  output << std::setprecision(std::numeric_limits<double>::max_digits10)
         << "{\"sample_rate\":" << wave.sample_rate << ",\"channels\":" << wave.source_channels
         << ",\"container_bits\":" << wave.container_bits << ",\"valid_bits\":" << wave.valid_bits
         << ",\"channel_mask\":" << wave.channel_mask
         << ",\"floating_point\":" << (wave.floating_point ? "true" : "false")
         << ",\"frames\":" << wave.mono.size() << ",\"mono_peak\":" << mono_peak(wave)
         << ",\"metadata_root\":";
  if (root) {
    output << *root;
  } else {
    output << "null";
  }
  output << ",\"detected_pitch\":";
  if (pitch) {
    output << "{\"midi_note\":" << pitch->midi_note << ",\"frequency_hz\":" << pitch->frequency_hz
           << ",\"periodicity\":" << pitch->periodicity << '}';
  } else {
    output << "null";
  }
  output << ",\"instrument\":";
  instrument_information(wave.instrument, output);
  output << ",\"loops\":[";
  bool separator = false;
  for (const auto &loop : wave.loops) {
    if (separator) { output << ','; }
    separator = true;
    output << "{\"identifier\":" << loop.identifier << ",\"type\":" << loop.type
           << ",\"start\":" << loop.start << ",\"end_inclusive\":" << loop.end_inclusive
           << ",\"fraction\":" << loop.fraction << ",\"play_count\":" << loop.play_count << '}';
  }
  output << "]}\n";
}

void append_wave(Program &program,
                 const std::filesystem::path &path,
                 const std::string_view root,
                 const unsigned highest) {
  if (program.samples.size() == maximum_samples) { throw Fault("Too many sample references"); }
  const auto wave = load_sample_wave(path);
  Sample sample{.filename = path.filename().generic_string(),
                .source = std::filesystem::absolute(path).lexically_normal(),
                .pitch = selected_root(wave, root),
                .highest = highest,
                .normalize = false,
                .fixed_pitch = false,
                .level_db = 0};
  program.samples.push_back(std::move(sample));
  order_samples(program);
}
}
