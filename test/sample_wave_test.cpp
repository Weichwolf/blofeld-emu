// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "sample_wave_fixture.hpp"
#include "samples/sample_wave.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld::samples;
using namespace sample_wave_fixture;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw blofeld::Fault(std::string(message)); }
}

void reject(const Bytes &bytes) {
  try {
    static_cast<void>(decode_sample_wave(bytes));
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault("Invalid own WAV fixture reported success");
}

void test_pcm() {
  const auto unsigned_pcm =
      decode_sample_wave(file({{"fmt ", format(8)}, {"data", {0, 128, 255}}}));
  constexpr double unsigned_max = 127.0 / 128.0;
  require(unsigned_pcm.mono == std::vector<double>{-1, 0, unsigned_max},
          "PCM8 uses an unsigned midpoint and preserves odd data padding");
  const auto signed_pcm = decode_sample_wave(
      file({{"data", {0, 128, 0xff, 0x7f, 0, 0}}, {"JUNK", {7}}, {"fmt ", format()}}));
  constexpr double signed_max = 32767.0 / 32768.0;
  require(signed_pcm.mono == std::vector<double>{-1, signed_max, 0},
          "PCM16 signed extrema and arbitrary ancillary-chunk order are independent expectations");
  constexpr unsigned bits24 = 24;
  const auto pcm24 = decode_sample_wave(
      file({{"fmt ", format(bits24)}, {"data", {0, 0, 0x80, 0, 0, 0x40, 0, 0, 0}}}));
  require(pcm24.mono == std::vector<double>{-1, 0.5, 0}, "PCM24 sign and scale are exact");
  constexpr unsigned bits32 = 32;
  const auto stereo = decode_sample_wave(
      file({{"fmt ", format(bits32, 2)}, {"data", {0, 0, 0, 0x80, 0, 0, 0, 0x40}}}));
  constexpr double stereo_mean = -0.25;
  require(stereo.mono == std::vector<double>{stereo_mean} && stereo.source_channels == 2,
          "PCM32 stereo analysis averages -1 and +0.5 without clipping or changing sources");
}

void test_float_and_extensible() {
  constexpr unsigned bits24 = 24;
  constexpr unsigned bits32 = 32;
  constexpr unsigned bits64 = 64;
  const auto f =
      decode_sample_wave(file({{"fmt ", format(bits32, 1, 3)}, {"data", {0, 0, 0xc0, 0x3f}}}));
  constexpr double above_full_scale = 1.5;
  require(f.floating_point && f.mono == std::vector<double>{above_full_scale},
          "IEEE float input retains finite values above full scale");
  const auto d = decode_sample_wave(
      file({{"fmt ", format(bits64, 1, 3)}, {"data", {0, 0, 0, 0, 0, 0, 0xe0, 0xbf}}}));
  require(d.mono == std::vector<double>{-0.5},
          "IEEE double input decodes its exact sign and exponent");
  constexpr unsigned wave_extensible = 0xfffe;
  auto ext = format(bits24, 1, wave_extensible);
  constexpr unsigned extra = 22;
  constexpr unsigned precision = 20;
  little(ext, extra, 2);
  little(ext, precision, 2);
  little(ext, 4, 4);
  for (const auto v : Bytes{1, 0, 0, 0, 0, 0, 0x10, 0, 0x80, 0, 0, 0xaa, 0, 0x38, 0x9b, 0x71}) {
    ext.push_back(v);
  }
  const auto precise = decode_sample_wave(file({{"fmt ", ext}, {"data", {0, 0, 0x40}}}));
  require(precise.valid_bits == precision && precise.channel_mask == 4 && precise.mono[0] == 0.5,
          "Extensible 20-bit precision is left aligned in the 24-bit container");
  reject(file({{"fmt ", ext}, {"data", {1, 0, 0x40}}}));
  ext.back() ^= 1;
  reject(file({{"fmt ", ext}, {"data", {0, 0, 0x40}}}));
}

void test_metadata() {
  const Bytes pcm(8, 0);
  const Bytes inst{60, 231, 250, 0, 127, 1, 127}; // -25 cents correction; -6 dB.
  const auto with_inst =
      decode_sample_wave(file({{"fmt ", format()}, {"data", pcm}, {"inst", inst}}));
  constexpr double corrected_root = 60.25;
  require(metadata_root(with_inst) == corrected_root && with_inst.instrument &&
              with_inst.instrument->gain_db == -6,
          "inst fine tune is a correction to the nominal key, with signed gain preserved");
  const auto wave = decode_sample_wave(
      file({{"smpl", sampler()}, {"fmt ", format()}, {"data", pcm}, {"inst", inst}}));
  constexpr double sampler_root = 60.5;
  require(metadata_root(wave) == sampler_root && wave.loops.size() == 1 &&
              wave.loops[0].start == 1 && wave.loops[0].end_inclusive == 3 &&
              wave.loops[0].type == 1 && wave.loops[0].play_count == 2 && wave.instrument,
          "smpl root precedence and inclusive alternating loop/count preserve independent fields");
  auto bad = sampler();
  constexpr unsigned loop_end_offset = 48;
  constexpr unsigned loop_type_offset = 40;
  bad[loop_end_offset] = 4;
  reject(file({{"fmt ", format()}, {"data", pcm}, {"smpl", bad}}));
  bad = sampler();
  bad[loop_type_offset] = 3;
  reject(file({{"fmt ", format()}, {"data", pcm}, {"smpl", bad}}));
  bad = sampler();
  bad[12] = 128;
  reject(file({{"fmt ", format()}, {"data", pcm}, {"smpl", bad}}));
  bad = inst;
  constexpr unsigned excessive_correction = 51;
  bad[1] = excessive_correction;
  reject(file({{"fmt ", format()}, {"data", pcm}, {"inst", bad}}));
}

void test_rejections() {
  const Bytes pcm{0, 0};
  reject({});
  reject(file({{"fmt ", format()}, {"data", {0}}}));
  reject(file({{"fmt ", format()}, {"data", {}}}));
  reject(file({{"fmt ", format()}, {"data", pcm}, {"data", pcm}}));
  reject(file({{"fmt ", format()}, {"data", pcm}, {"smpl", {}}}));
  reject(file({{"fmt ", format()}, {"data", pcm}, {"fact", {2, 0, 0, 0}}}));
  for (const auto &fmt :
       std::vector<Bytes>{format(12), format(16, 3), format(16, 0), format(16, 1, 6)}) {
    reject(file({{"fmt ", fmt}, {"data", pcm}}));
  }
  for (const auto at : {2U, 4U, 8U, 12U}) {
    auto fmt = format();
    constexpr unsigned malformed_field = 0xff;
    fmt[at] = malformed_field;
    reject(file({{"fmt ", fmt}, {"data", pcm}}));
  }
  constexpr unsigned bits32 = 32;
  for (const auto &value : std::vector<Bytes>{{0, 0, 0x80, 0x7f}, {0, 0, 0xc0, 0x7f}}) {
    reject(file({{"fmt ", format(bits32, 1, 3)}, {"data", value}}));
  }
  auto truncated = file({{"fmt ", format()}, {"data", pcm}});
  truncated.pop_back();
  reject(truncated);
  auto boundary = file({{"fmt ", format()}, {"data", pcm}});
  boundary.push_back(0);
  reject(boundary);
}

void test_pitch() {
  SampleWave wave;
  constexpr unsigned sample_rate = 44100;
  wave.sample_rate = sample_rate;
  constexpr unsigned frames = 11025;
  constexpr double frequency = 440;
  constexpr double root = 69;
  constexpr double semitone_tolerance = 0.02;
  constexpr double fundamental_level = 0.25;
  for (unsigned i = 0; i < frames; ++i) {
    const auto angle = 2 * std::numbers::pi * frequency * i / wave.sample_rate;
    wave.mono.push_back((fundamental_level * std::sin(angle)) + (0.5 * std::sin(2 * angle)));
  }
  const auto pitch = detect_sample_pitch(wave);
  require(pitch && std::abs(pitch->midi_note - root) < semitone_tolerance,
          "Independent 440 Hz fundamental survives a stronger second harmonic and decimation");
  wave.mono.assign(frames, 0);
  require(!detect_sample_pitch(wave), "Silence has no guessed pitch");
  wave.mono.assign(frames, 0.5);
  require(!detect_sample_pitch(wave), "DC has no guessed pitch");
  wave.mono.assign(512, 0);
  wave.mono[128] = 1;
  require(!detect_sample_pitch(wave), "A single impulse has no reliable stationary pitch");
  wave.mono.assign(2, 1);
  require(!detect_sample_pitch(wave), "Insufficient frames need a manual root");
}
}

int main() {
  try {
    test_pcm();
    test_float_and_extensible();
    test_metadata();
    test_rejections();
    test_pitch();
    std::cout << "Verified own independent PCM/float/stereo, sampler/instrument and pitch vectors; "
                 "no device playback claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
