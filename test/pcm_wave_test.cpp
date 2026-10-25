// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_recording.hpp"
#include "audio/pcm_wave.hpp"
#include "common/bus.hpp"
#include "pcm_transport_fixture.hpp"
#include "serial_pcm_fixture.hpp"
#include "storage/private_file.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <system_error>
#include <vector>

namespace blofeld::test::pcm {
namespace {
void literal_wave() {
  PcmRecording recording({}, 2);
  recording.append(frame());
  const auto bytes = encode_pcm_wave(recording, nominal);
  constexpr std::array<std::uint8_t, 76> expected{
      0x52, 0x49, 0x46, 0x46, 0x44, 0,    0,    0,    0x57, 0x41, 0x56, 0x45, 0x66,
      0x6d, 0x74, 0x20, 0x28, 0,    0,    0,    0xfe, 0xff, 2,    0,    0x80, 0xbb,
      0,    0,    0,    0xdc, 5,    0,    8,    0,    0x20, 0,    0x16, 0,    0x18,
      0,    0,    0,    0,    0,    1,    0,    0,    0,    0,    0,    0x10, 0,
      0x80, 0,    0,    0xaa, 0,    0x38, 0x9b, 0x71, 0x64, 0x61, 0x74, 0x61, 8,
      0,    0,    0,    0,    0x9f, 0xc3, 0xa5, 0,    0x56, 0x34, 0x12};
  require(std::ranges::equal(bytes, expected),
          "Independent complete WAVEFORMATEXTENSIBLE header and signed24 payload match literally");
  constexpr unsigned second = 740;
  recording.append(frame(second));
  require(encode_pcm_wave(recording, nominal).size() == payload_offset + 16,
          "Two complete stereo frames contribute exactly16 container bytes");
  PcmRecording extreme({.word_bits = 32}, 1);
  auto limits = frame();
  limits.samples[0] = std::numeric_limits<std::int32_t>::min();
  limits.samples[1] = std::numeric_limits<std::int32_t>::max();
  extreme.append(limits);
  const auto wide = encode_pcm_wave(extreme, nominal);
  constexpr std::array<std::uint8_t, 8> extrema{0, 0, 0, 0x80, 0xff, 0xff, 0xff, 0x7f};
  require(wide[valid_bits_offset] == 32 &&
              std::equal(extrema.begin(), extrema.end(), wide.begin() + payload_offset),
          "Full signed32 extrema survive without rounding, clipping or undefined shifts");
  constexpr unsigned precision = 20;
  PcmRecording narrow({.word_bits = precision}, 1);
  auto source = frame();
  constexpr std::int32_t literal = 0x12345;
  source.samples[0] = literal;
  source.samples[1] = -1;
  narrow.append(source);
  const auto packed = encode_pcm_wave(narrow, nominal);
  constexpr std::array<std::uint8_t, 8> expected20{0, 0x50, 0x34, 0x12, 0, 0xf0, 0xff, 0xff};
  require(packed[valid_bits_offset] == precision &&
              std::equal(expected20.begin(), expected20.end(), packed.begin() + payload_offset),
          "Literal20-bit words retain their valid bits left-aligned inside32-bit containers");
}

void timing_rejections() {
  for (unsigned variant = 0; variant < 4; ++variant) {
    PcmRecording recording({}, 2);
    auto input = frame();
    if (variant == 0) { input.uniform_clock = false; }
    if (variant == 1) { input.bit_ticks.reset(); }
    if (variant == 2) { input.last_tick += 1; }
    recording.append(input);
    if (variant == 3) { recording.append(frame(second_tick + 1)); }
    require(rejects([&] { (void)encode_pcm_wave(recording, nominal); }).find("sampling ticks") !=
                std::string::npos,
            "Jitter, missing timing evidence, wrong span and a lost inter-frame bit reject");
  }
  PcmRecording empty({}, 1);
  (void)rejects([&] { (void)encode_pcm_wave(empty, nominal); });
  empty.append(frame());
  for (const unsigned wrong : {0U, 44100U}) {
    (void)rejects(
        [&] { (void)encode_pcm_wave(empty, {.timebase_hz = timebase, .sample_rate = wrong}); });
  }
  const auto maximum = std::numeric_limits<std::uint32_t>::max();
  constexpr unsigned frame_ticks = 640;
  (void)rejects([&] {
    (void)encode_pcm_wave(
        empty,
        {.timebase_hz = static_cast<std::uint64_t>(maximum) * frame_ticks, .sample_rate = maximum});
  });
  PcmRecording endpoint({}, 1);
  constexpr unsigned duration = 630;
  endpoint.append(frame(std::numeric_limits<std::uint64_t>::max() - duration));
  require(encode_pcm_wave(endpoint, nominal).size() == payload_offset + 8,
          "A valid final sampling tick of UINT64_MAX does not require an invented successor");
}

void atomic_files(const std::filesystem::path &directory) {
  const auto path = directory / "audio.wav";
  const std::vector<std::uint8_t> old{1, 2, 3};
  PcmRecording recording({}, 1);
  recording.append(frame());
  for (const auto phase :
       {PrivateFilePhase::created, PrivateFilePhase::written, PrivateFilePhase::synced}) {
    write_private_file(path, old);
    (void)rejects([&] {
      save_pcm_wave(path, recording, nominal, [&](const PrivateFilePhase observed) {
        if (observed == phase) { throw Fault("Injected pre-publication WAV error"); }
      });
    });
    require(read_private_file(path, 1024) == old,
            "Pre-publication failures preserve the previous complete file");
  }
  std::vector<PrivateFilePhase> phases;
  save_pcm_wave(path, recording, nominal, [&](const auto phase) { phases.push_back(phase); });
  require(phases == std::vector{PrivateFilePhase::created,
                                PrivateFilePhase::written,
                                PrivateFilePhase::synced,
                                PrivateFilePhase::published,
                                PrivateFilePhase::directory_synced} &&
              read_private_file(path, 1024) == encode_pcm_wave(recording, nominal),
          "The complete recording publishes only after file sync and then syncs its directory");
  for (const auto phase : {PrivateFilePhase::published, PrivateFilePhase::directory_synced}) {
    write_private_file(path, old);
    (void)rejects([&] {
      save_pcm_wave(path, recording, nominal, [&](const PrivateFilePhase observed) {
        if (observed == phase) { throw Fault("Injected post-publication WAV error"); }
      });
    });
    require(
        read_private_file(path, 1024) == encode_pcm_wave(recording, nominal),
        "Post-rename errors stay visible and do not falsely promise restoration of the old file");
  }
  const auto permissions = std::filesystem::status(path).permissions();
  require(permissions == (std::filesystem::perms::owner_read | std::filesystem::perms::owner_write),
          "Audio references retain private0600 permissions");
  const auto link = directory / "link.wav";
  std::filesystem::remove(link);
  std::filesystem::create_symlink(path.filename(), link);
  (void)rejects([&] { save_pcm_wave(link, recording, nominal); });
  std::filesystem::remove(link);
  (void)rejects([&] { save_pcm_wave(directory / "wrong.bin", recording, nominal); });
  bool missing_parent = false;
  try {
    save_pcm_wave(directory / "missing-parent/audio.wav", recording, nominal);
  } catch (const std::filesystem::filesystem_error &error) {
    missing_parent = error.code() == std::errc::no_such_file_or_directory;
  }
  require(missing_parent, "The host filesystem reports its actual missing-directory failure");
  for (const auto &entry : std::filesystem::directory_iterator(directory)) {
    require(entry.path().filename().string().find(".tmp.") == std::string::npos,
            "Injected publication failures clean their temporary files");
  }
}
}

void wave_checks(const std::filesystem::path &directory) {
  literal_wave();
  timing_rejections();
  atomic_files(directory);
}
}
