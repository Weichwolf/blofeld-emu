// SPDX-License-Identifier: GPL-3.0-only
#include "audio/buffer.hpp"
#include "audio/wav.hpp"
#include "files/read.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unistd.h>
#include <vector>

namespace {
using blofeld::host::audio::Frame;
using blofeld::host::audio::Wav;
constexpr unsigned rate = 44100;
constexpr unsigned header_bytes = 44;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

template <typename Error> void rejects(auto &&action, const char *message) {
  bool rejected = false;
  try {
    action();
  } catch (const Error &) { rejected = true; }
  require(rejected, message);
}

void golden(const std::filesystem::path &root) {
  constexpr std::array<Frame, 4> frames{
      Frame{0, -1}, Frame{-0x800000, 0x7fffff}, Frame{0x123456, -0x123456}, Frame{1, -2}};
  const auto path = root / "golden.wav";
  Wav wav(path, rate);
  wav.write(std::span(frames).first(1));
  wav.write({});
  wav.write(std::span(frames).subspan(1));
  require(wav.frames() == frames.size() && !std::filesystem::exists(path),
          "Only completed WAV output becomes visible at the selected path");
  wav.finish();
  const auto bytes = blofeld::host::read_file(path, header_bytes + 24);
  const std::vector<std::uint8_t> expected{
      'R',  'I',  'F',  'F',  60,   0,    0,    0,    'W', 'A',  'V',  'E',  'f',  'm',
      't',  ' ',  16,   0,    0,    0,    1,    0,    2,   0,    0x44, 0xac, 0,    0,
      0x98, 0x09, 4,    0,    6,    0,    24,   0,    'd', 'a',  't',  'a',  24,   0,
      0,    0,    0,    0,    0,    0xff, 0xff, 0xff, 0,   0,    0x80, 0xff, 0xff, 0x7f,
      0x56, 0x34, 0x12, 0xaa, 0xcb, 0xed, 1,    0,    0,   0xfe, 0xff, 0xff};
  require(bytes == expected, "RIFF header and signed 24-bit stereo PCM match independent bytes");
  rejects<std::logic_error>([&] { wav.finish(); }, "A sealed WAV cannot be published twice");
  rejects<std::logic_error>([&] { wav.write(frames); }, "A sealed WAV cannot accept more frames");
}

void streaming(const std::filesystem::path &root) {
  constexpr std::size_t count = 4097;
  const std::vector<Frame> frames(count, Frame{0x123456, -0x123456});
  const auto path = root / "streaming.wav";
  {
    Wav wav(path, rate);
    wav.write(frames);
    wav.write(frames);
    require(wav.frames() == count * 2, "Count streamed frames once across partial packing blocks");
    wav.finish();
  }
  const auto bytes = blofeld::host::read_file(path, header_bytes + (count * 12));
  constexpr std::array<std::uint8_t, 6> pair{0x56, 0x34, 0x12, 0xaa, 0xcb, 0xed};
  require(bytes.size() == header_bytes + (count * 12), "No frame is dropped or duplicated");
  for (std::size_t offset = header_bytes; offset < bytes.size(); offset += pair.size()) {
    require(std::ranges::equal(std::span(bytes).subspan(offset, pair.size()), pair),
            "Packing remains exact at every block boundary");
  }
}

void failures(const std::filesystem::path &root) {
  const auto invalid = root / "invalid.wav";
  {
    Wav wav(invalid, rate);
    constexpr std::array frames{Frame{1, 2}, Frame{0x800000, 0}};
    rejects<std::runtime_error>([&] { wav.write(frames); }, "Do not truncate invalid samples");
    require(wav.frames() == 0, "Failed input is not accepted or counted");
    rejects<std::logic_error>([&] { wav.finish(); }, "Failed output cannot publish a partial file");
  }
  require(!std::filesystem::exists(invalid), "Failed output leaves no selected WAV");
  const auto partial = root / "partial.wav";
  {
    Wav wav(partial, rate);
    constexpr std::size_t count = 1025;
    constexpr std::int32_t below_minimum = -0x800001;
    std::vector<Frame> frames(count, Frame{0, 1});
    frames.back()[0] = below_minimum;
    rejects<std::runtime_error>([&] { wav.write(frames); },
                                "Reject invalid samples after a complete file-write block");
    rejects<std::logic_error>([&] { wav.finish(); },
                              "Partially written failed input cannot be finalized");
    require(wav.frames() == 0, "A failed multi-block submission is not accepted");
  }
  require(!std::filesystem::exists(partial), "Partially written failed input stays unpublished");
  const auto abandoned = root / "abandoned.wav";
  { const Wav wav(abandoned, rate); }
  require(!std::filesystem::exists(abandoned), "Destructor does not publish unfinished output");
  rejects<std::invalid_argument>([&] { const Wav wav(root / "zero.wav", 0); },
                                 "Reject zero sample rate before creating a file");
  const auto existing = root / "existing.wav";
  {
    std::ofstream output(existing);
    output << "keep";
  }
  {
    Wav wav(existing, rate);
    rejects<std::system_error>([&] { wav.finish(); }, "Never replace an existing output");
  }
  require(blofeld::host::read_file(existing, 4) == std::vector<std::uint8_t>{'k', 'e', 'e', 'p'},
          "Failed publication preserves existing contents");
  const auto link = root / "symlink.wav";
  std::filesystem::create_symlink("existing.wav", link);
  {
    Wav wav(link, rate);
    rejects<std::system_error>([&] { wav.finish(); }, "Do not follow output symlinks");
  }
  for (const auto &entry : std::filesystem::directory_iterator(root)) {
    require(entry.path().extension() == ".wav", "All owned temporary files are removed");
  }
}
}

int main(const int argc, char **argv) try {
  require(argc == 2, "Supply a build artifact directory");
  const auto root = std::filesystem::path(argv[1]) / std::to_string(::getpid());
  require(std::filesystem::create_directories(root), "Create owned unit directory");
  golden(root);
  streaming(root);
  failures(root);
  std::filesystem::remove_all(root);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
