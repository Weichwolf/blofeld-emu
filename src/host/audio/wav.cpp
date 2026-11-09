// SPDX-License-Identifier: GPL-3.0-only
#include "audio/wav.hpp"

#include "audio/buffer.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#if defined(__APPLE__)
#include <sys/fcntl.h>
#else
#include <fcntl.h>
#endif
#include <system_error>
#include <unistd.h>

namespace blofeld::host::audio {
namespace {
constexpr unsigned channels = 2, sample_bytes = 3, frame_bytes = channels * sample_bytes;
constexpr unsigned header_bytes = 44, riff_overhead = header_bytes - 8;
constexpr auto max_field = std::numeric_limits<std::uint32_t>::max();
constexpr std::uint64_t max_frames = (max_field - riff_overhead) / frame_bytes;
constexpr std::int32_t sample_minimum = -0x800000, sample_maximum = 0x7fffff;
constexpr std::size_t block_frames = 1024;
constexpr unsigned rate_offset = 24, byte_rate_offset = 28, payload_size_offset = 40;
constexpr std::uint32_t byte_mask = 0xff;

struct Extent {
  unsigned rate;
  std::uint64_t frames;
};

void little(const std::span<std::uint8_t> bytes, std::uint32_t value) {
  for (auto &byte : bytes) {
    byte = static_cast<std::uint8_t>(value & byte_mask);
    value >>= 8U;
  }
}

std::array<std::uint8_t, header_bytes> header(const Extent extent) {
  std::array<std::uint8_t, header_bytes> bytes{
      'R', 'I',      'F', 'F', 0,           0,   0,
      0,   'W',      'A', 'V', 'E',         'f', 'm',
      't', ' ',      16,  0,   0,           0,   1,
      0,   channels, 0,   0,   0,           0,   0,
      0,   0,        0,   0,   frame_bytes, 0,   sample_bytes * 8,
      0,   'd',      'a', 't', 'a',         0,   0,
      0,   0};
  const auto payload = static_cast<std::uint32_t>(extent.frames * frame_bytes);
  little(std::span(bytes).subspan(4, 4), riff_overhead + payload);
  little(std::span(bytes).subspan(rate_offset, 4), extent.rate);
  little(std::span(bytes).subspan(byte_rate_offset, 4), extent.rate * frame_bytes);
  little(std::span(bytes).subspan(payload_size_offset, 4), payload);
  return bytes;
}

void write_all(const int file, std::span<const std::uint8_t> bytes) {
  while (!bytes.empty()) {
    const auto count = ::write(file, bytes.data(), bytes.size());
    if (count < 0) {
      if (errno == EINTR) { continue; }
      throw std::system_error(errno, std::generic_category(), "Write WAV output");
    }
    if (count == 0) { throw std::runtime_error("WAV output made no progress"); }
    bytes = bytes.subspan(static_cast<std::size_t>(count));
  }
}

void pack(const std::span<const Frame> frames, const std::span<std::uint8_t> bytes) {
  std::size_t offset = 0;
  for (const auto &frame : frames) {
    for (const auto sample : frame) {
      if (sample < sample_minimum || sample > sample_maximum) {
        throw std::runtime_error("WAV input exceeds signed 24-bit PCM");
      }
      little(bytes.subspan(offset, sample_bytes), std::bit_cast<std::uint32_t>(sample));
      offset += sample_bytes;
    }
  }
}
}

struct Wav::State {
  enum class Phase : std::uint8_t { writing, failed, finished };
  std::filesystem::path path, temporary;
  int file = -1;
  unsigned rate;
  std::uint64_t frames = 0;
  std::array<std::uint8_t, block_frames * frame_bytes> buffer{};
  std::size_t used = 0;
  Phase phase = Phase::writing;

  State(const std::filesystem::path &target, const unsigned sample_rate)
      : path(target),
        temporary(target.string() + ".part." + std::to_string(::getpid())),
        rate(sample_rate) {
    if (rate == 0 || rate > max_field / frame_bytes) {
      throw std::invalid_argument("WAV sample rate is not representable");
    }
    constexpr unsigned private_mode = 0600;
    file = ::open(
        temporary.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC | O_NOFOLLOW, private_mode);
    if (file < 0) { throw std::system_error(errno, std::generic_category(), "Create WAV output"); }
  }

  ~State() {
    if (file >= 0) { ::close(file); }
    ::unlink(temporary.c_str());
  }

  State(const State &) = delete;
  State &operator=(const State &) = delete;
};

Wav::Wav(const std::filesystem::path &path, const unsigned rate)
    : state_(std::make_unique<State>(path, rate)) {
  write_all(state_->file, header({.rate = rate, .frames = 0}));
}

Wav::~Wav() = default;

void Wav::write(std::span<const Frame> frames) {
  if (state_->phase != State::Phase::writing) {
    throw std::logic_error("WAV output is not writable");
  }
  state_->phase = State::Phase::failed;
  if (frames.size() > max_frames - state_->frames) {
    throw std::runtime_error("WAV output exceeds the RIFF size limit");
  }
  const auto count = frames.size();
  while (!frames.empty()) {
    const auto available = (state_->buffer.size() - state_->used) / frame_bytes;
    const auto block = frames.first(std::min(frames.size(), available));
    const auto bytes = std::span(state_->buffer).subspan(state_->used, block.size() * frame_bytes);
    pack(block, bytes);
    state_->used += bytes.size();
    if (state_->used == state_->buffer.size()) {
      write_all(state_->file, state_->buffer);
      state_->used = 0;
    }
    frames = frames.subspan(block.size());
  }
  state_->frames += count;
  state_->phase = State::Phase::writing;
}

void Wav::finish() {
  if (state_->phase != State::Phase::writing) {
    throw std::logic_error("WAV output is not writable");
  }
  state_->phase = State::Phase::failed;
  write_all(state_->file, std::span(state_->buffer).first(state_->used));
  if (::lseek(state_->file, 0, SEEK_SET) < 0) {
    throw std::system_error(errno, std::generic_category(), "Seek WAV header");
  }
  write_all(state_->file, header({.rate = state_->rate, .frames = state_->frames}));
  if (::fsync(state_->file) != 0) {
    throw std::system_error(errno, std::generic_category(), "Flush WAV output");
  }
  const auto file = state_->file;
  state_->file = -1;
  if (::close(file) != 0) {
    throw std::system_error(errno, std::generic_category(), "Close WAV output");
  }
  if (::link(state_->temporary.c_str(), state_->path.c_str()) != 0) {
    throw std::system_error(errno, std::generic_category(), "Publish WAV output without replacing");
  }
  state_->phase = State::Phase::finished;
}

std::uint64_t Wav::frames() const noexcept {
  return state_->frames;
}
}
