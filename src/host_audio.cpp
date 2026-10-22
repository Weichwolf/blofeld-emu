// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/host_audio.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/serial_pcm.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <pulse/channelmap.h>
#include <pulse/error.h>
#include <pulse/sample.h>
#include <pulse/simple.h>
#include <span>
#include <string>
#include <string_view>

namespace blofeld {
namespace {
constexpr unsigned channels = 2;
constexpr unsigned precision = SerialPcmFormat::dsp_word_bits;
constexpr std::int32_t minimum_sample = -8388608;
constexpr std::int32_t maximum_sample = 8388607;
constexpr std::int64_t container_scale = 256; // Signed 24-bit PCM in the high bits of S32NE.

void check_target(const HostAudioTarget &target) {
  if (target.server.contains('\0') || target.sink.contains('\0')) {
    throw Fault("Host audio target contains an embedded NUL");
  }
}
}

struct HostAudio::Stream {
  struct Release {
    void operator()(pa_simple *value) const { pa_simple_free(value); }
  };

  std::unique_ptr<pa_simple, Release> connection;
  std::optional<std::uint64_t> previous_end;
  std::uint64_t bit_ticks = 0;
  std::uint64_t frame_span = 0;
  std::uint64_t written = 0;
  bool failed = false;

  void require_ready() const {
    if (failed) { throw Fault("Host audio stream is faulted; create a new transport"); }
  }

  [[noreturn]] void fail(const std::string_view operation, const int error) {
    failed = true;
    throw Fault("Host audio " + std::string(operation) + ": " + pa_strerror(error));
  }

  void validate(const std::span<const SerialPcmFrame> frames) const {
    if (frames.empty() || frames.size() > maximum_block_frames ||
        frames.size() > std::numeric_limits<std::uint64_t>::max() - written) {
      throw Fault("Invalid host audio block size or frame counter exhausted");
    }
    auto previous = previous_end;
    for (const auto &frame : frames) {
      if (frame.channels != channels || !frame.uniform_clock || !frame.bit_ticks ||
          *frame.bit_ticks != bit_ticks || frame.last_tick < frame.first_tick ||
          frame.last_tick - frame.first_tick != frame_span ||
          (previous &&
           (frame.first_tick < *previous || frame.first_tick - *previous != bit_ticks))) {
        throw Fault("Host audio frame shape/timing does not match the declared serial clock");
      }
      for (unsigned channel = 0; channel < channels; ++channel) {
        if (frame.samples.at(channel) < minimum_sample ||
            frame.samples.at(channel) > maximum_sample) {
          throw Fault("Host audio sample exceeds signed 24-bit precision");
        }
      }
      previous = frame.last_tick;
    }
  }
};

HostAudio::HostAudio(const SerialPcmFormat format,
                     const HostAudioClock clock,
                     const HostAudioTarget &target)
    : stream_(std::make_unique<Stream>()) {
  const SerialPcmReceiver checked(format);
  check_target(target);
  const pa_sample_spec spec{
      .format = PA_SAMPLE_S32NE, .rate = clock.sample_rate, .channels = channels};
  const auto denominator =
      static_cast<std::uint64_t>(clock.sample_rate) * format.slot_bits * channels;
  if (format.channels != channels || format.word_bits != precision ||
      pa_sample_spec_valid(&spec) == 0 || denominator == 0 || clock.timebase_hz < denominator ||
      clock.timebase_hz % denominator != 0) {
    throw Fault("Host audio requires stereo signed 24-bit PCM and an integral declared clock");
  }
  stream_->bit_ticks = clock.timebase_hz / denominator;
  stream_->frame_span =
      (static_cast<std::uint64_t>(format.slot_bits) * channels - 1) * stream_->bit_ticks;
  pa_channel_map map{};
  pa_channel_map_init_stereo(&map);
  int error = 0;
  stream_->connection.reset(pa_simple_new(target.server.empty() ? nullptr : target.server.c_str(),
                                          "Blofeld emulator",
                                          PA_STREAM_PLAYBACK,
                                          target.sink.empty() ? nullptr : target.sink.c_str(),
                                          "DSP PCM output",
                                          &spec,
                                          &map,
                                          nullptr,
                                          &error));
  if (!stream_->connection) { stream_->fail("connect", error); }
}

HostAudio::~HostAudio() = default;

void HostAudio::write(const std::span<const SerialPcmFrame> frames) {
  stream_->require_ready();
  stream_->validate(frames);
  std::array<std::int32_t, maximum_block_frames * channels> samples{};
  std::size_t offset = 0;
  for (const auto &frame : frames) {
    for (unsigned channel = 0; channel < channels; ++channel) {
      samples.at(offset++) = static_cast<std::int32_t>(frame.samples.at(channel) * container_scale);
    }
  }
  int error = 0;
  if (pa_simple_write(
          stream_->connection.get(), samples.data(), offset * sizeof(samples[0]), &error) < 0) {
    stream_->fail("write", error);
  }
  stream_->previous_end = frames.back().last_tick;
  stream_->written += frames.size();
}

void HostAudio::drain() {
  stream_->require_ready();
  int error = 0;
  if (pa_simple_drain(stream_->connection.get(), &error) < 0) { stream_->fail("drain", error); }
}

std::uint64_t HostAudio::frames_written() const {
  return stream_->written;
}

bool HostAudio::faulted() const {
  return stream_->failed;
}
}
