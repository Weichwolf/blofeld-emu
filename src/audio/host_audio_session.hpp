// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/host_audio.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <pulse/context.h>
#include <pulse/mainloop.h>
#include <pulse/operation.h>
#include <pulse/stream.h>
#include <string_view>

namespace blofeld {
struct HostAudioOutput::Session {
  static constexpr std::size_t batch_frames = 256;
  static constexpr unsigned channels = 2;
  static constexpr unsigned sample_bytes = 4;
  static constexpr unsigned frame_bytes = channels * sample_bytes;
  explicit Session(HostAudioConfig setup);
  ~Session();
  Session(const Session &) = delete;
  Session &operator=(const Session &) = delete;
  Session(Session &&) = delete;
  Session &operator=(Session &&) = delete;

  void connect();
  void refresh_stream();
  void poll();
  void check();
  void operation(pa_operation *pending, int &result, std::string_view name);
  void validate(std::span<const SerialPcmFrame> frames) const;
  [[nodiscard]] std::size_t write(std::span<const SerialPcmFrame> frames);
  [[noreturn]] void fail(std::string_view name);
  [[noreturn]] void timeout(std::string_view name);
  [[noreturn]] void incompatible(std::string_view reason);
  static void completed(pa_stream *stream, int success, void *data) noexcept;
  static void underflow(pa_stream *stream, void *data) noexcept;

  HostAudioConfig config;
  HostAudioStatus status{};
  HostAudioStreamInfo observed_stream;
  pa_mainloop *loop = nullptr;
  pa_context *context = nullptr;
  pa_stream *stream = nullptr;
  std::array<std::int32_t, batch_frames * channels> samples{};
  std::optional<std::uint64_t> previous_end;
  std::uint64_t bit_ticks = 0;
  std::uint64_t frame_span = 0;
  std::int64_t sample_limit = 0;
  std::int64_t scale = 0;
  bool faulted = false;
};
}
