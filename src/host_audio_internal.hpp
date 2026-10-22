// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/host_audio.hpp"
#include "blofeld/pcm_queue.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <pulse/context.h>
#include <pulse/mainloop.h>
#include <pulse/operation.h>
#include <pulse/sample.h>
#include <pulse/stream.h>
#include <span>
#include <string_view>

namespace blofeld {
struct HostAudio::Stream {
  struct ReleaseLoop {
    void operator()(pa_mainloop *value) const { pa_mainloop_free(value); }
  };

  struct ReleaseContext {
    void operator()(pa_context *value) const {
      pa_context_disconnect(value);
      pa_context_unref(value);
    }
  };

  struct ReleaseStream {
    void operator()(pa_stream *value) const {
      static_cast<void>(pa_stream_disconnect(value));
      pa_stream_unref(value);
    }
  };

  struct ReleaseOperation {
    void operator()(pa_operation *value) const {
      pa_operation_cancel(value);
      pa_operation_unref(value);
    }
  };

  std::unique_ptr<pa_mainloop, ReleaseLoop> loop;
  std::unique_ptr<pa_context, ReleaseContext> context;
  std::unique_ptr<pa_stream, ReleaseStream> connection;
  std::unique_ptr<pa_operation, ReleaseOperation> drain;
  HostAudioTarget target;
  pa_sample_spec spec{};
  PcmQueue<queue_capacity> queue;
  std::optional<HostAudioBoundary> previous;
  std::optional<std::uint64_t> previous_end;
  std::uint64_t bit_ticks = 0;
  std::uint64_t frame_span = 0;
  std::uint64_t accepted = 0;
  std::uint64_t written = 0;
  bool finishing = false;
  bool drained = false;
  bool drain_success = false;
  bool failed = false;
  bool retired = false;

  void close();
  void require_ready() const;
  [[noreturn]] void fail(std::string_view operation, int error);
  void validate(std::span<const SerialPcmFrame> frames) const;
  void boundary(const HostAudioBoundary &value);
  void connect_stream();
  void submit();
  void progress();
  static void drained_callback(pa_stream *source, int success, void *opaque) noexcept;
};
}
