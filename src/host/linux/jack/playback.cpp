// SPDX-License-Identifier: GPL-3.0-only
#include "audio/playback.hpp"

#include "audio/pcm_frame.hpp"
#include "audio/render_buffer.hpp"
#include "linux/jack/midi_queue.hpp"
#include "midi/ump.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <jack/jack.h>
#include <jack/midiport.h>
#include <jack/types.h>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>

namespace blofeld::host::audio {
namespace {
constexpr auto poll = std::chrono::microseconds(25);
constexpr unsigned maximum_rate = 384000;
constexpr unsigned periods_per_second_minimum = 100;

jack_client_t *open(const unsigned rate, const std::string_view name) {
  if (rate == 0 || rate > maximum_rate) { throw std::invalid_argument("Invalid JACK sample rate"); }
  jack_status_t status{};
  auto *client = jack_client_open(std::string(name).c_str(), JackNoStartServer, &status);
  if (client == nullptr) {
    throw std::runtime_error("Open JACK client (status " + std::to_string(status) + ')');
  }
  return client;
}

std::size_t quantum(jack_client_t *client, const unsigned rate) {
  if (jack_get_sample_rate(client) != rate) {
    throw std::runtime_error("JACK server sample rate must be " + std::to_string(rate) + " Hz");
  }
  const auto frames = jack_get_buffer_size(client);
  if (frames == 0 || frames > rate / periods_per_second_minimum) {
    throw std::runtime_error("Unsupported JACK buffer size: " + std::to_string(frames));
  }
  return frames;
}

jack_port_t *port(jack_client_t *client, const char *name, const char *type, unsigned long flags) {
  auto *result = jack_port_register(client, name, type, flags, 0);
  if (result == nullptr) { throw std::runtime_error(std::string("Register JACK port ") + name); }
  return result;
}
}

struct Playback::Impl {
  static_assert(std::atomic<bool>::is_always_lock_free);
  static_assert(std::atomic<const char *>::is_always_lock_free);

  Impl(const unsigned rate, const std::string_view name)
      : client(open(rate, name), jack_client_close),
        block(quantum(client.get(), rate)),
        pcm(2 * block),
        grants{.rate = rate,
               .sink_rate = rate,
               .target_frames = pcm.capacity(),
               .maximum_frames = pcm.capacity(),
               .minimum_request_frames = block,
               .position_history_frames = {},
               .maximum_position_uncertainty_ns = {},
               .sink = jack_get_client_name(client.get()),
               .sink_format = "JACK float32 stereo"} {
    left = port(client.get(), "audio_l", JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput);
    right = port(client.get(), "audio_r", JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput);
    input = port(client.get(), "midi_in", JACK_DEFAULT_MIDI_TYPE, JackPortIsInput);
    output = port(client.get(), "midi_out", JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput);
    jack_on_shutdown(client.get(), shutdown, this);
    if (jack_set_process_callback(client.get(), process, this) != 0 ||
        jack_set_xrun_callback(client.get(), xrun, this) != 0 ||
        jack_set_sample_rate_callback(client.get(), sample_rate, this) != 0 ||
        jack_activate(client.get()) != 0) {
      throw std::runtime_error("Activate JACK client");
    }
  }

  ~Impl() { stop(); }

  void stop() noexcept {
    if (!closed.exchange(true, std::memory_order_acq_rel)) {
      static_cast<void>(jack_deactivate(client.get()));
      client.reset();
    }
  }

  void fail(const char *reason) noexcept {
    const char *none = nullptr;
    error.compare_exchange_strong(none, reason, std::memory_order_relaxed);
  }

  static void shutdown(void *context) noexcept {
    static_cast<Impl *>(context)->fail("JACK server disconnected");
  }

  static int sample_rate(const jack_nframes_t rate, void *context) noexcept {
    auto &self = *static_cast<Impl *>(context);
    if (rate != self.grants.rate) { self.fail("JACK server sample rate changed"); }
    return 0;
  }

  static int xrun(void *context) noexcept {
    auto &self = *static_cast<Impl *>(context);
    if (!self.closed.load(std::memory_order_acquire) &&
        self.frontier.load(std::memory_order_relaxed) != 0) {
      self.underruns.fetch_add(1, std::memory_order_relaxed);
    }
    return 0;
  }

  static int process(const jack_nframes_t frames, void *context) noexcept [[clang::nonblocking]] {
    auto &self = *static_cast<Impl *>(context);
    self.cycle(frames);
    return 0;
  }

  void receive(void *buffer, const jack_nframes_t frames) noexcept {
    if (jack_midi_get_lost_event_count(buffer) != 0) {
      fail("JACK reported lost MIDI input events");
      return;
    }
    const auto count = jack_midi_get_event_count(buffer);
    for (std::uint32_t index = 0; index < count; ++index) {
      jack_midi_event_t event{};
      if (jack_midi_event_get(&event, buffer, index) != 0 || event.time >= frames ||
          !incoming.push(frame + event.time, {event.buffer, event.size})) {
        fail("Invalid or overflowing JACK MIDI input");
        return;
      }
    }
  }

  void transmit(void *buffer, const jack_nframes_t frames) noexcept {
    while (const auto message = outgoing.peek()) {
      if (message->frame >= frame + frames) { return; }
      if (message->size() > jack_midi_max_event_size(buffer)) {
        if (jack_midi_get_event_count(buffer) == 0) {
          fail("JACK MIDI output event exceeds buffer");
        }
        return;
      }
      const auto offset = static_cast<jack_nframes_t>(std::max(frame, message->frame) - frame);
      auto *bytes = jack_midi_event_reserve(buffer, offset, message->size());
      if (bytes == nullptr) {
        fail("JACK rejected a scheduled MIDI output event");
        return;
      }
      bytes = std::ranges::copy(message->first, bytes).out;
      std::ranges::copy(message->second, bytes);
      outgoing.consume(message->size());
    }
  }

  void cycle(const jack_nframes_t frames) noexcept {
    const std::span l(static_cast<float *>(jack_port_get_buffer(left, frames)), frames);
    const std::span r(static_cast<float *>(jack_port_get_buffer(right, frames)), frames);
    auto *midi = jack_port_get_buffer(output, frames);
    jack_midi_clear_buffer(midi);
    if (frames != block) { fail("JACK server buffer size changed"); }
    if (resetting.load(std::memory_order_acquire)) {
      pcm.discard();
      primed = false;
      late_frames = 0;
      resetting.store(false, std::memory_order_release);
    }
    if (closed.load(std::memory_order_acquire) ||
        error.load(std::memory_order_relaxed) != nullptr) {
      std::ranges::fill(l, 0.0F);
      std::ranges::fill(r, 0.0F);
      return;
    }
    const auto was_primed = primed;
    if (!primed) { primed = pcm.queued() == pcm.capacity(); }
    if (primed || frame != 0) {
      const auto current = jack_last_frame_time(client.get());
      if (previous_cycle) {
        const auto elapsed = current - *previous_cycle;
        if (elapsed > frames && elapsed < std::numeric_limits<jack_nframes_t>::max() / 2U) {
          const auto skipped = elapsed - frames;
          frame += skipped;
          if (was_primed) { late_frames += skipped; }
        }
      }
      previous_cycle = current;
    }
    if (primed) {
      const auto copied = pcm.render(l, r, &late_frames);
      submitted.fetch_add(copied, std::memory_order_relaxed);
      if (copied != frames) { underruns.fetch_add(1, std::memory_order_relaxed); }
    } else {
      std::ranges::fill(l, 0.0F);
      std::ranges::fill(r, 0.0F);
      if (frame == 0) { return; }
    }
    receive(jack_port_get_buffer(input, frames), frames);
    transmit(midi, frames);
    frame += frames;
    // The DSP can now advance through this cycle's input, including empty cycles.
    frontier.store(frame, std::memory_order_release);
  }

  std::unique_ptr<jack_client_t, decltype(&jack_client_close)> client;
  const std::size_t block;
  RenderBuffer pcm;
  const Grants grants;
  jack_port_t *left = nullptr, *right = nullptr, *input = nullptr, *output = nullptr;
  JackMidiQueue incoming, outgoing;
  std::atomic<bool> closed{false}, resetting{false};
  std::atomic<const char *> error{nullptr};
  std::atomic<std::uint64_t> frontier{0}, submitted{0}, underruns{0};
  std::optional<jack_nframes_t> previous_cycle;
  std::uint64_t frame = 0;
  std::size_t late_frames = 0;
  bool primed = false;
};

Playback::Playback(const unsigned rate, const std::string_view name)
    : impl_(std::make_unique<Impl>(rate, name)) {}

Playback::~Playback() = default;

const Grants &Playback::grants() const noexcept {
  return impl_->grants;
}

Playback::Producer Playback::producer() const {
  const auto seconds =
      std::chrono::duration<double>(static_cast<double>(impl_->block) / impl_->grants.rate);
  return Producer(std::chrono::duration_cast<std::chrono::nanoseconds>(seconds));
}

void Playback::stop() noexcept {
  impl_->stop();
}

void Playback::check() const {
  if (const auto *reason = impl_->error.load(std::memory_order_relaxed)) {
    throw std::runtime_error(reason);
  }
}

bool Playback::write(std::span<const Frame> frames, const std::stop_token &stop) {
  while (!frames.empty()) {
    check();
    if (stop.stop_requested() || impl_->closed.load(std::memory_order_relaxed)) { return false; }
    const auto consumed = impl_->pcm.produce(frames);
    frames = frames.subspan(consumed);
    if (consumed == 0) { std::this_thread::sleep_for(poll); }
  }
  return true;
}

bool Playback::reset(const std::stop_token &stop) {
  impl_->resetting.store(true, std::memory_order_release);
  while (impl_->resetting.load(std::memory_order_acquire)) {
    check();
    if (stop.stop_requested() || impl_->closed.load(std::memory_order_relaxed)) { return false; }
    std::this_thread::sleep_for(poll);
  }
  return true;
}

PlaybackStatus Playback::status() const noexcept {
  return {.submitted_frames = impl_->submitted.load(std::memory_order_relaxed),
          .underruns = impl_->underruns.load(std::memory_order_relaxed),
          .queued_frames = impl_->pcm.queued(),
          .server_buffer_frames = impl_->block,
          .latency_us = {},
          .started = midi_frontier() != 0,
          .failed = impl_->error.load(std::memory_order_relaxed) != nullptr,
          .position = {}};
}

bool Playback::fills_render_quantum(const std::size_t pending) const noexcept {
  if (midi_frontier() == 0) { return false; }
  const auto queued = impl_->pcm.queued();
  return queued < impl_->block && pending >= impl_->block - queued;
}

std::uint64_t Playback::midi_frontier() const noexcept {
  return impl_->frontier.load(std::memory_order_acquire);
}

std::optional<midi::Packet> Playback::read_midi(const std::uint64_t before) noexcept {
  const auto message = impl_->incoming.peek();
  if (!message || message->frame >= before) { return {}; }
  midi::Packet packet{.bytes = {}, .timestamp = message->frame};
  const auto bytes = std::min(message->first.size(), packet.bytes.size());
  std::ranges::copy(message->first.first(bytes), packet.bytes.data());
  const auto rest = std::min(message->second.size(), packet.bytes.size() - bytes);
  std::ranges::copy(message->second.first(rest), packet.bytes.data() + bytes);
  const auto count = bytes + rest;
  packet.size = static_cast<std::uint8_t>(count);
  impl_->incoming.consume(count);
  return packet;
}

void Playback::write_midi(const std::span<const std::uint8_t> message, const std::uint64_t frame) {
  check();
  if (!impl_->outgoing.push(frame, message)) { throw std::runtime_error("JACK MIDI output full"); }
}
}
