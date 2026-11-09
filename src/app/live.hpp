// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/consumer.hpp"
#include "audio/playback.hpp"
#include "display/publication.hpp"
#include "io/pcm_output.hpp"
#include "midi/input.hpp"
#include "midi/live_clock.hpp"
#include "options.hpp"
#include "process/signals.hpp"
#include "vm_context.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>
#include <vector>

namespace blofeld::app {
class Live {
public:
  Live(host::audio::Playback &audio, DisplayPublication &display, const Options &options);
  [[nodiscard]] vi::PcmOutput output() noexcept;
  int run(vi::VmContext &vm, const host::Signals &signals);

private:
  enum class Phase : std::uint8_t { booting, priming, audio_ready, playing, closed };
  [[nodiscard]] std::optional<std::uint64_t> startup(const host::Signals &signals);
  void feed(std::uint64_t origin, const std::stop_token &stop);
  void admit(LiveClock::Window window);
  void process(LiveClock::Window window,
               std::uint64_t now,
               std::optional<std::uint64_t> &ready,
               const std::stop_token &stop);
  [[nodiscard]] bool receive(host::midi::Input &input,
                             LiveClock::Window window,
                             std::uint64_t ready,
                             const std::stop_token &stop);
  void discard(std::uint64_t now);
  void connect();
  void control();
  void publish_display();
  void status(bool terminal) const;
  host::audio::Playback &audio_;
  DisplayPublication &display_;
  AudioConsumer consumer_;
  LiveClock clock_;
  const std::optional<std::int32_t> source_id_;
  const std::uint8_t group_;
  std::unique_ptr<host::midi::Input> input_;
  std::atomic<host::midi::Input *> active_input_{nullptr};
  std::optional<std::vector<host::midi::Source>> sources_;
  vi::VmContext *vm_ = nullptr;
  std::optional<host::midi::Packet> pending_;
  std::uint64_t ignored_ = 0, received_ = 0;
  std::atomic<Phase> phase_{Phase::booting};
  std::chrono::steady_clock::time_point next_display_;
  std::chrono::steady_clock::time_point next_connect_;
};
}
