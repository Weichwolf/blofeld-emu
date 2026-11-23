// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/consumer.hpp"
#include "audio/playback.hpp"
#include "display/publication.hpp"
#include "io/pcm_output.hpp"
#include "options.hpp"
#include "process/signals.hpp"
#include "vm_context.hpp"
#if defined(__APPLE__)
#include "macos/mcu_wake.hpp"
#include "macos/midi/input.hpp"
#include "macos/midi/output.hpp"
#include "midi/live_clock.hpp"
#endif

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>
#include <thread>
#include <vector>

namespace blofeld::app {
class Live {
public:
  Live(host::audio::Playback &audio, DisplayPublication &display, const Options &options);
  [[nodiscard]] vi::PcmOutput output() noexcept;

  struct Result {
    int status;
    bool save_storage;
  };

  [[nodiscard]] Result run(vi::VmContext &vm, const host::Signals &signals);

private:
  enum class Phase : std::uint8_t { booting, arming, priming, audio_ready, playing, closed };
  [[nodiscard]] std::optional<std::uint64_t> startup(const host::Signals &signals);
  [[nodiscard]] bool refill(std::uint64_t required, const std::stop_token &stop);
  [[nodiscard]] bool release_input(const AudioConsumer::InputGrant &grant, std::uint64_t required);
  [[nodiscard]] std::uint64_t arm_input(std::uint64_t origin);
  [[nodiscard]] bool clock_ready() const noexcept;
  void check_platform() const;
  void print_audio_grants() const;
  void discard();
  void discard_midi_replies();
  [[nodiscard]] std::jthread output_worker();
#if defined(__APPLE__)
  [[nodiscard]] std::uint64_t receive(host::midi::Input &input,
                                      std::uint64_t committed,
                                      const AudioConsumer::InputGrant &grant,
                                      std::uint64_t now);
  void connect();
  void send_midi(host::midi::Output &output, const std::stop_token &stop);
  std::optional<host::midi::Output> midi_output_;
#else
  void send_midi(const std::stop_token &stop);
  [[nodiscard]] std::uint64_t tick_at(std::uint64_t frame) const noexcept;
  [[nodiscard]] std::uint64_t frame_at(std::uint64_t tick) const noexcept;
  std::uint64_t origin_ = 0, lead_ = 0;
  std::atomic<bool> midi_failed_{false};
#endif
  void control();
  void attach_audio_source();
  void publish_display();
  void status(bool terminal) const;
  host::audio::Playback &audio_;
  DisplayPublication &display_;
  AudioConsumer consumer_;
#if defined(__APPLE__)
  McuWake mcu_wake_;
  LiveClock clock_;
  const std::optional<std::int32_t> source_id_;
  const std::uint8_t group_;
  std::unique_ptr<host::midi::Input> input_;
  std::atomic<host::midi::Input *> active_input_{nullptr};
  std::optional<std::vector<host::midi::Source>> sources_;
  std::optional<host::midi::Packet> pending_;
  std::uint64_t ready_host_ns_ = 0, last_requested_ = 0;
  std::chrono::steady_clock::time_point next_connect_;
#else
  std::optional<host::midi::Packet> pending_;
#endif
  vi::VmContext *vm_ = nullptr;
  std::uint64_t ignored_ = 0, received_ = 0;
  std::uint64_t late_packets_ = 0, uncorrelated_packets_ = 0, maximum_late_ticks_ = 0;
  std::atomic<Phase> phase_{Phase::booting};
  std::chrono::steady_clock::time_point next_display_;
};
}
