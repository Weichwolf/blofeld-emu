// SPDX-License-Identifier: GPL-3.0-only
#include "live.hpp"

#include "audio/pcm_clock.hpp"
#include "audio/playback.hpp"
#include "display/publication.hpp"
#include "io/midi_port.hpp"
#include "midi/input.hpp"
#include "midi/output.hpp"
#include "midi/ump.hpp"
#include "options.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <ratio>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

namespace blofeld::app {
namespace {
constexpr std::chrono::microseconds input_poll{250};
constexpr std::size_t packet_batch = 1024;

std::uint64_t host_time(const PcmClock::Point clock, const std::uint64_t tick) {
  const auto later = tick >= clock.ticks;
  const auto ticks = later ? tick - clock.ticks : clock.ticks - tick;
  using Nanoseconds = std::ratio<std::nano::den, vi::MidiPort::reference_hz>;
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (ticks > maximum / Nanoseconds::num) {
    throw std::overflow_error("MIDI output time exceeds the audio clock range");
  }
  const auto distance = ticks * Nanoseconds::num / Nanoseconds::den;
  if (distance > (later ? maximum - clock.host_ns : clock.host_ns)) {
    throw std::overflow_error("MIDI output time exceeds the host clock range");
  }
  return later ? clock.host_ns + distance : clock.host_ns - distance;
}

}

Live::Live(host::audio::Playback &audio, DisplayPublication &display, const Options &options)
    : audio_(audio),
      display_(display),
      consumer_(audio, true),
      clock_({.rate = audio.grants().rate,
              .queue_frames = audio.grants().maximum_frames,
              .uncertainty_ns = audio.grants().maximum_position_uncertainty_ns.value_or(0)}),
      source_id_(options.midi_source),
      group_(options.midi_group) {
  print_audio_grants();
  const auto &grants = audio.grants();
  if (!grants.position_history_frames || *grants.position_history_frames == 0 ||
      !grants.maximum_position_uncertainty_ns) {
    throw std::runtime_error("Live MIDI requires native positions, history and uncertainty grants");
  }
  std::cerr << "Live MIDI uses the native audio playhead; physical key-to-output latency is "
               "unmeasured.\n";
  connect();
  if (options.midi_destination) {
    midi_output_.emplace(*options.midi_destination);
    std::cerr << "MIDI output: ID=" << *options.midi_destination << '\n';
  }
}

void Live::connect() {
  auto sources = host::midi::Input::sources();
  const auto selected = std::ranges::find_if(sources, [&](const auto &source) {
    return source_id_ ? source.id == *source_id_ : sources.size() == 1;
  });
  if (selected != sources.end()) {
    input_ = std::make_unique<host::midi::Input>(
        host::midi::Input::Route{.source = selected->id, .group = group_});
    std::cerr << "MIDI input: " << selected->name << " ID=" << selected->id
              << " group=" << static_cast<unsigned>(group_) << '\n';
    active_input_.store(input_.get(), std::memory_order_release);
    return;
  }
  if (!sources_ || !std::ranges::equal(*sources_, sources, [](const auto &left, const auto &right) {
        return left.id == right.id && left.name == right.name;
      })) {
    std::cerr << "Waiting for native MIDI source; select an ID with --midi-source if ambiguous.\n";
    for (const auto &source : sources) {
      std::cerr << "MIDI source: " << source.id << ' ' << source.name << '\n';
    }
    sources_ = std::move(sources);
  }
}

void Live::control() {
  publish_display();
  const auto now = std::chrono::steady_clock::now();
  if (!input_ && now >= next_connect_) {
    next_connect_ = now + std::chrono::seconds(1);
    connect();
  }
}

void Live::discard() {
  const auto now = host::midi::Input::now();
  auto *input = active_input_.load(std::memory_order_acquire);
  if (input == nullptr) { return; }
  for (std::size_t count = 0; count < packet_batch; ++count) {
    if (!pending_) { pending_ = input->read(); }
    if (!pending_ || pending_->timestamp > now) { return; }
    ignored_ += pending_->size;
    pending_.reset();
  }
}

std::uint64_t Live::receive(host::midi::Input &input,
                            const std::uint64_t committed,
                            const AudioConsumer::InputGrant &grant,
                            const std::uint64_t now) {
  auto &port = vm_->midi();
  const auto exclusive = grant.exclusive;
  for (;;) {
    if (!pending_) { pending_ = input.read(); }
    if (!pending_ || pending_->timestamp > now) { return exclusive; }
    if (pending_->timestamp < ready_host_ns_) {
      ignored_ += pending_->size;
      pending_.reset();
      continue;
    }
    const auto intended = clock_.correlate(pending_->timestamp, grant.audio_epoch);
    const auto requested = std::max(last_requested_, intended.value_or(committed));
    if (requested >= exclusive) { return exclusive; }
    const auto admission = port.stage_input(pending_->data(), requested);
    if (admission.status == vi::MidiPort::InputStatus::full) {
      return std::min(exclusive, admission.tick);
    }
    if (admission.status == vi::MidiPort::InputStatus::refused) {
      const auto *error = port.error();
      throw std::runtime_error(error != nullptr ? error : "Live MIDI admission failed");
    }
    if (!intended) {
      ++uncorrelated_packets_;
    } else if (admission.tick > *intended) {
      ++late_packets_;
      maximum_late_ticks_ = std::max(maximum_late_ticks_, admission.tick - *intended);
    }
    last_requested_ = requested;
    received_ += pending_->size;
    pending_.reset();
  }
}

bool Live::release_input(const AudioConsumer::InputGrant &grant, const std::uint64_t required) {
  auto &port = vm_->midi();
  const auto known = port.input_limit();
  if (!known) { throw std::logic_error("Live MIDI lost its exclusive audio frontier"); }
  if (grant.playhead) { clock_.observe(*grant.playhead); }
  const bool ready = grant.started || phase_.load(std::memory_order_acquire) != Phase::priming;
  auto *const input = ready ? active_input_.load(std::memory_order_acquire) : nullptr;
  const auto cutoff =
      input == nullptr ? grant.exclusive : receive(*input, *known, grant, host::midi::Input::now());
  if (cutoff <= required || !port.advance_input(cutoff)) {
    const auto *error = port.error();
    throw std::runtime_error(
        error != nullptr ? error : "Neither MIDI receiver can retire input before the audio grant");
  }
  if (ready) {
    phase_.store(input == nullptr ? Phase::audio_ready : Phase::playing, std::memory_order_release);
  }
  return true;
}

void Live::send_midi(host::midi::Output &output, const std::stop_token &stop) {
  host::midi::DinEncoder encoder(group_);
  auto &port = vm_->midi();
  while (!stop.stop_requested() && port.await_output(stop)) {
    const auto event = port.read_output();
    if (!event) { continue; }
    const auto packets = encoder.push(event->byte);
    if (packets.size == 0) { continue; }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::nanoseconds(LiveClock::maximum_gap_ns);
    auto clock = consumer_.clock();
    while (!clock && !stop.stop_requested()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("Original MIDI output needs a native audio clock");
      }
      std::this_thread::sleep_for(input_poll);
      clock = consumer_.clock();
    }
    if (!clock) { return; }
    output.send(packets.data(), host_time(*clock, event->ticks));
  }
}

std::jthread Live::output_worker() {
  if (!midi_output_) { return {}; }
  return std::jthread([this, output = &*midi_output_](const std::stop_token &stop) {
    try {
      send_midi(*output, stop);
    } catch (const std::exception &error) {
      std::cerr << "MIDI output stopped: " << error.what() << '\n';
      auto &port = vm_->midi();
      while (port.await_output(stop)) { static_cast<void>(port.read_output()); }
    }
  });
}

void Live::discard_midi_replies() {
  if (midi_output_) { return; }
  while (vm_->midi().read_output()) {}
}

void Live::attach_audio_source() {
  vm_->midi().audio_source({
      .owner = this,
      .refill =
          [](void *owner, const std::uint64_t tick, const std::stop_token &stop) {
            return static_cast<Live *>(owner)->refill(tick, stop);
          },
      .before_mcu_wait =
          [](void *owner, std::uint64_t) noexcept {
            auto &live = *static_cast<Live *>(owner);
            if (!live.mcu_wake_.schedule()) { live.vm_->request_stop(); }
          },
      .before_checkpoint_wait =
          [](void *owner, const vi::CheckpointWait wait) noexcept {
            return static_cast<Live *>(owner)->mcu_wake_.wait_checkpoint(wait);
          },
  });
}

std::uint64_t Live::arm_input(const std::uint64_t origin) {
  ready_host_ns_ = host::midi::Input::now();
  mcu_wake_.start(audio_.device_period());
  vm_->midi().begin_rendering(origin);
  if (origin > std::numeric_limits<std::uint64_t>::max() - clock_.prime_ticks()) {
    throw std::overflow_error("Live priming timeline exhausted");
  }
  return origin + clock_.prime_ticks();
}

bool Live::clock_ready() const noexcept {
  return clock_.has_observation();
}

void Live::check_platform() const {
  mcu_wake_.check();
}

}
