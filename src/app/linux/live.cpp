// SPDX-License-Identifier: GPL-3.0-only
#include "live.hpp"

#include "audio/playback.hpp"
#include "display/publication.hpp"
#include "io/midi_port.hpp"
#include "linux/jack/midi_queue.hpp"
#include "midi/ump.hpp"
#include "options.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

namespace blofeld::app {
namespace {
constexpr auto ticks_per_second = std::uint64_t{vi::MidiPort::reference_hz};
constexpr unsigned pcm_block = 16;
constexpr std::uint64_t latency_limit_ms = 10;
constexpr unsigned ms_per_second = 1000;

cpu_set_t thread_siblings(const int cpu) {
  std::ifstream siblings("/sys/devices/system/cpu/cpu" + std::to_string(cpu) +
                         "/topology/thread_siblings_list");
  cpu_set_t mask{};
  for (int begin = 0; siblings >> begin;) {
    int last = begin;
    if (siblings.peek() == '-') {
      siblings.ignore();
      if (!(siblings >> last)) { return {}; }
    }
    if (begin < 0 || last < begin || last >= CPU_SETSIZE) { return {}; }
    for (int sibling = begin; sibling <= last; ++sibling) { CPU_SET(sibling, &mask); }
    if (siblings.peek() != ',') { break; }
    siblings.ignore();
  }
  return mask;
}

std::array<cpu_set_t, 2> owner_affinity() {
  cpu_set_t allowed{};
  if (::sched_getaffinity(0, sizeof(allowed), &allowed) != 0) { return {}; }
  const auto current = ::sched_getcpu();
  if (current < 0) { return {}; }
  auto mcu = thread_siblings(current);
  CPU_AND(&mcu, &mcu, &allowed);
  cpu_set_t dsp{};
  CPU_XOR(&dsp, &allowed, &mcu);
  if (CPU_COUNT(&mcu) < 2 || CPU_COUNT(&dsp) == 0) { return {}; }
  return {mcu, dsp};
}
}

Live::Live(host::audio::Playback &audio, DisplayPublication &display, const Options & /*options*/)
    : lead_(audio.grants().maximum_frames + pcm_block),
      audio_(audio),
      display_(display),
      consumer_(audio, true) {
  print_audio_grants();
  const auto quantum = audio.grants().minimum_request_frames.value_or(0);
  if (quantum == 0 || (lead_ + quantum) * ms_per_second > audio.grants().rate * latency_limit_ms) {
    throw std::runtime_error("Choose a JACK quantum of 128 frames or less at 44.1 kHz");
  }
  const auto &name = audio.grants().sink;
  std::cerr << "JACK ports: " << name << ":{audio_l,audio_r,midi_in,midi_out}\n";
}

void Live::start_owners() {
  vm_->start([masks = owner_affinity()](const vi::VmContext::Owner owner) {
    const auto &mask = masks[owner == vi::VmContext::Owner::mcu ? 0U : 1U];
    if (CPU_COUNT(&mask) != 0) {
      static_cast<void>(::pthread_setaffinity_np(::pthread_self(), sizeof(mask), &mask));
    }
    if (::sched_getscheduler(0) != SCHED_OTHER) { return; }
    // Keep PCM production above normal tasks and below a real-time JACK callback.
    // NOLINTNEXTLINE(misc-include-cleaner) sched.h declares the POSIX type.
    const sched_param realtime{.sched_priority = ::sched_get_priority_min(SCHED_RR)};
    const auto error = ::pthread_setschedparam(::pthread_self(), SCHED_RR, &realtime);
    if (error != 0 && error != EPERM) {
      throw std::system_error(error, std::generic_category(), "Set live VM scheduling");
    }
  });
}

void Live::control() {
  publish_display();
}

void Live::discard() {}

void Live::discard_midi_replies() {}

void Live::check_platform() const {
  if (midi_failed_.load(std::memory_order_acquire)) {
    throw std::runtime_error("JACK MIDI failed");
  }
}

bool Live::clock_ready() const noexcept {
  return audio_.midi_frontier() != 0;
}

std::uint64_t Live::tick_at(const std::uint64_t frame) const noexcept {
  const auto rate = audio_.grants().rate;
  const auto fraction = (frame % rate) * ticks_per_second;
  return origin_ + ((frame / rate) * ticks_per_second) + ((fraction + rate - 1) / rate);
}

std::uint64_t Live::frame_at(const std::uint64_t tick) const noexcept {
  const auto rate = audio_.grants().rate;
  const auto delta = tick >= origin_ ? tick - origin_ : 0;
  const auto fraction = (delta % ticks_per_second) * rate;
  return ((delta / ticks_per_second) * rate) +
         ((fraction + ticks_per_second - 1) / ticks_per_second);
}

std::uint64_t Live::arm_input(const std::uint64_t origin) {
  origin_ = origin;
  return tick_at(lead_);
}

void Live::attach_audio_source() {
  vm_->midi().audio_source({
      .owner = this,
      .refill =
          [](void *owner, const std::uint64_t tick, const std::stop_token &stop) {
            return static_cast<Live *>(owner)->refill(tick, stop);
          },
  });
}

bool Live::release_input(const AudioConsumer::InputGrant &grant, const std::uint64_t required) {
  const auto captured = audio_.midi_frontier();
  if (captured == 0) { return false; }
  auto cutoff = std::min(grant.exclusive, tick_at(captured + lead_));
  if (cutoff <= required) { return false; }

  auto &port = vm_->midi();
  for (;;) {
    if (!pending_) { pending_ = audio_.read_midi(captured); }
    if (!pending_) { break; }
    const auto requested = tick_at(pending_->timestamp + lead_);
    if (requested >= cutoff) { break; }
    const auto admission = port.stage_input(pending_->data(), requested);
    if (admission.status == vi::MidiPort::InputStatus::full) {
      cutoff = std::min(cutoff, admission.tick);
      break;
    }
    if (admission.status == vi::MidiPort::InputStatus::refused) {
      throw std::runtime_error("JACK MIDI admission failed");
    }
    if (admission.tick > requested) {
      ++late_packets_;
      maximum_late_ticks_ = std::max(maximum_late_ticks_, admission.tick - requested);
    }
    received_ += pending_->size;
    pending_.reset();
  }
  if (cutoff <= required || !port.advance_input(cutoff)) {
    throw std::runtime_error("JACK MIDI cannot retire its captured audio interval");
  }
  phase_.store(Phase::playing, std::memory_order_release);
  return true;
}

void Live::send_midi(const std::stop_token &stop) {
  host::midi::DinEncoder encoder(0);
  host::midi::Ump decoder(0);
  std::vector<std::uint8_t> exclusive;
  auto &port = vm_->midi();
  while (!stop.stop_requested() && port.await_output(stop)) {
    const auto event = port.read_output();
    if (!event) { continue; }
    const auto batch = encoder.push(event->byte);
    auto words = batch.data();
    while (!words.empty()) {
      const bool sysex = (words.front() >> 28U) == 3;
      const auto count = sysex ? 2U : 1U;
      const auto message = decoder.decode(words.first(count), event->ticks);
      words = words.subspan(count);
      if (!message) { throw std::runtime_error("Invalid DIN reply for JACK MIDI"); }
      const auto bytes = message->data();
      const auto frame = frame_at(event->ticks);
      if (!sysex) {
        audio_.write_midi(bytes, frame);
      } else {
        if (bytes.size() > host::audio::JackMidiQueue::capacity - exclusive.size()) {
          throw std::runtime_error("Firmware SysEx reply exceeds the JACK MIDI queue");
        }
        exclusive.insert(exclusive.end(), bytes.begin(), bytes.end());
        constexpr std::uint8_t end_of_exclusive = 0xf7;
        if (!bytes.empty() && bytes.back() == end_of_exclusive) {
          audio_.write_midi(exclusive, frame);
          exclusive.clear();
        }
      }
    }
  }
}

std::jthread Live::output_worker() {
  return std::jthread([this](const std::stop_token &stop) {
    try {
      send_midi(stop);
    } catch (const std::exception &error) {
      std::cerr << "JACK MIDI output stopped: " << error.what() << '\n';
      midi_failed_.store(true, std::memory_order_release);
      vm_->request_stop();
    }
  });
}
}
