// SPDX-License-Identifier: GPL-3.0-only
#include "midi/smf.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <jack/jack.h>
#include <jack/midiport.h>
#include <jack/types.h>
#include <memory>
#include <span>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace {
constexpr unsigned rate = 44100, period = 64, tail = rate / 2;
constexpr unsigned status_type = 0xf0, program_change = 0xc0, channel_pressure = 0xd0;
constexpr unsigned last_channel_status = 0xef, max_data = 127;
constexpr unsigned maximum_score_frames = rate * 35;
constexpr std::chrono::milliseconds poll_interval{20};

struct Event {
  unsigned frame;
  std::vector<jack_midi_data_t> bytes;
};

struct Score {
  std::vector<Event> events;
  unsigned end;
};

Score load_fixture(const std::filesystem::path &file) {
  std::ifstream input(file);
  std::vector<Event> events;
  unsigned frame = 0;
  unsigned status = 0;
  unsigned data1 = 0;
  unsigned data2 = 0;
  while (input >> frame >> status >> data1 >> data2) {
    if ((!events.empty() && frame < events.back().frame) || frame > maximum_score_frames ||
        status < 0x80 || status > last_channel_status || data1 > max_data || data2 > max_data) {
      throw std::runtime_error("Invalid MIDI event");
    }
    events.push_back(
        {.frame = frame,
         .bytes = {static_cast<jack_midi_data_t>(status), static_cast<jack_midi_data_t>(data1)}});
    const auto type = status & status_type;
    if (type != program_change && type != channel_pressure) {
      events.back().bytes.push_back(static_cast<jack_midi_data_t>(data2));
    }
  }
  if (!input.eof() || events.empty()) { throw std::runtime_error("Invalid MIDI event file"); }
  return {.events = std::move(events), .end = frame + tail};
}

Score load_smf(const std::filesystem::path &file) {
  using blofeld::host::midi::Smf;
  Smf score(file);
  std::vector<Event> events;
  while (const auto event = score.next()) {
    if (event->kind == Smf::Kind::meta) { continue; }
    const auto frame = event->time.ceil_ticks(rate);
    if (frame > maximum_score_frames || !event->first || !event->last) {
      throw std::runtime_error("JACK test score exceeds its time or MIDI packet bound");
    }
    events.push_back({.frame = static_cast<unsigned>(frame),
                      .bytes = {event->bytes.begin(), event->bytes.end()}});
  }
  const auto end = score.time().ceil_ticks(rate);
  if (events.empty() || end > maximum_score_frames) {
    throw std::runtime_error("Empty or overlong JACK test score");
  }
  return {.events = std::move(events), .end = static_cast<unsigned>(end)};
}

struct Replay {
  const Score score;
  jack_port_t *midi = nullptr, *left = nullptr, *right = nullptr;
  std::size_t next = 0;
  unsigned frame = 0;
  std::array<float, 2> peaks{};
  std::atomic<unsigned> xruns{0};
  std::atomic<bool> armed{false}, done{false}, failed{false};

  static int process(const jack_nframes_t frames, void *opaque) noexcept [[clang::nonblocking]] {
    auto &self = *static_cast<Replay *>(opaque);
    auto *output = jack_port_get_buffer(self.midi, frames);
    jack_midi_clear_buffer(output);
    if (!self.armed.load(std::memory_order_acquire) || self.done.load(std::memory_order_relaxed)) {
      return 0;
    }
    const auto end = self.frame + frames;
    while (self.next < self.score.events.size() && self.score.events[self.next].frame < end) {
      const auto &event = self.score.events[self.next++];
      if (jack_midi_event_write(
              output, event.frame - self.frame, event.bytes.data(), event.bytes.size()) != 0) {
        self.failed.store(true, std::memory_order_relaxed);
        self.done.store(true, std::memory_order_release);
        return 0;
      }
    }
    const std::span left(static_cast<const float *>(jack_port_get_buffer(self.left, frames)),
                         frames);
    const std::span right(static_cast<const float *>(jack_port_get_buffer(self.right, frames)),
                          frames);
    for (jack_nframes_t i = 0; i < frames; ++i) {
      self.peaks[0] = std::max(self.peaks[0], std::abs(left[i]));
      self.peaks[1] = std::max(self.peaks[1], std::abs(right[i]));
    }
    self.frame = end;
    if (end >= self.score.end && self.next == self.score.events.size()) {
      self.done.store(true, std::memory_order_release);
    }
    return 0;
  }

  static int xrun(void *opaque) noexcept {
    auto &self = *static_cast<Replay *>(opaque);
    if (self.armed.load(std::memory_order_acquire) && !self.done.load(std::memory_order_acquire)) {
      self.xruns.fetch_add(1, std::memory_order_relaxed);
    }
    return 0;
  }
};
}

int main(int argc, char **argv) try {
  if (argc != 2) { throw std::runtime_error("Usage: jack-nachtmusik EVENTS|SCORE.mid"); }
  const std::filesystem::path path(argv[1]);
  Replay replay{.score = path.extension() == ".mid" ? load_smf(path) : load_fixture(path)};
  const std::unique_ptr<jack_client_t, decltype(&jack_client_close)> client(
      jack_client_open("blofeld-qualification", JackNoStartServer, nullptr), jack_client_close);
  if (!client || jack_get_sample_rate(client.get()) != rate ||
      jack_get_buffer_size(client.get()) != period) {
    throw std::runtime_error("Expected JACK at 44.1 kHz with a 64-frame period");
  }
  replay.midi =
      jack_port_register(client.get(), "midi", JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0);
  replay.left =
      jack_port_register(client.get(), "left", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
  replay.right =
      jack_port_register(client.get(), "right", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
  if (replay.midi == nullptr || replay.left == nullptr || replay.right == nullptr ||
      jack_set_process_callback(client.get(), Replay::process, &replay) != 0 ||
      jack_set_xrun_callback(client.get(), Replay::xrun, &replay) != 0 ||
      jack_activate(client.get()) != 0 ||
      jack_connect(client.get(), jack_port_name(replay.midi), "blofeld-emu:midi_in") != 0 ||
      jack_connect(client.get(), "blofeld-emu:audio_l", jack_port_name(replay.left)) != 0 ||
      jack_connect(client.get(), "blofeld-emu:audio_r", jack_port_name(replay.right)) != 0) {
    throw std::runtime_error("Cannot connect the JACK replay client to blofeld-emu");
  }
  const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(40);
  while (jack_port_connected(replay.midi) == 0 && std::chrono::steady_clock::now() < timeout) {
    std::this_thread::sleep_for(poll_interval);
  }
  replay.armed.store(true, std::memory_order_release);
  while (!replay.done.load(std::memory_order_acquire) && jack_port_connected(replay.midi) != 0 &&
         std::chrono::steady_clock::now() < timeout) {
    std::this_thread::sleep_for(poll_interval);
  }
  if (jack_deactivate(client.get()) != 0) {
    throw std::runtime_error("Cannot deactivate JACK client");
  }
  std::cout << "MIDI events=" << replay.next << '/' << replay.score.events.size()
            << " peaks=" << replay.peaks[0] << '/' << replay.peaks[1]
            << " JACK xruns=" << replay.xruns.load() << '\n';
  return replay.done.load() && !replay.failed.load() && replay.next == replay.score.events.size() &&
                 replay.peaks[0] > 0.0F && replay.peaks[1] > 0.0F && replay.xruns.load() == 0
             ? 0
             : 1;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
