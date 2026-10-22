// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/host_audio.hpp"
#include "blofeld/pcm_queue.hpp"
#include "blofeld/serial_pcm.hpp"
#include "host_audio_fixture.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <thread>

namespace {
constexpr blofeld::HostAudioClock clock{.timebase_hz = 3072000000, .sample_rate = 48000};

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw blofeld::Fault(std::string(message)); }
}

void reject(const auto &operation, const std::string_view expected) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) {
    require(std::string_view(fault.what()).contains(expected), "Unexpected host audio diagnostic");
    return;
  }
  throw blofeld::Fault("Invalid host audio operation succeeded");
}

const blofeld::HostAudioBoundary &boundary() {
  static const blofeld::HostAudioBoundary value{.owner = std::make_shared<int>(0)};
  return value;
}

void pump_until(blofeld::HostAudio &audio, const auto &condition) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!condition()) {
    require(std::chrono::steady_clock::now() < deadline, "Asynchronous audio progress timed out");
    audio.poll(boundary());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void finish(blofeld::HostAudio &audio) {
  audio.finish();
  pump_until(audio, [&] { return audio.finished(); });
}

void queue_checks() {
  blofeld::PcmQueue<3> queue;
  const blofeld::SerialPcmFrame first{.first_tick = 1};
  auto second = first;
  second.first_tick = 2;
  auto third = first;
  third.first_tick = 3;
  require(queue.peek() == nullptr && !queue.pop() && queue.available() == 3,
          "An empty queue cannot expose or remove a manufactured frame");
  require(queue.push(first) && queue.push(second) && queue.push(third), "Fill bounded FIFO");
  const auto saved = queue;
  require(!queue.push(first) && !queue.pop(4) && queue == saved,
          "Rejected overflow/pop preserves every frame and cursor");
  require(queue.pop() && queue.push(first) && queue.peek()->first_tick == 2 &&
              queue.peek(1)->first_tick == 3 && queue.peek(2)->first_tick == 1 &&
              queue.peek(3) == nullptr,
          "FIFO wraps without reordering or leaking a stale slot");
  queue = saved;
  require(queue.peek()->first_tick == 1 && queue.pop(3) && queue.peek() == nullptr,
          "A value snapshot restores the full frame/cursor prefix");
}

void boundary_guards(const blofeld::HostAudioTarget &target) {
  const auto native = blofeld::test::host_audio_native_frame(false);
  for (unsigned problem = 0; problem < 4; ++problem) {
    blofeld::HostAudio audio({}, clock, target);
    auto current = boundary();
    current.ticks = 1;
    audio.poll(current);
    audio.write(std::span{&native, 1});
    switch (problem) {
      case 0: current.owner = std::make_shared<int>(1); break;
      case 1: ++current.restore_revision; break;
      case 2: current.ticks = 0; break;
      case 3: current.owner.reset(); break;
      default: throw blofeld::Fault("Unexpected transport boundary case");
    }
    reject([&] { audio.poll(current); }, "board owner/restore/time changed");
    require(audio.faulted() && audio.queued_frames() == 1 && audio.frames_written() == 0,
            "Changed admission retires queued output before any further host callback/write");
  }
}

void invalid_configuration(const blofeld::HostAudioTarget &unavailable) {
  reject(
      [&] {
        blofeld::HostAudio audio({}, clock, unavailable);
        pump_until(audio, [&] { return audio.ready(); });
      },
      "Host audio connect");
  for (const unsigned channels : {0U, 1U, 3U}) {
    reject([&] { const blofeld::HostAudio audio({.channels = channels}, clock, unavailable); },
           "PCM");
  }
  reject([&] { const blofeld::HostAudio audio({.word_bits = 16}, clock, unavailable); }, "24-bit");
  reject(
      [&] { const blofeld::HostAudio audio({}, {.sample_rate = clock.sample_rate}, unavailable); },
      "clock");
  reject([&] { const blofeld::HostAudio audio({}, {.timebase_hz = 1}, unavailable); }, "clock");
  reject(
      [&] {
        const blofeld::HostAudio audio(
            {},
            {.timebase_hz = clock.timebase_hz + 1, .sample_rate = clock.sample_rate},
            unavailable);
      },
      "clock");
  reject(
      [&] {
        const blofeld::HostAudio audio(
            {}, clock, {.server = {}, .sink = std::string("bad\0sink", 8)});
      },
      "NUL");
}

using Block = std::array<blofeld::SerialPcmFrame, blofeld::HostAudio::maximum_block_frames>;

Block block(const blofeld::SerialPcmFrame &native, std::uint64_t &next) {
  constexpr unsigned duration = 63000;
  constexpr unsigned period = 64000;
  Block result;
  for (auto &frame : result) {
    frame = native;
    frame.first_tick = next;
    frame.last_tick = next + duration;
    next += period;
  }
  return result;
}

void invalid_blocks(blofeld::HostAudio &audio, const Block &valid) {
  reject([&] { audio.write({}); }, "block size");
  std::array<blofeld::SerialPcmFrame, blofeld::HostAudio::maximum_block_frames + 1> oversized{};
  reject([&] { audio.write(oversized); }, "block size");
  constexpr std::int32_t below_minimum = -8388609;
  for (unsigned problem = 0; problem < 8; ++problem) {
    auto invalid = valid;
    auto &last = invalid.back();
    switch (problem) {
      case 0: last.channels = 1; break;
      case 1: last.bit_ticks.reset(); break;
      case 2: last.uniform_clock = false; break;
      case 3: --last.last_tick; break;
      case 4: ++last.first_tick; break;
      case 5: last.last_tick = 0; break;
      case 6: last.samples[0] = 8388608; break;
      case 7: last.samples[1] = below_minimum; break;
      default: throw blofeld::Fault("Unexpected invalid-frame case");
    }
    reject([&] { audio.write(invalid); }, problem < 6 ? "shape/timing" : "precision");
    require(audio.frames_written() == 0 && !audio.faulted(),
            "Whole-block validation must precede all output and retain a usable transport");
  }
}

void playback(const blofeld::HostAudioTarget &target, const bool secondary) {
  const auto native = blofeld::test::host_audio_native_frame(secondary);
  blofeld::HostAudio audio({}, clock, target);
  std::uint64_t next = native.first_tick;
  auto first = block(native, next);
  invalid_blocks(audio, first);
  audio.write(first);
  reject([&] { audio.write(first); }, "shape/timing");
  constexpr unsigned blocks = 16;
  for (unsigned index = 1; index < blocks; ++index) {
    const auto current = block(native, next);
    audio.write(current);
  }
  const auto extra = block(native, next);
  reject([&] { audio.write(extra); }, "queue capacity");
  require(audio.frames_written() == 0 && audio.queued_frames() == 4096,
          "Enqueueing never performs host I/O; a full queue retains its accepted prefix");
  finish(audio);
  require(audio.frames_written() == blocks * first.size() && !audio.faulted(),
          "Only complete accepted blocks count after successful server drain");
  std::cout << "PLAYED " << secondary << ' ' << audio.frames_written() << '\n' << std::flush;
}

void boundary_samples() {
  const auto native = blofeld::test::host_audio_native_frame(false);
  blofeld::HostAudio audio({}, clock);
  std::uint64_t next = native.first_tick;
  auto frames = block(native, next);
  constexpr std::int32_t minimum = -8388608;
  constexpr std::int32_t maximum = 8388607;
  constexpr std::array<std::array<std::int32_t, 2>, 4> pairs{
      {{minimum, maximum}, {1, -1}, {0, 0}, {maximum, minimum}}};
  for (unsigned i = 0; i < pairs.size(); ++i) {
    frames.at(i).samples[0] = pairs.at(i)[0];
    frames.at(i).samples[1] = pairs.at(i)[1];
  }
  audio.write(std::span{frames}.first(pairs.size()));
  finish(audio);
  require(audio.frames_written() == pairs.size(), "All precision boundary frames were accepted");
  std::cout << "EDGES 4\n" << std::flush;
}

void disconnected(const blofeld::HostAudioTarget &target) {
  const auto native = blofeld::test::host_audio_native_frame(false);
  blofeld::HostAudio audio({}, clock, target);
  std::uint64_t next = native.first_tick;
  const auto first = block(native, next);
  audio.write(first);
  pump_until(audio, [&] { return audio.ready(); });
  std::cout << "CONNECTED\n" << std::flush;
  if (std::cin.get() != 'x') { throw blofeld::Fault("Missing private-server disconnect signal"); }
  reject([&] { pump_until(audio, [] { return false; }); }, "Host audio connection:");
  require(audio.faulted(), "A real server disconnect must poison the transport");
  reject([&] { audio.write(first); }, "stream is faulted");
  reject([&] { audio.poll(boundary()); }, "stream is faulted");
  std::cout << "DISCONNECTED\n" << std::flush;
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3 && count != 4) {
      throw blofeld::Fault("Usage: host-audio-test private-server unavailable-server");
    }
    const blofeld::HostAudioTarget target{.server = arguments[1], .sink = {}};
    if (count == 4) {
      require(std::string_view(arguments[3]) == "--disconnect", "Unexpected test mode");
      disconnected(target);
      return 0;
    }
    invalid_configuration({.server = arguments[2], .sink = {}});
    queue_checks();
    boundary_guards(target);
    playback(target, false);
    playback(target, true);
    boundary_samples();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
