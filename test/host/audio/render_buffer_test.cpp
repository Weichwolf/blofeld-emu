// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_frame.hpp"
#include "audio/render_buffer.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
using blofeld::host::audio::FloatFrame;
using blofeld::host::audio::Frame;
using blofeld::host::audio::RenderBuffer;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void boundaries() {
  bool rejected = false;
  try {
    const RenderBuffer empty(0);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "A render buffer requires a bounded nonempty grant");
  RenderBuffer queue(5);
  constexpr std::int32_t maximum = 8388607;
  constexpr std::int32_t minimum = -8388608;
  constexpr std::array input{
      Frame{maximum, minimum}, Frame{0, 1}, Frame{-1, 0}, Frame{1, -1}, Frame{minimum, maximum}};
  require(queue.produce(input) == input.size() && queue.produce(input) == 0,
          "A full render queue retains exact capacity");
  const auto unit = std::ldexp(1.0F, -23);
  std::array<FloatFrame, 3> prefix{};
  require(queue.render(prefix) == prefix.size() &&
              prefix == std::array{FloatFrame{1.0F - unit, -1.0F},
                                   FloatFrame{0, unit},
                                   FloatFrame{-unit, 0}},
          "Both 24-bit extrema and unit values convert exactly to binary float");
  require(queue.produce(std::span(input).first(3)) == 3, "Released capacity wraps independently");
  std::array<FloatFrame, 7> output{};
  output.fill({1, 1});
  require(queue.render(output) == queue.capacity() && queue.queued() == 0 &&
              output == std::array{FloatFrame{unit, -unit},
                                   FloatFrame{-1.0F, 1.0F - unit},
                                   FloatFrame{1.0F - unit, -1.0F},
                                   FloatFrame{0, unit},
                                   FloatFrame{-unit, 0},
                                   FloatFrame{},
                                   FloatFrame{}},
          "A callback spans wrap, preserves ordering and reports unprovided zero padding");
  require(queue.produce(input) != 0, "A drained queue remains reusable");
  queue.discard();
  require(queue.queued() == 0 && queue.render(output) == 0,
          "A quiescent epoch reset cannot replay old frames");
  for (const auto &frame : output) { require(frame == FloatFrame{}, "Empty render is silent"); }
  require(queue.produce(input) != 0, "Planar rendering reads the same original PCM");
  std::array<float, 3> left{};
  std::array<float, 3> right{};
  require(queue.render(left, right) == left.size() &&
              left == std::array{1.0F - unit, 0.0F, -unit} &&
              right == std::array{-1.0F, unit, 0.0F},
          "JACK channel buffers preserve the exact 24-bit values and channel order");
}

void underrun_recovery() {
  RenderBuffer queue(8);
  constexpr std::int32_t unit = 1024;
  constexpr float sample = 1.0F / 8192.0F;
  constexpr std::array initial{Frame{unit, -unit}, Frame{2 * unit, -2 * unit}};
  std::array<FloatFrame, 3> output{};
  std::size_t late = 0;
  require(queue.produce(initial) == initial.size() && queue.render(output, &late) == 2 &&
              late == 1 &&
              output == std::array{FloatFrame{sample, -sample},
                                   FloatFrame{2 * sample, -2 * sample},
                                   FloatFrame{}},
          "An underrun silences the missing frame and preserves the submitted prefix");
  constexpr std::array resumed{
      Frame{3 * unit, -3 * unit}, Frame{4 * unit, -4 * unit}, Frame{5 * unit, -5 * unit}};
  std::array<float, 2> left{};
  std::array<float, 2> right{};
  require(queue.produce(resumed) == resumed.size() && queue.render(left, right, &late) == 2 &&
              late == 0 && left == std::array{4 * sample, 5 * sample} &&
              right == std::array{-4 * sample, -5 * sample},
          "The next callback skips expired PCM and resumes on the hardware timeline");
  late = 4;
  require(queue.produce(initial) == initial.size() && queue.render(left, right, &late) == 0 &&
              late == 4 && left == std::array<float, 2>{} && right == std::array<float, 2>{},
          "A skipped host cycle can consume a complete queue without replaying expired PCM");
}

void concurrent() {
  constexpr std::size_t count = 4096;
  constexpr std::size_t capacity = 31;
  RenderBuffer queue(capacity);
  std::atomic<bool> failed{false};
  std::jthread producer([&](const std::stop_token &stop) {
    for (std::size_t offset = 0; offset < count && !stop.stop_requested();) {
      const auto value = static_cast<std::int32_t>(offset);
      const std::array<Frame, 1> input{Frame{value, -value}};
      if (queue.produce(input) == 1) {
        ++offset;
      } else {
        std::this_thread::yield();
      }
    }
  });
  for (std::size_t offset = 0; offset < count;) {
    std::array<FloatFrame, 7> output{};
    const auto frames = queue.render(output);
    for (std::size_t index = 0; index < frames; ++index) {
      const auto expected = std::ldexp(static_cast<float>(offset + index), -23);
      if (output[index] != FloatFrame{expected, -expected}) { failed.store(true); }
    }
    offset += frames;
    if (frames == 0) { std::this_thread::yield(); }
  }
  producer.join();
  require(!failed.load(), "Concurrent render owns every original stereo frame exactly once");
}
}

int main() try {
  boundaries();
  underrun_recovery();
  concurrent();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
