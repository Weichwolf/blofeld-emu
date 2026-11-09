// SPDX-License-Identifier: GPL-3.0-only
#include "audio/render_position.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace {
using blofeld::host::audio::RenderPosition;
using blofeld::host::audio::RenderPositionStore;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void epochs() {
  RenderPositionStore clock;
  require(!clock.read(), "No clock exists until real frames have been submitted");
  const RenderPosition first{.epoch = 0, .first_frame = 0, .host_ns = 1, .frames = 29};
  require(clock.publish(first) && clock.read() == first,
          "Frame zero and epoch zero are valid observations");
  require(clock.clear() && !clock.read(), "Reset revokes the previous audio clock");
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  const RenderPosition next{
      .epoch = maximum, .first_frame = maximum - 1, .host_ns = maximum, .frames = 1};
  require(clock.publish(next) && clock.read() == next,
          "Publication preserves complete integer positions without narrowing");
  require(clock.clear() && clock.clear() && !clock.read(),
          "Repeated quiescent resets cannot resurrect an old observation");
}

RenderPosition point(const std::uint64_t index) {
  constexpr std::uint64_t epoch_frames = 1024;
  return {.epoch = index / epoch_frames,
          .first_frame = index * 3,
          .host_ns = index * 7,
          .frames = static_cast<std::uint32_t>(index % 16) + 1,
          .uncertainty_ns = index * 5};
}

void concurrent() {
  RenderPositionStore clock;
  std::atomic<bool> done{false};
  constexpr std::uint64_t publications = 20000;
  std::jthread writer([&] {
    for (std::uint64_t index = 1; index <= publications; ++index) {
      if (!clock.clear() || !clock.publish(point(index))) { std::terminate(); }
    }
    done.store(true, std::memory_order_release);
  });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  std::uint64_t observed = 0;
  do {
    if (const auto position = clock.read()) {
      const auto index = position->first_frame / 3;
      require(index >= observed && index <= publications && *position == point(index),
              "Readers receive complete ordered observations across clears and epochs");
      observed = index;
    }
    require(std::chrono::steady_clock::now() < deadline,
            "A contended clock read stays bounded while its writer progresses");
  } while (!done.load(std::memory_order_acquire));
  writer.join();
  require(clock.read() == point(publications),
          "A quiescent reader obtains the final publication after contention");
}
}

int main() try {
  epochs();
  concurrent();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
