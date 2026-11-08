// SPDX-License-Identifier: GPL-3.0-only
#include "audio/buffer.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
using blofeld::host::audio::Buffer;
using blofeld::host::audio::Frame;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void boundaries() {
  Buffer queue;
  std::array<Frame, Buffer::capacity> input{};
  for (unsigned index = 0; index < input.size(); ++index) {
    input[index] = {static_cast<std::int32_t>(index), -static_cast<std::int32_t>(index)};
  }
  require(queue.produce(input) == input.size() && queue.produce(input) == 0 &&
              queue.queued() == input.size() && !queue.consume(input.size() + 1),
          "The queue is bounded and invalid consumer ownership cannot advance it");
  const auto first = queue.readable();
  for (unsigned index = 0; index < first.size(); ++index) {
    require(first[index][0] == static_cast<std::int32_t>(index) &&
                first[index][1] == -static_cast<std::int32_t>(index),
            "Both signed channels retain exact order");
  }
  constexpr unsigned prefix = 17;
  require(queue.consume(prefix) && queue.produce(std::span(input).first(prefix)) == prefix &&
              queue.consume(Buffer::capacity - prefix),
          "Wrap producer and consumer independently");
  const auto wrapped = queue.readable();
  require(wrapped.size() == prefix && wrapped[0] == input[0], "Wrapped frames remain intact");
  queue.discard();
  require(queue.readable().empty(), "Explicit reset discards only queued frames");
  const auto filled = queue.produce(input);
  require(filled + queue.produce(std::span(input).subspan(filled)) == input.size(),
          "Fill both contiguous portions before cancellation");
  bool accepted = true;
  std::jthread producer([&](const std::stop_token &stop) { accepted = queue.wait_space(stop); });
  producer.request_stop();
  producer.join();
  require(!accepted, "Stop wakes a producer waiting on full capacity");
  queue.close();
  require(!queue.wait_space({}) && queue.produce(input) == 0,
          "Closed queues cannot accept new data");
}

void concurrent() {
  Buffer queue;
  constexpr unsigned count = 4096;
  constexpr unsigned batch = 17;
  std::atomic<bool> failed{false};
  std::jthread producer([&](const std::stop_token &stop) {
    for (unsigned offset = 0; offset < count;) {
      std::array<Frame, batch> input{};
      const auto remaining =
          std::min<unsigned>(static_cast<unsigned>(input.size()), count - offset);
      for (unsigned index = 0; index < remaining; ++index) {
        const auto value = static_cast<std::int32_t>(offset + index);
        input[index] = {value, -value};
      }
      const auto written = queue.produce(std::span(input).first(remaining));
      offset += static_cast<unsigned>(written);
      if (written == 0 && !queue.wait_space(stop)) {
        failed.store(true);
        break;
      }
    }
  });
  for (unsigned offset = 0; offset < count;) {
    const auto frames = queue.readable();
    if (frames.empty()) {
      std::this_thread::yield();
      continue;
    }
    for (unsigned index = 0; index < frames.size(); ++index) {
      const auto expected = static_cast<std::int32_t>(offset + index);
      if (frames[index] != Frame{expected, -expected}) { failed.store(true); }
    }
    if (!queue.consume(frames.size())) { failed.store(true); }
    offset += static_cast<unsigned>(frames.size());
  }
  producer.join();
  require(!failed.load() && queue.queued() == 0,
          "Distinct owners retain every frame across wraps/backpressure");
}
}

int main() try {
  boundaries();
  concurrent();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
