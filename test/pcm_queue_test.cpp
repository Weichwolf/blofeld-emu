// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_queue.hpp"
#include "audio/serial_pcm.hpp"
#include "serial_pcm_fixture.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stop_token>
#include <thread>

namespace blofeld::test::pcm {
namespace {
SerialPcmFrame numbered(const std::uint64_t index) {
  constexpr std::uint64_t period = 1000;
  constexpr std::uint64_t duration = 990;
  constexpr unsigned sample_cycle = 257;
  constexpr std::int32_t sample_bias = 128;
  SerialPcmFrame frame{
      .channels = 2, .first_tick = index * period, .last_tick = (index * period) + duration};
  frame.samples[0] = static_cast<std::int32_t>(index % sample_cycle) - sample_bias;
  frame.samples[1] = -frame.samples[0];
  return frame;
}

void literal_capacity_and_wrap() {
  constexpr unsigned duration = 990;
  constexpr unsigned second_tick = 1000;
  constexpr unsigned third_tick = 2000;
  constexpr std::int32_t second_word = -127;
  constexpr std::int32_t third_word = -126;
  PcmQueue queue(2);
  static_assert(noexcept(queue.try_push(SerialPcmFrame{})));
  SerialPcmFrame received{};
  static_assert(noexcept(queue.try_pop(received)));
  auto first = numbered(0);
  first.samples.back() = 1;
  require(queue.try_push(first) && queue.try_push(numbered(1)) && !queue.try_push(numbered(2)),
          "Capacity two admits exactly two complete frames and retains a rejected third");
  require(queue.try_pop(received) && received.samples[0] == -128 && received.samples[1] == 128 &&
              received.samples.back() == 1 && received.first_tick == 0 &&
              received.last_tick == duration && !received.bit_ticks,
          "Literal whole-frame fields, including unused channel storage, survive publication");
  require(queue.try_push(numbered(2)), "The freed slot is immediately reusable after wrap");
  require(queue.try_pop(received) && received.samples[0] == second_word &&
              received.first_tick == second_tick,
          "Wrapping cannot overwrite the second queued frame");
  require(queue.try_pop(received) && received.samples[0] == third_word &&
              received.first_tick == third_tick,
          "The third frame follows the preserved second frame");
  const auto saved = received;
  require(!queue.try_pop(received) && received == saved,
          "An empty read leaves the caller's last complete frame intact");
  for (const auto invalid : {std::size_t{0}, PcmQueue::maximum_capacity + 1}) {
    (void)rejects([&] { const PcmQueue bad(invalid); });
  }
}

void concurrent_fifo(const std::size_t capacity) {
  constexpr std::uint64_t frames = 100000;
  constexpr std::uint64_t period = 1000;
  constexpr std::uint64_t duration = 990;
  constexpr unsigned sample_cycle = 257;
  constexpr std::int32_t sample_bias = 128;
  PcmQueue queue(capacity);
  std::atomic<bool> mismatch{false};
  std::jthread consumer([&](const std::stop_token &stop) {
    SerialPcmFrame frame;
    for (std::uint64_t index = 0; index < frames && !stop.stop_requested();) {
      if (!queue.try_pop(frame)) {
        std::this_thread::yield();
        continue;
      }
      const auto expected = static_cast<std::int32_t>(index % sample_cycle) - sample_bias;
      if (frame.samples[0] != expected || frame.samples[1] != -expected ||
          frame.first_tick != index * period || frame.last_tick != (index * period) + duration ||
          frame.channels != 2) {
        mismatch.store(true, std::memory_order_relaxed);
        return;
      }
      ++index;
    }
  });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{10};
  for (std::uint64_t index = 0; index < frames;) {
    require(!mismatch.load(std::memory_order_relaxed),
            "Concurrent consumption must retain independent frame order and literal fields");
    require(std::chrono::steady_clock::now() < deadline, "Concurrent FIFO must make progress");
    if (queue.try_push(numbered(index))) {
      ++index;
    } else {
      std::this_thread::yield();
    }
  }
  consumer.join();
  require(!mismatch.load(std::memory_order_relaxed),
          "The final concurrently consumed frame matches");
}
}
}

int main() {
  try {
    using namespace blofeld::test::pcm;
    literal_capacity_and_wrap();
    constexpr std::array<std::size_t, 3> capacities{1, 7, 256};
    for (const auto capacity : capacities) { concurrent_fifo(capacity); }
    std::cout
        << "Bounded SPSC PCM: literal FIFO, capacity, wrap and300000 concurrent frames passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
