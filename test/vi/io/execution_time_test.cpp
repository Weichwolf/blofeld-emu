// SPDX-License-Identifier: GPL-3.0-only
#include "io/execution_time.hpp"

#include <atomic>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
using blofeld::vi::ExecutionTime;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void ordered_transactions() {
  ExecutionTime time;
  std::uint64_t data = 0;
  std::atomic<bool> done{false};
  bool ordered = true;
  constexpr std::uint64_t count = 64;
  constexpr std::uint64_t spacing = 100;
  std::jthread dsp([&](const std::stop_token &stop) {
    std::uint64_t now = 0;
    for (std::uint64_t expected = 0; expected <= count; ++expected) {
      const auto limit = time.dsp_limit(now, stop);
      if (!limit) {
        ordered = false;
        break;
      }
      ordered = ordered && *limit == (expected + 1) * spacing && data == expected;
      now = *limit;
    }
    done.store(true);
    done.notify_one();
  });
  for (std::uint64_t value = 1; value <= count; ++value) {
    require(time.enter_mcu(value * spacing, {}), "Admit a real shared-register transaction");
    require(data == value - 1, "The previous phase remains until its producer changes it");
    data = value;
    time.leave_mcu();
  }
  require(time.publish_mcu((count + 1) * spacing), "Publish completed local work without waiting");
  done.wait(false);
  dsp.join();
  require(ordered, "Native grants observe every committed phase in device-time order");
  require(!time.publish_mcu(0), "A published device frontier cannot move backwards");
}

void reset_and_cancellation() {
  ExecutionTime time;
  time.hold_dsp();
  constexpr std::uint64_t reset_time = 120;
  require(time.enter_mcu(reset_time, {}), "A held DSP cannot block MCU reset setup");
  time.leave_mcu();
  require(!time.dsp_limit(0, {}), "A reset interrupts a DSP grant wait");
  time.release_dsp(reset_time);
  require(time.dsp_origin() == reset_time, "A fresh DSP epoch uses the MCU reset-release time");
  bool cancelled = false;
  std::jthread dsp(
      [&](const std::stop_token &stop) { cancelled = !time.dsp_limit(reset_time, stop); });
  dsp.request_stop();
  dsp.join();
  require(cancelled, "Shutdown cancels a DSP waiting for a future device event");
  // libc++ mutates the source on request_stop(); libstdc++ accepts a const source.
#if defined(_LIBCPP_VERSION)
  std::stop_source stop;
#else
  const std::stop_source stop;
#endif
  stop.request_stop();
  require(!time.enter_mcu(reset_time, stop.get_token()),
          "Cancellation never grants a shared access or advances the source");
}

void publication_wakeups() {
  ExecutionTime time;
  constexpr std::uint64_t count = 512;
  constexpr std::uint64_t spacing = 100;
  std::atomic<std::uint64_t> acknowledged{0};
  bool ordered = true;
  std::jthread dsp([&](const std::stop_token &stop) {
    std::uint64_t now = 0;
    for (std::uint64_t expected = 1; expected <= count; ++expected) {
      const auto next = time.dsp_limit(now, stop);
      if (!next) {
        ordered = false;
        return;
      }
      ordered = ordered && *next == expected * spacing;
      now = *next;
      acknowledged.store(expected, std::memory_order_release);
      acknowledged.notify_one();
    }
  });
  for (std::uint64_t value = 1; value <= count; ++value) {
    require(time.publish_mcu(value * spacing) && time.publish_mcu(value * spacing),
            "A new frontier and its repeated publication retain actual device time");
    auto observed = acknowledged.load(std::memory_order_acquire);
    while (observed < value) {
      acknowledged.wait(observed, std::memory_order_acquire);
      observed = acknowledged.load(std::memory_order_acquire);
    }
  }
  dsp.join();
  require(ordered && acknowledged.load() == count,
          "Publication racing wait registration cannot lose a wakeup or reorder a grant");
  require(!time.publish_mcu(0), "Rejected backwards progress retains the final frontier");
}

void repeated_timestamp_transactions() {
  ExecutionTime time;
  constexpr std::uint64_t timestamp = 100;
  constexpr std::uint64_t count = 1024;
  bool completed = false;
  std::jthread dsp([&](const std::stop_token &stop) {
    completed = time.dsp_limit(timestamp, stop) == timestamp + 1;
  });
  for (std::uint64_t index = 0; index < count; ++index) {
    require(time.enter_mcu(timestamp, {}), "Each same-time transaction receives its own grant");
    require(!time.enter_mcu(timestamp, {}), "An outstanding transaction cannot be entered twice");
    time.leave_mcu();
  }
  require(time.publish_mcu(timestamp + 1), "Only genuine device progress advances the frontier");
  dsp.join();
  require(completed, "Reused timestamps cannot hide a request or reuse a stale grant");
  bool backwards = false;
  try {
    static_cast<void>(time.dsp_limit(timestamp - 1, {}));
  } catch (const std::logic_error &) { backwards = true; }
  require(backwards, "Atomic grants still reject backwards DSP progress");
}

void cancelled_transactions() {
  constexpr std::uint64_t timestamp = 100;
  ExecutionTime time;
  bool accepted = true;
  std::jthread mcu(
      [&](const std::stop_token &stop) { accepted = time.enter_mcu(timestamp, stop); });
  require(time.dsp_limit(0, {}) == timestamp,
          "The DSP sees publication while the MCU awaits actual boundary admission");
  mcu.request_stop();
  mcu.join();
  require(!accepted, "Cancellation withdraws an ungranted transaction");
  time.hold_dsp();
  require(time.enter_mcu(timestamp, {}), "Cancellation releases ownership for reset setup");
  time.leave_mcu();
  time.release_dsp(timestamp);

  bool cancelled = false;
  std::jthread dsp(
      [&](const std::stop_token &stop) { cancelled = !time.dsp_limit(timestamp, stop); });
  require(time.enter_mcu(timestamp, {}), "A real grant precedes cancellation of its DSP waiter");
  dsp.request_stop();
  dsp.join();
  require(cancelled, "Shutdown wakes a DSP even while the MCU still holds its granted access");
  time.leave_mcu();
}
}

int main() try {
  ordered_transactions();
  reset_and_cancellation();
  publication_wakeups();
  repeated_timestamp_transactions();
  cancelled_transactions();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
