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
  std::atomic<std::uint64_t> data{0};
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
      ordered = ordered && *limit == (expected + 1) * spacing && data.load() == expected;
      now = *limit;
    }
    done.store(true);
    done.notify_one();
  });
  for (std::uint64_t value = 1; value <= count; ++value) {
    require(time.enter_mcu(value * spacing, {}), "Admit a real shared-register transaction");
    require(data.load() == value - 1, "The previous phase remains until its producer changes it");
    data.store(value);
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
  const std::stop_source stop;
  stop.request_stop();
  require(!time.enter_mcu(reset_time, stop.get_token()),
          "Cancellation never grants a shared access or advances the source");
}
}

int main() try {
  ordered_transactions();
  reset_and_cancellation();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
