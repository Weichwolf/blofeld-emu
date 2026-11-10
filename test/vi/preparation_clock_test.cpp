// SPDX-License-Identifier: GPL-3.0-only
#include "preparation_clock.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <semaphore>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
using blofeld::vi::PreparationClock;
using Clock = PreparationClock::Clock;
constexpr std::chrono::nanoseconds step{10};
std::atomic<std::int64_t> ticks{0};

Clock::time_point now() noexcept {
  return Clock::time_point(std::chrono::nanoseconds(ticks.load(std::memory_order_relaxed)));
}

void require(const bool good, const char *message) {
  if (!good) { throw std::runtime_error(message); }
}

void advance(const int steps) {
  ticks.store((step * steps).count());
}

void intervals() {
  advance(0);
  PreparationClock clock(now);
  advance(1);
  require(clock.elapsed() == step * 1,
          "Ordinary startup time consumes the unchanged execution budget");
  {
    const PreparationClock::Scope mcu(&clock);
    advance(2);
    require(clock.elapsed() == step * 1, "An active MCU compiler freezes the execution clock");
    {
      const PreparationClock::Scope dsp(&clock);
      advance(3);
      require(clock.elapsed() == step * 1, "Overlapping compiler scopes exclude their union once");
    }
    advance(4);
    require(clock.elapsed() == step * 1,
            "Ending one compiler does not resume a still preparing owner");
  }
  advance(5);
  require(clock.elapsed() == step * 2,
          "50 wall ticks minus the40-10 compiler union leaves20 execution ticks");
  {
    const PreparationClock::Scope detached(nullptr);
    advance(6);
  }
  require(clock.elapsed() == step * 3,
          "Detached qualification owners do not alter the execution clock");
  bool caught = false;
  try {
    const PreparationClock::Scope compiling(&clock);
    advance(7);
    throw std::runtime_error("Authored compiler failure");
  } catch (const std::runtime_error &) { caught = true; }
  require(caught, "The authored compiler exception must propagate through its scope");
  advance(8);
  require(clock.elapsed() == step * 4,
          "Compiler exceptions close their exclusion scope during unwinding");
}

struct Owner {
  std::binary_semaphore entered{0}, leave{0}, ended{0};
};

void startup_budget() {
  advance(0);
  PreparationClock clock(now);
  const auto set = [](const Clock::duration value) {
    ticks.store(std::chrono::duration_cast<std::chrono::nanoseconds>(value).count());
  };
  constexpr std::chrono::seconds initial_execution{10};
  constexpr std::chrono::hours long_compilation{24};
  constexpr std::chrono::seconds before_deadline{49};
  constexpr std::chrono::seconds at_deadline{50};
  constexpr std::chrono::minutes budget{1};
  set(initial_execution);
  {
    const PreparationClock::Scope compiling(&clock);
    set(long_compilation);
    require(clock.elapsed() == initial_execution,
            "Long initial AOT cannot consume the firmware startup budget");
  }
  set(long_compilation + before_deadline);
  require(clock.elapsed() < budget,
          "Only49 additional execution seconds leave the original deadline open");
  set(long_compilation + at_deadline);
  require(clock.elapsed() == budget,
          "The unchanged minute deadline still expires after60 execution seconds");
}

bool acquire(std::binary_semaphore &gate) {
  constexpr std::chrono::seconds guard{2};
  return gate.try_acquire_for(guard);
}

void overlapping_owners() {
  advance(0);
  PreparationClock clock(now);
  Owner mcu;
  Owner dsp;
  std::atomic<bool> valid{true};
  std::atomic<bool> complete{false};
  const auto compile = [&](Owner &owner) {
    {
      const PreparationClock::Scope phase(&clock);
      owner.entered.release();
      if (!acquire(owner.leave)) { valid.store(false); }
    }
    owner.ended.release();
  };
  advance(1);
  std::jthread mcu_thread([&] { compile(mcu); });
  require(acquire(mcu.entered), "MCU compiler must enter before the test guard");
  std::jthread observer([&](const std::stop_token &stop) {
    Clock::duration previous{};
    while (!complete.load() && !stop.stop_requested()) {
      const auto elapsed = clock.elapsed();
      if (elapsed < previous) { valid.store(false); }
      previous = elapsed;
      std::this_thread::yield();
    }
  });
  advance(2);
  std::jthread dsp_thread([&] { compile(dsp); });
  require(acquire(dsp.entered), "DSP compiler must enter before the test guard");
  advance(3);
  mcu.leave.release();
  require(acquire(mcu.ended) && clock.elapsed() == step * 1,
          "Execution remains frozen after MCU exit while the DSP compiler is active");
  advance(5);
  dsp.leave.release();
  require(acquire(dsp.ended), "DSP compiler must finish before the test guard");
  advance(6);
  require(clock.elapsed() == step * 2,
          "Overlapping20-tick MCU and30-tick DSP compilation excludes40 wall ticks, not50");
  complete.store(true);
  observer.join();
  mcu_thread.join();
  dsp_thread.join();
  require(valid.load(),
          "Concurrent compiler transitions and observations preserve monotonic execution time");
}
}

int main() {
  try {
    intervals();
    startup_budget();
    overlapping_owners();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
