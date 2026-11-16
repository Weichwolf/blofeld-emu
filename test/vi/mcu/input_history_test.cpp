// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/epoch.hpp"
#include "mcu/input_history.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::mcu::Epoch;
using blofeld::vi::mcu::InputHistory;
constexpr auto data = Epoch::Signal::data;
constexpr auto irq = Epoch::Signal::interrupt;
constexpr std::uint64_t initial = 0x11, next = 0x22, high = 0xa0, combined = 0xa5;
constexpr std::uint64_t byte = 0xff, upper = 0xf0, lower = 0x0f;
constexpr std::uint64_t first = 10, second = 12, future = 20;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void receiving_time() {
  std::array<Epoch::Read, 8> storage{};
  InputHistory history(storage, {initial});
  require(history.write({.ticks = first, .value = next, .mask = byte}) &&
              history.at(first - 1, data) == initial && history.at(first, data) == next &&
              history.at(future, data) == next,
          "A future MCU read predicts the last actual output; replay never reads it early");
  require(history.write({.ticks = first, .value = 1, .mask = 1, .signal = irq}) &&
              history.write({.ticks = second, .value = high, .mask = upper}) &&
              history.write({.ticks = second, .value = 5, .mask = lower}) &&
              history.at(first - 1, irq) == 0 && history.at(first, irq) == 1 &&
              history.at(second - 1, data) == next && history.at(second, data) == combined,
          "Different signals, masked changes and same-instant source order remain distinct");
  require(history.retire(first) && !history.at(first - 1, data) &&
              history.at(first, data) == next && history.at(second, data) == combined,
          "A committed checkpoint retains its exact baseline and known later history");
  require(history.retire(second) && history.at(second, data) == combined &&
              history.at(future, irq) == 1,
          "Retiring one signal cannot erase another signal's actual retained level");
  require(!history.write({.ticks = second - 1, .value = next, .mask = byte}) &&
              history.error() == Epoch::Error::time && !history.at(future, data),
          "Late or reordered source history is a real error, never a replacement input");
}

void retained_replay() {
  std::array<Epoch::Read, 8> input_storage{};
  std::array<Epoch::Read, 8> read_storage{};
  InputHistory history(input_storage, {initial});
  Epoch epoch(read_storage, 2);
  require(epoch.record({.ticks = future, .value = initial, .mask = byte}) &&
              history.write({.ticks = first, .value = next, .mask = byte}) &&
              !epoch.changed({.ticks = first, .value = next, .mask = byte}) && epoch.cancelled() &&
              epoch.rollback(),
          "An actual output change invalidates an observation that used the earlier output");
  require(history.at(first - 1, data) == initial && history.at(first, data) == next &&
              history.at(future, data) == next,
          "Rollback preserves actual source history for both sides of the changed instant");
  require(epoch.record({.ticks = first - 1, .value = initial, .mask = byte}) &&
              epoch.record({.ticks = first, .value = next, .mask = byte}) &&
              epoch.attained(second) && epoch.checkpoint(first) && history.retire(first),
          "Replayed actual observations validate through an attained instruction boundary");
  require(epoch.record({.ticks = future, .value = 0, .mask = 1, .signal = irq}) &&
              !epoch.changed({.ticks = second, .value = 1, .mask = 1, .signal = irq}) &&
              epoch.cancelled(),
          "A changed interrupt input is subject to the same retained read-set validation");
  require(!epoch.changed({.ticks = first, .value = initial, .mask = byte}) &&
              epoch.error() == Epoch::Error::time && !epoch.cancelled() && !epoch.rollback(),
          "A real protocol error takes priority over an earlier cancellation flag");
}

void retained_irq_edge() {
  std::array<Epoch::Read, 8> storage{};
  InputHistory history(storage);
  require(
      history.write({.ticks = 3, .value = 1, .mask = 1, .signal = Epoch::Signal::interrupt}) &&
          history.write({.ticks = 4, .value = 0, .mask = 1, .signal = Epoch::Signal::interrupt}) &&
          history.raised(Epoch::Signal::interrupt, 2, 5),
      "An eligible IRQ pulse remains pending after its line was released");
  require(history.retire(5) && history.at(5, Epoch::Signal::interrupt) == 0 &&
              history.raised(Epoch::Signal::interrupt, 2, 5) &&
              !history.raised(Epoch::Signal::interrupt, 5, 6),
          "Checkpoint retirement preserves the pulse until its MCU eligibility window closes");
}

void input_pressure() {
  std::array<Epoch::Read, 8> storage{};
  InputHistory history(storage, {}, 3);
  require(!history.yield(), "A fresh source history has room for speculative input");
  for (std::uint64_t tick = 1; tick <= 5; ++tick) {
    require(history.write({.ticks = tick, .value = tick, .mask = byte}),
            "Retain actual transitions");
  }
  require(history.yield() && history.error() == Epoch::Error::none,
          "Reserve remaining slots before the real input history is exhausted");
  require(history.retire(5) && !history.yield() && history.at(5, Epoch::Signal::data) == 5,
          "Confirmed retirement frees history while preserving the actual source value");
}

void bounded_history() {
  std::array<Epoch::Read, 2> storage{};
  InputHistory history(storage);
  for (std::uint64_t tick = 1; tick < 10; ++tick) {
    require(history.write({.ticks = tick, .value = tick, .mask = byte}) &&
                history.at(tick, data) == tick && history.retire(tick),
            "Committed source slots wrap without changing retained input values");
  }
  require(history.write({.ticks = 10, .value = 10, .mask = byte}) &&
              history.write({.ticks = 11, .value = 11, .mask = byte}) &&
              history.write({.ticks = second, .value = 11, .mask = byte}),
          "Repeated equal outputs advance source order without consuming bounded storage");
  require(!history.write({.ticks = future, .value = future, .mask = byte}) &&
              history.error() == Epoch::Error::exhausted,
          "A full source history fails instead of discarding information needed for replay");
}
}

int main() try {
  receiving_time();
  retained_replay();
  bounded_history();
  input_pressure();
  retained_irq_edge();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
