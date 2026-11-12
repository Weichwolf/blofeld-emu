// SPDX-License-Identifier: GPL-3.0-only
#include "census.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
using blofeld::vi::Census;
constexpr std::uint32_t middle = 200, overflow = 70000;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

Census::Guest guest(void *context) noexcept {
  return *static_cast<Census::Guest *>(context);
}

void disabled() {
  Census census(false);
  const Census::Scope owner(census, Census::Actor::dsp);
  const Census::GuestScope source(nullptr, guest);
  const Census::Wait waiting;
  Census::recording(true);
  const auto active = Census::session_begin();
  Census::session_end(active, 1);
  require(!census.enabled() && !census.capture().valid &&
              census.sessions(Census::Actor::dsp).count == 0,
          "Disabled diagnostics never read a guest or accumulate sessions");
}

void owner_window() {
  Census census(true);
  std::atomic<unsigned> phase{0};
  std::jthread mcu([&] {
    const Census::Scope owner(census, Census::Actor::mcu);
    const Census::Wait waiting;
    phase.store(1);
    phase.notify_one();
    phase.wait(1);
  });
  phase.wait(0);
  Census::Snapshot first;
  Census::Snapshot last;
  bool isolated = false;
  bool absent = false;
  {
    const Census::Scope owner(census, Census::Actor::dsp);
    Census::Guest value{.instructions = 12, .cycles = 13, .idle = 14, .pc = 0x1000};
    {
      const Census::GuestScope source(&value, guest);
      first = census.capture();
      const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(2);
      while (std::chrono::steady_clock::now() < until) { std::this_thread::yield(); }
      value.instructions += 9;
      value.cycles += 5;
      value.idle += 7;
      last = census.capture();
    }
    absent = !census.capture().valid;
    const Census::GuestScope next(&value, guest);
    isolated = census.capture().guest.epoch == first.guest.epoch + 1;
    Census::session_end(false, 11);
    Census::recording(true);
    for (const auto retired : {0U, 1U, middle, overflow}) {
      Census::session_end(Census::session_begin(), retired);
    }
    Census::recording(false);
    Census::session_end(true, 4);
  }
  phase.store(2);
  phase.notify_one();
  mcu.join();
  require(absent, "An absent guest source cannot yield an invented snapshot");
  require(first.valid && last.valid && first.closed <= last.opened,
          "Both live owners are captured inside a bounded common window");
  require(first.guest.pc == 0x1000 && last.guest.instructions - first.guest.instructions == 9 &&
              last.guest.cycles - first.guest.cycles == 5 &&
              last.guest.idle - first.guest.idle == 7 && isolated,
          "Retirement, work cycles, WAIT advancement and execution epochs stay distinct");
  const auto &before = first.owners[0].wait;
  const auto &after = last.owners[0].wait;
  require(before.calls == 1 && after.calls == 1 && after.nanoseconds > before.nanoseconds &&
              after.nanoseconds - before.nanoseconds >= last.opened - first.closed &&
              after.nanoseconds - before.nanoseconds <= last.closed - first.opened,
          "A wait crossing both boundaries contributes only its window duration");
  require(last.owners[1].wait.calls == 0, "Wait attribution stays on its native execution owner");
  const auto sessions = census.sessions(Census::Actor::dsp);
  require(sessions.count == 5 && sessions.sum == overflow + middle + 1 + 4 &&
              sessions.maximum == overflow && sessions.bins[0] == 1 && sessions.bins[1] == 1 &&
              sessions.bins[middle] == 1 && sessions.bins.back() == 1,
          "Session census includes boundary overlap, zero retirements and explicit overflow");
}
}

int main() {
  try {
    disabled();
    owner_window();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
