// SPDX-License-Identifier: GPL-3.0-only
#include "io/midi_port.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <latch>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
using blofeld::vi::MidiPort;
enum class Completion : std::uint8_t { frontier, eof, cancelled, aborted };
constexpr std::uint64_t first_tick = 100, second_tick = 200;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void wait_for(const std::atomic<bool> &completed, const char *message) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!completed.load(std::memory_order_acquire)) {
    require(std::chrono::steady_clock::now() < deadline, message);
    std::this_thread::yield();
  }
}

void settle() {
  constexpr unsigned registration_yields = 256;
  for (unsigned index = 0; index < registration_yields; ++index) { std::this_thread::yield(); }
}

void split_frontiers(const Completion completion) {
  MidiPort port(MidiPort::InputMode::stream);
  std::latch entered{2};
  std::atomic<bool> first_done{false};
  std::atomic<bool> second_done{false};
  std::array<bool, 2> accepted{};
  // Start the farther receiver first, so both can park before the first grant.
  std::jthread dsp([&](const std::stop_token &stop) {
    entered.count_down();
    accepted[1] = port.await_input(second_tick, stop);
    second_done.store(true, std::memory_order_release);
  });
  std::jthread mcu([&](const std::stop_token &stop) {
    entered.count_down();
    accepted[0] = port.await_input(first_tick, stop);
    first_done.store(true, std::memory_order_release);
  });
  entered.wait();
  settle();
  require(port.advance_input(first_tick + 1), "Publish only the nearer receiver's input grant");
  wait_for(first_done, "The first receiver must consume its granted frontier");
  require(!second_done.load(std::memory_order_acquire),
          "The farther receiver cannot run beyond committed input knowledge");
  // The remaining receiver rechecks the first publication and parks again.
  // No private waiter access or synthetic traffic is used by this test.
  settle();
  switch (completion) {
    case Completion::frontier:
      require(port.advance_input(second_tick + 1), "Publish the remaining receiver's grant");
      break;
    case Completion::eof:
      require(port.finish_input(), "EOF releases the remaining receiver");
      break;
    case Completion::cancelled: dsp.request_stop(); break;
    case Completion::aborted: port.abort_input(); break;
    default: throw std::logic_error("Unknown waiter completion");
  }
  wait_for(second_done, "One receiver leaving must not hide the remaining wait registration");
  mcu.join();
  dsp.join();
  const bool successful = completion == Completion::frontier || completion == Completion::eof;
  require(accepted[0] && accepted[1] == successful && port.time() == 0 && !port.peek_input() &&
              !port.peek_input(MidiPort::Receiver::dsp) &&
              (port.error() != nullptr) == (completion == Completion::aborted),
          "Split grants, EOF, cancellation and abort retain admission, error and device-time "
          "semantics");
}
}

int main() try {
  constexpr unsigned rounds = 8;
  for (unsigned round = 0; round < rounds; ++round) {
    for (const auto completion :
         {Completion::frontier, Completion::eof, Completion::cancelled, Completion::aborted}) {
      split_frontiers(completion);
    }
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
