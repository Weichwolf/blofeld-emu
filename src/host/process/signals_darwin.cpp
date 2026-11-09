// SPDX-License-Identifier: GPL-3.0-only
#include "process/signals.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <limits>
#include <pthread.h>
#include <stdexcept>
#include <sys/_types/_sigset_t.h>
#include <sys/signal.h>
#include <system_error>
#include <thread>

extern "C" {
#include <signal.h>
}

namespace blofeld::host {
Signals::Signals() {
  sigemptyset(&selected_);
  sigaddset(&selected_, SIGINT);
  sigaddset(&selected_, SIGTERM);
  const auto error = pthread_sigmask(SIG_BLOCK, &selected_, &previous_);
  if (error != 0) {
    throw std::system_error(error, std::generic_category(), "Block termination signals");
  }
}

Signals::~Signals() {
  pthread_sigmask(SIG_SETMASK, &previous_, nullptr);
}

bool Signals::wait(const std::chrono::milliseconds timeout) const {
  if (timeout.count() < 0 || timeout.count() > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("Signal poll requires a bounded nonnegative timeout");
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  // Darwin lacks timed sigwait; kqueue does not observe thread-directed signals.
  for (;;) {
    sigset_t pending{};
    if (sigpending(&pending) != 0) {
      throw std::system_error(errno, std::generic_category(), "Inspect termination signals");
    }
    for (const auto signal : {SIGINT, SIGTERM}) {
      if (sigismember(&pending, signal) != 1) { continue; }
      int received = 0;
      const auto error = sigwait(&selected_, &received);
      if (error != 0) {
        throw std::system_error(error, std::generic_category(), "Consume termination signal");
      }
      return true;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) { return false; }
    constexpr std::chrono::milliseconds interval{1};
    std::this_thread::sleep_until(std::min(deadline, now + interval));
  }
}
}
