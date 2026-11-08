// SPDX-License-Identifier: GPL-3.0-only
#include "process/signals.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <pthread.h>
#include <stdexcept>

extern "C" {
#include <signal.h>
}

int main() try {
  constexpr std::chrono::milliseconds timeout{100};
  const blofeld::host::Signals signals;
  if (signals.wait(std::chrono::milliseconds(0))) {
    throw std::runtime_error("No signal was sent");
  }
  if (pthread_kill(pthread_self(), SIGINT) != 0 || !signals.wait(timeout)) {
    throw std::runtime_error("Termination signals must be received on the control thread");
  }
  if (signals.wait(std::chrono::milliseconds(0))) {
    throw std::runtime_error("A signal cannot be delivered twice");
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
