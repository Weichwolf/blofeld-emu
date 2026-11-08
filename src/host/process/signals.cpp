// SPDX-License-Identifier: GPL-3.0-only
#include "process/signals.hpp"

#include <bits/types/sigset_t.h>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <sys/poll.h>
#include <sys/signalfd.h>
#include <system_error>
#include <unistd.h>

// POSIX extensions are declared by the C API, outside the standard C++ wrapper.
extern "C" {
#include <signal.h>
}

namespace blofeld::host {
Signals::Signals() {
  sigset_t selected{};
  sigemptyset(&selected);
  sigaddset(&selected, SIGINT);
  sigaddset(&selected, SIGTERM);
  const auto error = pthread_sigmask(SIG_BLOCK, &selected, &previous_);
  if (error != 0) {
    throw std::system_error(error, std::generic_category(), "Block termination signals");
  }
  descriptor_ = signalfd(-1, &selected, SFD_CLOEXEC | SFD_NONBLOCK);
  if (descriptor_ < 0) {
    const auto reason = errno;
    pthread_sigmask(SIG_SETMASK, &previous_, nullptr);
    throw std::system_error(reason, std::generic_category(), "Create termination descriptor");
  }
}

Signals::~Signals() {
  ::close(descriptor_);
  pthread_sigmask(SIG_SETMASK, &previous_, nullptr);
}

bool Signals::wait(const std::chrono::milliseconds timeout) const {
  if (timeout.count() < 0 || timeout.count() > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("Signal poll requires a bounded nonnegative timeout");
  }
  pollfd descriptor{.fd = descriptor_, .events = POLLIN, .revents = 0};
  const auto events = ::poll(&descriptor, 1, static_cast<int>(timeout.count()));
  if (events < 0 && errno == EINTR) { return false; }
  if (events < 0) {
    throw std::system_error(errno, std::generic_category(), "Poll termination signals");
  }
  if (events == 0) { return false; }
  if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    throw std::runtime_error("Termination descriptor disconnected");
  }
  signalfd_siginfo information{};
  const auto bytes = ::read(descriptor_, &information, sizeof(information));
  if (bytes < 0 && errno == EAGAIN) { return false; }
  if (bytes < 0 || static_cast<std::size_t>(bytes) != sizeof(information)) {
    throw std::runtime_error("Read termination signal failed");
  }
  return true;
}
}
