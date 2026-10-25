// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <unistd.h>
#include <utility>

namespace blofeld::usbip_tcp {
class Socket {
public:
  explicit Socket(const int fd = -1) : fd_(fd) {}

  ~Socket() { reset(); }

  Socket(const Socket &) = delete;
  Socket &operator=(const Socket &) = delete;

  Socket(Socket &&other) noexcept : fd_(std::exchange(other.fd_, -1)) {}

  Socket &operator=(Socket &&other) noexcept {
    if (this != &other) {
      reset();
      fd_ = std::exchange(other.fd_, -1);
    }
    return *this;
  }

  [[nodiscard]] int get() const { return fd_; }

  void reset() {
    if (fd_ >= 0) {
      static_cast<void>(::close(fd_));
      fd_ = -1;
    }
  }

private:
  int fd_;
};
}
