// SPDX-License-Identifier: GPL-3.0-only
#include "audio/audio_wake.hpp"

#include "common/bus.hpp"

#include <cerrno>
#include <cstdint>
#include <string>
#include <sys/eventfd.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <system_error>
#include <unistd.h>

namespace blofeld {
namespace {
[[noreturn]] void failed(const char *operation, const int error) {
  throw Fault("Audio wake " + std::string(operation) + ": " +
              std::error_code(error, std::generic_category()).message());
}
}

AudioWake::AudioWake() : descriptor_(::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)) {
  if (descriptor_ < 0) { failed("create", errno); }
}

AudioWake::~AudioWake() {
  ::close(descriptor_);
}

int AudioWake::signal() const noexcept {
  const std::uint64_t token = 1;
  const auto count = ::write(descriptor_, &token, sizeof(token));
  if (count == static_cast<ssize_t>(sizeof(token))) { return 0; }
  const auto error = errno;
  // A saturated event counter is already readable: one wake covers all data.
  if (count < 0 && error == EAGAIN) { return 0; }
  return count < 0 ? error : EIO;
}

void AudioWake::wait() const {
  pollfd event{.fd = descriptor_, .events = POLLIN, .revents = 0};
  constexpr int native_poll_ms = 1;
  const auto ready = ::poll(&event, 1, native_poll_ms);
  if (ready < 0) {
    if (errno == EINTR) { return; }
    failed("poll", errno);
  }
  if (ready == 0) { return; }
  if (event.revents != POLLIN) { failed("event", EIO); }
  std::uint64_t tokens = 0;
  const auto count = ::read(descriptor_, &tokens, sizeof(tokens));
  if (count < 0 && (errno == EAGAIN || errno == EINTR)) { return; }
  if (count != static_cast<ssize_t>(sizeof(tokens))) { failed("read", count < 0 ? errno : EIO); }
}
}
