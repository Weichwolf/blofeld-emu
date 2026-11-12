// SPDX-License-Identifier: GPL-3.0-only
#include "diagnostics/thread_usage.hpp"

#include <cerrno>
#include <cstdint>
#include <pthread.h>
#include <thread>
#include <time.h>

namespace blofeld::host::diagnostics {
ThreadUsageReader::ThreadUsageReader() = default;

ThreadUsageReader::ThreadId ThreadUsageReader::current() noexcept {
  return identify(pthread_self());
}

int ThreadUsageReader::name_current(const char *name) noexcept {
  return pthread_setname_np(pthread_self(), name);
}

ThreadUsageReader::ThreadId
ThreadUsageReader::identify(const std::thread::native_handle_type thread) noexcept {
  ThreadId result;
  clockid_t clock{};
  result.error = pthread_getcpuclockid(thread, &clock);
  result.clock = clock;
  return result;
}

ThreadUsage ThreadUsageReader::read(const ThreadId thread) const noexcept {
  ThreadUsage result{.identifier = thread.identifier, .error = thread.error};
  if (result.error != 0) { return result; }
  timespec time{};
  if (clock_gettime(static_cast<clockid_t>(thread.clock), &time) != 0) {
    result.error = errno;
    return result;
  }
  constexpr std::uint64_t nanoseconds = 1000000000;
  result.count = 1;
  result.levels[0].cpu_nanoseconds = (static_cast<std::uint64_t>(time.tv_sec) * nanoseconds) +
                                     static_cast<std::uint64_t>(time.tv_nsec);
  return result;
}
}
