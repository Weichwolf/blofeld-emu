// SPDX-License-Identifier: GPL-3.0-only
#include "diagnostics/thread_usage.hpp"

#include <cerrno>
#include <cstdint>
#include <ctime>
#include <pthread.h>
#include <sys/syscall.h>
#include <thread>
#include <unistd.h>

namespace blofeld::host::diagnostics {
ThreadUsageReader::ThreadUsageReader() = default;

ThreadUsageReader::ThreadId ThreadUsageReader::current() noexcept {
  auto result = identify(pthread_self());
  result.identifier = static_cast<std::uint64_t>(gettid());
  return result;
}

int ThreadUsageReader::name_current(const char *name) noexcept {
  return pthread_setname_np(pthread_self(), name);
}

ThreadUsageReader::ThreadId
ThreadUsageReader::identify(const std::thread::native_handle_type thread) noexcept {
  ThreadId result;
  int clock{};
  result.error = pthread_getcpuclockid(thread, &clock);
  result.clock = clock;
  return result;
}

ThreadUsage ThreadUsageReader::read(const ThreadId thread) const noexcept {
  ThreadUsage result{.identifier = thread.identifier, .error = thread.error};
  if (result.error != 0) { return result; }
  std::timespec time{};
  if (syscall(SYS_clock_gettime, static_cast<int>(thread.clock), &time) != 0) {
    result.error = errno;
    return result;
  }
  constexpr std::uint64_t nanoseconds = 1000000000;
  result.count = levels_;
  result.levels[0].cpu_nanoseconds = (static_cast<std::uint64_t>(time.tv_sec) * nanoseconds) +
                                     static_cast<std::uint64_t>(time.tv_nsec);
  return result;
}
}
