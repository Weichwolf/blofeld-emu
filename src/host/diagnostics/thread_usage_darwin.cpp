// SPDX-License-Identifier: GPL-3.0-only
#include "diagnostics/thread_usage.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <libproc.h>
#include <mach/kern_return.h>
#include <mach/mach_time.h>
#include <pthread.h>
#include <stdexcept>
#include <sys/sysctl.h>
#include <thread>
#include <unistd.h>

namespace blofeld::host::diagnostics {
namespace {
constexpr int thread_counts = 34;

struct Counters {
  std::uint64_t instructions, cycles, user_mach, system_mach, energy;
};

struct Reply {
  std::uint16_t length = 0, reserved = 0;
  std::uint32_t padding = 0;
  std::array<Counters, ThreadUsage::maximum_levels> levels{};
};

static_assert(sizeof(Counters) == 5 * sizeof(std::uint64_t) && offsetof(Reply, levels) == 8);

bool nanoseconds(const std::uint64_t ticks,
                 const std::uint32_t numerator,
                 const std::uint32_t denominator,
                 std::uint64_t &result) noexcept {
  std::uint64_t whole = 0;
  std::uint64_t remainder = 0;
  return !__builtin_mul_overflow(ticks / denominator, numerator, &whole) &&
         !__builtin_mul_overflow(ticks % denominator, numerator, &remainder) &&
         !__builtin_add_overflow(whole, remainder / denominator, &result);
}
}

ThreadUsageReader::ThreadUsageReader() {
  std::size_t bytes = sizeof(levels_);
  mach_timebase_info_data_t scale{};
  if (sysctlbyname("hw.nperflevels", &levels_, &bytes, nullptr, 0) != 0 ||
      bytes != sizeof(levels_) || levels_ == 0 || levels_ > ThreadUsage::maximum_levels ||
      mach_timebase_info(&scale) != KERN_SUCCESS || scale.numer == 0 || scale.denom == 0) {
    throw std::runtime_error("Cannot identify native thread counter layout/timebase");
  }
  numerator_ = scale.numer;
  denominator_ = scale.denom;
}

ThreadUsageReader::ThreadId ThreadUsageReader::current() noexcept {
  return identify(pthread_self());
}

int ThreadUsageReader::name_current(const char *name) noexcept {
  return pthread_setname_np(name);
}

ThreadUsageReader::ThreadId
ThreadUsageReader::identify(const std::thread::native_handle_type thread) noexcept {
  ThreadId result;
  result.error = pthread_threadid_np(thread, &result.identifier);
  return result;
}

ThreadUsage ThreadUsageReader::read(const ThreadId thread) const noexcept {
  ThreadUsage result{.identifier = thread.identifier, .error = thread.error};
  if (result.error != 0) { return result; }
  Reply reply;
  const auto expected = offsetof(Reply, levels) + (levels_ * sizeof(Counters));
  const auto bytes =
      proc_pidinfo(getpid(), thread_counts, thread.identifier, &reply, sizeof(reply));
  if (bytes <= 0 || static_cast<std::size_t>(bytes) != expected || reply.length != levels_) {
    result.error = bytes <= 0 ? errno : EINVAL;
    return result;
  }
  result.count = levels_;
  result.pmc = true;
  for (unsigned i = 0; i < levels_; ++i) {
    const auto &level = reply.levels[i];
    std::uint64_t ticks = 0;
    if (__builtin_add_overflow(level.user_mach, level.system_mach, &ticks) ||
        !nanoseconds(ticks, numerator_, denominator_, result.levels[i].cpu_nanoseconds)) {
      result.error = ERANGE;
      return result;
    }
    result.levels[i].instructions = level.instructions;
    result.levels[i].cycles = level.cycles;
  }
  return result;
}
}
