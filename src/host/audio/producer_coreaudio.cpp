// SPDX-License-Identifier: GPL-3.0-only
#include "audio/playback.hpp"

#include <AudioToolbox/AudioWorkInterval.h>
#include <array>
#include <bit>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <limits>
#include <mach/kern_return.h>
#include <mach/mach_time.h>
#include <mach/message.h>
#include <mach/thread_act.h>
#include <mach/thread_policy.h>
#if defined(__aarch64__)
#include <mach/arm/boolean.h>
#include <mach/arm/kern_return.h>
#include <mach/arm/vm_types.h>
#else
#include <mach/i386/boolean.h>
#include <mach/i386/kern_return.h>
#include <mach/i386/vm_types.h>
#endif
#include <memory>
#include <os/clock.h>
#include <os/object.h>
#include <os/workgroup.h>
#include <pthread.h>
#include <stdexcept>
#include <string>
#include <sys/_pthread/_pthread_t.h>
#include <system_error>

namespace blofeld::host::audio {
namespace {
void require(const kern_return_t status, const char *operation) {
  if (status != KERN_SUCCESS) {
    throw std::runtime_error(std::string(operation) + ": Mach status=" + std::to_string(status));
  }
}

void require_workgroup(const int status) {
  if (status != 0) {
    throw std::system_error(status, std::generic_category(), "Native audio producer workgroup");
  }
}

std::uint32_t absolute_period(const std::chrono::nanoseconds duration) {
  if (duration <= std::chrono::nanoseconds::zero() || duration > std::chrono::seconds(1)) {
    throw std::invalid_argument("Audio producer period requires a bounded positive grant");
  }
  mach_timebase_info_data_t timebase{};
  require(mach_timebase_info(&timebase), "Query producer timebase");
  if (timebase.numer == 0 || timebase.denom == 0) {
    throw std::runtime_error("Invalid producer timebase");
  }
  const auto ticks = static_cast<std::uint64_t>(duration.count()) * timebase.denom / timebase.numer;
  if (ticks < 2 || ticks > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Audio producer period exceeds Mach scheduling bounds");
  }
  return static_cast<std::uint32_t>(ticks);
}

class Deadline {
public:
  explicit Deadline(const std::uint32_t period) : thread_(pthread_self()) {
    if (pthread_main_np() != 0) {
      throw std::logic_error("Native audio production requires a dedicated worker thread");
    }
    auto count = static_cast<mach_msg_type_number_t>(previous_.size());
    require(thread_policy_get(pthread_mach_thread_np(thread_),
                              THREAD_TIME_CONSTRAINT_POLICY,
                              previous_.data(),
                              &count,
                              &was_default_),
            "Read producer scheduling policy");
    if (count != previous_.size()) {
      throw std::runtime_error("Invalid producer scheduling policy");
    }
    const thread_time_constraint_policy_data_t policy{
        .period = period, .computation = period / 2, .constraint = period, .preemptible = 1};
    auto words = std::bit_cast<Policy>(policy);
    require(thread_policy_set(pthread_mach_thread_np(thread_),
                              THREAD_TIME_CONSTRAINT_POLICY,
                              words.data(),
                              static_cast<mach_msg_type_number_t>(words.size())),
            "Set producer scheduling deadline");
  }

  ~Deadline() {
    assert(pthread_equal(thread_, pthread_self()) != 0);
    const auto flavor = was_default_ != 0 ? unsigned{THREAD_STANDARD_POLICY}
                                          : unsigned{THREAD_TIME_CONSTRAINT_POLICY};
    const auto count = was_default_ != 0 ? THREAD_STANDARD_POLICY_COUNT : previous_.size();
    thread_policy_set(pthread_mach_thread_np(thread_),
                      flavor,
                      previous_.data(),
                      static_cast<mach_msg_type_number_t>(count));
  }

  Deadline(const Deadline &) = delete;
  Deadline &operator=(const Deadline &) = delete;

private:
  using Policy = std::array<integer_t, THREAD_TIME_CONSTRAINT_POLICY_COUNT>;
  pthread_t thread_;
  Policy previous_{};
  boolean_t was_default_ = 0;
};

class Workgroup {
public:
  Workgroup() : thread_(pthread_self()) {
    group_ = AudioWorkIntervalCreate("Native audio producer", OS_CLOCK_MACH_ABSOLUTE_TIME, nullptr);
    if (group_ == nullptr) { throw std::runtime_error("Cannot create audio producer workgroup"); }
    const auto status = os_workgroup_join(group_, &token_);
    if (status != 0) {
      os_release(group_);
      require_workgroup(status);
    }
  }

  ~Workgroup() {
    assert(pthread_equal(thread_, pthread_self()) != 0);
    if (active_) { static_cast<void>(os_workgroup_interval_finish(group_, nullptr)); }
    os_workgroup_leave(group_, &token_);
    os_release(group_);
  }

  Workgroup(const Workgroup &) = delete;
  Workgroup &operator=(const Workgroup &) = delete;

  void begin(const std::uint32_t period) {
    assert(!active_ && pthread_equal(thread_, pthread_self()) != 0);
    const auto now = mach_absolute_time();
    if (now > std::numeric_limits<std::uint64_t>::max() - period) {
      throw std::overflow_error("Audio producer deadline exhausted");
    }
    require_workgroup(os_workgroup_interval_start(group_, now, now + period, nullptr));
    active_ = true;
  }

  void end() {
    assert(pthread_equal(thread_, pthread_self()) != 0);
    if (!active_) { return; }
    require_workgroup(os_workgroup_interval_finish(group_, nullptr));
    active_ = false;
  }

private:
  pthread_t thread_;
  os_workgroup_interval_t group_ = nullptr;
  os_workgroup_join_token_s token_{};
  bool active_ = false;
};
}

struct Playback::Producer::Impl {
  explicit Impl(const std::chrono::nanoseconds duration)
      : period(absolute_period(duration)), deadline(period) {}

  std::uint32_t period;
  Deadline deadline;
  Workgroup workgroup;
};

Playback::Producer::Producer(const std::chrono::nanoseconds period)
    : impl_(std::make_unique<Impl>(period)) {}

Playback::Producer::Producer(Producer &&) noexcept = default;
Playback::Producer::~Producer() = default;

void Playback::Producer::begin_work() {
  impl_->workgroup.begin(impl_->period);
}

void Playback::Producer::end_work() {
  impl_->workgroup.end();
}
}
