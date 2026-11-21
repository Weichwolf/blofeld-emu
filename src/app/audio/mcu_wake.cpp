// SPDX-License-Identifier: GPL-3.0-only
#include "audio/mcu_wake.hpp"

#include "audio/deadline_coreaudio.hpp"
#include "io/midi_port.hpp"

#include <chrono>
#include <cstdint>

#if defined(__aarch64__)
#include <atomic>
#include <mach/kern_return.h>
#include <stdexcept>
#include <string>
#include <sys/sysctl.h>
#endif

namespace blofeld::app {
#if defined(__aarch64__)
namespace {
std::uint64_t counter() noexcept {
  std::uint64_t value;
  asm volatile("mrs %0, cntvct_el0" : "=r"(value));
  return value;
}

struct WaitRequest {
  std::uint64_t expected, deadline;
};

[[gnu::target("arch=armv8.7-a")]]
bool wait_word(const std::uint64_t &word, const WaitRequest request) noexcept {
  bool ready;
  for (;;) {
    std::uint64_t observed;
    asm volatile("ldaxr %0, [%1]" : "=r"(observed) : "r"(&word) : "memory");
    ready = observed != request.expected;
    if (ready || counter() >= request.deadline) { break; }
    asm volatile("wfet %0" : : "r"(request.deadline) : "memory");
  }
  asm volatile("clrex" ::: "memory");
  return ready;
}
}

void McuWake::start(const std::chrono::nanoseconds device_period) {
  period_.store(host::audio::device_period_ticks(device_period), std::memory_order_release);
  int feature = 0;
  auto bytes = sizeof(feature);
  if (sysctlbyname("hw.optional.arm.FEAT_WFxT", &feature, &bytes, nullptr, 0) != 0 ||
      bytes != sizeof(feature) || feature != 1) {
    return;
  }
  std::uint64_t frequency;
  asm("mrs %0, cntfrq_el0" : "=r"(frequency));
  constexpr std::uint64_t ns_per_second = 1'000'000'000;
  budget_.store(static_cast<std::uint64_t>(device_period.count()) * frequency / ns_per_second,
                std::memory_order_release);
}

bool McuWake::schedule() noexcept {
  if (!scheduled_) {
    const auto period = period_.load(std::memory_order_acquire);
    if (period == 0) { return true; }
    schedule_status_ = host::audio::set_current_deadline(period);
    scheduled_ = true;
  }
  return schedule_status_ == KERN_SUCCESS;
}

void McuWake::check() const {
  if (schedule_status_ != KERN_SUCCESS) {
    throw std::runtime_error("Set MCU scheduling deadline: Mach status=" +
                             std::to_string(schedule_status_));
  }
}

bool McuWake::wait_checkpoint(const vi::CheckpointWait wait) noexcept {
  const auto budget = budget_.load(std::memory_order_acquire);
  if (budget == 0) { return false; }
  const auto now = counter();
  if (wait.command != checkpoint_) {
    checkpoint_ = wait.command;
    checkpoint_deadline_ = now + budget;
  }
  return now < checkpoint_deadline_ &&
         wait_word(wait.generation, {.expected = wait.observed, .deadline = checkpoint_deadline_});
}
#else
void McuWake::start(std::chrono::nanoseconds) {}

bool McuWake::schedule() noexcept {
  return true;
}

void McuWake::check() const {}

bool McuWake::wait_checkpoint(vi::CheckpointWait) noexcept {
  return false;
}
#endif
}
