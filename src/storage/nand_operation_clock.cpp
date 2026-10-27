// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_operation_clock.hpp"

#include "common/bus.hpp"
#include "storage/nand.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace blofeld {
namespace {
constexpr unsigned read_microseconds = 15;
constexpr unsigned reset_microseconds = 5;
constexpr unsigned program_reset_microseconds = 10;
constexpr unsigned erase_reset_microseconds = 500;
constexpr unsigned microseconds_per_second = 1000000;
constexpr std::uint8_t reset_command = 0xff;
}

NandOperationClock::NandOperationClock(Nand device,
                                       const std::uint32_t timebase_hz,
                                       const NandOperationContract contract)
    : device_(std::move(device)), timebase_hz_(timebase_hz) {
  if (contract != NandOperationContract::reference_maximum) {
    fail("requires the explicit maximum-duration reference contract");
  }
  if (timebase_hz_ == 0) { fail("zero reference timebase"); }
  if (device_.pending() != NandPending::none) { fail("initial operation has no start time"); }
}

[[noreturn]] void NandOperationClock::fail(const std::string &reason) const {
  throw Fault("NAND operation clock ticks=" + std::to_string(ticks_) +
              " timebase-hz=" + std::to_string(timebase_hz_) + " page=" + hex(device_.page(), 6) +
              " column=" + hex(device_.column(), 3) + ": " + reason);
}

void NandOperationClock::healthy() const {
  if (!healthy_) { fail("incomplete operation requires coordinated recovery"); }
}

void NandOperationClock::schedule(const NandPending operation, const unsigned microseconds) {
  const auto scaled = static_cast<std::uint64_t>(microseconds) * timebase_hz_;
  const auto duration =
      (scaled / microseconds_per_second) + (scaled % microseconds_per_second != 0 ? 1U : 0U);
  if (duration > std::numeric_limits<std::uint64_t>::max() - ticks_) {
    fail("completion deadline exceeds the reference counter");
  }
  event_ = {.ticks = ticks_ + duration, .operation = operation};
}

std::optional<NandCompletionEvent> NandOperationClock::next_event() const {
  healthy();
  return event_;
}

std::optional<NandCompletionEvent> NandOperationClock::advance(const std::uint64_t ticks) {
  healthy();
  healthy_ = false;
  if (ticks < ticks_) { fail("reference time moved backwards"); }
  std::optional<NandCompletionEvent> completed;
  if (event_ && ticks >= event_->ticks) {
    ticks_ = event_->ticks;
    completed = event_;
    device_.observe_completion();
    event_.reset();
  }
  ticks_ = ticks;
  healthy_ = true;
  return completed;
}

unsigned NandOperationClock::reset_duration() const {
  switch (device_.pending()) {
    case NandPending::read: return reset_microseconds;
    case NandPending::program: return program_reset_microseconds;
    case NandPending::erase: return erase_reset_microseconds;
    case NandPending::reset: fail("repeated reset timing is unqualified");
    case NandPending::none:
      if (device_.ready() != true) { fail("reset with unknown power readiness is unqualified"); }
      return reset_microseconds;
    default: fail("invalid pending operation");
  }
}

void NandOperationClock::observe_identification(const NandIdentification identification) {
  healthy();
  healthy_ = false;
  device_.observe_identification(identification);
  healthy_ = true;
}

void NandOperationClock::observe_power_ready() {
  healthy();
  healthy_ = false;
  device_.observe_power_ready();
  healthy_ = true;
}

void NandOperationClock::observe_wp(const std::optional<bool> high) {
  healthy();
  healthy_ = false;
  device_.observe_wp(high);
  healthy_ = true;
}

void NandOperationClock::command(const std::uint8_t value) {
  healthy();
  healthy_ = false;
  const auto duration = value == reset_command ? reset_duration() : 0;
  device_.command(value);
  if (value == reset_command) { schedule(NandPending::reset, duration); }
  healthy_ = true;
}

void NandOperationClock::address(const std::uint8_t value) {
  healthy();
  healthy_ = false;
  const auto before = device_.pending();
  device_.address(value);
  if (before != NandPending::read && device_.pending() == NandPending::read) {
    schedule(NandPending::read, read_microseconds);
  }
  healthy_ = true;
}

void NandOperationClock::write(const std::uint8_t value) {
  healthy();
  healthy_ = false;
  device_.write(value);
  healthy_ = true;
}

std::uint8_t NandOperationClock::read(const std::uint8_t mask) {
  healthy();
  healthy_ = false;
  const auto result = device_.read(mask);
  healthy_ = true;
  return result;
}

void NandOperationClock::observe_completion(const NandOutcome outcome) {
  healthy();
  healthy_ = false;
  device_.observe_completion(outcome);
  healthy_ = true;
}

void NandOperationClock::power_loss() {
  healthy();
  healthy_ = false;
  device_.power_loss();
  event_.reset();
  healthy_ = true;
}
}
