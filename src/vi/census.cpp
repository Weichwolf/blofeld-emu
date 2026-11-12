// SPDX-License-Identifier: GPL-3.0-only
#include "census.hpp"

#include "diagnostics/thread_usage.hpp"
#include "process/configuration.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace blofeld::vi {
namespace {
constexpr std::size_t histogram_limit = 65536;

struct Context {
  Census *census = nullptr;
  unsigned actor = 0;
  void *guest = nullptr;
  Census::GuestScope::Read read = nullptr;
  std::uint64_t epoch = 0;
  bool recording = false;
};

thread_local Context current;

std::uint64_t now() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count());
}
}

struct alignas(64) Census::Slot {
  host::diagnostics::ThreadUsageReader::ThreadId thread;
  std::atomic<bool> registered{false};
  std::atomic<std::uint64_t> sequence{0}, calls{0}, wait_nanoseconds{0}, entered{0};
  int naming_error = 0;
  std::array<std::uint64_t, histogram_limit + 1> bins{};
  std::uint64_t sessions = 0, retired = 0;
  std::uint32_t maximum = 0;
};

struct Census::Impl {
  host::diagnostics::ThreadUsageReader reader;
  std::array<Slot, 2> slots;
};

Census::Census() : Census(host::inherited_setting("BLOFELD_CENSUS") == "1") {}

Census::Census(const bool enabled) : impl_(enabled ? std::make_unique<Impl>() : nullptr) {}

Census::~Census() = default;

Census::Scope::Scope(Census &census, const Actor actor) noexcept {
  assert(current.census == nullptr);
  if (!census.enabled()) { return; }
  current.census = &census;
  current.actor = static_cast<unsigned>(actor);
  auto &slot = census.impl_->slots[current.actor];
  slot.naming_error = host::diagnostics::ThreadUsageReader::name_current(
      actor == Actor::mcu ? "blofeld-mcu" : "blofeld-dsp");
  assert(!slot.registered.load());
  slot.thread = host::diagnostics::ThreadUsageReader::current();
  slot.registered.store(true, std::memory_order_release);
}

Census::Scope::~Scope() {
  current = {};
}

Census::GuestScope::GuestScope(void *context, const Read read) noexcept {
  if (current.census == nullptr) { return; }
  assert(current.read == nullptr);
  current.guest = context;
  current.read = read;
  ++current.epoch;
}

Census::GuestScope::~GuestScope() {
  current.guest = nullptr;
  current.read = nullptr;
}

Census::Wait::Wait() noexcept {
  if (current.census == nullptr) { return; }
  slot_ = &current.census->impl_->slots[current.actor];
  assert(slot_->entered.load() == 0);
  begun_ = now();
  slot_->sequence.fetch_add(1);
  slot_->calls.fetch_add(1);
  slot_->entered.store(begun_);
  slot_->sequence.fetch_add(1);
}

Census::Wait::~Wait() {
  if (slot_ == nullptr) { return; }
  const auto ended = now();
  slot_->sequence.fetch_add(1);
  slot_->wait_nanoseconds.fetch_add(ended - begun_);
  slot_->entered.store(0);
  slot_->sequence.fetch_add(1);
}

Census::WaitUsage Census::wait_usage(const Slot &slot) noexcept {
  const auto before = slot.sequence.load();
  if ((before & 1U) != 0) { return {}; }
  WaitUsage result{.calls = slot.calls.load(), .nanoseconds = slot.wait_nanoseconds.load()};
  const auto entered = slot.entered.load();
  const auto stamp = now();
  if (entered != 0 && stamp >= entered) { result.nanoseconds += stamp - entered; }
  result.coherent = before == slot.sequence.load();
  return result;
}

Census::Snapshot Census::capture() const noexcept {
  Snapshot result;
  if (!impl_ || current.census != this || current.read == nullptr) { return result; }
  result.opened = now();
  result.guest = current.read(current.guest);
  result.guest.epoch = current.epoch;
  result.valid = true;
  for (unsigned i = 0; i < result.owners.size(); ++i) {
    const auto &slot = impl_->slots[i];
    auto &owner = result.owners[i];
    if (!slot.registered.load(std::memory_order_acquire)) {
      owner.native.error = ESRCH;
      result.valid = false;
      continue;
    }
    owner.native = impl_->reader.read(slot.thread);
    owner.wait = wait_usage(slot);
    owner.naming_error = slot.naming_error;
    result.valid &= owner.native.error == 0 && owner.wait.coherent;
  }
  result.closed = now();
  return result;
}

void Census::recording(const bool active) noexcept {
  current.recording = active;
}

bool Census::session_begin() noexcept {
  return current.recording;
}

void Census::session_end(const bool active_at_entry, const std::uint32_t retired) noexcept {
  if (current.census == nullptr || (!active_at_entry && !current.recording)) { return; }
  auto &slot = current.census->impl_->slots[current.actor];
  ++slot.sessions;
  slot.retired += retired;
  slot.maximum = std::max(slot.maximum, retired);
  ++slot.bins[std::min<std::size_t>(retired, histogram_limit)];
}

Census::Sessions Census::sessions(const Actor actor) const noexcept {
  if (!impl_) { return {}; }
  const auto &slot = impl_->slots[static_cast<unsigned>(actor)];
  return {.bins = slot.bins, .count = slot.sessions, .sum = slot.retired, .maximum = slot.maximum};
}
}
