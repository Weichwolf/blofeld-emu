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
#include <ostream>
#include <span>
#include <stdexcept>

namespace blofeld::vi {
namespace {
constexpr std::size_t histogram_limit = 65536;

struct Context {
  Census *census = nullptr;
  unsigned actor = 0;
  void *guest = nullptr;
  Census::GuestScope::Read read = nullptr;
  std::uint64_t epoch = 0;
  bool recording = false, unit_open = false;
  std::uint32_t unit_retired = 0;
};

thread_local Context current;

std::uint64_t now() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count());
}
}

struct alignas(64) Census::Slot {
  std::atomic<std::uint64_t> sequence{0}, calls{0}, wait_nanoseconds{0}, entered{0};
  std::uint64_t sessions = 0, retired = 0;
  std::uint64_t unit_count = 0, unit_sum = 0, unit_crossings = 0, unit_unattributed = 0;
  host::diagnostics::ThreadUsageReader::ThreadId thread;
  std::array<std::uint64_t, 3> unit_weights{};
  std::array<DiscardedWrite, 5> discarded_writes{};
  Intervals intervals{};
  std::array<std::uint64_t, histogram_limit + 1> bins{}, unit_bins{};
  int naming_error = 0;
  std::uint32_t maximum = 0, unit_maximum = 0;
  std::atomic<bool> registered{false};
};

struct Census::Impl {
  std::array<Slot, 2> slots;
  std::uint64_t begin = 0, end = 0;
  host::diagnostics::ThreadUsageReader reader;

  [[nodiscard]] unsigned phase(const std::uint64_t ticks) const noexcept {
    if (ticks < begin) { return static_cast<unsigned>(Phase::startup); }
    return static_cast<unsigned>(ticks < end ? Phase::active : Phase::after);
  }
};

Census::Census() : Census(host::inherited_setting("BLOFELD_CENSUS") == "1") {}

Census::Census(const bool enabled) : impl_(enabled ? std::make_unique<Impl>() : nullptr) {}

Census::~Census() = default;

void Census::interval(const std::uint64_t begin, const std::uint64_t end) {
  if (!impl_ || begin == 0 || begin >= end || impl_->end != 0 ||
      impl_->slots[0].registered.load() || impl_->slots[1].registered.load()) {
    throw std::logic_error(
        "Census interval requires an enabled, unstarted owner pair and ordered bounds");
  }
  impl_->begin = begin;
  impl_->end = end;
}

const Census::Intervals &Census::intervals(const Actor actor) const {
  if (!impl_) { throw std::logic_error("No census interval storage"); }
  return impl_->slots[static_cast<unsigned>(actor)].intervals;
}

void Census::validate_interval(const std::uint64_t begin, const std::uint64_t end) const {
  if (impl_ && impl_->end != 0 && (impl_->begin != begin || impl_->end != end)) {
    throw std::runtime_error("Census bounds disagree with the actual musical interval");
  }
}

bool Census::tracing() noexcept {
  return current.census != nullptr && current.census->impl_->end != 0;
}

void Census::event(const Event event,
                   const std::uint64_t ticks,
                   const std::uint64_t amount) noexcept {
  if (!tracing()) { return; }
  auto &owner = *current.census->impl_;
  owner.slots[current.actor].intervals[owner.phase(ticks)][static_cast<unsigned>(event)] += amount;
}

void Census::native_session(const NativeSession session) noexcept {
  if (!tracing()) { return; }
  auto &owner = *current.census->impl_;
  const auto phase = owner.phase(session.begin) == owner.phase(session.end)
                         ? owner.phase(session.end)
                         : static_cast<unsigned>(Phase::crossing);
  auto &counts = owner.slots[current.actor].intervals[phase];
  ++counts[static_cast<unsigned>(Event::session)];
  counts[static_cast<unsigned>(Event::retired)] += session.retired;
  if (session.retired == 0) { ++counts[static_cast<unsigned>(Event::empty)]; }
  for (auto index = static_cast<unsigned>(Event::yielded);
       index < static_cast<unsigned>(Event::count);
       ++index) {
    if ((session.endings & (std::uint64_t{1} << index)) != 0) { ++counts[index]; }
  }
}

void Census::report_intervals(std::ostream &output) const {
  if (!impl_ || impl_->end == 0) { return; }
  constexpr std::array names{"rollback",
                             "discarded",
                             "reinstall",
                             "destructive",
                             "data",
                             "interrupt",
                             "mode",
                             "reply",
                             "generation",
                             "announce",
                             "pressure",
                             "session",
                             "retired",
                             "empty",
                             "yielded",
                             "waiting",
                             "fault",
                             "peer-due",
                             "irq-step",
                             "timer-deadline",
                             "media-deadline",
                             "audio-deadline",
                             "midi-deadline",
                             "peer-deadline",
                             "dma-change",
                             "priority-change",
                             "timer-change",
                             "pending-interrupt",
                             "audio-change",
                             "media-change",
                             "peer-change",
                             "stopping",
                             "resetting",
                             "coupling-service",
                             "coupling-output",
                             "other-return"};
  static_assert(names.size() == static_cast<unsigned>(Event::count));
  constexpr std::array actors{"mcu", "dsp"};
  constexpr std::array phases{"startup", "active", "after", "crossing"};
  for (unsigned actor = 0; actor < actors.size(); ++actor) {
    for (unsigned phase = 0; phase < phases.size(); ++phase) {
      output << "INTERVAL actor=" << actors[actor] << " phase=" << phases[phase]
             << " begin=" << impl_->begin << " end=" << impl_->end;
      const auto &counts = impl_->slots[actor].intervals[phase];
      for (unsigned index = 0; index < names.size(); ++index) {
        output << ' ' << names[index] << '=' << counts[index];
      }
      output << '\n';
    }
  }
}

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
  unit_exit();
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

void Census::discarded_write(const unsigned space,
                             const bool readonly,
                             const Store store) noexcept {
  if (current.census == nullptr || space > 2) { return; }
  const auto index = space == 0 ? 0U : (2U * space) - (readonly ? 1U : 0U);
  auto &record = current.census->impl_->slots[current.actor].discarded_writes[index];
  if (record.count++ == 0) {
    record.first = record.minimum = record.maximum = store.address;
    record.pc = store.pc;
    record.value = store.value;
  }
  record.minimum = std::min(record.minimum, store.address);
  record.maximum = std::max(record.maximum, store.address);
}

std::span<const Census::DiscardedWrite> Census::discarded_writes() const noexcept {
  if (!impl_) { return {}; }
  return impl_->slots[static_cast<unsigned>(Actor::dsp)].discarded_writes;
}

void Census::unit_entry() noexcept {
  unit_exit();
  current.unit_open = current.census != nullptr && current.recording;
  current.unit_retired = 0;
}

void Census::unit_retired(const std::uint32_t count) noexcept {
  if (current.unit_open) {
    current.unit_retired += count;
  } else if (current.census != nullptr && current.recording) {
    current.census->impl_->slots[current.actor].unit_unattributed += count;
  }
}

void Census::unit_exit() noexcept {
  if (!current.unit_open) { return; }
  auto &slot = current.census->impl_->slots[current.actor];
  const auto count = current.unit_retired;
  ++slot.unit_count;
  slot.unit_sum += count;
  slot.unit_maximum = std::max(slot.unit_maximum, count);
  ++slot.unit_bins[std::min<std::size_t>(count, histogram_limit)];
  const auto band = count <= 32 ? 1U : 2U;
  slot.unit_weights[count < 8 ? 0U : band] += count;
  current.unit_open = false;
}

Census::Units Census::units() const noexcept {
  if (!impl_) { return {}; }
  const auto &slot = impl_->slots[static_cast<unsigned>(Actor::dsp)];
  return {.histogram = {.bins = slot.unit_bins,
                        .count = slot.unit_count,
                        .sum = slot.unit_sum,
                        .maximum = slot.unit_maximum},
          .region_retirements = slot.unit_weights,
          .boundary_crossings = slot.unit_crossings,
          .unattributed = slot.unit_unattributed};
}

void Census::recording(const bool active) noexcept {
  if (current.unit_open && current.recording != active) {
    ++current.census->impl_->slots[current.actor].unit_crossings;
    unit_exit();
  }
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
