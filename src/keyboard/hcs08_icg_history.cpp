// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_icg_history.hpp"

#include "common/bus.hpp"
#include "keyboard/hcs08_icg_events.hpp"
#include "keyboard/hcs08_icg_status.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned status1 = 0x4a, control2_address = 0x49;
constexpr std::array faults{Hcs08IcgFault::loss_of_lock,
                            Hcs08IcgFault::loss_of_clock,
                            Hcs08IcgFault::loss_of_clock_and_lock};

Hcs08KnownByte intersect(const Hcs08KnownByte first, const Hcs08KnownByte second) {
  const auto known =
      static_cast<std::uint8_t>(first.known & second.known & ~(first.value ^ second.value));
  return {.value = static_cast<std::uint8_t>(first.value & known), .known = known};
}
} // namespace

Hcs08IcgHistory::Hcs08IcgHistory(const Hcs08IcgHistoryContract contract) {
  if (contract != Hcs08IcgHistoryContract::candidate_gt8a_unknown_events) {
    fail("configure", status1, "requires explicit candidate unknown-event contract");
  }
  histories_[0].emplace(Hcs08IcgStatusContract::candidate_gt8a_observed_bits);
}

void Hcs08IcgHistory::context(const ExecutionContext value) {
  context_ = value;
  for (unsigned index = 0; index < size_; ++index) { history(index).context(value); }
}

void Hcs08IcgHistory::reset() {
  history(0).reset();
  size_ = 1;
}

void Hcs08IcgHistory::observe(const Hcs08IcgStatusPoint point) {
  auto next = *this;
  for (unsigned index = 0; index < size_; ++index) { next.history(index).observe(point); }
  *this = next;
}

void Hcs08IcgHistory::control2(const std::uint8_t value) {
  auto next = *this;
  for (unsigned index = 0; index < size_; ++index) { next.history(index).control2(value); }
  *this = next;
}

void Hcs08IcgHistory::signal(const Hcs08IcgFault event) {
  auto next = *this;
  for (unsigned index = 0; index < size_; ++index) { next.history(index).signal(event); }
  *this = next;
}

Hcs08IcgStatus &Hcs08IcgHistory::history(const unsigned index) {
  if (index >= size_) { fail("access", status1, "event history index is out of range"); }
  auto &entry = histories_.at(index);
  if (!entry) { fail("access", status1, "event history is absent"); }
  return *entry;
}

const Hcs08IcgStatus &Hcs08IcgHistory::history(const unsigned index) const {
  if (index >= size_) { fail("access", status1, "event history index is out of range"); }
  const auto &entry = histories_.at(index);
  if (!entry) { fail("access", status1, "event history is absent"); }
  return *entry;
}

void Hcs08IcgHistory::append(const Hcs08IcgStatus &value) {
  for (unsigned index = 0; index < size_; ++index) {
    if (history(index).state() == value.state()) { return; }
  }
  if (size_ == maximum_histories) {
    fail("expand", status1, "event history exceeds the complete finite state space");
  }
  histories_[size_] = value;
  ++size_;
}

void Hcs08IcgHistory::unobserved_interval() {
  auto next = *this;
  next.size_ = 0;
  for (unsigned index = 0; index < size_; ++index) {
    auto prior = history(index);
    prior.observe({});
    next.append(prior);
    for (const auto event : faults) {
      auto candidate = prior;
      candidate.signal(event);
      next.append(candidate);
    }
  }
  *this = next;
}

Hcs08KnownByte Hcs08IcgHistory::read_known(const std::uint16_t address) {
  auto next = *this;
  auto result = next.history(0).read_known(address);
  for (unsigned index = 1; index < size_; ++index) {
    result = intersect(result, next.history(index).read_known(address));
  }
  *this = next;
  return result;
}

std::uint8_t Hcs08IcgHistory::read(const std::uint16_t address) {
  auto next = *this;
  const auto byte = next.read_known(address);
  if (byte.known != Hcs08KnownByte::all) {
    fail("read", address, "unknown live bits known=" + hex(byte.known));
  }
  *this = next;
  return byte.value;
}

void Hcs08IcgHistory::write(const ByteWrite access, const std::uint8_t known) {
  auto next = *this;
  for (unsigned index = 0; index < size_; ++index) { next.history(index).write(access, known); }
  *this = next;
}

Hcs08IcgFaultAction Hcs08IcgHistory::action(const bool interrupt_masked) const {
  std::optional<Hcs08IcgFaultAction> result;
  for (unsigned index = 0; index < size_; ++index) {
    const auto pending = history(index).action();
    const auto effective = interrupt_masked && pending == Hcs08IcgFaultAction::interrupt
                               ? Hcs08IcgFaultAction::none
                               : pending;
    if (result && *result != effective) {
      fail("action", control2_address, "unobserved event history affects IRQ/reset selection");
    }
    result = effective;
  }
  if (!result) { fail("action", status1, "missing event history"); }
  return *result;
}

[[noreturn]] void Hcs08IcgHistory::fail(const std::string &operation,
                                        const unsigned address,
                                        const std::string &reason) const {
  throw Fault("HCS08 ICG history " + operation + " address=" + hex(address) +
              " pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr) + " agent=CPU: " + reason);
}
} // namespace blofeld
