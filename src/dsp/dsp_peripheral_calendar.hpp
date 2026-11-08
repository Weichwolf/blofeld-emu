// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
struct PeripheralEventIdentity final {};

struct PeripheralCalendarBoundary {
  std::uint64_t ticks = 0;
  bool native_through = true;
  bool events_through = true;
  bool operator==(const PeripheralCalendarBoundary &) const = default;
};
enum class PeripheralCalendarCause : std::uint8_t { native, external, peripheral, peer };

struct PeripheralCalendarState {
  PeripheralCalendarBoundary horizon;
  std::shared_ptr<const PeripheralEventIdentity> identity;
  std::optional<std::uint64_t> pending = std::nullopt;
  std::optional<std::uint64_t> last_native = std::nullopt;
  std::optional<std::uint64_t> last_external = std::nullopt;
  std::optional<std::uint64_t> last_peripheral = std::nullopt;
  std::optional<std::uint64_t> last_peer = std::nullopt;
  bool advanced = false;
  bool operator==(const PeripheralCalendarState &) const = default;
};

// Callers query actual sources under their execution guards, then supply
// observed deadlines and actual completed causes. No state copies are needed
// to validate and commit a completed boundary.
// Peer causes are actual accepted bus writes, ordered by the peer owner. They
// may follow DSP/input work at the same tick but cannot invent an earlier event.
// New same-tick events require an explicitly open input boundary and no completed
// native cycle at that tick. Otherwise only an already pending event may remain.
class PeripheralCalendar {
public:
  explicit PeripheralCalendar(PeripheralCalendarBoundary initial = {});
  PeripheralCalendar(const PeripheralCalendar &) = delete;
  PeripheralCalendar &operator=(const PeripheralCalendar &) = delete;
  void bind(const std::shared_ptr<const PeripheralEventIdentity> &identity,
            std::optional<std::uint64_t> observed);
  void verify(const std::shared_ptr<const PeripheralEventIdentity> &identity,
              std::optional<std::uint64_t> observed) const;
  void
  caused(PeripheralCalendarCause cause, std::uint64_t ticks, std::optional<std::uint64_t> observed);
  void finish(PeripheralCalendarBoundary target);
  [[nodiscard]] PeripheralCalendarState state() const;

private:
  static void deadline_horizon(PeripheralCalendarBoundary boundary,
                               bool advanced,
                               std::optional<std::uint64_t> observed);
  static void causal_time(const PeripheralCalendarState &state,
                          PeripheralCalendarCause cause,
                          std::uint64_t ticks);
  [[nodiscard]] static std::optional<std::uint64_t> &
  cause_stamp(PeripheralCalendarState &state, PeripheralCalendarCause cause, std::uint64_t ticks);
  static void source(const PeripheralCalendarState &state,
                     const std::shared_ptr<const PeripheralEventIdentity> &identity,
                     std::optional<std::uint64_t> observed);
  static void future(const PeripheralCalendarState &state, std::optional<std::uint64_t> observed);
  static void horizon(const PeripheralCalendarState &state, PeripheralCalendarBoundary next);
  [[noreturn]] static void fail(const char *reason);
  PeripheralCalendarState state_;
};
}
