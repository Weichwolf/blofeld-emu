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
  std::uint64_t causes = 0;
  std::uint64_t peripheral_events = 0;
  bool advanced = false;
  bool operator==(const PeripheralCalendarState &) const = default;
};
class PeripheralCalendar;

class PeripheralCalendarCheckpoint {
public:
  PeripheralCalendarCheckpoint(const PeripheralCalendarCheckpoint &) = default;
  PeripheralCalendarCheckpoint &operator=(const PeripheralCalendarCheckpoint &) = default;

private:
  friend class PeripheralCalendar;
  PeripheralCalendarCheckpoint(PeripheralCalendarState state,
                               const PeripheralCalendar *owner,
                               std::weak_ptr<const bool> lifetime);
  PeripheralCalendarState state_;
  const PeripheralCalendar *owner_;
  std::weak_ptr<const bool> lifetime_;
};

// Pure value-state calendar. Callers query actual sources under their native
// state guards, then supply observed deadlines and actual completed causes.
// Peripheral payload/protocol state is restored separately by the board owner.
// Peer causes are actual accepted bus writes, ordered by the peer owner. They
// may follow DSP/input work at the same tick but cannot invent an earlier event.
class PeripheralCalendar {
public:
  explicit PeripheralCalendar(PeripheralCalendarBoundary initial = {});
  PeripheralCalendar(const PeripheralCalendarCheckpoint &checkpoint,
                     const std::shared_ptr<const PeripheralEventIdentity> &identity,
                     std::optional<std::uint64_t> observed,
                     PeripheralCalendarBoundary restored);
  PeripheralCalendar(const PeripheralCalendar &) = delete;
  PeripheralCalendar &operator=(const PeripheralCalendar &) = delete;
  void bind(const std::shared_ptr<const PeripheralEventIdentity> &identity,
            std::optional<std::uint64_t> observed);
  void verify(const std::shared_ptr<const PeripheralEventIdentity> &identity,
              std::optional<std::uint64_t> observed) const;
  [[nodiscard]] bool first(std::optional<std::uint64_t> native,
                           std::optional<std::uint64_t> external,
                           PeripheralCalendarBoundary target) const;
  void
  caused(PeripheralCalendarCause cause, std::uint64_t ticks, std::optional<std::uint64_t> observed);
  void finish(PeripheralCalendarBoundary target);
  [[nodiscard]] PeripheralCalendarState state() const;
  [[nodiscard]] PeripheralCalendarCheckpoint checkpoint() const;
  void restore(const PeripheralCalendarCheckpoint &checkpoint,
               const std::shared_ptr<const PeripheralEventIdentity> &identity,
               std::optional<std::uint64_t> observed,
               PeripheralCalendarBoundary restored);

private:
  friend class DspFetchScheduledRun;
  void reset_unbound(PeripheralCalendarBoundary restored);
  [[nodiscard]] static PeripheralCalendarState
  completed(const PeripheralCalendarState &state, PeripheralCalendarBoundary target, bool advance);
  static void causal_time(const PeripheralCalendarState &state,
                          PeripheralCalendarCause cause,
                          std::uint64_t ticks);
  static void
  record_cause(PeripheralCalendarState &state, PeripheralCalendarCause cause, std::uint64_t ticks);
  static void source(const PeripheralCalendarState &state,
                     const std::shared_ptr<const PeripheralEventIdentity> &identity,
                     std::optional<std::uint64_t> observed);
  static void future(const PeripheralCalendarState &state, std::optional<std::uint64_t> observed);
  static void horizon(const PeripheralCalendarState &state, PeripheralCalendarBoundary next);
  static void restored_source(const PeripheralCalendarCheckpoint &checkpoint,
                              const std::shared_ptr<const PeripheralEventIdentity> &identity,
                              std::optional<std::uint64_t> observed,
                              PeripheralCalendarBoundary restored);
  [[noreturn]] static void fail(const char *reason);
  PeripheralCalendarState state_;
  std::shared_ptr<const bool> lifetime_ = std::make_shared<const bool>(true);
};
}
