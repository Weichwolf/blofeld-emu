// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "diagnostics/thread_usage.hpp"

#include <array>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <span>

namespace blofeld::vi {
class Census {
  struct Slot;
  struct Impl;

public:
  enum class Actor : std::uint8_t { mcu, dsp };
  enum class Event : std::uint8_t {
    rollback,
    discarded,
    reinstall,
    destructive,
    data,
    interrupt,
    mode,
    reply,
    generation,
    announce,
    pressure,
    session,
    retired,
    empty,
    yielded,
    waiting,
    fault,
    peer_due,
    irq_step,
    timer_deadline,
    media_deadline,
    audio_deadline,
    midi_deadline,
    peer_deadline,
    dma_change,
    priority_change,
    timer_change,
    pending_interrupt,
    audio_change,
    media_change,
    peer_change,
    stopping,
    resetting,
    coupling_service,
    coupling_output,
    other_return,
    count
  };
  enum class Phase : std::uint8_t { startup, active, after, crossing, count };
  using Counts = std::array<std::uint64_t, static_cast<unsigned>(Event::count)>;
  using Intervals = std::array<Counts, static_cast<unsigned>(Phase::count)>;

  struct NativeSession {
    std::uint64_t begin = 0, end = 0, endings = 0;
    std::uint32_t retired = 0;
  };

  struct Guest {
    std::uint64_t instructions = 0, cycles = 0, idle = 0, epoch = 0;
    std::uint64_t region_instructions = 0, precise_instructions = 0;
    std::uint64_t batched_instructions = 0, deferred_instructions = 0;
    std::uint32_t pc = 0;
  };

  struct WaitUsage {
    std::uint64_t calls = 0, nanoseconds = 0;
    bool coherent = false;
  };

  struct OwnerUsage {
    host::diagnostics::ThreadUsage native;
    WaitUsage wait;
    int naming_error = 0;
  };

  struct Snapshot {
    std::uint64_t opened = 0, closed = 0;
    Guest guest;
    std::array<OwnerUsage, 2> owners{};
    bool valid = false;
  };

  struct DiscardedWrite {
    std::uint64_t count = 0;
    std::uint32_t first = 0, minimum = 0, maximum = 0, pc = 0, value = 0;
  };

  struct Store {
    std::uint32_t address = 0, pc = 0, value = 0;
  };

  struct Sessions {
    std::span<const std::uint64_t> bins;
    std::uint64_t count = 0, sum = 0;
    std::uint32_t maximum = 0;
  };

  struct Units {
    Sessions histogram;
    std::array<std::uint64_t, 3> region_retirements{};
    std::uint64_t boundary_crossings = 0, unattributed = 0;
  };

  [[nodiscard]] Units units() const noexcept;
  static void unit_entry() noexcept asm("blofeld_census_unit_entry");
  static void unit_retired(std::uint32_t count) noexcept asm("blofeld_census_unit_retired");
  static void unit_exit() noexcept;

  Census();
  explicit Census(bool enabled);
  ~Census();
  Census(const Census &) = delete;
  Census &operator=(const Census &) = delete;

  [[nodiscard]] bool enabled() const noexcept { return impl_ != nullptr; }

  [[nodiscard]] Snapshot capture() const noexcept;
  [[nodiscard]] Sessions sessions(Actor actor) const noexcept;
  [[nodiscard]] std::span<const DiscardedWrite> discarded_writes() const noexcept;
  static void discarded_write(unsigned space, bool readonly, Store store) noexcept
      asm("blofeld_census_discarded_write");
  void interval(std::uint64_t begin, std::uint64_t end);
  void validate_interval(std::uint64_t begin, std::uint64_t end) const;
  [[nodiscard]] const Intervals &intervals(Actor actor) const;
  void report_intervals(std::ostream &output) const;
  [[nodiscard]] static bool tracing() noexcept;
  static void event(Event event, std::uint64_t ticks, std::uint64_t amount = 1) noexcept;
  static void native_session(NativeSession session) noexcept;

  class Scope {
  public:
    Scope(Census &census, Actor actor) noexcept;
    ~Scope();
    Scope(const Scope &) = delete;
    Scope &operator=(const Scope &) = delete;
  };

  class GuestScope {
  public:
    using Read = Guest (*)(void *) noexcept;
    GuestScope(void *context, Read read) noexcept;
    ~GuestScope();
    GuestScope(const GuestScope &) = delete;
    GuestScope &operator=(const GuestScope &) = delete;
  };

  class Wait {
  public:
    Wait() noexcept;
    ~Wait();
    Wait(const Wait &) = delete;
    Wait &operator=(const Wait &) = delete;

  private:
    Slot *slot_ = nullptr;
    std::uint64_t begun_ = 0;
  };

  static void recording(bool active) noexcept;
  [[nodiscard]] static bool session_begin() noexcept;
  static void session_end(bool active_at_entry, std::uint32_t retired) noexcept;

private:
  [[nodiscard]] static WaitUsage wait_usage(const Slot &slot) noexcept;
  std::unique_ptr<Impl> impl_;
};
}
