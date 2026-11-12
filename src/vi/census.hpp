// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "diagnostics/thread_usage.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <span>

namespace blofeld::vi {
class Census {
  struct Slot;
  struct Impl;

public:
  enum class Actor : std::uint8_t { mcu, dsp };

  struct Guest {
    std::uint64_t instructions = 0, cycles = 0, idle = 0, epoch = 0;
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

  struct Sessions {
    std::span<const std::uint64_t> bins;
    std::uint64_t count = 0, sum = 0;
    std::uint32_t maximum = 0;
  };

  Census();
  explicit Census(bool enabled);
  ~Census();
  Census(const Census &) = delete;
  Census &operator=(const Census &) = delete;

  [[nodiscard]] bool enabled() const noexcept { return impl_ != nullptr; }

  [[nodiscard]] Snapshot capture() const noexcept;
  [[nodiscard]] Sessions sessions(Actor actor) const noexcept;

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
