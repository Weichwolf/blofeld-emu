// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "mcu/devices.hpp"
#include "mcu/io.hpp"
#include "mcu/memory.hpp"

#include <array>
#include <atomic>
#include <cpu12/core.hpp>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <stop_token>

namespace blofeld::vi::mcu {
class StateProbe {
public:
  using Digest = std::array<std::uint8_t, 32>;
  static constexpr std::size_t sections = 9, retained = 64, pending_limit = 128;

  struct Record {
    std::uint64_t ordinal = 0, instructions = 0, cycles = 0, ticks = 0, idle = 0;
    std::uint16_t pc = 0;
    std::uint64_t reset_epoch = 0;
    bool reset_held = false;
    std::array<Digest, sections> parts{};
    Digest chain{};
  };

  class Scope {
  public:
    explicit Scope(StateProbe *probe);
    ~Scope();
    Scope(const Scope &) = delete;
    Scope &operator=(const Scope &) = delete;
  };

  explicit StateProbe(std::stop_source *stop = nullptr) : stop_(stop) {}

  void end_at(std::uint64_t ticks);
  void pcm_complete() noexcept;
  [[nodiscard]] bool stopping(std::uint64_t ticks) const noexcept;
  [[nodiscard]] static StateProbe *active() noexcept;
  [[nodiscard]] static Record capture(const cpu12::Core &core,
                                      const Memory &memory,
                                      const Devices &devices,
                                      const Io &io,
                                      std::uint64_t instructions);
  void executed(std::uint32_t instructions, bool replaying) noexcept;
  void wait(Record record, bool speculative);
  void commit(std::uint64_t instructions, cpu12::Progress progress, std::uint64_t ticks);
  void rollback(std::uint64_t discarded) noexcept;
  void finish(Record record) noexcept;
  void report(std::ostream &output) const;

  [[nodiscard]] std::uint64_t count() const noexcept { return count_; }

  [[nodiscard]] std::size_t pending() const noexcept { return pending_count_; }

private:
  [[nodiscard]] static Digest timer_state(const Timer &timer);
  [[nodiscard]] static Digest clock_state(const Clock &clock);
  [[nodiscard]] static Digest sci_state(const Sci &sci, const char *midi_error);
  [[nodiscard]] static Digest usb_state(const Usb &usb);
  [[nodiscard]] static Digest gpio_state(const Io::Snapshot &local);
  [[nodiscard]] static Digest flash_state(const McuFlash &flash);
  [[nodiscard]] static Digest lcd_state(const DisplayPort &display);
  void append(Record record);
  std::array<Record, retained> records_{};
  std::array<Record, pending_limit> pending_{};
  std::optional<Record> first_, final_, ending_;
  std::stop_source *stop_ = nullptr;
  std::atomic<std::uint64_t> end_{0};
  std::atomic<bool> pcm_done_{false};
  Digest chain_{};
  std::size_t pending_count_ = 0;
  std::uint64_t count_ = 0, executed_ = 0, discarded_ = 0, replayed_ = 0;
  std::uint64_t confirmed_instructions_ = 0, confirmed_cycles_ = 0, confirmed_ticks_ = 0;
};
}
