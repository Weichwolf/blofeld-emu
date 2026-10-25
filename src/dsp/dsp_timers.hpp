// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld {

struct DspTimerWrite {
  static constexpr std::uint32_t word_mask = 0xffffff;
  std::uint32_t address = 0;
  std::uint32_t value = 0;
};

struct DspTimerPin {
  bool output = false;
  std::optional<bool> level;
};

// DSP56371 modes zero, event-three and capture-six (TRM=0). Clocks and synchronized
// input samples are explicit observations; raw pads do not invent timer events.
class DspTimers {
public:
  DspTimers();
  void reset();
  [[nodiscard]] static bool handles(std::uint32_t address);
  [[nodiscard]] static bool is_control(std::uint32_t address);
  [[nodiscard]] std::uint32_t read(std::uint32_t address,
                                   std::uint32_t mask = DspTimerWrite::word_mask) const;
  void write(DspTimerWrite access);
  void set_tio_drive(unsigned index, std::optional<bool> level);
  void observe_synchronized_tio(unsigned index, bool level);
  [[nodiscard]] DspTimerPin observe_tio(unsigned index) const;
  void advance_clk2(std::uint64_t edges);
  [[nodiscard]] std::vector<std::uint32_t> interrupt_vectors() const;
  void acknowledge_interrupt(std::uint32_t vector);

private:
  struct Timer {
    std::uint32_t control = 0;
    std::uint32_t flags = 0;
    std::uint32_t count = 0;
    std::optional<std::uint32_t> load;
    std::optional<std::uint32_t> compare;
    bool first_clock = false;
    bool reload_pending = false;
    std::uint32_t captured = 0;
    std::optional<bool> synchronized;
    bool synchronized_qualified = false;
    bool halted = false;
  };

  struct CounterAdvance {
    std::uint32_t compare = 0;
    std::uint64_t edges = 0;
  };

  void write_checked(DspTimerWrite access);
  void validate_control(std::size_t index, std::uint32_t value) const;
  void write_control(std::size_t index, std::uint32_t value);
  [[nodiscard]] bool prescaler_enabled() const;
  [[nodiscard]] std::uint64_t advance_prescaler(std::uint64_t edges);
  static void advance_timer(Timer &timer, std::size_t index, std::uint64_t edges);
  static void advance_free(Timer &timer, CounterAdvance advance);
  static void advance_capture(Timer &timer, std::size_t index, std::uint64_t edges);
  std::array<Timer, 3> timers_{};
  std::array<std::optional<bool>, 2> external_{};
  std::uint32_t prescaler_load_ = 0;
  std::uint32_t prescaler_count_ = 0;
};

}
