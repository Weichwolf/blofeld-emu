// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <optional>

namespace blofeld::vi::dsp {
// DSP56371UM 11. CLK/2 counters advance analytically between real events.
// GPIO timer, event counter and capture modes; other modes fail on enable.
class Timers {
public:
  struct Write {
    dsp56300::Word address = 0, value = 0;
  };

  void reset() noexcept;
  [[nodiscard]] static bool contains(dsp56300::Word address) noexcept;
  [[nodiscard]] bool read(dsp56300::Word address, dsp56300::Word &value) const noexcept;
  [[nodiscard]] bool write(Write access) noexcept;
  [[nodiscard]] bool advance(std::uint64_t cycles) noexcept;
  [[nodiscard]] bool input(unsigned index, bool level) noexcept;
  [[nodiscard]] std::optional<std::uint64_t> deadline() const noexcept;
  [[nodiscard]] std::optional<dsp56300::Word> interrupt() const noexcept;

  [[nodiscard]] unsigned requests() const noexcept { return requests_; }

  [[nodiscard]] unsigned take_requests() noexcept;

  [[nodiscard]] const char *error() const noexcept { return error_; }

private:
  struct Timer {
    // TLR/TCPR have unspecified reset contents. Deterministic VM power-on
    // values are zero, retained across RESET. TCR and TCSR reset as documented.
    dsp56300::Word control = 0, load = 0, compare = 0, count = 0, captured = 0;
    bool preload = false, halted = false, level = false;
  };

  [[nodiscard]] bool prescaler_active() const noexcept;
  [[nodiscard]] std::uint64_t prescale(std::uint64_t ticks) noexcept;
  void count(unsigned index, std::uint64_t ticks) noexcept;
  void event(unsigned index, dsp56300::Word value) noexcept;
  [[nodiscard]] bool control(unsigned index, dsp56300::Word value) noexcept;
  [[nodiscard]] bool fail(const char *message) noexcept;
  std::array<Timer, 3> timers_;
  dsp56300::Word prescaler_ = 0, prescaler_count_ = 0;
  std::uint64_t cycles_ = 0;
  unsigned requests_ = 0;
  const char *error_ = nullptr;
};
}
