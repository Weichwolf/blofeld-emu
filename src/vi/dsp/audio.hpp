// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/clock.hpp"
#include "dsp/dax.hpp"
#include "dsp/esai.hpp"
#include "io/pcm_output.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <stop_token>

namespace blofeld::vi::dsp {
// Nominal VM clock binding: EXTAL, HCKT_1 and ACI share 256 * 44100 Hz.
// Clock relations are a firmware-derived hypothesis, qualified separately from PCM delivery.
class Audio {
public:
  Audio(Esai &esai, Dax &dax, PcmOutput output, const std::stop_token &stop)
      : esai_(esai), dax_(dax), output_(output), stop_(stop) {}

  struct Epoch {
    std::uint64_t id = 0, origin = 0;
  };

  void reset(Epoch epoch) noexcept;
  [[nodiscard]] bool configure(std::uint64_t cycles, const Clock &clock);
  [[nodiscard]] std::optional<std::uint64_t> deadline() const noexcept;
  // Consume only the next due event. The caller services its real DMA requests
  // before advancing to another event or submitting a complete PCM block.
  [[nodiscard]] std::optional<std::uint64_t> transmit_deadline(unsigned slots) const noexcept;
  [[nodiscard]] bool clock();
  // Advance one event on the supplied copies and reject any second event
  // through the horizon. The live clock and PCM producer stay unchanged.
  [[nodiscard]] bool preview_clock(Esai &esai, Dax &dax, std::uint64_t through) const noexcept;
  [[nodiscard]] bool submit(bool partial = false);

  [[nodiscard]] const char *error() const noexcept { return error_; }

private:
  struct Pulse {
    std::uint32_t step = 0, step_remainder = 0, denominator = 1, remainder = 0;
    std::uint64_t floor = 0;
    [[nodiscard]] std::optional<std::uint64_t> deadline() const noexcept;
    [[nodiscard]] bool
    configure(std::uint64_t now, std::uint32_t numerator, std::uint32_t next_denominator) noexcept;
    [[nodiscard]] bool advance() noexcept;
  };

  struct Projection {
    std::optional<Clock> current, previous;
    std::uint64_t since = 0, origin = 0;
    [[nodiscard]] bool configure(std::uint64_t cycles, const Clock &clock) noexcept;
    [[nodiscard]] std::optional<std::uint64_t> at(std::uint64_t cycles) noexcept;
  };

  [[nodiscard]] bool capture(const Esai::Slot &slot, std::uint64_t cycles) noexcept;
  [[nodiscard]] bool clock(Esai &esai, Dax &dax) noexcept;
  [[nodiscard]] bool fail(const char *message) noexcept;
  Esai &esai_;
  Dax &dax_;
  PcmOutput output_;
  const std::stop_token &stop_;
  Pulse transmit_, digital_;
  Projection projection_;
  std::array<PcmOutput::Frame, 16> frames_{};
  std::array<std::uint64_t, 16> ticks_{};
  PcmOutput::Frame frame_{};
  std::optional<std::uint64_t> last_tick_;
  std::uint64_t epoch_ = 0, next_frame_ = 0;
  unsigned used_ = 0, frame_ticks_ = 0;
  std::uint16_t underruns_ = 0;
  bool have_left_ = false, underrun_ = false;
  const char *error_ = nullptr;
};
}
