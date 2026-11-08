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

  void reset(std::uint64_t epoch) noexcept;
  [[nodiscard]] bool configure(std::uint64_t cycles, Clock::Ratio ratio);
  [[nodiscard]] std::optional<std::uint64_t> deadline() const noexcept;
  // Consume only the next due event. The caller services its real DMA requests
  // before advancing to another event or submitting a complete PCM block.
  [[nodiscard]] bool clock();
  [[nodiscard]] bool submit(bool partial = false);

  [[nodiscard]] const char *error() const noexcept { return error_; }

private:
  struct Pulse {
    std::uint32_t numerator = 0, denominator = 1, remainder = 0;
    std::uint64_t floor = 0;
    [[nodiscard]] std::optional<std::uint64_t> deadline() const noexcept;
    [[nodiscard]] bool configure(std::uint64_t now,
                                 std::uint32_t next_numerator,
                                 std::uint32_t next_denominator) noexcept;
    [[nodiscard]] bool advance() noexcept;
  };

  [[nodiscard]] bool capture(const Esai::Slot &slot) noexcept;
  [[nodiscard]] bool fail(const char *message) noexcept;
  Esai &esai_;
  Dax &dax_;
  PcmOutput output_;
  const std::stop_token &stop_;
  Pulse transmit_, digital_;
  std::array<PcmOutput::Frame, 16> frames_{};
  PcmOutput::Frame frame_{};
  std::uint64_t epoch_ = 0, next_frame_ = 0;
  unsigned used_ = 0, frame_ticks_ = 0;
  std::uint16_t underruns_ = 0;
  bool have_left_ = false, underrun_ = false;
  const char *error_ = nullptr;
};
}
