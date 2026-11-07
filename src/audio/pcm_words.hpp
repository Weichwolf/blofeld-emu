// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/serial_pcm.hpp"
#include "dsp/dsp_esai.hpp"

#include <cstdint>
#include <optional>
#include <string_view>

namespace blofeld {
struct PcmWordClock {
  std::uint64_t ticks;
  std::uint64_t high_half_period;
};

// Borrow each actual transmitter latch once. The consumer may receive a complete
// frame before its presentation interval ends; timestamps retain the device clock.
// No bit decoding, allocation or ownership of the DSP's sample memory.
class PcmWords {
public:
  explicit PcmWords(SerialPcmFormat format, std::uint64_t after_ticks = 0);
  [[nodiscard]] std::optional<SerialPcmFrame>
  observe(const DspEsaiSlot &slot, unsigned lane, PcmWordClock clock);
  void reset();

  [[nodiscard]] std::uint64_t completed_frames() const { return frames_; }

  [[nodiscard]] bool locked() const { return locked_; }

  [[nodiscard]] bool faulted() const { return faulted_; }

  bool operator==(const PcmWords &) const = default;

private:
  [[noreturn]] void fail(std::uint64_t ticks, std::string_view reason);
  [[nodiscard]] std::uint64_t timing(const DspEsaiSlot &slot, unsigned lane, PcmWordClock clock);
  SerialPcmFormat format_;
  SerialPcmFrame frame_;
  std::uint64_t after_ticks_ = 0;
  std::uint64_t sequence_ = 0;
  std::uint64_t frames_ = 0;
  unsigned next_ = 0;
  bool locked_ = false;
  bool faulted_ = false;
};
}
