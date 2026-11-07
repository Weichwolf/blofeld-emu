// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/serial_pcm.hpp"

#include <cstdint>
#include <optional>
#include <string_view>

namespace blofeld {
// Bounded receiver; copies retain alignment, partial words and the clock phase.
// Success allocates nothing. Fault diagnostics throw and are not RT callbacks.
class SerialPcmReceiver {
public:
  explicit SerialPcmReceiver(SerialPcmFormat format);
  [[nodiscard]] std::optional<SerialPcmFrame> observe(std::uint64_t ticks, SerialPcmPins pins);
  [[nodiscard]] std::optional<SerialPcmFrame> observe_ordered(SerialPcmStamp stamp,
                                                              SerialPcmPins pins);
  void reset();
  bool operator==(const SerialPcmReceiver &) const = default;

  [[nodiscard]] std::uint64_t completed_frames() const { return frames_; }

  [[nodiscard]] bool locked() const { return locked_; }

  [[nodiscard]] bool faulted() const { return faulted_; }

private:
  [[noreturn]] void fail(SerialPcmStamp stamp, std::string_view reason);
  [[nodiscard]] std::optional<SerialPcmFrame>
  sample(SerialPcmStamp stamp, bool active, std::optional<bool> data);
  [[nodiscard]] std::int32_t signed_word() const;
  SerialPcmFormat format_;
  SerialPcmFrame frame_;
  std::optional<bool> previous_clock_;
  std::optional<SerialPcmStamp> previous_stamp_;
  std::uint64_t frames_ = 0;
  std::int64_t word_modulus_ = 0;
  std::uint32_t word_sign_ = 0;
  std::uint32_t word_ = 0;
  unsigned slot_ = 0;
  unsigned bit_ = 0;
  bool seen_inactive_ = false;
  bool locked_ = false;
  bool faulted_ = false;
};
}
