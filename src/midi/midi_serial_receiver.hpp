// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace blofeld {
struct MidiSerialByte {
  std::uint8_t value = 0;
  std::uint64_t start_ticks = 0, stop_sample_ticks = 0;
  bool operator==(const MidiSerialByte &) const = default;
};

// Passive 31250-baud 8-N-1 sink. The caller reports every wire change and
// eventually advances through the last stop sample; no device clock is driven.
// A changed level takes effect before a sample at the same tick. Repeated
// unchanged observations are inert; contradictory same-tick updates refuse.
// Unknown idle is unconnected. Unknown active samples quarantine the decoder.
class MidiSerialReceiver {
public:
  explicit MidiSerialReceiver(std::uint64_t timebase_hz);
  [[nodiscard]] std::optional<MidiSerialByte> observe(std::uint64_t ticks,
                                                      std::optional<bool> level);

  [[nodiscard]] std::optional<std::uint64_t> next_sample_ticks() const noexcept { return next_; }

  [[nodiscard]] std::uint64_t received_bytes() const noexcept { return received_; }

  [[nodiscard]] bool faulted() const noexcept { return faulted_; }

private:
  [[nodiscard]] std::optional<MidiSerialByte> sample(bool level);
  void schedule();
  [[noreturn]] void fail(std::uint64_t ticks, std::string_view reason);
  std::uint64_t timebase_;
  std::optional<std::uint64_t> last_, next_;
  std::optional<bool> previous_;
  std::uint64_t start_ = 0, received_ = 0;
  unsigned slot_ = 0, word_ = 0;
  bool faulted_ = false;
};
}
