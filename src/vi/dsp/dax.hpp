// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <dsp56300/memory.hpp>
#include <optional>

namespace blofeld::vi::dsp {
// DSP56371UM 10: complete audio-word uploads, without a serial bit encoder.
// The execution owner supplies subframe events from the selected clock source.
class Dax {
public:
  struct Write {
    dsp56300::Word address = 0, value = 0;
  };

  struct Clock {
    bool external = false;
    unsigned frame_ticks = 0;
  };

  struct Frame {
    std::array<dsp56300::Word, 2> samples{};
    std::optional<dsp56300::Word> non_audio;
    unsigned known = 0, index = 0;
    bool repeated = false;
  };

  void reset() noexcept;
  void connected(bool active) noexcept;

  [[nodiscard]] bool connected() const noexcept { return connected_; }

  [[nodiscard]] static bool contains(dsp56300::Word address) noexcept;
  [[nodiscard]] static bool data_register(dsp56300::Word address) noexcept;
  [[nodiscard]] bool read(dsp56300::Word address, dsp56300::Word &value) noexcept;
  [[nodiscard]] bool write(Write access) noexcept;
  [[nodiscard]] Clock clock() const noexcept;
  // A starts a frame, captures non-audio bits and requests refill. B completes
  // its pair. Unknown first data stays unknown; underruns repeat the last pair.
  [[nodiscard]] std::optional<Frame> clock_subframe() noexcept;
  [[nodiscard]] bool request() const noexcept;
  [[nodiscard]] std::optional<dsp56300::Word> interrupt() const noexcept;

private:
  struct Pending {
    unsigned channel = 0;
    dsp56300::Word value = 0;
  };

  void reset_transmit() noexcept;
  void transfer() noexcept;
  void upload(unsigned channel) noexcept;
  std::array<std::optional<dsp56300::Word>, 2> buffers_;
  std::optional<Pending> pending_;
  std::optional<dsp56300::Word> non_audio_;
  dsp56300::Word control_ = 0, status_ = 0;
  Frame current_, previous_;
  unsigned write_channel_ = 0, channel_ = 0, frame_ = 0, clear_writes_ = 0;
  bool connected_ = false, observed_underrun_ = false;
};
}
