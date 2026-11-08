// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <dsp56300/memory.hpp>
#include <optional>

namespace blofeld::vi::dsp {
// DSP56371 ESAI_1 transmit word transactions. The owner supplies real HCKT slot
// events; configuration alone does not advance the device or manufacture PCM.
class Esai {
public:
  struct Write {
    dsp56300::Word address = 0, value = 0;
  };

  struct Format {
    unsigned word_bits = 0, slots = 0, slot_ticks = 0;
  };

  struct Slot {
    unsigned index = 0, enabled = 0, known = 0;
    std::array<dsp56300::Word, 6> samples{};
    bool repeated = false;
  };

  Esai() noexcept { reset(); }

  void reset() noexcept;
  void connected(bool active) noexcept;

  [[nodiscard]] bool connected() const noexcept { return connected_; }

  [[nodiscard]] bool active() const noexcept;

  [[nodiscard]] static bool contains(dsp56300::Word address) noexcept;
  [[nodiscard]] static bool data_register(dsp56300::Word address) noexcept;
  [[nodiscard]] bool read(dsp56300::Word address, dsp56300::Word &value) noexcept;
  [[nodiscard]] bool write(Write access) noexcept;
  [[nodiscard]] std::optional<Format> format() const noexcept;
  [[nodiscard]] std::optional<Slot> clock_slot() noexcept;
  [[nodiscard]] bool request() const noexcept;

  [[nodiscard]] const char *error() const noexcept { return error_; }

private:
  static constexpr dsp56300::Word all_slots = 0xffffffff;
  void reset_transmit() noexcept;
  void accept_data() noexcept;
  std::array<dsp56300::Word, 6> data_{};
  dsp56300::Word control_ = 0, clock_ = 0, common_ = 0;
  dsp56300::Word receive_control_ = 0, receive_clock_ = 0, status_ = 0;
  dsp56300::Word transmit_mask_ = all_slots, frame_mask_ = all_slots;
  dsp56300::Word receive_mask_ = all_slots;
  unsigned written_ = 0, known_ = 0, slot_ = 0, active_transmitters_ = 0;
  bool connected_ = false, skip_ = false, observed_underrun_ = false;
  const char *error_ = nullptr;
};
}
