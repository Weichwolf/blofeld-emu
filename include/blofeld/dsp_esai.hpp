// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
enum class DspEsaiInstance : std::uint8_t { primary, secondary };

struct DspEsaiInputs {
  std::optional<bool> clock;
  std::optional<bool> frame_sync;
};

// Bits0..5 identify TX0..TX5, independently of physical port pad numbering.
struct DspEsaiOutputs {
  std::uint32_t driven = 0;
  std::uint32_t known = 0;
  std::uint32_t levels = 0;
  bool operator==(const DspEsaiOutputs &) const = default;
};

// DSP56371UM chapters8/9: externally clocked, bit-sync normal/network transmitter.
// No implicit board clock, RX data, DMA, interrupt delivery or host audio synthesis.
class DspEsai {
public:
  explicit DspEsai(DspEsaiInstance instance);
  void reset();
  void set_active(bool active);
  [[nodiscard]] static bool handles(unsigned offset);
  [[nodiscard]] std::uint32_t read(unsigned offset);
  void write(unsigned offset, std::uint32_t value);
  void observe(DspEsaiInputs inputs);

  [[nodiscard]] DspEsaiOutputs outputs() const;

  [[nodiscard]] bool needs_transmit_clock() const;

  [[nodiscard]] std::uint64_t frames_started() const { return frames_; }

  [[nodiscard]] std::uint64_t slots_started() const { return slots_; }

private:
  struct Format {
    unsigned slot = 0;
    unsigned word = 0;
  };

  [[nodiscard]] std::string address(unsigned offset) const;
  [[nodiscard]] Format format() const;
  [[nodiscard]] unsigned enabled() const;
  [[nodiscard]] unsigned frame_slots() const;
  void validate() const;
  void require_observed_clock() const;
  void reset_paths();
  void write_checked(unsigned offset, std::uint32_t value);
  void write_control(std::uint32_t value);
  void write_data(unsigned channel, std::uint32_t value);
  void clear_error(unsigned written);
  void observe_checked(DspEsaiInputs inputs);
  void launch();
  void start_frame();
  void start_slot();
  void emit_bit();
  [[nodiscard]] unsigned data_bit(unsigned position) const;
  DspEsaiInstance instance_;
  std::array<std::uint32_t, 6> data_{};
  std::array<std::uint32_t, 6> shift_{};
  std::uint32_t control_ = 0;
  std::uint32_t clock_control_ = 0;
  std::uint32_t common_control_ = 0;
  std::uint32_t receive_control_ = 0;
  std::uint32_t status_ = 0;
  std::uint32_t transmit_mask_ = UINT32_MAX;
  std::uint32_t latched_mask_ = UINT32_MAX;
  std::uint32_t receive_mask_ = UINT32_MAX;
  unsigned known_ = 0;
  unsigned shift_known_ = 0;
  unsigned written_ = 0;
  unsigned clear_written_ = 0;
  unsigned running_ = 0;
  unsigned stop_after_slot_ = 0;
  unsigned slot_ = 0;
  unsigned position_ = 0;
  std::uint64_t frames_ = 0;
  std::uint64_t slots_ = 0;
  std::optional<bool> previous_clock_;
  bool previous_sync_ = false;
  bool active_ = false;
  bool baseline_ = false;
  bool frame_ = false;
  bool pending_frame_ = false;
  bool slot_driven_ = false;
  bool skip_ = false;
  bool clear_armed_ = false;
  DspEsaiOutputs output_;
};
}
