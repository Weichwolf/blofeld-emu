// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace blofeld {
enum class DspEsaiInstance : std::uint8_t { primary, secondary };
enum class DspEsaiClockTiming : std::uint8_t { unqualified, reference_reset_edges };
enum class DspEsaiConfigurationContract : std::uint8_t {
  documented_reset,
  cold_unclocked_reference,
  cold_before_bit_clock_reference
};

struct DspEsaiInputs {
  std::optional<bool> clock = std::nullopt;
  std::optional<bool> frame_sync = std::nullopt;
  std::optional<bool> high_clock = std::nullopt;
  DspEsaiClockTiming timing = DspEsaiClockTiming::unqualified;
};

// Bit0 is SCKT, bit1 is FST. Only output directions drive these logical signals.
struct DspEsaiClockOutputs {
  unsigned driven = 0;
  unsigned known = 0;
  unsigned levels = 0;
  bool operator==(const DspEsaiClockOutputs &) const = default;
};

// Bits0..5 identify TX0..TX5, independently of physical port pad numbering.
struct DspEsaiOutputs {
  std::uint32_t driven = 0;
  std::uint32_t known = 0;
  std::uint32_t levels = 0;
  bool operator==(const DspEsaiOutputs &) const = default;
};

// A borrowed view of the actual transmitter latch, valid until the next write/clock.
// No serial decoding or guessed data; lane knowledge and ownership remain explicit.
struct DspEsaiSlot {
  std::span<const std::uint32_t, 6> words;
  std::uint64_t sequence;
  unsigned index, channels, slot_bits, word_bits, shift;
  unsigned driven, known, high_edges_per_bit;
  bool msb_first;
};

// DSP56371UM chapters8/9: normal/network transmitter with explicit clock inputs.
// No implicit board clock, RX data, DMA timing or host audio synthesis.
class DspEsai {
public:
  explicit DspEsai(DspEsaiInstance instance);
  void reset();
  // Explicit software-reference hypothesis, retained through hardware/software reset.
  void set_configuration_contract(DspEsaiConfigurationContract contract);
  void set_active(bool active);
  [[nodiscard]] static bool handles(unsigned offset);
  [[nodiscard]] std::uint32_t read(unsigned offset);
  void write(unsigned offset, std::uint32_t value);
  void observe(DspEsaiInputs inputs);
  // Periodic HCKT transitions from the already observed level. The caller must
  // stop at high_clock_edges_until_event() and synchronize before register/pad changes.
  // Status, DMA requests and IRQs retain the existing transmitter implementation;
  // quiet clock spans advance arithmetically between observable events.
  void advance_high_clock(std::uint64_t edges);
  // Pure horizon to a slot load, frame/status transition or required external
  // synchronization. nullopt means no running generator/transmitter event.
  [[nodiscard]] std::optional<std::uint64_t> high_clock_edges_until_event() const;

  [[nodiscard]] DspEsaiOutputs outputs() const;
  [[nodiscard]] std::optional<DspEsaiSlot> transmitted_slot() const;
  [[nodiscard]] DspEsaiClockOutputs clock_outputs() const;
  [[nodiscard]] unsigned required_clock_pads() const;
  [[nodiscard]] bool transmit_request() const;
  // Pure data/even-data/exception request; does not read SAISR or clear TUE.
  [[nodiscard]] std::optional<std::uint32_t> interrupt_vector() const;

  [[nodiscard]] bool needs_transmit_clock() const;

  [[nodiscard]] std::uint64_t frames_started() const { return frames_; }

  [[nodiscard]] std::uint64_t slots_started() const { return slots_; }

private:
  friend class DspBus;

  enum class Configuration : std::uint8_t { common, transmit_clock };

  struct Format {
    unsigned slot = 0;
    unsigned word = 0;
  };

  [[nodiscard]] std::string address(unsigned offset) const;
  [[nodiscard]] Format format() const;
  [[nodiscard]] unsigned enabled() const;
  [[nodiscard]] unsigned frame_slots() const;
  [[nodiscard]] bool cold_configuration_allowed() const;
  void validate() const;
  void require_observed_clock() const;
  [[nodiscard]] bool clock_generator() const;
  [[nodiscard]] bool frame_generator() const;
  [[nodiscard]] bool generator_running() const;
  [[nodiscard]] unsigned high_clock_divisor() const;
  void rebaseline();
  void observe_high_clock(DspEsaiInputs inputs);
  void advance_high_clock_checked(std::uint64_t edges);
  [[nodiscard]] std::uint64_t high_edges_for_toggles(std::uint64_t toggles) const;
  [[nodiscard]] unsigned launches_until_event() const;
  void skip_quiet_clock_toggles(std::uint64_t toggles);
  void observe_bit_clock(DspEsaiInputs inputs);
  void clock_edge(bool clock, std::optional<bool> sync);
  void generate_frame();
  void update_frame_flag();
  void reset_paths();
  void write_checked(unsigned offset, std::uint32_t value);
  void write_control(std::uint32_t value);
  void write_configuration(Configuration kind, std::uint32_t value);
  void write_data(unsigned channel, std::uint32_t value);
  void clear_error(unsigned written);
  void observe_checked(DspEsaiInputs inputs);
  void launch();
  void start_frame();
  void start_slot();
  void emit_bit();
  [[nodiscard]] unsigned data_bit(unsigned position) const;
  DspEsaiInstance instance_;
  DspEsaiConfigurationContract configuration_contract_ =
      DspEsaiConfigurationContract::documented_reset;
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
  unsigned latched_slot_ = 0;
  unsigned position_ = 0;
  std::uint64_t frames_ = 0;
  std::uint64_t slots_ = 0;
  std::optional<bool> previous_clock_;
  DspEsaiInputs observed_;
  unsigned high_phase_ = 0;
  unsigned frame_position_ = 0;
  bool generated_clock_ = false;
  bool generated_sync_ = false;
  bool frame_clocked_ = false;
  bool previous_sync_ = false;
  bool active_ = false;
  bool cold_configuration_ = true;
  bool baseline_ = false;
  bool frame_ = false;
  bool pending_frame_ = false;
  bool slot_driven_ = false;
  bool skip_ = false;
  bool clear_armed_ = false;
  DspEsaiOutputs output_;
};
}
