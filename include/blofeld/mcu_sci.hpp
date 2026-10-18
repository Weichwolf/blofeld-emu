// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"

#include <cstdint>
#include <optional>
#include <string_view>

namespace blofeld {

struct SciClockSlice {
  unsigned bus_cycles = 0;
  std::optional<bool> receive_level = std::nullopt;
};

// MC9S12UF32 SCI in normal NRZ mode. Clock slices have a constant RX pad level;
// the board scheduler must split a slice at every external signal transition.
class McuSci {
public:
  void reset();
  [[nodiscard]] static std::string_view register_name(std::uint16_t offset);
  [[nodiscard]] std::uint8_t read(std::uint16_t offset);
  void write(ByteWrite write);
  void advance(SciClockSlice slice);
  [[nodiscard]] bool receiver_enabled() const;
  [[nodiscard]] bool interrupt_pending() const;
  [[nodiscard]] unsigned bus_cycles_until_sample() const;
  [[nodiscard]] std::optional<bool> transmit_level() const;

private:
  static constexpr unsigned reset_status = 0xc0;

  struct TxFrame {
    unsigned bits = 0;
    unsigned length = 0;
  };
  enum class RxStage : std::uint8_t { idle, start, data, stop };

  struct Receiver {
    RxStage stage = RxStage::idle;
    unsigned phase = 0;
    unsigned bit = 0;
    unsigned samples = 0;
    unsigned start_samples = 0;
    unsigned preceding_ones = 0;
    unsigned idle_ticks = 0;
    unsigned falling_age = 16;
    unsigned word = 0;
    unsigned errors = 0;
    bool previous_level = true;
    bool previous_bit = false;
    bool sampled = false;
    bool active = false;
    bool idle_armed = true;
  };

  [[nodiscard]] unsigned character_bits() const;
  [[nodiscard]] bool busy() const;
  [[nodiscard]] TxFrame data_frame() const;
  [[nodiscard]] TxFrame idle_frame() const;
  [[nodiscard]] std::optional<TxFrame> prepare_transmission();
  void transmit_tick();
  void receive_tick(bool level);
  void receive_start(bool level);
  void receive_data(bool level);
  void receive_complete();
  void track_idle(bool level);
  void write_control2(std::uint8_t value);
  void write_data(std::uint8_t value);
  void require_transfer_known() const;
  std::uint16_t baud_ = 4;
  std::uint8_t baud_high_ = 0;
  std::uint8_t control1_ = 0;
  std::uint8_t control2_ = 0;
  std::uint8_t control3_ = 0;
  std::uint8_t status_ = reset_status;
  std::uint8_t seen_tx_ = 0;
  std::uint8_t seen_rx_ = 0;
  std::uint16_t received_ = 0;
  std::uint16_t transmit_ = 0;
  unsigned divider_phase_ = 0;
  unsigned transmitter_phase_ = 0;
  bool generator_started_ = false;
  bool baud_high_pending_ = false;
  bool preamble_queued_ = false;
  std::optional<TxFrame> transmitting_;
  std::optional<TxFrame> prepared_;
  Receiver receiver_;
};

}
