// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_sci.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <optional>
#include <string_view>
#include <utility>

namespace blofeld {
namespace {
constexpr unsigned oversampling = 16;
constexpr unsigned data_empty = 0x80;
constexpr unsigned transmission_complete = 0x40;
constexpr unsigned receive_full = 0x20;
constexpr unsigned idle_flag = 0x10;
constexpr unsigned overrun = 8;
constexpr unsigned noise = 4;
constexpr unsigned framing_error = 2;
constexpr unsigned parity_error = 1;
constexpr unsigned receiver_flags = 0x3f;
constexpr unsigned transmitter_flags = 0xc0;
constexpr unsigned transmitter_enable = 8;
constexpr unsigned receiver_enable = 4;
constexpr unsigned unsupported_control2 = 3;
constexpr unsigned transmitter_interrupts = 0xc0;
constexpr unsigned receive_interrupt = 0x20;
constexpr unsigned idle_interrupt = 0x10;
constexpr unsigned loop_mode = 0x80;
constexpr unsigned stop_in_wait = 0x40;
constexpr unsigned nine_bit_mode = 0x10;
constexpr unsigned idle_after_stop = 4;
constexpr unsigned parity_enable = 2;
constexpr unsigned odd_parity = 1;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned ninth_bit = 0x100;
constexpr unsigned high_baud_mask = 0x1f;
constexpr unsigned transmit_high_bit = 0x40;
constexpr unsigned status2_mask = 0x0a;
constexpr unsigned maximum_clock_slice = 1048576;
constexpr unsigned sample_first = 8;
constexpr unsigned sample_last = 10;
constexpr unsigned transfer_phase = 9;
}

bool McuSci::runs_in_wait() const {
  return (control1_ & stop_in_wait) == 0;
}

void McuSci::reset() {
  *this = McuSci{};
}

std::string_view McuSci::register_name(const std::uint16_t offset) {
  switch (offset) {
    case hc12::sci_baud_high: return "SCIBDH";
    case hc12::sci_baud_low: return "SCIBDL";
    case hc12::sci_control1: return "SCICR1";
    case hc12::sci_control2: return "SCICR2";
    case hc12::sci_status1: return "SCISR1";
    case hc12::sci_status2: return "SCISR2";
    case hc12::sci_data_high: return "SCIDRH";
    case hc12::sci_data_low: return "SCIDRL";
    default: return {};
  }
}

unsigned McuSci::character_bits() const {
  return (control1_ & nine_bit_mode) != 0 ? 9 : 8;
}

bool McuSci::receiver_enabled() const {
  return (control2_ & receiver_enable) != 0;
}

bool McuSci::interrupt_pending() const {
  const bool transmit = (status_ & control2_ & transmitter_interrupts) != 0;
  const bool receive =
      (control2_ & receive_interrupt) != 0 && (status_ & (receive_full | overrun)) != 0;
  const bool idle = (control2_ & idle_interrupt) != 0 && (status_ & idle_flag) != 0;
  if (transmit || receive || idle) { return true; }
  if ((control2_ & receive_interrupt) != 0) { require_transfer_known(); }
  return false;
}

bool McuSci::busy() const {
  return transmitting_.has_value() || prepared_.has_value() || receiver_.stage != RxStage::idle;
}

void McuSci::require_transfer_known() const {
  // The guide specifies the RT8/9/10 stop decision but does not unambiguously
  // specify all data/flag transfer edges. Do not hide that unresolved interval.
  if (receiver_.stage == RxStage::stop && receiver_.sampled) {
    throw Fault("Unverified SCI stop-to-data flag transfer phase at SCISR1 offset=" +
                hex(hc12::sci_status1));
  }
}

std::uint8_t McuSci::read(const std::uint16_t offset) {
  switch (offset) {
    case hc12::sci_baud_high:
      if (baud_high_pending_) {
        throw Fault("Unspecified SCI baud-high readback before low-byte commit");
      }
      return static_cast<std::uint8_t>(baud_ >> 8U);
    case hc12::sci_baud_low: return static_cast<std::uint8_t>(baud_ & byte_mask);
    case hc12::sci_control1: return control1_;
    case hc12::sci_control2: return control2_;
    case hc12::sci_status1:
      require_transfer_known();
      seen_tx_ = static_cast<std::uint8_t>(status_ & transmitter_flags);
      seen_rx_ = static_cast<std::uint8_t>(status_ & receiver_flags);
      return status_;
    case hc12::sci_status2:
      return static_cast<std::uint8_t>(control3_ | (receiver_.active ? 1U : 0U));
    case hc12::sci_data_high:
      require_transfer_known();
      return static_cast<std::uint8_t>(((received_ & ninth_bit) >> 1U) |
                                       ((transmit_ & ninth_bit) >> 2U));
    case hc12::sci_data_low: {
      require_transfer_known();
      if ((seen_rx_ & idle_flag) != 0) { receiver_.idle_armed = false; }
      status_ = static_cast<std::uint8_t>(status_ & ~static_cast<unsigned>(seen_rx_));
      seen_rx_ = 0;
      return static_cast<std::uint8_t>(received_ & byte_mask);
    }
    default: throw Fault("Unknown SCI register read at " + hex(offset));
  }
}

void McuSci::write_control2(const std::uint8_t value) {
  if ((value & unsupported_control2) != 0) {
    throw Fault("Unimplemented SCI wakeup or break generation");
  }
  const unsigned previous = control2_;
  control2_ = value;
  if ((value & (transmitter_enable | receiver_enable)) != 0) { generator_started_ = true; }
  if ((value & transmitter_enable) != 0 && (previous & transmitter_enable) == 0) {
    status_ = static_cast<std::uint8_t>(status_ & ~transmission_complete);
    if (transmitting_) {
      preamble_queued_ = true;
    } else {
      transmitting_ = idle_frame();
      transmitter_phase_ = 0;
    }
  }
  if (!receiver_enabled()) { receiver_ = Receiver{}; }
}

void McuSci::write_data(const std::uint8_t value) {
  transmit_ = static_cast<std::uint16_t>((transmit_ & ninth_bit) | value);
  status_ = static_cast<std::uint8_t>(status_ & ~static_cast<unsigned>(seen_tx_));
  seen_tx_ = 0;
  if ((status_ & data_empty) == 0) {
    status_ = static_cast<std::uint8_t>(status_ & ~transmission_complete);
  }
}

void McuSci::write(const ByteWrite write) {
  const auto value = write.value;
  switch (write.address) {
    case hc12::sci_baud_high:
      baud_high_ = static_cast<std::uint8_t>(value & high_baud_mask);
      baud_high_pending_ = true;
      return;
    case hc12::sci_baud_low: {
      const auto next = (static_cast<unsigned>(baud_high_) << 8U) | value;
      if (busy()) { throw Fault("Unimplemented active SCI baud write timing"); }
      baud_ = static_cast<std::uint16_t>(next);
      baud_high_pending_ = false;
      divider_phase_ = 0;
      return;
    }
    case hc12::sci_control1:
      if ((value & loop_mode) != 0) { throw Fault("Unimplemented SCI loop/single-wire mode"); }
      if (((value ^ control1_) & ~stop_in_wait) != 0 && busy()) {
        throw Fault("Unimplemented active SCI format change");
      }
      control1_ = value;
      return;
    case hc12::sci_control2: write_control2(value); return;
    case hc12::sci_status1: return; // Read-only status, including no write-one clearing.
    case hc12::sci_status2: control3_ = static_cast<std::uint8_t>(value & status2_mask); return;
    case hc12::sci_data_high:
      transmit_ =
          static_cast<std::uint16_t>((transmit_ & byte_mask) | ((value & transmit_high_bit) << 2U));
      return;
    case hc12::sci_data_low: write_data(value); return;
    default: throw Fault("Unknown SCI register write at " + hex(write.address));
  }
}

McuSci::TxFrame McuSci::idle_frame() const {
  const auto length = character_bits() + 2;
  return {.bits = (1U << length) - 1, .length = length};
}

McuSci::TxFrame McuSci::data_frame() const {
  const auto width = character_bits();
  const bool parity = (control1_ & parity_enable) != 0;
  const auto data_width = width - (parity ? 1U : 0U);
  unsigned data = transmit_ & ((1U << data_width) - 1);
  if (parity) {
    const auto parity_bit = (static_cast<unsigned>(std::popcount(data)) & 1U) ^
                            ((control1_ & odd_parity) != 0 ? 1U : 0U);
    data |= parity_bit << data_width;
  }
  return {.bits = (data << 1U) | (1U << (width + 1)), .length = width + 2};
}

std::optional<McuSci::TxFrame> McuSci::prepare_transmission() {
  if ((control2_ & transmitter_enable) == 0) { return {}; }
  if (preamble_queued_) {
    preamble_queued_ = false;
    return idle_frame();
  }
  if ((status_ & data_empty) == 0) {
    const auto frame = data_frame();
    status_ = static_cast<std::uint8_t>(status_ | data_empty);
    return frame;
  }
  return {};
}

void McuSci::transmit_tick() {
  ++transmitter_phase_;
  if (!transmitting_) {
    if (transmitter_phase_ == oversampling) {
      transmitter_phase_ = 0;
      transmitting_ = prepare_transmission();
      if (transmitting_) { status_ = static_cast<std::uint8_t>(status_ & ~transmission_complete); }
    }
    return;
  }
  const auto length = transmitting_->length * oversampling;
  if (transmitter_phase_ >= length - oversampling + transfer_phase && !prepared_) {
    prepared_ = prepare_transmission();
  }
  if (transmitter_phase_ == length) {
    transmitting_ = std::exchange(prepared_, {});
    transmitter_phase_ = 0;
    if (!transmitting_ && (status_ & data_empty) != 0) {
      status_ = static_cast<std::uint8_t>(status_ | transmission_complete);
    }
  }
}

std::optional<bool> McuSci::transmit_level() const {
  if (transmitting_) {
    return ((transmitting_->bits >> (transmitter_phase_ / oversampling)) & 1U) != 0;
  }
  if ((control2_ & transmitter_enable) != 0) { return true; }
  return {};
}

void McuSci::track_idle(const bool level) {
  auto &receiver = receiver_;
  const auto threshold = (character_bits() + 2) * oversampling;
  const bool eligible = (control1_ & idle_after_stop) == 0 || receiver.stage == RxStage::idle;
  if (!level || !eligible) {
    receiver.idle_ticks = 0;
  } else if (receiver.idle_ticks < threshold) {
    ++receiver.idle_ticks;
  }
  if (receiver.idle_ticks == threshold) {
    receiver.active = false;
    if (receiver.idle_armed) { status_ = static_cast<std::uint8_t>(status_ | idle_flag); }
  }
}

void McuSci::receive_start(const bool level) {
  auto &receiver = receiver_;
  if (receiver.phase == 3 || receiver.phase == 5 || receiver.phase == 7) {
    receiver.start_samples += level ? 1U : 0U;
  }
  if (receiver.phase == 7) {
    if (receiver.start_samples >= 2) {
      receiver.stage = RxStage::idle;
      return;
    }
    if (receiver.start_samples != 0) { receiver.errors |= noise; }
  }
  if (receiver.phase >= sample_first && receiver.phase <= sample_last && level) {
    receiver.errors |= noise;
  }
}

void McuSci::receive_data(const bool level) {
  auto &receiver = receiver_;
  if (receiver.sampled || receiver.phase < sample_first || receiver.phase > sample_last) { return; }
  receiver.samples += level ? 1U : 0U;
  if (receiver.phase != sample_last) { return; }
  receiver.sampled = true;
  const bool bit = receiver.samples >= 2;
  if (receiver.samples != 0 && receiver.samples != 3) { receiver.errors |= noise; }
  if (receiver.stage == RxStage::stop) {
    if (!bit) { receiver.errors |= framing_error; }
    return;
  }
  if (bit) { receiver.word |= 1U << receiver.bit; }
  // Re-synchronize to a falling edge validated by consecutive data majorities.
  if (receiver.previous_bit && !bit && receiver.falling_age < oversampling) {
    receiver.phase = receiver.falling_age + 1;
  }
  receiver.previous_bit = bit;
}

void McuSci::receive_complete() {
  auto &receiver = receiver_;
  if ((status_ & receive_full) != 0) {
    status_ = static_cast<std::uint8_t>(status_ | overrun);
    return;
  }
  const unsigned data = receiver.word;
  if ((control1_ & parity_enable) != 0) {
    const auto parity = (static_cast<unsigned>(std::popcount(data)) & 1U);
    const auto expected = (control1_ & odd_parity) != 0 ? 1U : 0U;
    if (parity != expected) { receiver.errors |= parity_error; }
  }
  // The shift-register bits feed SCIDR and the parity checker in parallel.
  // In nine-bit format R8 remains the received ninth bit, including parity.
  received_ = static_cast<std::uint16_t>(data);
  status_ = static_cast<std::uint8_t>(status_ | receive_full | receiver.errors);
  receiver.idle_armed = true;
}

void McuSci::receive_tick(const bool level) {
  auto &receiver = receiver_;
  const auto preceding = receiver.preceding_ones;
  receiver.preceding_ones = level ? std::min(preceding + 1, 3U) : 0;
  if (!level && receiver.previous_level) {
    receiver.falling_age = 0;
  } else {
    receiver.falling_age = std::min(receiver.falling_age + 1, oversampling);
  }
  receiver.previous_level = level;
  track_idle(level);
  if ((status_ & framing_error) != 0) { return; }
  if (receiver.stage == RxStage::idle) {
    if (!level && preceding >= 3) {
      receiver.stage = RxStage::start;
      receiver.phase = 1;
      receiver.bit = 0;
      receiver.samples = 0;
      receiver.start_samples = 0;
      receiver.word = 0;
      receiver.errors = 0;
      receiver.sampled = false;
      receiver.active = true;
      receiver.previous_bit = false;
    }
    return;
  }
  ++receiver.phase;
  if (receiver.stage == RxStage::start) {
    receive_start(level);
  } else {
    receive_data(level);
  }
  if (receiver.phase != oversampling) { return; }
  receiver.phase = 0;
  receiver.samples = 0;
  receiver.sampled = false;
  switch (receiver.stage) {
    case RxStage::start: receiver.stage = RxStage::data; break;
    case RxStage::data:
      ++receiver.bit;
      if (receiver.bit == character_bits()) { receiver.stage = RxStage::stop; }
      break;
    case RxStage::stop:
      receive_complete();
      receiver.stage = RxStage::idle;
      break;
    case RxStage::idle: break;
    default: throw Fault("Invalid SCI receiver stage");
  }
}

void McuSci::advance(const SciClockSlice slice) {
  if (slice.bus_cycles > maximum_clock_slice) {
    throw Fault("SCI clock slice exceeds cycle limit");
  }
  if (slice.bus_cycles == 0 || !generator_started_ || baud_ == 0) { return; }
  const auto ticks = (divider_phase_ + slice.bus_cycles) / baud_;
  if (ticks != 0 && receiver_enabled() && !slice.receive_level) {
    throw Fault("Unknown SCI RX pad level");
  }
  const bool receive_level = slice.receive_level.value_or(false); // Unused without an RX sample.
  divider_phase_ = (divider_phase_ + slice.bus_cycles) % baud_;
  for (unsigned index = 0; index < ticks; ++index) {
    transmit_tick();
    if (receiver_enabled()) { receive_tick(receive_level); }
  }
}

unsigned McuSci::bus_cycles_until_sample() const {
  if (!generator_started_ || baud_ == 0) { return 0; }
  return baud_ - divider_phase_;
}

}
