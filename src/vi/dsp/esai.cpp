// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/esai.hpp"

#include <array>
#include <cstddef>
#include <dsp56300/memory.hpp>
#include <optional>

namespace blofeld::vi::dsp {
namespace {
using dsp56300::Word;
constexpr Word tx_first = 0xffff80, tx_last = 0xffff85;
constexpr Word common_address = 0xffff90, status_address = 0xffff91, skip_address = 0xffff92;
constexpr Word receive_control_address = 0xffff97, receive_clock_address = 0xffff98;
constexpr Word control_address = 0xffff95, clock_address = 0xffff96;
constexpr Word transmit_mask_low = 0xffff99, receive_mask_low = 0xffff9b, end_address = 0xffff9c;
constexpr Word transmitters = 0x3f, reset_bit = 0x80000;
constexpr Word frame_sync = 0x2000, underrun = 0x4000, empty = 0x8000;
constexpr Word even_empty = 0x10000, odd_empty = 0x20000;
constexpr Word empty_flags = empty | even_empty | odd_empty;
constexpr Word half_word = 0xffff, control_mask = 0xfbffff, common_mask = 0x1c7;
constexpr Word receive_control_mask = 0xf9ffcf;
constexpr unsigned word_bits = 24, format_shift = 10;

struct Length {
  unsigned slot = 0, word = 0;
};

constexpr std::array<Length, 32> lengths{Length{.slot = 8, .word = 8},
                                         Length{.slot = 12, .word = 12},
                                         Length{.slot = 16, .word = 16},
                                         Length{.slot = 20, .word = 20},
                                         Length{.slot = 12, .word = 8},
                                         Length{.slot = 16, .word = 12},
                                         Length{.slot = 20, .word = 16},
                                         Length{.slot = 24, .word = 20},
                                         Length{.slot = 16, .word = 8},
                                         Length{.slot = 20, .word = 12},
                                         Length{.slot = 24, .word = 16},
                                         Length{},
                                         Length{.slot = 20, .word = 8},
                                         Length{.slot = 24, .word = 12},
                                         Length{},
                                         Length{.slot = 32, .word = 20},
                                         Length{.slot = 24, .word = 8},
                                         Length{},
                                         Length{.slot = 32, .word = 16},
                                         Length{},
                                         Length{},
                                         Length{.slot = 32, .word = 12},
                                         Length{},
                                         Length{},
                                         Length{.slot = 32, .word = 8},
                                         Length{},
                                         Length{},
                                         Length{},
                                         Length{},
                                         Length{},
                                         Length{.slot = 24, .word = 24},
                                         Length{.slot = 32, .word = 24}};
}

void Esai::reset_transmit() noexcept {
  status_ &= ~(frame_sync | underrun | empty_flags);
  written_ = slot_ = active_transmitters_ = 0;
  skip_ = observed_underrun_ = false;
}

void Esai::reset() noexcept {
  data_ = {};
  known_ = 0;
  control_ = clock_ = common_ = receive_control_ = receive_clock_ = status_ = 0;
  transmit_mask_ = frame_mask_ = receive_mask_ = all_slots;
  connected_ = false;
  error_ = nullptr;
  reset_transmit();
}

void Esai::connected(const bool active) noexcept {
  if (connected_ && !active) { reset_transmit(); }
  connected_ = active;
}

bool Esai::contains(const Word address) noexcept {
  return data_register(address) || (address >= common_address && address <= skip_address) ||
         (address >= control_address && address <= end_address);
}

bool Esai::data_register(const Word address) noexcept {
  return address >= tx_first && address <= tx_last;
}

bool Esai::read(const Word address, Word &value) noexcept {
  switch (address) {
    case common_address: value = common_; return true;
    case status_address:
      value = status_;
      if ((status_ & underrun) != 0) { observed_underrun_ = true; }
      return true;
    case control_address: value = control_; return true;
    case clock_address: value = clock_; return true;
    case receive_control_address: value = receive_control_; return true;
    case receive_clock_address: value = receive_clock_; return true;
    default: break;
  }
  if (address < transmit_mask_low || address > end_address) { return false; }
  const auto mask = address < receive_mask_low ? transmit_mask_ : receive_mask_;
  value = ((address & 1) != 0 ? mask : mask >> 16U) & half_word;
  return true;
}

void Esai::accept_data() noexcept {
  const auto enabled = control_ & transmitters;
  if (enabled == 0 || (written_ & enabled) != enabled) { return; }
  status_ &= ~empty_flags;
  if (observed_underrun_) {
    status_ &= ~underrun;
    observed_underrun_ = false;
  }
}

bool Esai::write(const Write access) noexcept {
  if (access.value > dsp56300::word_mask || error_ != nullptr) { return false; }
  if (data_register(access.address)) {
    const auto index = access.address - tx_first;
    data_[index] = access.value;
    written_ |= 1U << index;
    known_ |= 1U << index;
    accept_data();
    return true;
  }
  switch (access.address) {
    case common_address: common_ = access.value & common_mask; return true;
    case status_address: return true; // Read-only status.
    case skip_address:
      skip_ = true;
      status_ &= ~empty_flags;
      if (observed_underrun_) { status_ &= ~underrun; }
      observed_underrun_ = false;
      return true;
    case control_address:
      control_ = access.value & control_mask;
      active_transmitters_ &= control_ & transmitters;
      if ((control_ & reset_bit) != 0) { reset_transmit(); }
      return true;
    case clock_address: clock_ = access.value; return true;
    case receive_control_address:
      receive_control_ = access.value & receive_control_mask;
      return true;
    case receive_clock_address: receive_clock_ = access.value; return true;
    default: break;
  }
  if (access.address < transmit_mask_low || access.address > end_address) { return false; }
  auto &mask = access.address < receive_mask_low ? transmit_mask_ : receive_mask_;
  const auto value = access.value & half_word;
  mask =
      (access.address & 1) != 0 ? (mask & ~half_word) | value : (mask & half_word) | (value << 16U);
  return true;
}

std::optional<Esai::Format> Esai::format() const noexcept {
  constexpr Word alignment_or_lsb = 0xc0;
  constexpr Word alc = 0x100;
  constexpr Word internal_word_clock = 0x600000;
  constexpr Word internal_hck = 0x800000;
  constexpr Word mode_mask = 0x300;
  constexpr Word network_mode = 0x100;
  constexpr Word length_mask = 31;
  if ((control_ & alignment_or_lsb) != 0 || (common_ & alc) != 0 ||
      (control_ & mode_mask) != network_mode || (clock_ & internal_hck) != 0 ||
      (clock_ & internal_word_clock) != internal_word_clock) {
    return {};
  }
  const auto length = lengths[(control_ >> format_shift) & length_mask];
  const auto slots = ((clock_ >> 9U) & length_mask) + 1;
  if (length.word == 0 || slots < 2) { return {}; }
  const auto prescale = (clock_ & 0x100) != 0 ? 1U : 8U;
  constexpr Word byte_mask = 0xff;
  const auto divider = 2 * prescale * ((clock_ & byte_mask) + 1) * (((clock_ >> 14U) & 15) + 1);
  return Format{.word_bits = length.word, .slots = slots, .slot_ticks = divider * length.slot};
}

std::optional<Esai::Slot> Esai::clock_slot() noexcept {
  if (error_ != nullptr || !connected_ || (control_ & transmitters) == 0 ||
      (control_ & reset_bit) != 0) {
    return {};
  }
  const auto selected = format();
  if (!selected) {
    error_ = "Unsupported ESAI_1 transmit format or clock source";
    return {};
  }
  if (slot_ == 0) {
    active_transmitters_ = control_ & transmitters;
    frame_mask_ = transmit_mask_;
  }
  Slot result{.index = slot_};
  status_ &= ~frame_sync;
  if (slot_ == 0) { status_ |= frame_sync; }
  if ((frame_mask_ & (Word{1} << slot_)) != 0) {
    result.enabled = skip_ ? 0 : active_transmitters_;
    result.known = known_ & result.enabled;
    const auto sample_mask = dsp56300::word_mask << (word_bits - selected->word_bits);
    for (std::size_t index = 0; index < data_.size(); ++index) {
      result.samples[index] = data_[index] & sample_mask;
    }
    result.repeated = !skip_ && (written_ & active_transmitters_) != active_transmitters_;
    if (result.repeated) { status_ |= underrun; }
    status_ |= empty | ((slot_ & 1) == 0 ? even_empty : odd_empty);
    written_ &= ~active_transmitters_;
    skip_ = false;
  }
  slot_ = (slot_ + 1) % selected->slots;
  return result;
}

bool Esai::active() const noexcept {
  return connected_ && (control_ & transmitters) != 0 && (control_ & reset_bit) == 0;
}

bool Esai::request() const noexcept {
  return active() && (status_ & empty) != 0;
}
}
