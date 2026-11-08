// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dax.hpp"

#include <array>
#include <dsp56300/memory.hpp>
#include <optional>

namespace blofeld::vi::dsp {
namespace {
using dsp56300::Word;
constexpr Word control_address = 0xffffd0, non_audio_address = 0xffffd1;
constexpr Word data_a = 0xffffd2, data_b = 0xffffd3, status_address = 0xffffd4;
constexpr Word empty = 1, underrun = 2, block_end = 4, start_block = 0x20;
constexpr Word control_mask = 0x3f, non_audio_mask = 0xfc00;
constexpr unsigned block_frames = 192;
}

void Dax::reset_transmit() noexcept {
  status_ = 0;
  buffers_ = {};
  pending_.reset();
  current_ = previous_ = {};
  write_channel_ = channel_ = frame_ = clear_writes_ = 0;
  observed_underrun_ = false;
}

void Dax::reset() noexcept {
  control_ = 0;
  connected_ = false;
  reset_transmit();
  // XNADR is explicitly unaffected by all DAX reset states.
}

void Dax::connected(const bool active) noexcept {
  if (connected_ && !active) { reset_transmit(); }
  connected_ = active;
}

bool Dax::contains(const Word address) noexcept {
  return address >= control_address && address <= status_address;
}

bool Dax::data_register(const Word address) noexcept {
  return address >= non_audio_address && address <= data_b;
}

bool Dax::read(const Word address, Word &value) noexcept {
  if (address == control_address) {
    value = control_;
    return true;
  }
  if (address != status_address) { return false; }
  value = status_;
  if ((status_ & underrun) != 0 && !observed_underrun_) {
    observed_underrun_ = true;
    clear_writes_ = 0;
  }
  return true;
}

void Dax::transfer() noexcept {
  if (pending_ && !buffers_[pending_->channel]) {
    buffers_[pending_->channel] = pending_->value;
    pending_.reset();
  }
}

bool Dax::write(const Write access) noexcept {
  if (access.value > dsp56300::word_mask) { return false; }
  if (access.address == control_address) {
    control_ = access.value & control_mask;
    return true;
  }
  if (access.address == non_audio_address) {
    non_audio_ = access.value & non_audio_mask;
    return true;
  }
  if (access.address == status_address) { return true; } // Read-only status.
  if (access.address != data_a && access.address != data_b) { return false; }
  // Both addresses reach XADR. An unpolled write replaces only XADR, never
  // an occupied channel buffer. Successive writes alternate channel ownership.
  pending_ = Pending{.channel = write_channel_, .value = access.value};
  transfer();
  write_channel_ ^= 1U;
  if (write_channel_ == 0) { status_ &= ~(empty | block_end); }
  if (observed_underrun_ && ++clear_writes_ == 2) {
    status_ &= ~underrun;
    observed_underrun_ = false;
    clear_writes_ = 0;
  }
  return true;
}

Dax::Clock Dax::clock() const noexcept {
  constexpr std::array<unsigned, 4> divisors{1024, 256, 384, 512};
  const auto source = (control_ >> 3U) & 3;
  return {.external = source != 0, .frame_ticks = divisors[source]};
}

void Dax::upload(const unsigned channel) noexcept {
  auto &buffer = buffers_[channel];
  if (buffer) {
    current_.samples[channel] = *buffer;
    current_.known |= 1U << channel;
    buffer.reset();
  } else {
    status_ |= underrun;
    current_.repeated = true;
  }
  transfer();
  if (!buffers_[0] && !pending_) { status_ |= empty; }
}

std::optional<Dax::Frame> Dax::clock_subframe() noexcept {
  if (!connected_) { return {}; }
  if (channel_ == 0) {
    if ((control_ & start_block) != 0) {
      frame_ = 0;
      control_ &= ~start_block;
    }
    current_ = {.non_audio = non_audio_, .index = frame_};
    upload(0);
    if (frame_ + 1 == block_frames) { status_ |= block_end; }
    channel_ = 1;
    return {};
  }
  upload(1);
  if (current_.repeated) {
    current_.samples = previous_.samples;
    current_.known = previous_.known;
  } else {
    previous_ = current_;
  }
  channel_ = 0;
  frame_ = (frame_ + 1) % block_frames;
  return current_;
}

bool Dax::request() const noexcept {
  return connected_ && (status_ & empty) != 0;
}

std::optional<Word> Dax::interrupt() const noexcept {
  if (!connected_) { return {}; }
  constexpr Word underrun_vector = 0x28;
  constexpr Word block_vector = 0x2a;
  constexpr Word empty_vector = 0x2e;
  if ((control_ & status_ & underrun) != 0) { return underrun_vector; }
  if ((status_ & empty) == 0) { return {}; }
  if ((control_ & status_ & block_end) != 0) { return block_vector; }
  return (control_ & empty) != 0 ? std::optional<Word>{empty_vector} : std::nullopt;
}
}
