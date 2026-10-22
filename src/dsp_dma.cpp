// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_dma.hpp"

#include "blofeld/bus.hpp"
#include "dsp_dma_registers.hpp"

#include <cstdint>
#include <string>

namespace blofeld {
namespace {
std::string location(const unsigned address) {
  return " at X:" + hex(address, 6);
}
}

void DspDma::reset() {
  *this = DspDma{};
}

bool DspDma::handles(const unsigned address) {
  return address >= dma::last_control && address <= dma::status;
}

bool DspDma::busy(const unsigned channel) const {
  const auto &state = channels_.at(channel);
  return (state.control & dma::enable) != 0 || state.pending ||
         (active_ && active_->channel == channel) || state.settling != 0;
}

unsigned DspDma::read_status(const unsigned mask) const {
  unsigned result = 0;
  for (unsigned index = 0; index < channels_.size(); ++index) {
    const auto &channel = channels_.at(index);
    if (busy(index) && !cycle_) {
      throw Fault("Unqualified DSP DMA cycle timing" + location(dma::status));
    }
    if (!channel.done_known && (mask & (1U << index)) != 0) {
      throw Fault("Unqualified DSP DMA repeating DTD lifetime" + location(dma::status));
    }
    if (channel.done) { result |= 1U << index; }
  }
  const auto selected = active_ ? std::optional{active_->channel} : select();
  constexpr unsigned active_flag = 0x100;
  constexpr unsigned channel_shift = 9;
  if (selected) { result |= active_flag | (*selected << channel_shift); }
  return result & mask;
}

std::uint32_t DspDma::read_checked(const unsigned address, const std::uint32_t mask) const {
  if (!handles(address) || mask > dma::word_mask) {
    throw Fault("Invalid DSP DMA read" + location(address));
  }
  if (address == dma::status) { return read_status(mask); }
  if (address >= dma::first_offset - 3) {
    const auto offset = dma::first_offset - address;
    if ((known_offsets_ & (1U << offset)) == 0) {
      throw Fault("Unknown DSP DMA offset" + location(address));
    }
    return offsets_.at(offset) & mask;
  }
  const auto index = (dma::first_source - address) / 4;
  const auto reg = (dma::first_source - address) % 4;
  const auto &channel = channels_.at(index);
  if (reg == 3) { return channel.control & mask; }
  if ((channel.known & (1U << reg)) == 0) {
    throw Fault("Unknown DSP DMA address/counter" + location(address));
  }
  if (reg == 0) { return channel.source & mask; }
  return (reg == 1 ? channel.destination : channel.counter) & mask;
}

std::uint32_t DspDma::read(const unsigned address, const std::uint32_t mask) const {
  try {
    return read_checked(address, mask);
  } catch (const Fault &error) {
    throw Fault("DSP DMA register read" + location(address) + ": " + error.what());
  }
}

void DspDma::validate_enable(const Channel &channel) const {
  if ((channel.control & dma::continuous) != 0 ||
      (mode(channel) != 1 && mode(channel) != 2 && mode(channel) != 3 && mode(channel) != 5)) {
    throw Fault("Unsupported DSP DMA continuous/transfer mode");
  }
  constexpr unsigned last_device_request = 22;
  if (mode(channel) != 3 && request(channel) > last_device_request) {
    throw Fault("Reserved DSP56371 DMA request source");
  }
  if (channel.known != dma::known_all) { throw Fault("Unknown DSP DMA address/counter preload"); }
  static_cast<void>(endpoint(channel, false));
  static_cast<void>(endpoint(channel, true));
  static_cast<void>(layout(channel));
  if ((channel.control & dma::three_dimensions) == 0 &&
      (addressing(channel, false) >= 6 || addressing(channel, true) >= 6)) {
    throw Fault("Reserved DSP DMA non-3D address mode");
  }
  const auto needed = offset_mask(channel);
  if ((known_offsets_ & needed) != needed) { throw Fault("Unknown DSP DMA required offset"); }
}

void DspDma::write_control(const unsigned index, const unsigned value) {
  auto &channel = channels_.at(index);
  const bool enabled = (value & dma::enable) != 0;
  const bool old_enabled = (channel.control & dma::enable) != 0;
  if (busy(index) && ((value ^ channel.control) & ~(dma::enable | dma::interrupt)) != 0) {
    throw Fault("DSP DMA configuration changed while busy");
  }
  channel.control = value;
  if (enabled && !old_enabled) {
    if (channel.pending || (active_ && active_->channel == index) || channel.settling != 0) {
      throw Fault("DSP DMA reenabled before prior transfer stopped");
    }
    validate_enable(channel);
    channel.settling = dma::settling_cycles;
    channel.done_known = true;
    channel.pending = mode(channel) == 3;
    channel.first_sample = true;
  }
  if (!enabled && old_enabled) {
    channel.settling = 0;
    if (mode(channel) == 2 || mode(channel) == 3) { channel.pending = false; }
    if (!channel.pending && (!active_ || active_->channel != index)) {
      channel.done = true;
      channel.done_known = true;
      if ((channel.control & dma::interrupt) != 0) { channel.interrupt_pending = true; }
    }
  }
}

void DspDma::write_checked(const unsigned address, const unsigned value) {
  if (!handles(address) || value > dma::word_mask || address == dma::status) {
    throw Fault("Invalid/read-only DSP DMA write");
  }
  if (address >= dma::first_offset - 3) {
    const auto offset = dma::first_offset - address;
    for (unsigned index = 0; index < channels_.size(); ++index) {
      if (busy(index) && (offset_mask(channels_.at(index)) & (1U << offset)) != 0) {
        throw Fault("DSP DMA offset changed while in use");
      }
    }
    offsets_.at(offset) = value;
    known_offsets_ |= 1U << offset;
    return;
  }
  const auto index = (dma::first_source - address) / 4;
  const auto reg = (dma::first_source - address) % 4;
  if (reg == 3) {
    write_control(index, value);
    return;
  }
  auto &channel = channels_.at(index);
  if (busy(index) || !channel.done || !channel.done_known) {
    throw Fault("DSP DMA address/counter changed before idle/done");
  }
  if (reg == 0) { channel.source = value; }
  if (reg == 1) { channel.destination = value; }
  if (reg == 2) {
    channel.counter = value;
    channel.reload = value;
  }
  channel.known |= 1U << reg;
}

void DspDma::write(const unsigned address, const std::uint32_t value) {
  auto next = *this;
  try {
    next.write_checked(address, value);
  } catch (const Fault &error) {
    throw Fault("DSP DMA register write" + location(address) + ": " + error.what());
  }
  *this = next;
}

std::uint32_t DspDma::requested_sources() const {
  std::uint32_t result = 0;
  for (const auto &channel : channels_) {
    if ((channel.control & dma::enable) != 0 && mode(channel) != 3) {
      result |= 1U << request(channel);
    }
  }
  return result;
}

bool DspDma::active() const {
  for (unsigned index = 0; index < channels_.size(); ++index) {
    if (busy(index)) { return true; }
  }
  return false;
}

std::uint8_t DspDma::interrupt_sources() const {
  unsigned sources = 0;
  for (unsigned index = 0; index < channels_.size(); ++index) {
    const auto &channel = channels_.at(index);
    if (channel.interrupt_pending && (channel.control & dma::interrupt) != 0) {
      sources |= 1U << index;
    }
  }
  return static_cast<std::uint8_t>(sources);
}

void DspDma::acknowledge_interrupt(const unsigned channel) {
  if (channel >= channels_.size() || (interrupt_sources() & (1U << channel)) == 0) {
    throw Fault("DSP DMA interrupt acknowledgement has no pending channel=" +
                std::to_string(channel));
  }
  channels_.at(channel).interrupt_pending = false;
}
}
