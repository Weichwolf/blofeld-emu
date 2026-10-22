// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_dma.hpp"
#include "dsp_dma_registers.hpp"

#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <string>

namespace blofeld {
void DspDma::sample(const DspDmaInputs inputs) {
  if (inputs.timing != DspDmaTiming::reference_sampled_cycles ||
      (inputs.grant != DspDmaGrant::available && inputs.grant != DspDmaGrant::wait) ||
      (inputs.requests & ~inputs.known_requests) != 0 ||
      (cycle_ &&
       (*cycle_ == std::numeric_limits<std::uint64_t>::max() || inputs.cycle != *cycle_ + 1))) {
    throw Fault("Missing/inconsistent DSP DMA reference cycle/grant");
  }
  const auto required = requested_sources();
  if ((inputs.known_requests & required) != required) {
    throw Fault("Unknown DSP DMA request input");
  }
  for (unsigned index = 0; index < channels_.size(); ++index) { sample_channel(index, inputs); }
  known_requests_ = inputs.known_requests;
  requests_ = inputs.requests;
  cycle_ = inputs.cycle;
}

void DspDma::sample_channel(const unsigned index, const DspDmaInputs inputs) {
  auto &channel = channels_.at(index);
  if (channel.settling != 0) {
    --channel.settling;
    if (channel.settling == 0) { channel.done = false; }
  }
  if ((channel.control & dma::enable) == 0 || mode(channel) == 3) { return; }
  const auto bit = 1U << request(channel);
  // APR23 section5.6: a peripheral's held request also survives late DMA enable.
  const bool held_peripheral = request(channel) >= 10 && channel.first_sample;
  if (!held_peripheral && (bit & ~known_requests_ & inputs.requests) != 0) {
    throw Fault("DSP DMA request requires a known baseline before a rising edge");
  }
  channel.first_sample = false;
  const auto rising = inputs.requests & known_requests_ & ~requests_;
  if ((rising & bit) == 0 && (!held_peripheral || (inputs.requests & bit) == 0)) { return; }
  if (channel.pending || (active_ && active_->channel == index)) {
    throw Fault("DSP DMA regular request repeated before prior word was serviced");
  }
  channel.pending = true;
}

std::optional<unsigned> DspDma::select() const {
  for (unsigned rank = 4; rank != 0; --rank) {
    const auto priority = rank - 1;
    for (unsigned step = 0; step < channels_.size(); ++step) {
      const auto index = (next_channel_.at(priority) + step) % channels_.size();
      const auto &channel = channels_.at(index);
      if (channel.pending && ((channel.control >> dma::priority_shift) & 3U) == priority) {
        return index;
      }
    }
  }
  return std::nullopt;
}

bool DspDma::admitted(const DspDmaGrant grant,
                      const DspDmaPhase &phase,
                      const DspDmaAdmission &admission) {
  if (!admission) { return grant == DspDmaGrant::available; }
  const auto callback = admission;
  DspDmaGrant decision = DspDmaGrant::unknown;
  try {
    decision = callback(phase);
  } catch (const std::exception &) { throw; } catch (...) {
    throw Fault("Non-standard DSP DMA admission exception");
  }
  if (decision != DspDmaGrant::available && decision != DspDmaGrant::wait) {
    throw Fault("Unknown DSP DMA phase admission");
  }
  return grant == DspDmaGrant::available && decision == DspDmaGrant::available;
}

std::optional<DspDmaWord> DspDma::advance_checked(const DspDmaInputs inputs, const DspDmaIo &io) {
  sample(inputs);
  if (inputs.grant == DspDmaGrant::wait && !io.admit) { return std::nullopt; }
  if (!active_) {
    const auto selected = select();
    if (!selected) { return std::nullopt; }
    auto &channel = channels_.at(*selected);
    if (!io.read || !io.write) { throw Fault("Missing DSP DMA bus access callback"); }
    DspDmaWord word{.channel = *selected,
                    .source = endpoint(channel, false),
                    .destination = endpoint(channel, true)};
    if (!admitted(inputs.grant,
                  {.channel = word.channel, .source = word.source, .destination = word.destination},
                  io.admit)) {
      return std::nullopt;
    }
    word.value = io.read(word.source);
    if (word.value > dma::word_mask) { throw Fault("DSP DMA read exceeds24 bits"); }
    channel.pending = false;
    active_ = word;
    return std::nullopt;
  }
  const auto word = *active_;
  if (!admitted(inputs.grant,
                {.kind = DspDmaPhaseKind::destination_write,
                 .channel = word.channel,
                 .source = word.source,
                 .destination = word.destination,
                 .value = word.value},
                io.admit)) {
    return std::nullopt;
  }
  auto &channel = channels_.at(word.channel);
  const bool last = complete(channel);
  const auto low_mask = (1U << layout(channel).low) - 1;
  const bool line_finished = (channel.counter & low_mask) == 0;
  update_addresses(channel);
  if (!io.write) { throw Fault("Missing DSP DMA destination callback"); }
  io.write(word.destination, word.value);
  const auto priority = (channel.control >> dma::priority_shift) & 3U;
  next_channel_.at(priority) = (word.channel + 1) % channels_.size();
  active_.reset();
  const bool disabled = (channel.control & dma::enable) == 0;
  if (last && mode(channel) != 5) { channel.control &= ~dma::enable; }
  if (last || disabled) {
    if ((channel.control & dma::interrupt) != 0) { channel.interrupt_pending = true; }
    channel.settling = 0;
    channel.done = true;
    // The manual does not pin the repeating-mode DTD lifetime between triggers.
    // Do not turn that unspecified interval into a successful hardware read.
    channel.done_known = disabled || mode(channel) != 5;
  }
  if (!last && !disabled && (mode(channel) == 3 || (mode(channel) == 2 && !line_finished))) {
    channel.pending = true;
  }
  return word;
}

std::optional<DspDmaWord> DspDma::advance(const DspDmaInputs inputs, const DspDmaIo &io) {
  auto next = *this;
  try {
    const auto result = next.advance_checked(inputs, io);
    *this = next;
    return result;
  } catch (const std::exception &error) {
    std::string context = "DSP DMA reference cycle=" + std::to_string(inputs.cycle);
    const auto selected = active_ ? std::optional{active_->channel} : next.select();
    if (selected) {
      const auto &channel = channels_.at(*selected);
      context += " channel=" + std::to_string(*selected) + " source=" + hex(channel.source, 6) +
                 " destination=" + hex(channel.destination, 6);
    }
    throw Fault(context + ": " + error.what());
  }
}
}
