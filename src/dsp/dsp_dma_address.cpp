// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_dma_registers.hpp"

namespace blofeld {
unsigned DspDma::mode(const Channel &channel) {
  return (channel.control >> dma::mode_shift) & 7U;
}

unsigned DspDma::request(const Channel &channel) {
  return (channel.control >> dma::request_shift) & dma::request_mask;
}

unsigned DspDma::addressing(const Channel &channel, const bool destination) {
  const auto bits = (channel.control >> dma::address_shift) & dma::address_mask;
  if ((channel.control & dma::three_dimensions) == 0) {
    return (bits >> (destination ? 3U : 0U)) & 7U;
  }
  const bool source_three = (bits & 4U) == 0;
  if (destination == source_three) { return bits >> 3U; }
  return destination ? 7U : 6U;
}

DspDma::Layout DspDma::layout(const Channel &channel) {
  if ((channel.control & dma::three_dimensions) == 0) {
    constexpr unsigned single_bits = 24;
    return {.low =
                addressing(channel, false) < 4 || addressing(channel, true) < 4 ? 12U : single_bits,
            .middle = 0};
  }
  const auto selection = (channel.control >> dma::address_shift) & 3U;
  if (selection == 0) { return {.low = 6, .middle = 6}; }
  if (selection == 1) { return {.low = 6, .middle = 12}; }
  if (selection == 2) { return {.low = 12, .middle = 6}; }
  throw Fault("Reserved DSP DMA triple-counter layout");
}

DspDmaAddress DspDma::endpoint(const Channel &channel, const bool destination) {
  const auto space = (channel.control >> (destination ? 2U : 0U)) & 3U;
  if (space == 3) { throw Fault("Reserved DSP DMA memory space"); }
  return {.space = static_cast<DspDmaSpace>(space),
          .address = destination ? channel.destination : channel.source};
}

bool DspDma::complete(const Channel &channel) {
  return channel.counter == 0;
}

unsigned DspDma::offset_mask(const Channel &channel) {
  unsigned result = 0;
  for (const bool destination : {false, true}) {
    const auto kind = addressing(channel, destination);
    if (kind < 4) { result |= 1U << kind; }
    if (kind >= 6) { result |= 3U << (kind == 6 ? 0U : 2U); }
  }
  return result;
}

unsigned DspDma::next_address(const Channel &channel, const bool destination) const {
  const auto kind = addressing(channel, destination);
  const auto address = destination ? channel.destination : channel.source;
  if (kind == 4) { return address; }
  const auto dimensions = layout(channel);
  const auto low_mask = (1U << dimensions.low) - 1;
  const auto middle_mask = (1U << dimensions.middle) - 1;
  unsigned offset = 1;
  if ((channel.counter & low_mask) == 0 && kind < 4) { offset = offsets_.at(kind); }
  if ((channel.counter & low_mask) == 0 && kind >= 6) {
    const auto middle = (channel.counter >> dimensions.low) & middle_mask;
    const auto first = kind == 6 ? 0U : 2U;
    offset = offsets_.at(first + (middle == 0 ? 1U : 0U));
  }
  // Both operands are24-bit; their sum fits unsigned without host overflow.
  return (address + offset) & dma::word_mask;
}

void DspDma::update_addresses(Channel &channel) const {
  channel.source = next_address(channel, false);
  channel.destination = next_address(channel, true);
  const auto dimensions = layout(channel);
  const auto low_mask = (1U << dimensions.low) - 1;
  const auto middle_mask = (1U << dimensions.middle) - 1;
  if ((channel.counter & low_mask) != 0) {
    --channel.counter;
  } else if (((channel.counter >> dimensions.low) & middle_mask) != 0) {
    channel.counter = (channel.counter - (1U << dimensions.low)) | (channel.reload & low_mask);
  } else if (channel.counter != 0) {
    const auto shift = dimensions.low + dimensions.middle;
    const auto reload_mask = (1U << shift) - 1;
    channel.counter = (channel.counter - (1U << shift)) | (channel.reload & reload_mask);
  } else {
    channel.counter = channel.reload;
  }
}
}
