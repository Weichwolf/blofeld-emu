// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dma.hpp"

#include "execution.hpp"

#include <algorithm>
#include <array>
#include <dsp56300/memory.hpp>
#include <optional>

namespace blofeld::vi::dsp {
namespace {
using dsp56300::Word;
constexpr Word status_address = 0xfffff4, first_offset = 0xfffff3, first_source = 0xffffef;
constexpr Word last_control = 0xffffd8, enable = 0x800000, irq_enable = 0x400000;
constexpr Word continuous = 0x10000, three_dimensions = 0x400;
constexpr unsigned channel_count = 6, priority_shift = 17, transfer_shift = 19;
constexpr unsigned word_bits = 24, request_mask = 31;

struct Counter {
  Word value = 0, reload = 0;
};

Word mask(const unsigned bits) noexcept {
  return (Word{1} << bits) - 1;
}

Word next_count(const Counter counter, const std::array<unsigned, 3> widths) noexcept {
  Word result = counter.value;
  unsigned shift = 0;
  for (const auto width : widths) {
    if (width == 0) { continue; }
    const auto field = mask(width) << shift;
    if ((result & field) != 0) {
      result -= Word{1} << shift;
      break;
    }
    result = (result & ~field) | (counter.reload & field);
    shift += width;
  }
  return result;
}
}

void Dma::reset() noexcept {
  channels_ = {};
  offsets_ = {};
  last_.fill(channel_count - 1);
  last_valid_.fill(false);
  active_.reset();
  failure_.reset();
  pending_ = interrupts_ = 0;
  done_ = mask(channel_count);
}

bool Dma::contains(const Word address) noexcept {
  return address >= last_control && address <= status_address;
}

bool Dma::read(const Word address, Word &value) const noexcept {
  if (!contains(address)) { return false; }
  if (address == status_address) {
    value = done_;
    if (active_) { value |= 0x100U | (*active_ << 9U); }
    return true;
  }
  if (address > first_source) {
    const auto &offset = offsets_[first_offset - address];
    if (!offset) { return false; }
    value = *offset;
    return true;
  }
  const auto relative = first_source - address;
  const auto &channel = channels_[relative / 4];
  const auto field = relative % 4;
  if (field < 3 && (channel.known & (1U << field)) == 0) { return false; }
  switch (field) {
    case 0: value = channel.source; return true;
    case 1: value = channel.destination; return true;
    case 2: value = channel.count; return true;
    case 3: value = channel.control; return true;
    default: return false;
  }
}

bool Dma::write(const Write access) noexcept {
  if (!contains(access.address) || access.value > dsp56300::word_mask) { return false; }
  if (access.address == status_address) { return true; } // Read-only register.
  if (access.address > first_source) {
    offsets_[first_offset - access.address] = access.value;
    return true;
  }
  const auto relative = first_source - access.address;
  const auto index = relative / 4;
  auto &channel = channels_[index];
  const auto field = relative % 4;
  channel.known |= 1U << field;
  switch (field) {
    case 0: channel.source = access.value; return true;
    case 1: channel.destination = access.value; return true;
    case 2: channel.count = channel.reload = access.value; return true;
    case 3: break;
    default: return false;
  }
  const bool previous = (channel.control & enable) != 0;
  channel.control = access.value;
  const auto bit = 1U << index;
  if ((access.value & enable) != 0) {
    done_ &= ~bit;
    if (!previous && ((access.value >> transfer_shift) & 7) == 3) { pending_ |= bit; }
  } else if ((pending_ & bit) == 0) {
    done_ |= bit;
    if (previous && (access.value & irq_enable) != 0) { interrupts_ |= bit; }
  }
  return true;
}

bool Dma::trigger(const unsigned source) noexcept {
  if (source >= 32) { return false; }
  for (unsigned index = 0; index < channel_count; ++index) {
    const auto control = channels_[index].control;
    if ((control & enable) != 0 && ((control >> 11U) & request_mask) == source &&
        ((control >> transfer_shift) & 7) != 3) {
      pending_ |= 1U << index;
    }
  }
  return true;
}

unsigned Dma::select() const noexcept {
  unsigned best = 0;
  for (unsigned index = 0; index < channel_count; ++index) {
    if ((pending_ & (1U << index)) == 0) { continue; }
    const auto priority = (channels_[index].control >> priority_shift) & 3;
    best = std::max(priority, best);
  }
  const auto previous = last_[best];
  if (last_valid_[best] && (pending_ & (1U << previous)) != 0 &&
      ((channels_[previous].control >> priority_shift) & 3) == best &&
      (channels_[previous].control & continuous) != 0) {
    return previous;
  }
  for (unsigned step = 1; step <= channel_count; ++step) {
    const auto index = (previous + step) % channel_count;
    if ((pending_ & (1U << index)) != 0 &&
        ((channels_[index].control >> priority_shift) & 3) == best) {
      return index;
    }
  }
  return 0; // service() calls only with a nonempty pending set.
}

bool Dma::fail_operand(const Address address, Failure context, const char *reason) noexcept {
  active_.reset();
  context.address = address.address;
  context.space = address.space == dsp56300::Space::x ? 'X' : 'Y';
  if (address.space == dsp56300::Space::program) { context.space = 'P'; }
  context.reason = reason;
  failure_ = context;
  return false;
}

bool Dma::fail_register(const Word address, const char *reason) noexcept {
  return fail_operand({.address = address}, {.operation = Access::execute}, reason);
}

std::optional<Word> Dma::increment(const unsigned mode,
                                   const Channel &channel,
                                   const unsigned low_bits,
                                   const unsigned middle_bits) noexcept {
  if (mode == 4) { return 0; }
  if (mode == 5 || (channel.count & mask(low_bits)) != 0) { return 1; }
  unsigned offset = mode;
  if (mode >= 6) {
    offset = (mode - 6) * 2;
    if (((channel.count >> low_bits) & mask(middle_bits)) == 0) { ++offset; }
  }
  if (!offsets_[offset]) {
    static_cast<void>(fail_register(first_offset - offset, "Unknown DMA offset"));
  }
  return offsets_[offset];
}

std::optional<Dma::Plan> Dma::prepare(const unsigned index) noexcept {
  const auto &channel = channels_[index];
  const auto control = channel.control;
  const auto address = first_source - (index * 4);
  if ((channel.known & 7) != 7 || ((control >> transfer_shift) & 7) > 5 || (control & 3) == 3 ||
      ((control >> 2U) & 3) == 3) {
    static_cast<void>(fail_register(address, "Unknown DMA operand or unsupported transfer fields"));
    return {};
  }
  const auto modes = (control >> 4U) & 63;
  unsigned source_mode = modes & 7;
  unsigned destination_mode = modes >> 3U;
  unsigned low = word_bits;
  unsigned middle = 0;
  if ((control & three_dimensions) != 0) {
    if ((modes & 3) == 3) {
      static_cast<void>(fail_register(address - 3, "Reserved DMA counter mode"));
      return {};
    }
    const auto selected = modes >> 3U;
    source_mode = (modes & 4) != 0 ? selected : 6;
    destination_mode = (modes & 4) != 0 ? 7 : selected;
    low = (modes & 3) == 2 ? 12U : 6U;
    middle = (modes & 3) == 1 ? 12U : 6U;
  } else if (source_mode >= 6 || destination_mode >= 6) {
    static_cast<void>(fail_register(address - 3, "Reserved DMA address mode"));
    return {};
  } else if (source_mode < 4 || destination_mode < 4) {
    low = 12;
  }
  const auto source_increment = increment(source_mode, channel, low, middle);
  if (!source_increment) { return {}; }
  const auto destination_increment = increment(destination_mode, channel, low, middle);
  if (!destination_increment) { return {}; }
  constexpr std::array spaces{dsp56300::Space::x, dsp56300::Space::y, dsp56300::Space::program};
  return Plan{.source = {.space = spaces[control & 3], .address = channel.source},
              .destination = {.space = spaces[(control >> 2U) & 3], .address = channel.destination},
              .next_source = (channel.source + *source_increment) & dsp56300::word_mask,
              .next_destination =
                  (channel.destination + *destination_increment) & dsp56300::word_mask,
              .next_count = next_count({.value = channel.count, .reload = channel.reload},
                                       {low, middle, word_bits - low - middle}),
              .line_end = (channel.count & mask(low)) == 0,
              .block_end = channel.count == 0};
}

void Dma::advance(const unsigned index, const Plan &plan) noexcept {
  auto &channel = channels_[index];
  const auto bit = 1U << index;
  const auto mode = (channel.control >> transfer_shift) & 7;
  channel.source = plan.next_source;
  channel.destination = plan.next_destination;
  channel.count = plan.next_count;
  last_[(channel.control >> priority_shift) & 3] = index;
  last_valid_[(channel.control >> priority_shift) & 3] = true;
  if (mode == 1 || mode == 5 || (mode == 2 && plan.line_end) || plan.block_end ||
      (channel.control & enable) == 0) {
    pending_ &= ~bit;
  }
  if (plan.block_end || (channel.control & enable) == 0) {
    done_ |= bit;
    if ((channel.control & irq_enable) != 0) { interrupts_ |= bit; }
    if (mode <= 3) { channel.control &= ~enable; }
    static_cast<void>(trigger(4 + index));
  }
}
}
