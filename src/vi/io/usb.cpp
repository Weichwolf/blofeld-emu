// SPDX-License-Identifier: GPL-3.0-only
#include "io/usb.hpp"

#include <array>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <optional>

namespace blofeld::vi {
namespace {
constexpr std::uint16_t first = 0x302, end = 0x400, byte_mask = 0xff;
constexpr std::uint16_t status = 0x302, interrupts = 0x304, test = 0x308, timestamp = 0x30a;
constexpr std::uint16_t configuration_status = 0x30c, select = 0x30e, data = 0x310;
constexpr std::uint16_t endpoint_begin = 0x314, endpoint_end = 0x324;
constexpr std::uint16_t setup_begin = 0x338, setup_end = 0x340;
constexpr std::uint16_t buffer_begin = 0x380, buffer_second = 0x3c0;
constexpr std::uint16_t selector_mask = 0x0f77, buffer_select_mask = 0x77;
constexpr std::uint16_t enabled = 0x8000, status_clear = 0x1807, interrupt_mask = 0x7807;
constexpr std::uint16_t transfer = 0x2000, transfer_enable = 0x1000, local_writable = 0xd07f;
constexpr std::uint16_t configuration_request = 0x2000, configuration_done = 0x1000;
constexpr std::uint16_t auxiliary_writable = 0xdfff, auxiliary_clear = 0x2000;
constexpr std::uint16_t out_status_writable = 0x500f, out_status_clear = 0xa000;
constexpr std::uint16_t in_status_writable = 0x100f, in_status_clear = 0x2000;
constexpr std::uint32_t count_mask = 0x0773;
constexpr unsigned counts = 8, alternates = 9;
constexpr std::uint16_t auxiliary_interrupts = 0x5000, status_interrupts = 0x1800;
constexpr std::uint16_t short_packet = 0x8000, short_enable = 0x4000;

std::uint16_t replace(const std::uint16_t previous, const Usb::Write access) noexcept {
  return (access.offset & 1U) == 0
             ? static_cast<std::uint16_t>((previous & byte_mask) | (unsigned{access.value} << 8U))
             : static_cast<std::uint16_t>((previous & ~byte_mask) | access.value);
}

std::uint8_t byte(const std::uint16_t value, const Usb::Read access) noexcept {
  return static_cast<std::uint8_t>((access.offset & 1U) == 0 ? value >> 8U : value & byte_mask);
}
}

bool Usb::contains(const std::uint16_t offset) noexcept {
  return offset >= first && offset < end;
}

std::optional<unsigned> Usb::configuration_index() const noexcept {
  const auto index = unsigned{selector_} >> 8U;
  if (index >= 1 && index <= 5) { return index - 1U; }
  if (index == counts) { return 5; }
  if (index == alternates) { return 6; }
  return {};
}

unsigned Usb::window(const std::uint16_t offset) const noexcept {
  return (offset >= buffer_second ? unsigned{selector_} >> 4U : unsigned{selector_}) & 7U;
}

bool Usb::mapping_pending() const noexcept {
  return (selector_ & buffer_select_mask) != bound_buffers_;
}

bool Usb::commit_mapping() noexcept {
  const auto changed = mapping_pending();
  bound_buffers_ = static_cast<std::uint8_t>(selector_ & buffer_select_mask);
  return changed;
}

std::array<cpu12::Ram, 2> Usb::regions(const std::uint16_t register_base) noexcept {
  std::array<cpu12::Ram, 2> result{};
  constexpr std::uint16_t last_register_base = 0xfc00;
  if (register_base > last_register_base) { return result; }
  for (unsigned index = 0; index < result.size(); ++index) {
    const auto selected = (unsigned{bound_buffers_} >> (index * 4U)) & 7U;
    if (selected >= buffers_.size()) { continue; }
    auto &buffer = buffers_[selected];
    result[index] = {
        .first = static_cast<std::uint16_t>(register_base + buffer_begin + (index * buffer_bytes)),
        .bytes = buffer.bytes,
        .known = buffer.known};
  }
  return result;
}

bool Usb::read_byte(const std::uint16_t offset, std::uint8_t &value) const noexcept {
  if (!contains(offset)) { return false; }
  const auto aligned = static_cast<std::uint16_t>(offset & ~1U);
  switch (aligned) {
    case status: value = byte(status_, {.offset = offset}); return true;
    case interrupts: value = byte(interrupts_, {.offset = offset}); return true;
    case timestamp: value = 0; return true; // No host SOF has been received.
    case configuration_status: value = byte(configuration_status_, {.offset = offset}); return true;
    case select: value = byte(selector_, {.offset = offset}); return true;
    default: break;
  }
  if (offset >= data && offset < endpoint_begin) {
    const auto index = configuration_index();
    if (!index) { return false; }
    const auto shift = (endpoint_begin - offset - 1U) * 8U;
    value = static_cast<std::uint8_t>((configurations_[*index] >> shift) & byte_mask);
    return true;
  }
  if (offset >= endpoint_begin && offset < endpoint_end) {
    value = byte(endpoints_[(unsigned{offset} - endpoint_begin) / 2U], {.offset = offset});
    return true;
  }
  if (offset >= setup_begin && offset < setup_end) { return false; }
  if (offset >= buffer_begin) {
    const auto selected = window(offset);
    const auto position = offset % buffer_bytes;
    if (selected >= buffers_.size() || buffers_[selected].known[position] == 0) { return false; }
    value = buffers_[selected].bytes[position];
    return true;
  }
  value = 0; // Documented reserved/test fields in normal mode.
  return true;
}

bool Usb::write_configuration(const Write access, const std::uint16_t control) noexcept {
  const auto index = configuration_index();
  if (!index) { return false; }
  // Only UNCIR/UNASR require MEN=0. Endpoint configuration remains writable.
  if (*index >= 5 && (control & enabled) != 0) { return true; }
  if (access.bytes != 2 || (access.offset != data && access.offset != data + 2)) { return false; }
  if (access.offset == data) {
    staged_high_ = access.value;
    return true;
  }
  if (!staged_high_) { return false; }
  const auto value = (std::uint32_t{*staged_high_} << 16U) | access.value;
  configurations_[*index] = *index == 5 ? value & count_mask : value;
  staged_high_.reset();
  return true;
}

bool Usb::write_byte(const Write access) noexcept {
  const auto offset = access.offset;
  if (!contains(offset)) { return false; }
  const auto aligned = static_cast<std::uint16_t>(offset & ~1U);
  const auto lane_mask =
      static_cast<std::uint16_t>((offset & 1U) == 0 ? byte_mask << 8U : byte_mask);
  const auto requested =
      static_cast<std::uint16_t>((offset & 1U) == 0 ? access.value << 8U : access.value);
  switch (aligned) {
    case status: status_ &= static_cast<std::uint16_t>(~(requested & status_clear)); return true;
    case interrupts: interrupts_ = replace(interrupts_, access) & interrupt_mask; return true;
    case test: return (requested & 15U) == 0;
    case timestamp: return true;
    case configuration_status:
      // No host configuration request exists yet; DONEECRU cannot create one.
      return (requested & configuration_done) == 0 ||
             (configuration_status_ & configuration_request) == 0;
    case select: selector_ = replace(selector_, access) & selector_mask; return true;
    default: break;
  }
  if (offset >= endpoint_begin && offset < endpoint_end) {
    const auto index = (unsigned{offset} - endpoint_begin) / 2U;
    auto &current = endpoints_[index];
    auto writable = local_writable;
    auto clear = transfer;
    if (index == 4 || index == 6) {
      writable = auxiliary_writable;
      clear = auxiliary_clear;
    }
    if (index == 5) {
      writable = out_status_writable;
      clear = out_status_clear;
    }
    if (index == 7) {
      writable = in_status_writable;
      clear = in_status_clear;
    }
    current =
        static_cast<std::uint16_t>((current & ~(lane_mask & writable)) | (requested & writable));
    current &= static_cast<std::uint16_t>(~(requested & clear));
    return true;
  }
  if (offset >= setup_begin && offset < setup_end) { return false; }
  if (offset >= buffer_begin) {
    const auto selected = window(offset);
    if (selected >= buffers_.size()) { return false; }
    const auto position = offset % buffer_bytes;
    buffers_[selected].bytes[position] = static_cast<std::uint8_t>(access.value);
    buffers_[selected].known[position] = 1;
  }
  return true;
}

bool Usb::read(const Read access, std::uint16_t &value) const noexcept {
  if (access.bytes != 1 && access.bytes != 2) { return false; }
  std::uint8_t high = 0;
  std::uint8_t low = 0;
  if (!read_byte(access.offset, high)) { return false; }
  value = high;
  if (access.bytes == 1) { return true; }
  if (!read_byte(static_cast<std::uint16_t>(access.offset + 1), low)) { return false; }
  value = static_cast<std::uint16_t>((unsigned{high} << 8U) | low);
  return true;
}

bool Usb::write(const Write access, const std::uint16_t control) noexcept {
  if ((access.bytes != 1 && access.bytes != 2) || (access.bytes == 1 && access.value > byte_mask)) {
    return false;
  }
  if (access.offset >= data && access.offset < endpoint_begin) {
    return write_configuration(access, control);
  }
  if (access.bytes == 1) { return access.value <= byte_mask && write_byte(access); }
  if (access.bytes != 2) { return false; }
  return write_byte(
             {.offset = access.offset, .value = static_cast<std::uint16_t>(access.value >> 8U)}) &&
         write_byte({.offset = static_cast<std::uint16_t>(access.offset + 1),
                     .value = static_cast<std::uint16_t>(access.value & byte_mask)});
}

bool Usb::interrupts_enabled() const noexcept {
  if (interrupts_ != 0) { return true; }
  for (unsigned index = 0; index < endpoints_.size(); ++index) {
    const auto mask = index == 5 ? auxiliary_interrupts : transfer_enable;
    if ((endpoints_[index] & mask) != 0) { return true; }
  }
  return false;
}

std::optional<std::uint16_t> Usb::pending() const noexcept {
  constexpr std::uint16_t status_vector = 0xffd4;
  constexpr std::uint16_t setup_vector = 0xffd2;
  constexpr std::uint16_t configuration_vector = 0xffd0;
  constexpr std::uint16_t sof_vector = 0xffbe;
  constexpr std::uint16_t resume_enable = 0x4000;
  constexpr std::uint16_t suspended = 0x10;
  if ((status_ & interrupts_ & status_interrupts) != 0 ||
      ((interrupts_ & resume_enable) != 0 && (status_ & suspended) == 0)) {
    return status_vector;
  }
  if ((status_ & interrupts_ & 3U) != 0) { return setup_vector; }
  if ((configuration_status_ & interrupts_ & configuration_request) != 0) {
    return configuration_vector;
  }
  constexpr std::array<std::uint16_t, 8> vectors{
      0xffce, 0xffcc, 0xffca, 0xffc8, 0xffc4, 0xffc4, 0xffc2, 0xffc2};
  for (unsigned index = 0; index < endpoints_.size(); ++index) {
    const auto value = endpoints_[index];
    if ((value & transfer) != 0 && (value & transfer_enable) != 0) { return vectors[index]; }
    if (index == 5 && (value & short_packet) != 0 && (value & short_enable) != 0) {
      return vectors[index];
    }
  }
  return (status_ & interrupts_ & 4U) != 0 ? std::optional{sof_vector} : std::nullopt;
}
}
