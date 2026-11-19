// SPDX-License-Identifier: GPL-3.0-only
#include "io/usb.hpp"

#include <algorithm>
#include <array>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <optional>
#include <span>

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
constexpr std::uint16_t clocked = 0x4000, phy_suspended = 0x0800, disconnected = 0x0020;
constexpr std::uint16_t setup_received = 1, setup_overrun = 2, frame_received = 4;
constexpr std::uint16_t enumerated = 0x80, suspended = 0x10, resetting = 8;
constexpr std::uint16_t suspend_changed = 0x1000, reset_changed = 0x0800;
constexpr std::uint16_t max_frame = 0x07ff, stall = 0x8000;
constexpr std::uint16_t data_valid = 0x4000, payload_size = 0x007f;
constexpr std::uint16_t transfer = 0x2000, transfer_enable = 0x1000, local_writable = 0xd07f;
constexpr std::uint16_t configuration_request = 0x2000, configuration_done = 0x1000;
constexpr std::uint16_t configuration_valid = 0x8000;
constexpr std::uint8_t in_direction = 0x80, endpoint_number = 0x0f, reserved_address = 0x70;
constexpr unsigned first_bulk_slot = 2, configuration_shift = 8;
constexpr std::uint8_t set_configuration = 9;
constexpr std::uint16_t auxiliary_writable = 0xdfff, auxiliary_clear = 0x2000;
constexpr std::uint16_t out_status_writable = 0x500f, out_status_clear = 0xa000;
constexpr std::uint16_t in_status_writable = 0x100f, in_status_clear = 0x2000;
constexpr std::uint32_t count_mask = 0x0773;
constexpr unsigned counts = 8, alternates = 9;
constexpr unsigned count_index = 5;
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

bool link_active(const std::uint16_t control) noexcept {
  constexpr auto required = enabled | clocked;
  return (control & (required | phy_suspended | disconnected)) == required;
}

bool firmware_request(const Usb::Setup &packet, const std::uint16_t control) noexcept {
  constexpr std::uint8_t type_mask = 0x60;
  constexpr std::uint8_t standard = 0;
  constexpr std::uint8_t reserved = 0x60;
  constexpr std::uint8_t get_descriptor = 6;
  constexpr std::uint8_t set_descriptor = 7;
  constexpr std::uint8_t synch_frame = 12;
  constexpr std::uint16_t descriptor_support = 0x10;
  constexpr std::uint16_t synch_frame_support = 1;
  const auto type = packet[0] & type_mask;
  if (type != standard) { return type != reserved; }
  return packet[1] == get_descriptor ||
         (packet[1] == set_descriptor && (control & descriptor_support) != 0) ||
         (packet[1] == synch_frame && (control & synch_frame_support) != 0);
}

unsigned packet_size(const std::uint32_t description) noexcept {
  constexpr unsigned size_shift = 19;
  constexpr std::uint32_t size_mask = 0x7ff;
  return (description >> size_shift) & size_mask;
}

bool full_speed_packet(const unsigned bytes) noexcept {
  return bytes == 8 || bytes == 16 || bytes == 32 || bytes == 64;
}
}

bool Usb::contains(const std::uint16_t offset) noexcept {
  return offset >= first && offset < end;
}

bool Usb::observe_link(const Link link, const std::uint16_t control) noexcept {
  if (!link_active(control)) { return false; }
  std::uint16_t state = 0;
  switch (link) {
    case Link::disconnected: break;
    case Link::reset: state = resetting; break;
    case Link::full_speed: state = enumerated; break;
    case Link::suspended: state = enumerated | suspended; break;
    default: return false;
  }
  const auto changed = status_ ^ state;
  status_ &= static_cast<std::uint16_t>(~(enumerated | suspended | resetting));
  status_ |= state;
  if ((changed & resetting) != 0) {
    status_ |= reset_changed;
    if (link == Link::reset) {
      configuration_status_ = 0;
      configuration_status_pending_ = false;
      halted_ = 0;
      setup_.reset();
      status_ &= static_cast<std::uint16_t>(~(setup_received | setup_overrun | frame_received));
      std::fill_n(endpoints_.begin(), buffers_.size(), std::uint16_t{0});
    }
  }
  if ((changed & suspended) != 0) { status_ |= suspend_changed; }
  return true;
}

bool Usb::accepts_packets(const std::uint16_t control) const noexcept {
  return link_active(control) && (status_ & (enumerated | suspended | resetting)) == enumerated;
}

bool Usb::receive_frame(const std::uint16_t frame, const std::uint16_t control) noexcept {
  if (frame > max_frame || !accepts_packets(control)) { return false; }
  frame_ = frame;
  status_ |= frame_received;
  return true;
}

bool Usb::receive_setup(const Setup &packet, const std::uint16_t control) noexcept {
  if (!accepts_packets(control) || configuration_status_pending_) { return false; }
  if (packet[0] == 0 && packet[1] == set_configuration) {
    constexpr unsigned count_field = 3;
    const auto configuration = packet[2];
    const auto limit = configurations_[count_index] & count_field;
    if (limit == count_field || configuration > limit || packet[3] != 0 || packet[4] != 0 ||
        packet[5] != 0 || packet[6] != 0 || packet[7] != 0 ||
        (status_ & (setup_received | setup_overrun)) != 0) {
      return false;
    }
    configuration_status_ =
        static_cast<std::uint16_t>(configuration_valid | configuration_request |
                                   (unsigned{configuration} << configuration_shift));
    configuration_status_pending_ = true;
    restart_control();
    return true;
  }
  if (!firmware_request(packet, control)) { return false; }
  if ((status_ & setup_received) != 0) { status_ |= setup_overrun; }
  setup_ = packet;
  status_ |= setup_received;
  restart_control();
  return true;
}

void Usb::restart_control() noexcept {
  constexpr auto previous = static_cast<std::uint16_t>(~(stall | data_valid));
  endpoints_[0] &= previous;
  endpoints_[1] &= previous;
}

std::optional<Usb::LocalEndpoint> Usb::local_endpoint(const std::uint8_t address,
                                                      const bool in,
                                                      const std::uint16_t control) const noexcept {
  if ((address & reserved_address) != 0 || ((address & in_direction) != 0) != in ||
      configuration_status_pending_ || !accepts_packets(control)) {
    return {};
  }
  const auto number = unsigned{address} & endpoint_number;
  if (number == 0) {
    const auto size = packet_size(configurations_[0]);
    if (!setup_ || !full_speed_packet(size)) { return {}; }
    return LocalEndpoint{.slot = in ? 0U : 1U, .maximum = size};
  }
  constexpr unsigned address_mask = 0x1f;
  constexpr unsigned direction_shift = 3;
  constexpr unsigned type_mask = 0x60;
  constexpr unsigned bulk_type = 0x40;
  constexpr unsigned config_shift = 7;
  constexpr unsigned field = 0x0f;
  constexpr unsigned interface_shift = 11;
  constexpr unsigned alternate_shift = 15;
  constexpr unsigned first_interface_shift = 4;
  constexpr unsigned second_interface_shift = 8;
  constexpr unsigned interface_mask = 7;
  const auto configuration = (unsigned{configuration_status_} >> configuration_shift) & field;
  if (number > 3 || configuration == 0 || (configuration_status_ & configuration_valid) == 0) {
    return {};
  }
  const auto code = number | ((unsigned{address} & in_direction) >> direction_shift);
  const auto shift = configuration == 1 ? first_interface_shift : second_interface_shift;
  const auto limit = (configurations_[count_index] >> shift) & interface_mask;
  std::optional<LocalEndpoint> endpoint;
  // NISOF specifies high-speed isochronous scheduling. It has no role in local bulk routing.
  for (unsigned slot = first_bulk_slot; slot < buffers_.size(); ++slot) {
    const auto description = configurations_[slot - 1];
    if ((description & (address_mask | type_mask)) != (code | bulk_type) ||
        ((description >> config_shift) & field) != configuration ||
        ((description >> alternate_shift) & field) != 0) {
      continue;
    }
    const auto interface = (description >> interface_shift) & field;
    const auto size = packet_size(description);
    if (endpoint || interface > limit || !full_speed_packet(size)) { return {}; }
    endpoint = LocalEndpoint{.slot = slot, .maximum = size};
  }
  return endpoint;
}

bool Usb::halted(const unsigned slot) const noexcept {
  return (endpoints_[slot] & stall) != 0 || (halted_ & (1U << slot)) != 0;
}

void Usb::record_halt(const unsigned slot) noexcept {
  if (slot < first_bulk_slot) { return; }
  halted_ |= static_cast<std::uint8_t>(1U << slot);
  endpoints_[slot] &= static_cast<std::uint16_t>(~stall);
}

Usb::InputPacket Usb::input(const std::uint8_t address, const std::uint16_t control) noexcept {
  if (address == in_direction && configuration_status_pending_) {
    if (!accepts_packets(control)) { return {}; }
    return {.transfer = (configuration_status_ & configuration_request) != 0 ? Transfer::nak
                                                                             : Transfer::ready,
            .bytes = {}};
  }
  const auto endpoint = local_endpoint(address, true, control);
  if (!endpoint) { return {}; }
  const auto packet = input_packet(*endpoint);
  if (packet.transfer == Transfer::stall) { record_halt(endpoint->slot); }
  return packet;
}

Usb::InputPacket Usb::input_packet(const LocalEndpoint endpoint) const noexcept {
  const auto flags = endpoints_[endpoint.slot];
  if (halted(endpoint.slot)) { return {.transfer = Transfer::stall, .bytes = {}}; }
  if ((flags & data_valid) == 0) { return {.transfer = Transfer::nak, .bytes = {}}; }
  const auto bytes = static_cast<unsigned>(flags & payload_size);
  const auto &buffer = buffers_[endpoint.slot];
  if (bytes > endpoint.maximum ||
      !std::ranges::all_of(std::span(buffer.known).first(bytes),
                           [](const auto known) { return known != 0; })) {
    return {};
  }
  return {.transfer = Transfer::ready, .bytes = std::span(buffer.bytes).first(bytes)};
}

bool Usb::acknowledge_input(const std::uint8_t address, const std::uint16_t control) noexcept {
  if (address == in_direction && configuration_status_pending_) {
    if (!accepts_packets(control) || (configuration_status_ & configuration_request) != 0) {
      return false;
    }
    configuration_status_pending_ = false;
    return true;
  }
  const auto endpoint = local_endpoint(address, true, control);
  if (!endpoint || input_packet(*endpoint).transfer != Transfer::ready) { return false; }
  auto &flags = endpoints_[endpoint->slot];
  flags = static_cast<std::uint16_t>((flags & ~data_valid) | transfer);
  return true;
}

Usb::Transfer Usb::output(const std::uint8_t address,
                          const std::span<const std::uint8_t> packet,
                          const std::uint16_t control) noexcept {
  const auto endpoint = local_endpoint(address, false, control);
  if (!endpoint || packet.size() > endpoint->maximum) { return Transfer::unavailable; }
  const auto slot = endpoint->slot;
  auto &flags = endpoints_[slot];
  if (halted(slot)) {
    record_halt(slot);
    return Transfer::stall;
  }
  if ((flags & data_valid) != 0) { return Transfer::nak; }
  auto &buffer = buffers_[slot];
  std::ranges::copy(packet, buffer.bytes.begin());
  std::ranges::fill(std::span(buffer.known).first(packet.size()), std::uint8_t{1});
  const auto bytes = static_cast<std::uint16_t>(packet.size());
  flags = static_cast<std::uint16_t>((flags & ~payload_size) | transfer | data_valid | bytes);
  return Transfer::ready;
}

std::optional<unsigned> Usb::configuration_index() const noexcept {
  const auto index = unsigned{selector_} >> 8U;
  if (index >= 1 && index <= 5) { return index - 1U; }
  if (index == counts) { return count_index; }
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
    case timestamp: value = byte(frame_, {.offset = offset}); return true;
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
  if (offset >= setup_begin && offset < setup_end) {
    if (!setup_) { return false; }
    // Acknowledging the interrupt clears its flag, not the last received bytes.
    value = (*setup_)[offset - setup_begin];
    return true;
  }
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
      // DONEECRU acknowledges the update and self-clears when SETECR is negated.
      if ((requested & configuration_done) != 0 &&
          (configuration_status_ & configuration_request) != 0) {
        configuration_status_ &= static_cast<std::uint16_t>(~configuration_request);
        halted_ = 0;
      }
      return true;
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
  if ((status_ & interrupts_ & status_interrupts) != 0 ||
      ((interrupts_ & resume_enable) != 0 && (status_ & suspended) == 0)) {
    return status_vector;
  }
  if ((status_ & interrupts_ & (setup_received | setup_overrun)) != 0) { return setup_vector; }
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
  return (status_ & interrupts_ & frame_received) != 0 ? std::optional{sof_vector} : std::nullopt;
}
}
