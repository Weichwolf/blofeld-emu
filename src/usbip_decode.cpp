// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/usb_transfer.hpp"
#include "blofeld/usbip.hpp"
#include "usbip_fields.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <linux/usbip.h>
#include <optional>
#include <span>
#include <utility>

namespace blofeld {
namespace {
using usbip::Field;
using usbip::word;
constexpr std::size_t maximum_frame = maximum_usb_transfer + usbip_transfer_header;

UsbTransferRequest header(const std::span<const std::uint8_t> bytes) {
  const auto direction = word(bytes, Field::direction);
  const auto endpoint = word(bytes, Field::endpoint);
  const auto length = word(bytes, Field::length);
  const auto flags = word(bytes, Field::flags_status_target);
  constexpr unsigned endpoint_mask = 15;
  constexpr unsigned input_bit = 0x80;
  constexpr unsigned host_memory_flags =
      USBIP_URB_NO_TRANSFER_DMA_MAP | USBIP_URB_NO_INTERRUPT | USBIP_URB_FREE_BUFFER |
      USBIP_URB_DMA_MAP_SINGLE | USBIP_URB_DMA_MAP_PAGE | USBIP_URB_DMA_MAP_SG |
      USBIP_URB_MAP_LOCAL | USBIP_URB_SETUP_MAP_SINGLE | USBIP_URB_SETUP_MAP_LOCAL |
      USBIP_URB_DMA_SG_COMBINED | USBIP_URB_ALIGNED_TEMP_BUFFER;
  constexpr unsigned supported_flags =
      host_memory_flags | USBIP_URB_SHORT_NOT_OK | USBIP_URB_ZERO_PACKET | USBIP_URB_DIR_IN;
  if (direction > 1 || endpoint > endpoint_mask || length > maximum_usb_transfer ||
      (flags & ~supported_flags) != 0 || (direction == 0 && (flags & USBIP_URB_DIR_IN) != 0)) {
    throw Fault("Unsupported USB/IP direction, endpoint, transfer length or flags");
  }
  const auto noniso = [](const auto value) {
    return value == 0 || value == std::numeric_limits<std::uint32_t>::max();
  };
  if (!noniso(word(bytes, Field::frame)) || !noniso(word(bytes, Field::packets))) {
    throw Fault("USB/IP isochronous packet/frame scheduling is unsupported");
  }
  UsbTransferRequest result{
      .sequence = word(bytes, Field::sequence),
      .address = static_cast<std::uint8_t>(endpoint | (direction != 0 ? input_bit : 0)),
      .length = length,
      .setup = {},
      .output = {},
      .short_not_ok = (flags & USBIP_URB_SHORT_NOT_OK) != 0,
      .zero_packet = (flags & USBIP_URB_ZERO_PACKET) != 0};
  std::ranges::copy(bytes.subspan(static_cast<std::size_t>(Field::setup), 8), result.setup.begin());
  return result;
}

UsbUnlinkRequest unlink_request(const std::span<const std::uint8_t> bytes) {
  constexpr unsigned padding_bytes = 24;
  if (word(bytes, Field::direction) != 0 || word(bytes, Field::endpoint) != 0 ||
      std::ranges::any_of(bytes.subspan(static_cast<std::size_t>(Field::length), padding_bytes),
                          [](const auto value) { return value != 0; })) {
    throw Fault("Invalid USB/IP unlink fields/padding");
  }
  return {.sequence = word(bytes, Field::sequence),
          .target = word(bytes, Field::flags_status_target)};
}
}

UsbipDecoder::UsbipDecoder(const std::uint32_t device) : device_(device) {
  constexpr unsigned lower_word = 0xffff;
  if ((device >> 16U) == 0 || (device & lower_word) == 0) {
    throw Fault("USB/IP requires a nonzero bus/device identity");
  }
}

void UsbipDecoder::require_active() const {
  if (faulted_ || closed_) {
    throw Fault("Closed/faulted USB/IP decoder requires a new connection");
  }
}

std::size_t UsbipDecoder::input_space() const {
  require_active();
  return maximum_frame - (bytes_.size() - consumed_);
}

void UsbipDecoder::feed(const std::span<const std::uint8_t> bytes) {
  require_active();
  if (bytes.size() > maximum_frame || bytes_.size() - consumed_ > maximum_frame - bytes.size()) {
    faulted_ = true;
    throw Fault("USB/IP stream exceeds its bounded frame buffer");
  }
  if (consumed_ != 0) {
    bytes_.erase(bytes_.begin(), bytes_.begin() + static_cast<std::ptrdiff_t>(consumed_));
    consumed_ = 0;
  }
  bytes_.insert(bytes_.end(), bytes.begin(), bytes.end());
}

std::optional<UsbipCommand> UsbipDecoder::next() {
  require_active();
  const auto bytes = std::span(bytes_).subspan(consumed_);
  if (bytes.size() < usbip_transfer_header) { return std::nullopt; }
  try {
    if (word(bytes, Field::device) != device_) {
      throw Fault("USB/IP request targets another device");
    }
    std::optional<UsbipCommand> result;
    std::size_t size = usbip_transfer_header;
    const auto command = word(bytes, Field::command);
    if (command == usbip::submit) {
      auto request = header(bytes);
      if ((request.address & 0x80U) == 0) { size += request.length; }
      if (bytes.size() < size) { return std::nullopt; }
      request.output.assign(bytes.begin() + static_cast<std::ptrdiff_t>(usbip_transfer_header),
                            bytes.begin() + static_cast<std::ptrdiff_t>(size));
      validate_usb_transfer(request);
      result = std::move(request);
    } else if (command == usbip::unlink) {
      result = unlink_request(bytes.first(usbip_transfer_header));
    } else {
      throw Fault("Unknown USB/IP transfer command");
    }
    consumed_ += size;
    if (consumed_ == bytes_.size()) {
      bytes_.clear();
      consumed_ = 0;
    }
    return result;
  } catch (const Fault &) {
    faulted_ = true;
    throw;
  }
}

void UsbipDecoder::finish() {
  require_active();
  if (bytes_.size() != consumed_) {
    faulted_ = true;
    throw Fault("USB/IP EOF with an incomplete or unconsumed transfer");
  }
  closed_ = true;
}
}
