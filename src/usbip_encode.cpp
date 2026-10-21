// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/usb_transfer.hpp"
#include "blofeld/usbip.hpp"
#include "usbip_fields.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::usbip {
std::uint32_t word(const std::span<const std::uint8_t> bytes, const Field field) {
  const auto at = static_cast<std::size_t>(field);
  if (at > bytes.size() || bytes.size() - at < 4) { throw Fault("Truncated USB/IP header field"); }
  std::uint32_t result = 0;
  for (unsigned i = 0; i < 4; ++i) { result = (result << 8U) | bytes[at + i]; }
  return result;
}

void put(const std::span<std::uint8_t> bytes, const Field field, std::uint32_t value) {
  const auto at = static_cast<std::size_t>(field);
  if (at > bytes.size() || bytes.size() - at < 4) { throw Fault("Truncated USB/IP output field"); }
  constexpr unsigned byte_mask = 0xff;
  for (unsigned i = 4; i > 0; --i) {
    bytes[at + i - 1] = static_cast<std::uint8_t>(value & byte_mask);
    value >>= 8U;
  }
}
}

namespace blofeld {
namespace {
std::int32_t status(const UsbTransferStatus value) {
  // Linux error numbers are part of the wire protocol, independent of host errno.
  constexpr std::int32_t linux_pipe = 32;
  constexpr std::int32_t linux_remote_io = 121;
  switch (value) {
    case UsbTransferStatus::success: return 0;
    case UsbTransferStatus::stalled: return -linux_pipe;
    case UsbTransferStatus::short_packet: return -linux_remote_io;
    default: throw Fault("Unknown USB transfer completion status");
  }
}
}

std::vector<std::uint8_t> encode_usbip_completion(const UsbTransferCompletion &completion) {
  if (completion.transferred > maximum_usb_transfer ||
      (completion.input_direction && completion.input.size() != completion.transferred) ||
      (!completion.input_direction && !completion.input.empty())) {
    throw Fault("Invalid USB/IP completion payload/length");
  }
  std::vector<std::uint8_t> result(usbip_transfer_header, 0);
  usbip::put(result, usbip::Field::command, usbip::return_submit);
  usbip::put(result, usbip::Field::sequence, completion.sequence);
  usbip::put(result,
             usbip::Field::flags_status_target,
             std::bit_cast<std::uint32_t>(status(completion.status)));
  usbip::put(result, usbip::Field::length, completion.transferred);
  result.insert(result.end(), completion.input.begin(), completion.input.end());
  return result;
}

std::vector<std::uint8_t> encode_usbip_unlink(const std::uint32_t sequence, const bool cancelled) {
  constexpr std::int32_t linux_connection_reset = 104;
  const std::int32_t status = cancelled ? -linux_connection_reset : 0;
  std::vector<std::uint8_t> result(usbip_transfer_header, 0);
  usbip::put(result, usbip::Field::command, usbip::return_unlink);
  usbip::put(result, usbip::Field::sequence, sequence);
  usbip::put(result, usbip::Field::flags_status_target, std::bit_cast<std::uint32_t>(status));
  return result;
}
}
