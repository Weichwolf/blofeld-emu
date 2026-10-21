// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/usbip_export.hpp"
#include "usbip_management_detail.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::usbip_management {
unsigned
network(const std::span<const std::uint8_t> bytes, const std::size_t offset, const unsigned width) {
  if (width == 0 || width > 4 || offset > bytes.size() || bytes.size() - offset < width) {
    throw Fault("Truncated USB/IP management field");
  }
  unsigned value = 0;
  for (unsigned i = 0; i < width; ++i) { value = (value << 8U) | bytes[offset + i]; }
  return value;
}

void append(std::vector<std::uint8_t> &bytes, const std::uint32_t value, const Width width) {
  constexpr unsigned byte_mask = 0xff;
  for (auto i = static_cast<unsigned>(width); i > 0; --i) {
    bytes.push_back(static_cast<std::uint8_t>((value >> ((i - 1) * 8U)) & byte_mask));
  }
}

std::vector<std::uint8_t> header(const unsigned operation, const unsigned status) {
  std::vector<std::uint8_t> result;
  append(result, version, Width::word);
  append(result, operation, Width::word);
  append(result, status, Width::longword);
  return result;
}

void validate_busid(const std::string_view busid) {
  if (busid.empty() || busid.size() >= busid_bytes || std::ranges::any_of(busid, [](const auto c) {
        return !((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                 c == '-' || c == '.');
      })) {
    throw Fault("Invalid USB/IP bus identifier");
  }
}
}

namespace blofeld {
std::size_t UsbipManagementDecoder::consume(const std::span<const std::uint8_t> bytes) {
  using namespace usbip_management;
  if (faulted_ || closed_) { throw Fault("Closed/faulted USB/IP management decoder"); }
  if (request_) { return 0; }
  std::size_t consumed = 0;
  try {
    if (bytes_.size() < header_bytes) {
      consumed = std::min(bytes.size(), header_bytes - bytes_.size());
      bytes_.insert(
          bytes_.end(), bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(consumed));
      if (bytes_.size() < header_bytes) { return consumed; }
    }
    constexpr unsigned operation_offset = 2;
    constexpr unsigned status_offset = 4;
    const auto operation = network(bytes_, operation_offset, 2);
    if (network(bytes_, 0, 2) != version || network(bytes_, status_offset, 4) != 0 ||
        (operation != (request_bit | list) && operation != (request_bit | import))) {
      throw Fault("Unsupported USB/IP management version, operation or request status");
    }
    if (operation == (request_bit | list)) {
      request_ = UsbipListRequest{};
      return consumed;
    }
    const auto needed = header_bytes + busid_bytes - bytes_.size();
    const auto count = std::min(bytes.size() - consumed, needed);
    const auto remainder = bytes.subspan(consumed, count);
    bytes_.insert(bytes_.end(), remainder.begin(), remainder.end());
    consumed += count;
    if (bytes_.size() != header_bytes + busid_bytes) { return consumed; }
    const auto id = std::span(bytes_).subspan(header_bytes);
    const auto zero = std::ranges::find(id, 0);
    if (zero == id.end() ||
        std::ranges::any_of(std::span(zero, id.end()), [](const auto c) { return c != 0; })) {
      throw Fault("Unterminated or nonzero-padded USB/IP bus identifier");
    }
    const std::string busid(id.begin(), zero);
    validate_busid(busid);
    request_ = UsbipImportRequest{.busid = busid};
    return consumed;
  } catch (const Fault &) {
    faulted_ = true;
    throw;
  }
}

void UsbipManagementDecoder::finish() {
  if (faulted_ || closed_ || !request_) {
    faulted_ = true;
    throw Fault("USB/IP management EOF without a complete valid request");
  }
  closed_ = true;
}
}
