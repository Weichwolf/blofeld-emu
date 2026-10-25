// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "mcu/mcu_usb.hpp"
#include "usb/usb_device.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace blofeld {
struct UsbipListRequest {};

struct UsbipImportRequest {
  std::string busid;
};

using UsbipManagementRequest = std::variant<UsbipListRequest, UsbipImportRequest>;

// consume() returns only the bytes belonging to this management request. A
// coalesced transfer frame remains with the caller for the post-import decoder.
class UsbipManagementDecoder {
public:
  [[nodiscard]] std::size_t consume(std::span<const std::uint8_t> bytes);

  [[nodiscard]] const std::optional<UsbipManagementRequest> &request() const { return request_; }

  void finish();

private:
  std::vector<std::uint8_t> bytes_;
  std::optional<UsbipManagementRequest> request_;
  bool faulted_ = false;
  bool closed_ = false;
};

// Administrative identity in the virtual exporting host, never a manufacturer
// identity or a claim that this path is a physical sysfs device.
struct UsbipLocation {
  std::string path;
  std::string busid;
  std::uint16_t bus = 0;
  std::uint16_t device = 0;
};

struct UsbipManagementReply {
  std::vector<std::uint8_t> bytes;
  bool imported = false;
  bool close = true;
};

class UsbipExport {
public:
  UsbipExport(UsbDeviceDescriptors descriptors,
              UsbipLocation location,
              const McuBus &bus,
              ExecutionContext context);
  [[nodiscard]] UsbipManagementReply reply(const UsbipManagementRequest &request);

  [[nodiscard]] bool available() const { return !claimed_; }

  [[nodiscard]] std::uint32_t device_id() const;

  // Lease bookkeeping only. The caller separately owns disconnect/reset and
  // must discard the whole export when descriptor/configuration evidence expires.
  void release() { claimed_ = false; }

private:
  [[nodiscard]] std::vector<std::uint8_t> device_record() const;
  UsbDeviceDescriptors descriptors_;
  UsbipLocation location_;
  UsbConfigurationInfo configuration_;
  std::vector<UsbInterfaceDescription> interfaces_;
  bool claimed_ = false;
};
}
