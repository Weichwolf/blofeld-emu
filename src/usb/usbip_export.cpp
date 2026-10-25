// SPDX-License-Identifier: GPL-3.0-only
#include "usb/usbip_export.hpp"

#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "usb/usb_descriptor_detail.hpp"
#include "usb/usb_device.hpp"
#include "usb/usbip_management_detail.hpp"

#include <algorithm>
#include <cstdint>
#include <utility>
#include <variant>
#include <vector>

namespace blofeld {
namespace {
UsbConfigurationInfo observed(const McuBus &bus, const ExecutionContext context) {
  try {
    return bus.usb_host_configuration();
  } catch (const Fault &fault) {
    throw Fault("USB/IP UCCSR/SETECR observation mcu-pc=" + hex(context.pc) +
                " ccr=" + hex(context.ccr, 2) + ": " + fault.what());
  }
}

std::vector<UsbInterfaceDescription> selected(const UsbDeviceDescriptors &descriptors,
                                              const UsbConfigurationInfo &configuration) {
  usb_descriptor::validate_device(descriptors.device());
  static_cast<void>(usb_descriptor::configuration_length(descriptors.configuration()));
  if (configuration.configuration == 0) { return {}; }
  if (configuration.configuration != descriptors.configuration()[5] ||
      configuration.interfaces != descriptors.interface_count()) {
    throw Fault(
        "USB active configuration/interface observations disagree with captured descriptors");
  }
  std::vector<UsbInterfaceDescription> result;
  for (unsigned i = 0; i < configuration.interfaces; ++i) {
    const auto alternate = configuration.alternatives.at(i);
    const auto alternatives = descriptors.alternatives();
    const auto found = std::ranges::find_if(alternatives, [i, alternate](const auto &entry) {
      return entry.number == i && entry.alternate == alternate;
    });
    if (found == alternatives.end()) {
      throw Fault("USB selected interface alternative is absent from descriptors");
    }
    result.push_back(*found);
  }
  return result;
}

void validate_location(const UsbipLocation &location) {
  usbip_management::validate_busid(location.busid);
  constexpr char first_printable = ' ';
  constexpr char last_printable = '~';
  if (location.bus == 0 || location.device == 0 || location.path.empty() ||
      location.path.front() != '/' || location.path.size() >= usbip_management::path_bytes ||
      std::ranges::any_of(location.path,
                          [](const auto c) { return c < first_printable || c > last_printable; })) {
    throw Fault("Invalid USB/IP virtual path or bus/device location");
  }
}
}

UsbipExport::UsbipExport(UsbDeviceDescriptors descriptors,
                         UsbipLocation location,
                         const McuBus &bus,
                         const ExecutionContext context)
    : descriptors_(std::move(descriptors)),
      location_(std::move(location)),
      configuration_(observed(bus, context)),
      interfaces_(selected(descriptors_, configuration_)) {
  validate_location(location_);
}

std::uint32_t UsbipExport::device_id() const {
  constexpr unsigned bus_shift = 16;
  return (static_cast<std::uint32_t>(location_.bus) << bus_shift) | location_.device;
}

std::vector<std::uint8_t> UsbipExport::device_record() const {
  using namespace usbip_management;
  std::vector<std::uint8_t> result(path_bytes + busid_bytes, 0);
  std::ranges::copy(location_.path, result.begin());
  std::ranges::copy(location_.busid, result.begin() + path_bytes);
  append(result, location_.bus, Width::longword);
  append(result, location_.device, Width::longword);
  // McuUsb's successful transfer API currently qualifies full speed only.
  constexpr unsigned linux_usb_speed_full = 2;
  append(result, linux_usb_speed_full, Width::longword);
  constexpr unsigned vendor_offset = 8;
  constexpr unsigned product_offset = 10;
  constexpr unsigned revision_offset = 12;
  const auto descriptor = descriptors_.device();
  for (const auto offset : {vendor_offset, product_offset, revision_offset}) {
    append(result, usb_descriptor::little_word(descriptor, offset), Width::word);
  }
  result.insert(result.end(), descriptor.begin() + 4, descriptor.begin() + 7);
  result.push_back(configuration_.configuration);
  constexpr unsigned configuration_count_offset = 17;
  result.push_back(descriptor[configuration_count_offset]);
  result.push_back(static_cast<std::uint8_t>(interfaces_.size()));
  return result;
}

UsbipManagementReply UsbipExport::reply(const UsbipManagementRequest &request) {
  using namespace usbip_management;
  if (std::holds_alternative<UsbipListRequest>(request)) {
    auto result = header(list, 0);
    append(result, available() ? 1U : 0U, Width::longword);
    if (available()) {
      const auto record = device_record();
      result.insert(result.end(), record.begin(), record.end());
      for (const auto &entry : interfaces_) {
        result.insert(result.end(), {entry.device_class, entry.subclass, entry.protocol, 0});
      }
    }
    return {.bytes = std::move(result)};
  }
  if (!available() || std::get<UsbipImportRequest>(request).busid != location_.busid) {
    return {.bytes = header(import, 1)};
  }
  auto result = header(import, 0);
  const auto record = device_record();
  result.insert(result.end(), record.begin(), record.end());
  claimed_ = true;
  return {.bytes = std::move(result), .imported = true, .close = false};
}
}
