// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "mcu/mcu_bus.hpp"
#include "usb/usbip_export.hpp"
#include "usb/usbip_session.hpp"
#include "usb_device_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::usbip_session {
using Bytes = std::vector<std::uint8_t>;
inline constexpr std::array<std::uint8_t, 8> list{1, 0x11, 0x80, 5, 0, 0, 0, 0};

inline Bytes import(const std::string &id = "1-2") {
  constexpr std::array<std::uint8_t, 8> header{1, 0x11, 0x80, 3, 0, 0, 0, 0};
  constexpr unsigned request_bytes = 40;
  Bytes bytes(request_bytes, 0);
  std::ranges::copy(header, bytes.begin());
  std::ranges::copy(id, bytes.begin() + header.size());
  return bytes;
}

inline std::shared_ptr<UsbipExport> publication(McuBus &bus) {
  return std::make_shared<UsbipExport>(
      usb_device::capture(bus),
      UsbipLocation{.path = "/blofeld-emu/usb/1-2", .busid = "1-2", .bus = 1, .device = 2},
      bus,
      usb_host::context);
}

inline Bytes drain(UsbipSession &session) {
  Bytes bytes;
  while (!session.output().empty()) {
    const auto output = session.output();
    bytes.insert(bytes.end(), output.begin(), output.end());
    session.consume_output(output.size());
  }
  return bytes;
}

inline void attach(UsbipSession &session, const std::shared_ptr<UsbipExport> &device) {
  session.receive(import(), device);
  constexpr unsigned import_reply_bytes = 320;
  usb_host::require(drain(session).size() == import_reply_bytes && session.imported(),
                    "A real captured publication is required before transfer traffic");
}
}
