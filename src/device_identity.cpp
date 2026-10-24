// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/device_identity.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace blofeld {
McuFlashImage provision_virtual_identity(const McuFlashImage &unprovisioned,
                                         const std::string_view serial) {
  constexpr unsigned flash_first = 0x8000;
  constexpr unsigned erased = 0xff;
  constexpr unsigned first_printable = 0x20;
  constexpr unsigned last_printable = 0x7e;
  constexpr auto offset = device_identity::serial_address - flash_first;
  if (serial.empty() || serial.size() >= device_identity::serial_bytes) {
    throw Fault("Virtual device serial must contain 1..31 printable ASCII bytes");
  }
  for (const char character : serial) {
    const auto byte = static_cast<unsigned char>(character);
    if (byte < first_printable || byte > last_printable) {
      throw Fault("Virtual device serial contains a non-printable or non-ASCII byte");
    }
  }
  unprovisioned.validate();
  for (std::size_t n = 0; n < device_identity::serial_bytes; ++n) {
    const auto address = device_identity::serial_address + n;
    if (!unprovisioned.known[offset + n]) {
      throw Fault("Virtual device factory provisioning address=" + hex(address) +
                  ": unknown serial storage cannot be replaced");
    }
    if (unprovisioned.bytes[offset + n] != erased) {
      throw Fault("Virtual device factory provisioning address=" + hex(address) +
                  ": existing unit data cannot be replaced");
    }
  }
  auto image = unprovisioned;
  for (std::size_t n = 0; n <= serial.size(); ++n) {
    image.bytes[offset + n] = n < serial.size() ? static_cast<std::uint8_t>(serial[n]) : 0;
    image.fresh.reset((offset + n) / 2);
  }
  image.validate();
  return image;
}
}
