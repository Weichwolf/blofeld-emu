// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/usb_transfer.hpp"

#include "blofeld/bus.hpp"

#include <algorithm>

namespace blofeld {
void validate_usb_transfer(const UsbTransferRequest &request) {
  constexpr unsigned input_bit = 0x80;
  constexpr unsigned reserved_address = 0x70;
  const bool input = (request.address & input_bit) != 0;
  const bool control = (request.address & 15U) == 0;
  if ((request.address & reserved_address) != 0 || request.length > maximum_usb_transfer ||
      (input && !request.output.empty()) || (!input && request.output.size() != request.length)) {
    throw Fault("Invalid USB host transfer address/length/payload");
  }
  if ((request.short_not_ok && !input) || (request.zero_packet && (input || control))) {
    throw Fault("Unsupported USB host transfer option for its direction/type");
  }
  if (control) {
    const auto length =
        static_cast<unsigned>(request.setup[6]) | (static_cast<unsigned>(request.setup[7]) << 8U);
    if (length != request.length ||
        (length != 0 && input != ((request.setup[0] & input_bit) != 0))) {
      throw Fault("USB control setup direction/length disagrees with its host transfer");
    }
  } else if (std::ranges::any_of(request.setup, [](const auto value) { return value != 0; })) {
    throw Fault("USB bulk transfer carries unexpected setup bytes");
  }
}
}
