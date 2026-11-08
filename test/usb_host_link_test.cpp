// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/mcu_bus.hpp"
#include "usb/usb_host_link.hpp"
#include "usb_endpoint_reset_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
namespace fixture = test::endpoint_reset;
using test::usb_host::reject;
using test::usb_host::require;

void full_speed_attachment() {
  McuBus bus(fixture::own_image());
  UsbHostLink link(1);
  require(!bus.usb_host_connectable() && !link.poll(bus, 0),
          "A host cannot invent device clocks, pull-up or known decoder state");
  fixture::prepare(bus);
  bus.observe_usb_decoder({});
  constexpr unsigned reset_start = 100;
  constexpr unsigned reset_end = 200;
  constexpr unsigned recovery_end = 210;
  require(!link.poll(bus, 0) && !link.poll(bus, reset_start - 1) &&
              (fixture::word(bus, fixture::status) & 8U) == 0,
          "The host waits a complete hundred-millisecond attachment debounce before reset");
  require(!link.poll(bus, reset_start) && (fixture::word(bus, fixture::status) & 8U) != 0,
          "Host attachment actually asserts USB reset");
  require(
      !link.poll(bus, reset_end - 1) && (fixture::word(bus, fixture::status) & 8U) != 0,
      "The chosen hundred-millisecond root-port reset stays asserted for its complete interval");
  constexpr unsigned link_status = 0x440;
  constexpr unsigned link_mask = 0x448;
  require(!link.poll(bus, reset_end) &&
              (fixture::word(bus, fixture::status) & link_mask) == link_status &&
              bus.usb_host_address() == 0 && bus.usb_host_configuration().configuration == 0,
          "Speed negotiation releases reset without addressing or configuring the device");
  for (unsigned frame = reset_end + 1; frame <= recovery_end; ++frame) {
    require(link.poll(bus, frame) == (frame == recovery_end) &&
                fixture::word(bus, fixture::timestamp) == frame,
            "Real SOFs advance during recovery; control admission waits ten more milliseconds");
  }
  reject([&] { static_cast<void>(link.poll(bus, recovery_end - 1)); }, "backwards");
  constexpr unsigned missed_frame = recovery_end + 2;
  reject([&] { static_cast<void>(link.poll(bus, missed_frame)); }, "missed");
}

void disconnect_and_invalid_period() {
  reject([] { const UsbHostLink invalid(0); }, "positive");
  McuBus bus(fixture::own_image());
  fixture::prepare(bus);
  UsbHostLink link(1);
  require(!link.poll(bus, 0), "The host begins connection debounce");
  bus.write(fixture::control, fixture::mce_high);
  require(!link.poll(bus, 1), "A disconnect cancels incomplete debounce");
  bus.write(fixture::control, fixture::men_high);
  constexpr unsigned reattached = 2;
  constexpr unsigned debounce_end = reattached + 100;
  require(!link.poll(bus, reattached) && !link.poll(bus, debounce_end - 1) &&
              (fixture::word(bus, fixture::status) & 8U) == 0,
          "A new connection restarts the full debounce interval");
  require(!link.poll(bus, debounce_end), "Stable connection begins root-port reset");
  bus.write(fixture::control, fixture::mce_high);
  reject([&] { static_cast<void>(link.poll(bus, debounce_end + 1)); }, "disconnected");
}
}

int main() {
  try {
    full_speed_attachment();
    disconnect_and_invalid_period();
    std::cout << "USB host reset, recovery, SOF and disconnect contracts passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
