// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/usb_device.hpp"
#include "blofeld/usbip_export.hpp"
#include "usb_device_fixture.hpp"
#include "usb_device_native_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_device;
using namespace blofeld::test::usb_device_native;

void test_native_descriptor_export() {
  const auto flash = firmware();
  McuBus bus(flash);
  prepare_native(bus);
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(initial);
  const auto descriptors = capture_native(bus, cpu);
  auto expected_device = device;
  expected_device[7] = full_speed_maximum;
  require(std::ranges::equal(descriptors.device(), expected_device) &&
              std::ranges::equal(descriptors.configuration(), configuration),
          "All captured bytes originate in actual own guest ROM loads and USB RAM stores");
  UsbipExport exported(descriptors,
                       {.path = "/blofeld-emu/usb/1-2", .busid = "1-2", .bus = 1, .device = 2},
                       bus,
                       {.pc = cpu.state().pc, .ccr = cpu.state().ccr});
  const auto reply = exported.reply(UsbipImportRequest{.busid = "1-2"});
  constexpr unsigned import_reply_bytes = 320;
  require(reply.imported && !reply.close && reply.bytes.size() == import_reply_bytes &&
              !cpu.pending_interrupt(),
          "Native descriptor capture plus actual MCU configuration yields an import record without "
          "a fake guest responder");
}
}

int main() {
  try {
    test_native_descriptor_export();
    std::cout << "Verified three descriptor requests through own HC12 ISRs, actual status/IRQs and "
                 "USB/IP import metadata. No original enumeration/host attachment claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
