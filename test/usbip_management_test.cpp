// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "usb/usb_device.hpp"
#include "usb/usbip.hpp"
#include "usb/usbip_export.hpp"
#include "usb_device_fixture.hpp"
#include "usb_endpoint_reset_fixture.hpp"
#include "usb_host_fixture.hpp"
#include "usbip_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <variant>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_device;
constexpr std::array<std::uint8_t, 8> list_request{1, 0x11, 0x80, 5, 0, 0, 0, 0};
constexpr unsigned management_header = 8;
constexpr unsigned import_bytes = 40;
constexpr unsigned record_bytes = 312;
constexpr std::uint8_t version_low = 0x11;
constexpr std::uint8_t wire_input_endpoint = 0x81;
constexpr unsigned expected_device_id = 0x00010002;

UsbipLocation location() {
  return {.path = "/blofeld-emu/usb/1-2", .busid = "1-2", .bus = 1, .device = 2};
}

Bytes import_request(const std::string &busid = "1-2") {
  Bytes result{1, version_low, 0x80, 3, 0, 0, 0, 0};
  result.resize(import_bytes, 0);
  std::ranges::copy(busid, result.begin() + management_header);
  return result;
}

void test_management_fragmentation_and_handoff() {
  const auto wire_import = import_request();
  for (std::size_t split = 0; split <= wire_import.size(); ++split) {
    UsbipManagementDecoder decoder;
    require(decoder.consume(std::span(wire_import).first(split)) == split,
            "Every fixed import split consumes only its available prefix");
    require(decoder.request().has_value() == (split == wire_import.size()),
            "Import never becomes ready before all32 identifier bytes arrive");
    require(decoder.consume(std::span(wire_import).subspan(split)) == wire_import.size() - split,
            "Every import split consumes its remaining bytes");
    const auto &parsed = decoder.request();
    if (!parsed) { throw Fault("Missing independently expected import request"); }
    require(std::get<UsbipImportRequest>(*parsed).busid == "1-2",
            "Every import split retains the independently supplied bus identifier");
    decoder.finish();
    reject([&decoder] { static_cast<void>(decoder.consume({})); }, "Closed/faulted");
  }
  for (std::size_t split = 0; split <= list_request.size(); ++split) {
    UsbipManagementDecoder decoder;
    require(decoder.consume(std::span(list_request).first(split)) == split &&
                decoder.consume(std::span(list_request).subspan(split)) ==
                    list_request.size() - split,
            "Every list split consumes exactly its request");
    const auto &parsed = decoder.request();
    if (!parsed) { throw Fault("Missing independently expected list request"); }
    require(std::holds_alternative<UsbipListRequest>(*parsed),
            "Every list split produces the independently expected request type");
    decoder.finish();
  }
  constexpr unsigned transfer_sequence = 19;
  auto request = test::usbip_fixture::submit(
      test::usb_host::request(transfer_sequence, wire_input_endpoint, 7));
  auto coalesced = wire_import;
  coalesced.insert(coalesced.end(), request.begin(), request.end());
  UsbipManagementDecoder decoder;
  const auto consumed = decoder.consume(coalesced);
  require(consumed == import_bytes && decoder.consume(std::span(coalesced).subspan(consumed)) == 0,
          "An already complete import leaves coalesced transfer bytes untouched");
  UsbipDecoder transfers(test::usbip_fixture::device);
  transfers.feed(std::span(coalesced).subspan(consumed));
  const auto submission = transfers.next();
  require(submission && std::get<UsbTransferRequest>(*submission).sequence == transfer_sequence,
          "The exact unconsumed suffix enters the existing transfer decoder");
  transfers.finish();
}

void test_management_rejections() {
  const auto wire = import_request();
  for (std::size_t cut = 0; cut < wire.size(); ++cut) {
    UsbipManagementDecoder decoder;
    static_cast<void>(decoder.consume(std::span(wire).first(cut)));
    reject([&decoder] { decoder.finish(); }, "EOF");
    reject([&decoder, &wire] { static_cast<void>(decoder.consume(wire)); }, "Closed/faulted");
  }
  for (const auto change :
       {std::array<unsigned, 2>{1, 0x12}, {2, 0}, {3, 7}, {7, 1}, {8, 0}, {9, '/'}, {39, 1}}) {
    auto invalid = wire;
    invalid.at(change[0]) = static_cast<std::uint8_t>(change[1]);
    UsbipManagementDecoder decoder;
    reject([&] { static_cast<void>(decoder.consume(invalid)); }, "USB/IP");
    reject([&] { static_cast<void>(decoder.consume(wire)); }, "Closed/faulted");
  }
  auto invalid = Bytes(import_bytes, 'a');
  std::ranges::copy(std::span(wire).first(management_header), invalid.begin());
  UsbipManagementDecoder decoder;
  reject([&] { static_cast<void>(decoder.consume(invalid)); }, "Unterminated");
}

Bytes expected_record() {
  // Independent fixed wire positions, not the production encoder's field helpers.
  enum Offset : std::uint16_t {
    path_size = 256,
    bus_last = 291,
    device_last = 295,
    speed_last = 299,
    vendor_high = 300,
    product_high = 302,
    revision_high = 304,
    configuration_byte = 309,
    configurations_byte = 310,
    interfaces_byte = 311
  };

  Bytes result(record_bytes, 0);
  const std::string path = "/blofeld-emu/usb/1-2";
  std::ranges::copy(path, result.begin());
  result[path_size] = '1';
  result[path_size + 1] = '-';
  result[path_size + 2] = '2';
  result[bus_last] = 1;
  result[device_last] = 2;
  result[speed_last] = 2;
  constexpr unsigned vendor_low = 0x34;
  constexpr unsigned vendor_msb = 0x12;
  constexpr unsigned product_low = 0x78;
  constexpr unsigned product_msb = 0x56;
  result[vendor_high] = vendor_msb;
  result[vendor_high + 1] = vendor_low;
  result[product_high] = product_msb;
  result[product_high + 1] = product_low;
  result[revision_high] = 1;
  result[revision_high + 1] = 2;
  result[configuration_byte] = 1;
  result[configurations_byte] = 1;
  result[interfaces_byte] = 2;
  return result;
}

void test_qualified_record_and_lease() {
  McuBus bus(descriptor_flash);
  prepare_device(bus);
  const auto descriptors = capture(bus);
  bus.observe_usb_configuration({.configuration = 1, .interface = 1, .alternate = 1});
  bus.write(test::endpoint_reset::configuration_status, 0x10);
  UsbipExport exported(descriptors, location(), bus, context);
  require(exported.device_id() == expected_device_id && exported.available(),
          "Explicit virtual bus/device numbers form the independent transfer identity");
  Bytes expected{1, version_low, 0, 5, 0, 0, 0, 0, 0, 0, 0, 1};
  const auto record = expected_record();
  expected.insert(expected.end(), record.begin(), record.end());
  constexpr std::array<std::uint8_t, 8> selected_interfaces{1, 1, 0, 0, 0xff, 0x12, 0x34, 0};
  expected.insert(expected.end(), selected_interfaces.begin(), selected_interfaces.end());
  const auto listed = exported.reply(UsbipListRequest{});
  require(listed.bytes == expected && listed.close && !listed.imported && exported.available(),
          "DEVLIST returns exact captured identity, explicit configuration and selected-alt class "
          "fields");
  constexpr std::array<std::uint8_t, 8> failure{1, 0x11, 0, 3, 0, 0, 0, 1};
  const auto wrong = exported.reply(UsbipImportRequest{.busid = "1-3"});
  require(std::ranges::equal(wrong.bytes, failure) && wrong.close && !wrong.imported &&
              exported.available(),
          "A wrong bus ID fails import without returning identity or taking the lease");
  expected = {1, version_low, 0, 3, 0, 0, 0, 0};
  expected.insert(expected.end(), record.begin(), record.end());
  const auto imported = exported.reply(UsbipImportRequest{.busid = "1-2"});
  require(imported.bytes == expected && imported.imported && !imported.close &&
              !exported.available(),
          "IMPORT includes no interface suffix and acquires exactly one existing export lease");
  require(std::ranges::equal(exported.reply(UsbipImportRequest{.busid = "1-2"}).bytes, failure),
          "A second import cannot claim the same active export");
  constexpr std::array<std::uint8_t, 12> empty_list{1, 0x11, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0};
  require(std::ranges::equal(exported.reply(UsbipListRequest{}).bytes, empty_list),
          "A leased export is absent from the available-device list");
  constexpr unsigned expected_list_bytes = 332;
  exported.release();
  require(exported.available() &&
              exported.reply(UsbipListRequest{}).bytes.size() == expected_list_bytes,
          "Lease release retains qualified identity without inventing a board reset");
}

void test_unknown_observations_and_location() {
  McuBus bus(descriptor_flash);
  prepare_device(bus);
  const auto descriptors = capture(bus);
  bus.observe_usb_configuration({.configuration = 1});
  reject([&] { const UsbipExport unused(descriptors, location(), bus, context); },
         "pending firmware");
  bus.write(test::endpoint_reset::configuration_status, 0x10);
  bus.write(test::endpoint_reset::control, test::endpoint_reset::mce_high);
  constexpr unsigned three_second_alternatives = 0x31;
  test::endpoint_reset::configure(bus, {.bank = 9, .value = three_second_alternatives});
  bus.write(test::endpoint_reset::control, test::endpoint_reset::men_high);
  bus.observe_usb_configuration({.configuration = 1, .interface = 1, .alternate = 2});
  bus.write(test::endpoint_reset::configuration_status, 0x10);
  reject([&] { const UsbipExport unused(descriptors, location(), bus, context); },
         "alternative is absent");
  bus.observe_usb_decoder({.reset = true});
  bus.observe_usb_decoder({.enumerated = true});
  UsbipExport unconfigured(descriptors, location(), bus, context);
  const auto reply = unconfigured.reply(UsbipListRequest{});
  constexpr unsigned configuration_position = 321;
  constexpr unsigned configuration_count_position = 322;
  constexpr unsigned interface_count_position = 323;
  constexpr unsigned unconfigured_reply_bytes = 324;
  require(
      reply.bytes.size() == unconfigured_reply_bytes && reply.bytes[configuration_position] == 0 &&
          reply.bytes[configuration_count_position] == 1 &&
          reply.bytes[interface_count_position] == 0,
      "Actual bus reset advertises zero active interfaces, never a guessed default alternative");
  McuBus unready(descriptor_flash);
  reject([&] { const UsbipExport unused(descriptors, location(), unready, context); },
         "mcu-pc=0x8123");
  McuBus different_counts(descriptor_flash);
  prepare(different_counts);
  reject([&] { const UsbipExport unused(descriptors, location(), different_counts, context); },
         "observations disagree");
  for (const auto &where : std::array<UsbipLocation, 6>{
           UsbipLocation{.path = "relative", .busid = "1-2", .bus = 1, .device = 2},
           {.path = "/valid", .busid = "1/2", .bus = 1, .device = 2},
           {.path = "/valid", .busid = "", .bus = 1, .device = 2},
           {.path = "/valid", .busid = "1-2", .bus = 0, .device = 2},
           {.path = "/valid", .busid = "1-2", .bus = 1, .device = 0},
           {.path = "/newline\n", .busid = "1-2", .bus = 1, .device = 2}}) {
    reject([&] { const UsbipExport unused(descriptors, where, bus, context); }, "USB/IP");
  }
}
}

int main() {
  try {
    test_management_fragmentation_and_handoff();
    test_management_rejections();
    test_qualified_record_and_lease();
    test_unknown_observations_and_location();
    std::cout
        << "Verified exact USB/IP discovery/import records from MCU descriptor transfers, stream "
           "handoff, observations and leases. No host attachment or original enumeration claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
