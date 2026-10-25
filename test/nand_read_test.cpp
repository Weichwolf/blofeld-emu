// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using blofeld::Nand;
constexpr unsigned last_page = Nand::pages - 1;
constexpr unsigned last_column = Nand::page_bytes - 1;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned reset_command = 0xff;
constexpr unsigned spare_command = 0x50;

struct Selection {
  unsigned page;
  unsigned column;
};

void require(const bool value, const std::string &message) {
  if (!value) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &reason) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) {
    const std::string message = fault.what();
    require(message.contains(reason) && message.contains("page=") && message.contains("column=") &&
                message.contains("command="),
            "Rejected NAND operations must include their reason and address/command context: " +
                message);
    return;
  }
  throw blofeld::Fault("Unsupported NAND operation unexpectedly succeeded: " + reason);
}

void addresses(Nand &device, const Selection selection) {
  device.address(static_cast<std::uint8_t>(selection.column));
  device.address(static_cast<std::uint8_t>(selection.page & byte_mask));
  device.address(static_cast<std::uint8_t>((selection.page >> 8U) & byte_mask));
  device.address(static_cast<std::uint8_t>((selection.page >> 16U) & byte_mask));
}

std::array<std::uint8_t, Nand::page_bytes> own_page(const unsigned page) {
  std::array<std::uint8_t, Nand::page_bytes> result{};
  for (unsigned column = 0; column < result.size(); ++column) {
    constexpr unsigned multiplier = 73;
    result[column] =
        static_cast<std::uint8_t>((page ^ (column * multiplier) ^ (column >> 8U)) & byte_mask);
  }
  return result;
}

void test_rows_and_pointer_areas() {
  struct Start {
    std::uint8_t command;
    unsigned address;
    unsigned column;
  };

  constexpr std::array<Start, 6> starts{{{.command = 0, .address = 0, .column = 0},
                                         {.command = 0, .address = 255, .column = 255},
                                         {.command = 1, .address = 0, .column = 256},
                                         {.command = 1, .address = 255, .column = 511},
                                         {.command = spare_command, .address = 0, .column = 512},
                                         {.command = spare_command, .address = 15, .column = 527}}};
  for (const unsigned row : {0U, 1U, 0xffU, 0x100U, 0xffffU, 0x10000U, last_page}) {
    const auto contents = own_page(row);
    for (const auto start : starts) {
      Nand device;
      device.observe_bytes(row, 0, contents);
      require(!device.ready().has_value(), "Observed contents must not qualify power/readiness");
      device.observe_power_ready();
      device.command(start.command);
      addresses(device, {.page = row, .column = start.address});
      require(device.ready() == false && device.page() == row && device.column() == start.column,
              "Four independent address bytes must select the exact NAND row and area column");
      reject([&] { static_cast<void>(device.read()); }, "no completed page transfer");
      device.address(byte_mask); // Documented surplus address cycles are ignored.
      device.observe_completion();
      reject([&] { device.address(byte_mask); }, "no qualified read command/address phase");
      for (unsigned column = start.column; column < Nand::page_bytes; ++column) {
        require(device.read() == contents[column], "NAND main/spare bytes must remain in order");
      }
      require(!device.ready().has_value(), "Unqualified sequential-row entry must remain unknown");
      reject([&] { static_cast<void>(device.read()); }, "no completed page transfer");
      reject([&] { device.command(0); }, "post-page");
    }
  }
}

void test_unknown_bytes_and_reset() {
  Nand device;
  reject([&] { device.command(0); }, "unqualified readiness");
  reject([&] { device.address(0); }, "no qualified read command");
  reject([&] { device.observe_completion(); }, "no pending reset/read");
  device.observe_power_ready();
  // The manufacturer's power-on Read1 pointer also permits address-only entry.
  addresses(device, {.page = last_page, .column = 0});
  device.observe_completion();
  reject([&] { static_cast<void>(device.read()); }, "unknown contents");
  require(device.column() == 0, "Unknown reads must not advance the NAND column");
  const std::array<std::uint8_t, 3> bytes{0x91, 0, 0xff};
  device.observe_bytes(last_page, 0, bytes);
  for (const auto value : bytes) {
    require(device.read() == value, "Observed bytes must retry exactly");
  }
  reject([&] { static_cast<void>(device.read()); }, "unknown contents");
  require(device.column() == bytes.size(), "Unobserved bytes must not turn into erased defaults");
  device.command(reset_command);
  require(device.ready() == false, "NAND reset must await a separate completion observation");
  device.command(reset_command);
  device.observe_completion();
  reject([&] { device.address(0); }, "no qualified read command");
  device.command(0);
  addresses(device, {.page = last_page, .column = 0});
  device.command(reset_command); // Abort a pending read without changing persistent observations.
  device.observe_completion();
  device.command(0);
  addresses(device, {.page = last_page, .column = 0});
  device.observe_completion();
  require(device.read() == bytes[0], "Reset/read abort must preserve observed NAND cells");
}

void test_strict_rejections() {
  Nand device;
  device.command(reset_command); // Reset is the only qualified command before observed readiness.
  device.observe_completion();
  constexpr std::array<std::uint8_t, 4> unsupported{0x41, 0x7a, 0x90, 0x30};
  for (const auto command : unsupported) {
    reject([&] { device.command(command); }, "unsupported value");
  }
  device.command(spare_command);
  reject([&] { device.address(0x10); }, "upper spare-column bits");
  device.address(15);
  device.address(byte_mask);
  device.address(byte_mask);
  reject([&] { device.address(2); }, "row exceeds device");
  device.address(1);
  reject([&] { device.command(0); }, "unqualified readiness");
  device.observe_completion();
  const std::array<std::uint8_t, 1> byte{0x57};
  reject([&] { device.observe_bytes(Nand::pages, 0, byte); }, "page exceeds device");
  constexpr unsigned past_last_column = last_column + 1;
  reject([&] { device.observe_bytes(last_page, past_last_column, byte); }, "range exceeds page");
  device.observe_bytes(last_page, last_column, byte);
  require(device.read() == byte[0],
          "Rejected addresses/commands must preserve the valid pending read");
  reject([&] { device.observe_power_ready(); }, "not a cold device");
}
}

int main() {
  try {
    test_rows_and_pointer_areas();
    test_unknown_bytes_and_reset();
    test_strict_rejections();
    std::cout << "NAND single-page read: 42 row/area vectors and strict unknown/reset/boundary "
                 "checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
