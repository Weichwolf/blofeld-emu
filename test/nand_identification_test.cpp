// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "nand_persistence_fixture.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_file.hpp"
#include "storage/nand_image.hpp"
#include "storage/nand_operation_clock.hpp"
#include "storage/nand_store.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
constexpr std::uint8_t read_id = 0x90, reset = 0xff, read_status = 0x70;
constexpr unsigned all_bits = 0xff, ready_wp = 0xc0;
// Samsung Rev1.1 p33 Figure15, K9F1208U0C. No fifth byte is supplied there.
constexpr std::array<std::uint8_t, 4> documented_prefix{0xec, 0x76, 0x5a, 0x3f};
// An own fifth-byte observation tests the protocol window, not a chip claim.
constexpr std::array<std::uint8_t, 5> own_identity{0xec, 0x76, 0x5a, 0x3f, 0xa5};

void require(const bool value, const std::string &reason) {
  if (!value) { throw Fault(reason); }
}

void reject(const auto &action, const std::string &reason) {
  try {
    action();
  } catch (const Fault &fault) {
    const std::string text = fault.what();
    require(text.contains(reason) && text.contains("page=") && text.contains("column="),
            "Wrong NAND identification refusal: " + text);
    return;
  }
  throw Fault("Expected identification refusal: " + reason);
}

NandIdentification identity(const bool fifth = true) {
  NandIdentification result;
  const auto count = fifth ? own_identity.size() : documented_prefix.size();
  for (std::size_t index = 0; index < count; ++index) {
    result.bytes[index] = {.value = own_identity[index], .known = all_bits};
  }
  return result;
}

void select(auto &device) {
  device.command(read_id);
  device.address(0);
}

void compare(auto &device, const std::size_t begin = 0) {
  for (std::size_t index = begin; index < own_identity.size(); ++index) {
    require(device.read() == own_identity[index], "Independent ID bytes and order");
  }
  reject([&] { static_cast<void>(device.read()); }, "exceeds the supplied");
}

void phases_and_unknown_data() {
  Nand device;
  reject([&] { device.command(read_id); }, "unqualified readiness");
  device.observe_power_ready();
  device.command(read_id);
  reject([&] { static_cast<void>(device.read()); }, "no completed page transfer");
  for (unsigned address = 1; address <= all_bits; ++address) {
    reject([&] { device.address(static_cast<std::uint8_t>(address)); }, "single address 00h");
  }
  device.address(0);
  reject([&] { device.address(0); }, "no qualified read command/address phase");
  reject([&] { static_cast<void>(device.read()); }, "unknown bits byte=0");
  reject([&] { static_cast<void>(device.read(0)); }, "empty read mask");
  require(device.ready() == true && device.pending() == NandPending::none && device.page() == 0 &&
              device.column() == 0,
          "ID needs no fabricated read timer, array address or busy transition");
  reject([&] { device.observe_identification(identity()); }, "cold device configuration");
  device.power_loss();
  device.observe_identification(identity(false));
  device.observe_power_ready();
  select(device);
  for (const auto value : documented_prefix) {
    require(device.read() == value, "The explicitly supplied four-byte prefix is readable");
  }
  reject([&] { static_cast<void>(device.read()); }, "unknown bits byte=4");
  reject([&] { static_cast<void>(device.read()); }, "unknown bits byte=4");
}

void masks_and_nonadvancing_failures() {
  for (std::size_t position = 0; position < own_identity.size(); ++position) {
    for (unsigned mask = 1; mask <= all_bits; ++mask) {
      Nand device;
      auto observed = identity();
      observed.bytes[position] = {.value = static_cast<std::uint8_t>(own_identity[position] & mask),
                                  .known = static_cast<std::uint8_t>(mask)};
      device.observe_identification(observed);
      device.observe_power_ready();
      select(device);
      for (std::size_t index = 0; index < position; ++index) {
        require(device.read() == own_identity[index], "Known prefix precedes partial ID");
      }
      const auto saved = device;
      if (mask != all_bits) {
        reject([&] { static_cast<void>(device.read()); }, "unknown bits byte=");
      }
      reject([&] { static_cast<void>(device.read(0)); }, "empty read mask");
      const auto expected = own_identity[position] & mask;
      require(device.read(static_cast<std::uint8_t>(mask)) == expected,
              "A refused read cannot consume the partial ID byte");
      compare(device, position + 1);
      auto restored = saved;
      require(restored.read(static_cast<std::uint8_t>(mask)) == expected,
              "Trusted partial-read copy retains ID knowledge and cursor");
      compare(restored, position + 1);
    }
  }
}

void immutable_configuration() {
  Nand device;
  auto first = identity();
  first.bytes[0] = {.value = 0x0c, .known = 0x0f};
  device.observe_identification(first);
  NandIdentification more;
  constexpr std::uint8_t high_nibble = 0xf0;
  constexpr std::uint8_t observed_high = 0xe0;
  more.bytes[0] = {.value = observed_high, .known = high_nibble};
  device.observe_identification(more);
  auto conflict = identity();
  conflict.bytes[0].value ^= 1;
  reject([&] { device.observe_identification(conflict); }, "conflicting immutable byte=0");
  device.observe_identification({});
  device.observe_power_ready();
  select(device);
  compare(device);
  Nand atomic;
  auto bad = identity();
  bad.bytes.back() = {.value = 1, .known = 0};
  reject([&] { atomic.observe_identification(bad); }, "noncanonical byte=4");
  atomic.observe_power_ready();
  select(atomic);
  reject([&] { static_cast<void>(atomic.read()); }, "unknown bits byte=0");
}

void resets_and_array_isolation() {
  constexpr unsigned row = 37;
  constexpr unsigned column = 17;
  constexpr NandByte cell{.value = 0x42, .known = all_bits};
  for (unsigned cut = 0; cut <= own_identity.size() + 1; ++cut) {
    Nand device;
    device.observe_cell({.page = row, .column = column}, cell);
    device.observe_identification(identity());
    device.observe_wp(true);
    device.observe_power_ready();
    device.command(read_id);
    if (cut != 0) {
      device.address(0);
      for (unsigned index = 0; index + 1 < cut; ++index) {
        require(device.read() == own_identity[index], "Literal pre-reset ID prefix");
      }
    }
    const auto partial = device;
    device.command(reset);
    require(device.pending() == NandPending::reset && device.ready() == false,
            "Reset still requires actual completion and aborts ID phase");
    reject([&] { device.command(read_id); }, "unqualified readiness");
    device.observe_completion();
    select(device);
    compare(device);
    device.command(read_status);
    require(device.read(ready_wp) == ready_wp, "ID does not fabricate or destroy ready/WP");
    select(device);
    require(device.read() == own_identity[0], "New 90h restarts after real status command");
    device = partial;
    device.power_loss();
    require(!device.ready().has_value(), "Power loss invalidates readiness");
    device.observe_identification(identity());
    device.observe_power_ready();
    select(device);
    compare(device);
    require(device.cell({.page = row, .column = column}) == cell &&
                device.cell({.page = row, .column = column + 1}) == NandByte{},
            "ID/reset/copies retain observed cells and neighboring unknownness");
    device.observe_wp(true);
    reject([&] { device.command(0x80); }, "pointer");
    device.command(0);
    device.address(0);
    device.address(row);
    device.address(0);
    device.address(0);
    device.observe_completion();
    reject([&] { static_cast<void>(device.read(1)); }, "unknown contents");
    require(device.column() == 0, "Refused array data cannot advance its cursor");
    device.command(0);
    device.address(column);
    device.address(row);
    device.address(0);
    device.address(0);
    device.observe_completion();
    require(device.read() == cell.value, "ID traffic preserves the actual observed array byte");
  }
}

void routed_owners() {
  constexpr unsigned timebase_hz = 1000000000;
  NandOperationClock clock(Nand{}, timebase_hz, NandOperationContract::reference_maximum);
  clock.observe_identification(identity());
  clock.observe_power_ready();
  select(clock);
  for (const auto value : own_identity) {
    require(clock.read() == value && !clock.next_event() && clock.ticks() == 0,
            "ID does not invent an array timer or elapsed time");
  }
  const test::PersistenceDirectory directory;
  const auto path = directory.file();
  std::vector<std::uint8_t> before;
  {
    NandStore store(path, NandImage{});
    before = encode_nand_image(load_nand_image(path));
    const auto timestamp = std::filesystem::last_write_time(path);
    unsigned publications = 0;
    store.set_observer([&](NandFilePhase) { ++publications; });
    store.observe_identification(identity());
    store.observe_power_ready();
    select(store);
    compare(store);
    require(publications == 0 && timestamp == std::filesystem::last_write_time(path) &&
                encode_nand_image(load_nand_image(path)) == before,
            "ID observations/traffic must not publish a cell-image transaction");
  }
  NandStore reopened(path);
  reopened.observe_power_ready();
  select(reopened);
  reject([&] { static_cast<void>(reopened.read()); }, "unknown bits byte=0");
  require(encode_nand_image(load_nand_image(path)) == before,
          "Cell-file reload must not manufacture an ID observation");
}
}

int main() {
  try {
    phases_and_unknown_data();
    masks_and_nonadvancing_failures();
    immutable_configuration();
    resets_and_array_isolation();
    routed_owners();
    std::cout << "NAND ID: 1275 masked-byte cases, 255 address refusals, reset/copy/owner checks\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
