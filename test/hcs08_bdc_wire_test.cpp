// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_bdc_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_wire_fixture.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <sstream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_wire;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr unsigned command_delay = 16, timeout = 512, sync_low = 128, sync_delay = 16;
constexpr unsigned byte_values = 256, byte_mask = 255, target_address = 0x200;
constexpr unsigned status_opcode = 0xe4, write_opcode = 0xc0, read_opcode = 0xe0;

constexpr unsigned active_status = 0xc8;

void all_bytes() {
  Target target;
  Hcs08Bdc bdc(target.cpu, target.mapping, chip, Hcs08BdcReset::bkgd_held_low);
  Hcs08BdcWire wire(bdc, wire_contract, high, wire_clock);
  for (unsigned value = 0; value < byte_values; ++value) {
    for (const auto byte : {write_opcode, 2U, 0U, value}) {
      send(wire, static_cast<std::uint8_t>(byte));
    }
    require(bdc.state().serviced == static_cast<std::uint64_t>(value) * 2,
            "No WRITE_BYTE side effect occurs before the separate command delay");
    clocks(wire, command_delay);
    require(target.memory.known[target_address] && target.memory.bytes[target_address] == value,
            "Every independently supplied byte reaches the actual target mapping");
    const auto distinct = static_cast<std::uint8_t>(value ^ byte_mask);
    target.memory.install(target_address + 1, std::span(&distinct, 1));
    for (const auto byte : {read_opcode, 2U, 1U}) { send(wire, static_cast<std::uint8_t>(byte)); }
    clocks(wire, command_delay);
    wire.host_drive(released, {.pc = algorithm});
    require(receive(wire) == distinct, "Target reply is independently seeded, not a round trip");
    wire.host_drive(high, {.pc = algorithm});
  }
  require(wire.state().received_bytes == static_cast<std::uint64_t>(byte_values) * 7 &&
              wire.state().transmitted_bytes == byte_values &&
              bdc.state().serviced == static_cast<std::uint64_t>(byte_values) * 2,
          "All byte commands and real replies retain exact framing counts");
}

void sync_and_timeout() {
  Target target;
  Hcs08Bdc bdc(target.cpu, target.mapping, chip, Hcs08BdcReset::bkgd_held_low);
  Hcs08BdcWire wire(bdc, wire_contract, high, wire_clock);
  send(wire, write_opcode);
  send(wire, 2);
  wire.host_drive(low, {.pc = algorithm});
  clocks(wire, sync_low);
  require(wire.state().sync == Hcs08BdcSync::wait_release && bdc.state().received == 0 &&
              !target.memory.known[target_address],
          "A real long-low SYNC request aborts incomplete frames without target writes");
  wire.host_drive(high, {.pc = algorithm});
  wire.host_drive(released, {.pc = algorithm});
  clocks(wire, sync_delay);
  require(wire.level(), "The target leaves all sixteen complete synchronization-delay cycles idle");
  wire.clock();
  require(!wire.level(), "The delayed target SYNC pulse starts on its own clock edge");
  clocks(wire, sync_low - 1);
  require(!wire.level(), "The last of128 target SYNC low cycles remains low");
  wire.clock();
  require(wire.level() && wire.target_drive() == high,
          "SYNC ends in one active-high speedup cycle");
  wire.clock();
  require(wire.level() && wire.target_drive() == released && wire.state().syncs == 1,
          "Target releases after the exact SYNC speedup pulse");
  wire.host_drive(high, {.pc = algorithm});
  send(wire, write_opcode);
  send(wire, 2);
  clocks(wire, timeout);
  require(bdc.state().received == 0 && wire.state().timeouts == 1 &&
              !target.memory.known[target_address] && bdc.status() == active_status,
          "Documented communication timeout aborts framing without resetting mode or memory");
}

void active_resynchronization() {
  // Own host stimulus exceeds128*64 periods; this is no silicon detector threshold.
  constexpr unsigned conservative_host_low = 16384;
  constexpr unsigned response_end = 145;
  Target target;
  const auto memory = target.memory;
  const auto cpu = target.cpu.state();
  Hcs08Bdc bdc(target.cpu, target.mapping, chip, Hcs08BdcReset::bkgd_held_low);
  Hcs08BdcWire wire(bdc, wire_contract, high, wire_clock);
  for (unsigned pass = 0; pass < 2; ++pass) {
    wire.host_drive(low, {.pc = algorithm});
    clocks(wire, conservative_host_low);
    require(wire.state().sync == Hcs08BdcSync::wait_release && bdc.status() == active_status,
            "Conservatively long host SYNC remains admissible in already active background mode");
    wire.host_drive(high, {.pc = algorithm});
    for (unsigned age = 0; age <= response_end; ++age) {
      wire.clock();
      auto expected_drive = released;
      if (age >= sync_delay && age < sync_delay + sync_low) {
        expected_drive = low;
      } else if (age == sync_delay + sync_low) {
        expected_drive = high;
      }
      require(wire.target_drive() == expected_drive && wire.level() == (expected_drive != low),
              "Independent16/128/one response shape survives a second active-mode SYNC");
      if (age == 0) { wire.host_drive(released, {.pc = algorithm}); }
    }
    require(wire.state().syncs == pass + 1 && wire.state().sync == Hcs08BdcSync::idle,
            "The response completes before a subsequent real command");
    wire.host_drive(high, {.pc = algorithm});
    send(wire, hcs08_bdc::background);
    clocks(wire, command_delay);
    require(bdc.state().serviced == pass + 1 && bdc.status() == active_status &&
                target.cpu.state() == cpu && target.memory.retired == 0 &&
                target.memory.bytes == memory.bytes && target.memory.known == memory.known &&
                target.memory.flash.image() == memory.flash.image() &&
                target.memory.flash.state() == memory.flash.state(),
            "Actual BACKGROUND after either SYNC retains held native CPU and untouched NVM/RAM");
  }
}

void strict_failures() {
  for (unsigned failure = 0; failure < 3; ++failure) {
    Target target;
    Hcs08Bdc bdc(target.cpu, target.mapping, chip, Hcs08BdcReset::bkgd_held_low);
    Hcs08BdcWire wire(bdc, wire_contract, high, wire_clock);
    const auto saved = wire.checkpoint();
    switch (failure) {
      case 0:
        require(rejected([&] {
                  wire.host_drive(Hcs08BdcDrive::unknown, {.pc = algorithm});
                }).contains("unknown or invalid host drive"),
                "Unknown drive");
        break;
      case 1:
        send(wire, status_opcode);
        wire.host_drive(released, {.pc = algorithm});
        wire.host_drive(low, {.pc = algorithm});
        clocks(wire, 7);
        require(rejected([&] { wire.clock(); }).contains("opposing active"),
                "A host cannot hold low across the target's one-bit speedup pulse");
        break;
      case 2:
        wire.host_drive(low, {.pc = algorithm});
        wire.clock();
        wire.host_drive(high, {.pc = algorithm});
        require(
            rejected([&] { wire.host_drive(low, {.pc = algorithm}); }).contains("preceding bit"),
            "Early overlapping host bits cannot silently replace each other");
        break;
      default: throw Fault("Invalid own wire failure case");
    }
    require(wire.faulted() && rejected([&] {
                                static_cast<void>(wire.checkpoint());
                              }).contains("coordinated recovery"),
            "Wire failures cannot become trusted replay checkpoints");
    if (failure == 0 || failure == 2) {
      wire.restore_checkpoint(saved);
      require(!wire.faulted() && wire.level(),
              "Explicit wire-only restore after no target effects");
    }
  }
}

unsigned suffix(Hcs08BdcWire &wire, std::ostream &trace) {
  const auto tick = [&] {
    wire.clock();
    const auto before = wire.state();
    const auto level = wire.level();
    require(before == wire.state(), "Passive line observation cannot change serial time/state");
    trace << wire.state().cycles << ' ' << level << ' '
          << static_cast<unsigned>(wire.target_drive()) << '\n';
  };
  for (unsigned n = 0; n < 7; ++n) { tick(); }
  unsigned value = wire.level() ? 1U : 0U;
  for (unsigned n = 0; n < 6; ++n) { tick(); }
  for (unsigned bit = 0; bit < 7; ++bit) {
    wire.host_drive(low, {.pc = algorithm});
    for (unsigned n = 0; n < 3; ++n) { tick(); }
    wire.host_drive(released, {.pc = algorithm});
    for (unsigned n = 0; n < 8; ++n) { tick(); }
    value = (value << 1U) | (wire.level() ? 1U : 0U);
    for (unsigned n = 0; n < 6; ++n) { tick(); }
  }
  return value;
}

void reply_replay() {
  Target target;
  constexpr std::array<std::uint8_t, 1> data{0xa5};
  target.memory.install(target_address, data);
  Hcs08Bdc bdc(target.cpu, target.mapping, chip, Hcs08BdcReset::bkgd_held_low);
  Hcs08BdcWire wire(bdc, wire_contract, high, wire_clock);
  send(wire, read_opcode);
  send(wire, 2);
  send(wire, 0);
  clocks(wire, command_delay);
  wire.host_drive(low, {.pc = algorithm});
  clocks(wire, 4);
  wire.host_drive(released, {.pc = algorithm});
  const auto memory = target.memory;
  const auto cpu = target.cpu.checkpoint();
  const auto controller = bdc.checkpoint();
  const auto serial = wire.checkpoint();
  std::ostringstream expected;
  require(suffix(wire, expected) == data[0], "Independent held response byte across a mid-bit cut");
  for (unsigned replay = 0; replay < 2; ++replay) {
    auto restored = memory;
    Hcs08 core(restored, BLOFELD_HCS08_SLA);
    core.restore_checkpoint(cpu);
    blofeld::test::hcs08_bdc::Memory mapping(restored);
    Hcs08Bdc other(core, mapping, chip, Hcs08BdcReset::normal);
    other.restore_checkpoint(controller);
    Hcs08BdcWire fresh(other, wire_contract, high, wire_clock);
    fresh.restore_checkpoint(serial);
    std::ostringstream actual;
    require(suffix(fresh, actual) == data[0] && actual.str() == expected.str() &&
                fresh.state() == wire.state() && other.state() == bdc.state() &&
                core.state() == target.cpu.state() && restored.bytes == target.memory.bytes &&
                restored.flash.image() == target.memory.flash.image(),
            "Fresh CPU/memory/BDC/wire owners reproduce every clock/level/drive and endpoint");
  }
}

void asynchronous_boundary() {
  constexpr unsigned too_early = 257;
  constexpr unsigned exact = 258;
  constexpr unsigned later = 260;
  constexpr unsigned first = 2;
  constexpr unsigned release = 50;
  constexpr Hcs08BdcWireClock timing{.period_ticks = 16, .first_edge_ticks = 1};
  for (const auto next : {too_early, exact, later}) {
    Target target;
    Hcs08Bdc bdc(target.cpu, target.mapping, chip, Hcs08BdcReset::bkgd_held_low);
    constexpr std::array<std::uint8_t, 1> status{0xe4};
    blofeld::test::hcs08_bdc::command(bdc, status);
    Hcs08BdcWire wire(bdc, wire_contract, high, timing);
    wire.clock();
    wire.host_drive(low, {.pc = algorithm}, first);
    clocks(wire, 3);
    wire.host_drive(released, {.pc = algorithm}, release);
    while (wire.state().next_clock_ticks <= next) { wire.clock(); }
    require(
        wire.state().perceived.has_value() && wire.target_drive() == released,
        "Asynchronous host can reach nominal end before the sixteenth perceived-clock interval");
    if (next == too_early) {
      require(rejected([&] {
                wire.host_drive(low, {.pc = algorithm}, next);
              }).contains("preceding bit"),
              "One tick below sixteen actual clock periods still rejects");
    } else {
      wire.host_drive(low, {.pc = algorithm}, next);
      require(!wire.faulted() && wire.state().transmit_bits == 1 &&
                  wire.state().last_fall == next && wire.state().bit_pending,
              "Exact or later sixteen-period host edge completes the sampled bit without an extra "
              "clock");
      const auto skipped = wire.state().next_clock_ticks;
      require(rejected([&] {
                wire.host_drive(high, {.pc = algorithm}, skipped);
              }).contains("unobserved BDC clock"),
              "Host timestamps cannot skip a required target clock edge");
    }
  }
}
} // namespace

int main() {
  try {
    all_bytes();
    sync_and_timeout();
    active_resynchronization();
    strict_failures();
    reply_replay();
    asynchronous_boundary();
    std::cout << "HCS08 BDC serial bytes, independent replies, SYNC/timeout and visible conflicts "
                 "passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
