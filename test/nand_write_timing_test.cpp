// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/nand_write_timing.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

namespace {
using blofeld::NandWriteKind;
using blofeld::NandWriteLatch;
using blofeld::NandWritePins;
using blofeld::NandWriteTiming;
constexpr unsigned ghz = 1000000000;
constexpr unsigned clearance = 100;
constexpr unsigned first_byte = 0x96;
constexpr unsigned second_byte = 0x29;
constexpr unsigned third_byte = 0x42;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned minimum_pulse = 21;
constexpr unsigned minimum_high = 15;
constexpr unsigned minimum_cycle = 42;
constexpr unsigned relaxed_pulse = 30;
constexpr unsigned second_pulse = 200;
// Independent primary-table conversion vectors: Hz, CS, CLS/ALS/WP, DS, hold, WH, WC.
constexpr std::array<std::array<unsigned, 7>, 3> vectors{{
    {{1000000000, 31, 21, 20, 5, 15, 42}},
    {{1080000000, 34, 23, 22, 6, 17, 46}},
    {{216000000, 7, 5, 5, 2, 4, 10}},
}};

void require(const bool value, const std::string &message) {
  if (!value) { throw blofeld::Fault(message); }
}

void reject(const auto &action, const std::string &reason) {
  try {
    action();
  } catch (const blofeld::Fault &fault) {
    require(std::string(fault.what()).contains(reason),
            "Wrong write timing fault: " + std::string(fault.what()));
    return;
  }
  throw blofeld::Fault("Missing NAND write timing failure: " + reason);
}

NandWritePins pins(const NandWriteKind kind = NandWriteKind::command) {
  return {.ce_n = false,
          .re_n = true,
          .we_n = true,
          .cle = kind == NandWriteKind::command,
          .ale = kind == NandWriteKind::address,
          .data = {.value = first_byte, .known = byte_mask}};
}

void observe(NandWriteTiming &model, const std::uint64_t tick, const NandWritePins value) {
  require(!model.observe(tick, value), "No latch is permitted outside rising WE");
}

void change_signal(NandWritePins &value, const unsigned field, const bool after_latch) {
  switch (field) {
    case 0: value.ce_n = after_latch; break;
    case 1: value.cle = after_latch; break;
    case 2: value.ale = !after_latch; break;
    case 3: value.data.value = after_latch ? third_byte : second_byte; break;
    default: throw blofeld::Fault("Invalid independent signal index");
  }
}

void setup_and_hold(const std::array<unsigned, 7> &row, const unsigned field) {
  constexpr std::array<const char *, 4> setups{"tCS", "tCLS", "tALS", "tDS"};
  constexpr std::array<const char *, 4> holds{"tCH", "tCLH", "tALH", "tDH"};
  constexpr std::array kinds{
      NandWriteKind::command, NandWriteKind::data, NandWriteKind::address, NandWriteKind::command};
  constexpr std::array<unsigned, 4> columns{1, 2, 2, 3};
  NandWriteTiming model(row[0]);
  auto value = pins();
  if (field == 0) { value.ce_n = true; }
  if (field == 2) { value.cle = false; }
  observe(model, 0, value);
  if (field == 0) {
    value.ce_n = false;
    observe(model, clearance, value);
  }
  value.we_n = false;
  observe(model, clearance, value);
  const auto changed_at = clearance + (field == 0 ? 0U : 10U);
  change_signal(value, field, false);
  observe(model, changed_at, value);
  const auto edge = changed_at + row[columns[field]];
  value.we_n = true;
  reject([&] { static_cast<void>(model.observe(edge - 1, value)); }, setups[field]);
  require(model.observe(edge, value) ==
              NandWriteLatch{.kind = kinds[field], .value = value.data.value},
          "Exact independent setup boundary permits precisely the selected latch");
  change_signal(value, field, true);
  reject([&] { observe(model, edge + row[4] - 1, value); }, holds[field]);
  observe(model, edge + row[4], value);
}

void setup_and_hold_boundaries() {
  for (const auto &row : vectors) {
    for (unsigned field = 0; field < 4; ++field) { setup_and_hold(row, field); }
  }
}

void pulse_cycle_and_kinds() {
  for (const auto kind : {NandWriteKind::command, NandWriteKind::address, NandWriteKind::data}) {
    NandWriteTiming model(ghz);
    auto value = pins(kind);
    observe(model, 0, value);
    value.we_n = false;
    observe(model, clearance, value);
    value.we_n = true;
    reject([&] { static_cast<void>(model.observe(clearance + minimum_pulse - 1, value)); }, "tWP");
    require(model.observe(clearance + minimum_pulse, value) ==
                NandWriteLatch{.kind = kind, .value = first_byte},
            "All three latch classes use the same independent minimum pulse boundary");
    value.we_n = false;
    reject([&] { observe(model, clearance + minimum_pulse + minimum_high - 1, value); }, "tWH");
    reject([&] { observe(model, clearance + minimum_cycle - 1, value); }, "tWC");
    observe(model, clearance + minimum_cycle, value);
    value.we_n = true;
    require(model.observe(clearance + minimum_cycle + minimum_pulse, value).has_value(),
            "Exact minimum repeated write cycle is valid");
  }
}

void rounded_pulses_and_unknown_controls() {
  for (const auto &row : vectors) {
    NandWriteTiming model(row[0]);
    auto value = pins();
    observe(model, 0, value);
    value.we_n = false;
    observe(model, clearance, value);
    value.we_n = true;
    reject([&] { static_cast<void>(model.observe(clearance + row[2] - 1, value)); }, "tWP");
    require(model.observe(clearance + row[2], value).has_value(), "Rounded pulse boundary");
    value.we_n = false;
    reject([&] { observe(model, clearance + row[2] + row[5] - 1, value); }, "tWH");
    reject([&] { observe(model, clearance + row[6] - 1, value); }, "tWC");
    observe(model, clearance + row[6], value);
  }
  for (const auto member : {&NandWritePins::ce_n,
                            &NandWritePins::re_n,
                            &NandWritePins::we_n,
                            &NandWritePins::cle,
                            &NandWritePins::ale}) {
    NandWriteTiming model(ghz);
    auto value = pins();
    (value.*member).reset();
    reject([&] { observe(model, 0, value); }, "unknown control");
  }
  NandWriteTiming model(ghz);
  auto value = pins();
  observe(model, 0, value);
  value.re_n = false;
  observe(model, clearance, value);
  value.re_n = true;
  value.we_n = false;
  reject([&] { observe(model, second_pulse, value); }, "unordered simultaneous");
}

void strict_inputs_and_snapshots() {
  reject([] { const NandWriteTiming bad(0); }, "zero");
  NandWriteTiming model(ghz);
  auto value = pins();
  value.we_n = false;
  reject([&] { observe(model, 0, value); }, "initial WE-high");
  value.we_n = true;
  value.ce_n.reset();
  reject([&] { observe(model, 0, value); }, "unknown control");
  value = pins();
  value.data.known = 0;
  reject([&] { observe(model, 0, value); }, "noncanonical");
  value = pins();
  observe(model, 0, value);
  value.we_n = false;
  observe(model, clearance, value);
  auto copy = model;
  value.data = {.value = 0x80, .known = 0x80};
  observe(model, clearance + 1, value);
  value.we_n = true;
  reject([&] { static_cast<void>(model.observe(clearance + relaxed_pulse, value)); },
         "eight known");
  auto complete = pins();
  require(copy.observe(clearance + relaxed_pulse, complete) ==
              NandWriteLatch{.kind = NandWriteKind::command, .value = first_byte},
          "Independent copy retains pending low pulse and data knowledge");
  reject([&] { observe(copy, clearance + relaxed_pulse - 1, complete); }, "backwards");
  value = pins();
  value.we_n = false;
  observe(copy, second_pulse, value);
  value.re_n = false;
  reject([&] { observe(copy, second_pulse + 1, value); }, "RE asserted during");
  value.re_n = true;
  value.cle = false;
  observe(copy, second_pulse + 1, value);
  value.cle = true;
  reject([&] { observe(copy, second_pulse + 2, value); }, "twice");
  NandWriteTiming overflow(ghz);
  observe(overflow, std::numeric_limits<std::uint64_t>::max(), pins());
  value = pins();
  value.we_n = false;
  reject([&] { observe(overflow, std::numeric_limits<std::uint64_t>::max(), value); }, "overflow");
}
}

int main() {
  try {
    setup_and_hold_boundaries();
    pulse_cycle_and_kinds();
    rounded_pulses_and_unknown_controls();
    strict_inputs_and_snapshots();
    std::cout << "NAND input setup/hold, pulse/cycle, latch classes and copied pending writes "
                 "verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
