// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_operation_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <utility>

namespace {
using namespace blofeld;
constexpr unsigned timebase = 1000000000;
constexpr unsigned completed = 15000;
constexpr unsigned half_end = 255;
constexpr unsigned second_half = 256;
constexpr unsigned last_column = 527;
constexpr unsigned byte_mask = 255;
constexpr unsigned byte_bias = 165;
constexpr unsigned own_byte = 0x96;

void require(const bool value, const std::string &message) {
  if (!value) { throw Fault(message); }
}

std::uint8_t expected(const unsigned column) {
  return static_cast<std::uint8_t>((column ^ byte_bias) & byte_mask);
}

NandOperationClock prepared(const bool missing = false) {
  Nand device(NandReadContinuation::k9f1208_p);
  if (missing) {
    device.observe_cell({.page = 0, .column = 0}, {.value = own_byte, .known = byte_mask});
  } else {
    std::array<std::uint8_t, Nand::page_bytes> bytes{};
    for (unsigned index = 0; index < bytes.size(); ++index) { bytes[index] = expected(index); }
    device.observe_bytes(0, 0, bytes);
  }
  device.observe_power_ready();
  NandOperationClock clock(std::move(device), timebase, NandOperationContract::reference_maximum);
  clock.command(0);
  for (unsigned index = 0; index < 4; ++index) { clock.address(0); }
  const auto event = clock.advance(completed);
  require(event && event->ticks == completed && event->operation == NandPending::read,
          "Real selected address completion occurs at the independent15us reference bound");
  return clock;
}

void pause(const unsigned count) {
  auto clock = prepared();
  for (unsigned index = 0; index < count; ++index) {
    require(clock.read() == expected(index), "Every actual preceding cell follows its own address");
  }
  const auto saved = clock;
  for (unsigned replay = 0; replay < 2; ++replay) {
    auto restored = saved;
    restored.observe_read_end(NandReadEnd::ce_high);
    require(!restored.faulted() && restored.ticks() == completed && !restored.next_event() &&
                restored.device().page() == 0 && restored.device().column() == count &&
                restored.device().ready() == true &&
                restored.device().pending() == NandPending::none,
            "Mid-page CE does not consume a byte, increment a row or invent an array operation");
    require(restored.read() == expected(count) && restored.device().column() == count + 1,
            "A following selected read continues at the exact next buffered column");
    require(restored.device().cell({.page = 0, .column = count}).value == expected(count),
            "Read pause and continuation preserve the original observed cell");
  }
}

void missing() {
  auto clock = prepared(true);
  require(clock.read() == own_byte, "Exactly one observed cell exists");
  clock.observe_read_end(NandReadEnd::ce_high);
  bool rejected = false;
  try {
    static_cast<void>(clock.read());
  } catch (const Fault &error) {
    rejected = std::string(error.what()).contains("unknown contents");
  }
  require(rejected && clock.faulted() && clock.device().column() == 1 &&
              clock.device().cell({.page = 0, .column = 1}).known == 0,
          "CE pause never creates unknown data; selected missing data still faults before advance");
}

void final_byte() {
  auto clock = prepared();
  for (unsigned index = 0; index < Nand::page_bytes; ++index) {
    require(clock.read() == expected(index), "The full528-byte reference page is unchanged");
  }
  clock.observe_read_end(NandReadEnd::ce_high);
  require(!clock.next_event() && clock.device().column() == Nand::page_bytes &&
              clock.device().page() == 0 && clock.device().ready() == true,
          "At the final-byte boundary CE still terminates sequential continuation");
  bool rejected = false;
  try {
    static_cast<void>(clock.read());
  } catch (const Fault &error) {
    rejected = std::string(error.what()).contains("no completed page transfer");
  }
  require(rejected && clock.faulted() && clock.device().page() == 0,
          "End termination never invents the next page or permits a data read from idle");
}

void checks() {
  for (const unsigned count : {0U, 1U, half_end, second_half, last_column}) { pause(count); }
  missing();
  final_byte();
  std::cout << "protocol-cases=7 pause-copies=10 independent-columns=0,1,255,256,527 "
               "missing-data=strict final-page=strict\n";
}
}

int main() {
  try {
    checks();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
