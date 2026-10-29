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
constexpr std::uint64_t completed = 15000;
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

void idle_pauses_after_end() {
  auto clock = prepared();
  for (unsigned index = 0; index < Nand::page_bytes; ++index) {
    require(clock.read() == expected(index), "Every original cell precedes termination");
  }
  clock.observe_read_end(NandReadEnd::ce_high);
  require(clock.device().quiescent_read_pause(),
          "A terminated ready read has no active protocol work");
  const auto saved = clock;
  for (unsigned replay = 0; replay < 2; ++replay) {
    auto restored = saved;
    for (unsigned pause = 0; pause < 4; ++pause) {
      restored.observe_read_end(NandReadEnd::ce_high);
    }
    require(!restored.faulted() && !restored.next_event() && restored.device().page() == 0 &&
                restored.device().column() == Nand::page_bytes &&
                restored.device().ready() == true &&
                restored.device().pending() == NandPending::none &&
                restored.device().cell({.page = 0, .column = last_column}).value ==
                    expected(last_column),
            "Idle CE pauses retain cells, cursor, readiness and absence of work");
    restored.command(0);
    for (unsigned index = 0; index < 4; ++index) { restored.address(0); }
    require(!restored.device().quiescent_read_pause(),
            "Actual new address request is not a completed read");
    const auto event = restored.advance(2 * completed);
    require(event && event->ticks == 2 * completed && restored.read() == expected(0),
            "Only actual command/address/completion starts a fresh selected page");
  }
}

void quiescent_phase_guards() {
  Nand cold(NandReadContinuation::k9f1208_p);
  require(!cold.quiescent_read_pause(), "Cold NAND cannot claim a completed read");
  cold.observe_power_ready();
  require(!cold.quiescent_read_pause(), "Power Ready alone is not a buffered read");
  const auto ready = prepared();
  require(ready.device().quiescent_read_pause(),
          "Completed buffered data is quiescent without consumption");
  constexpr std::uint8_t status = 0x70;
  constexpr std::uint8_t program = 0x80;
  constexpr std::uint8_t erase = 0x60;
  constexpr std::uint8_t reset = 0xff;
  for (const auto command : {status, program, erase, reset}) {
    auto changed = ready;
    changed.observe_wp(true);
    changed.command(command);
    require(!changed.device().quiescent_read_pause(),
            "Status, staged program/erase and pending reset cannot use read-only CE abstraction");
  }
}

void aborted_idle_pause() {
  auto clock = prepared();
  clock.command(0);
  for (unsigned index = 0; index < 4; ++index) { clock.address(0); }
  require(!clock.device().quiescent_read_pause(), "A pending read cannot use an idle pause");
  clock.observe_read_end(NandReadEnd::ce_high);
  constexpr unsigned abort_duration = 5000;
  const auto event = clock.advance(completed + abort_duration);
  require(event && event->ticks == completed + abort_duration && clock.device().ready() == true &&
              clock.device().quiescent_read_pause(),
          "Completed CE abort is idle and ready, without claiming a valid page");
  clock.observe_read_end(NandReadEnd::ce_high);
  bool refused = false;
  try {
    static_cast<void>(clock.read());
  } catch (const Fault &error) {
    refused = std::string(error.what()).contains("no completed page transfer");
  }
  require(refused && clock.faulted() && clock.device().column() == 0,
          "Repeated idle CE after abort cannot recreate the discarded page or consume a byte");
}

void checks() {
  for (const unsigned count : {0U, 1U, half_end, second_half, last_column}) { pause(count); }
  missing();
  final_byte();
  idle_pauses_after_end();
  quiescent_phase_guards();
  aborted_idle_pause();
  std::cout
      << "protocol-cases=10 pause-copies=12 idle-read-copies=2 independent-columns=0,1,255,256,527 "
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
