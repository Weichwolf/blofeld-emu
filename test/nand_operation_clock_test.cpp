// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand.hpp"
#include "storage/nand_operation_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace {
using namespace blofeld;
constexpr auto contract = NandOperationContract::reference_maximum;
constexpr std::uint32_t ghz = 1000000000;
constexpr std::uint64_t started = 100;
constexpr std::uint8_t reset_command = 0xff, status_command = 0x70, ready_mask = 0x40;
constexpr std::uint8_t program_command = 0x80, program_confirm = 0x10;
constexpr std::uint8_t erase_command = 0x60, erase_confirm = 0xd0;
constexpr std::uint8_t payload = 0xa5;
constexpr unsigned read_ticks = 15000, reset_ticks = 5000, stationary_polls = 100;
constexpr unsigned erase_reset_ticks = 500000;
// Independent p.13 conversions: Hz, 15us read, 5us/10us/500us reset.
constexpr std::array<std::array<unsigned, 5>, 4> vectors{{
    {{1000000000, 15000, 5000, 10000, 500000}},
    {{540000000, 8100, 2700, 5400, 270000}},
    {{1000001, 16, 6, 11, 501}},
    {{3, 1, 1, 1, 1}},
}};

void require(const bool value, const std::string &message) {
  if (!value) { throw Fault(message); }
}

void reject(const auto &operation, const std::string &reason) {
  try {
    operation();
  } catch (const Fault &fault) {
    require(std::string(fault.what()).contains(reason),
            "Wrong fault: " + std::string(fault.what()));
    return;
  }
  throw Fault("Missing NAND clock fault: " + reason);
}

void no_event(NandOperationClock &clock, const std::uint64_t ticks) {
  require(!clock.advance(ticks), "No completion event is due");
}

Nand observed(const bool erased = false,
              const NandReadContinuation profile = NandReadContinuation::unqualified) {
  Nand nand(profile);
  nand.observe_power_ready();
  if (erased) {
    nand.observe_wp(true);
    nand.observe_block(0, {.valid = true, .program_protected = false, .erase_protected = false});
    nand.command(erase_command);
    for (unsigned index = 0; index < 3; ++index) { nand.address(0); }
    nand.command(erase_confirm);
    nand.observe_completion(NandOutcome::pass);
  } else {
    nand.observe_bytes(0, 0, std::array{payload});
  }
  return nand;
}

void page_address(NandOperationClock &clock) {
  for (unsigned index = 0; index < 4; ++index) { clock.address(0); }
}

void begin_read(NandOperationClock &clock, const std::uint64_t ticks = started) {
  no_event(clock, ticks);
  clock.command(0);
  page_address(clock);
  require(clock.device().ready() == false && clock.device().pending() == NandPending::read,
          "Actual final address starts a busy page transfer");
}

void begin_write(NandOperationClock &clock, const bool erase) {
  if (erase) {
    clock.command(erase_command);
    for (unsigned index = 0; index < 3; ++index) { clock.address(0); }
    clock.command(erase_confirm);
  } else {
    clock.command(0);
    clock.command(program_command);
    page_address(clock);
    clock.write(payload);
    clock.command(program_confirm);
  }
  require(!clock.next_event() && clock.device().ready() == false,
          "Program/erase success has no fabricated clock event");
}

void write_busy_ce_pause() {
  for (const auto profile : {NandReadContinuation::k9f1208_p, NandReadContinuation::k9f1208_j}) {
    for (const bool erase : {false, true}) {
      NandOperationClock clock(observed(true, profile), ghz, contract);
      no_event(clock, started);
      begin_write(clock, erase);
      clock.observe_read_end(NandReadEnd::ce_high);
      require(!clock.faulted() && !clock.next_event() && clock.device().ready() == false &&
                  clock.device().pending() == (erase ? NandPending::erase : NandPending::program),
              "CE high during write busy neither cancels the operation nor invents a deadline");
      no_event(clock, started + erase_reset_ticks);
      clock.observe_completion(NandOutcome::pass);
      require(clock.device().ready() == true && clock.device().pending() == NandPending::none,
              "The explicit completion remains necessary after CE deselection");
    }
  }
}

void independent_read_intervals() {
  for (const auto &row : vectors) {
    NandOperationClock clock(observed(), row[0], contract);
    no_event(clock, started);
    clock.command(0);
    for (unsigned index = 0; index < 3; ++index) {
      clock.address(0);
      require(!clock.next_event() && clock.device().ready() == true,
              "Incomplete address does not start an array transfer");
    }
    clock.address(0);
    const NandCompletionEvent expected{.ticks = started + row[1], .operation = NandPending::read};
    require(clock.next_event() == expected, "Independent ceil-to-ticks read deadline");
    clock.address(payload);
    require(clock.next_event() == expected, "Ignored surplus address cannot restart the timer");
    clock.command(status_command);
    for (unsigned poll = 0; poll < stationary_polls; ++poll) {
      require(clock.read(ready_mask) == 0, "Host polling cannot advance NAND time");
    }
    no_event(clock, expected.ticks - 1);
    require(clock.device().ready() == false, "One tick before completion stays busy");
    auto saved = clock;
    require(clock.advance(expected.ticks) == expected &&
                saved.advance(expected.ticks + 1) == expected,
            "Fine/coarse advances and copied owner report the exact independent event tick");
    require(clock.device().ready() == true && clock.read(ready_mask) == ready_mask &&
                !clock.next_event(),
            "Completion changes actual protocol Ready and consumes the event once");
    no_event(clock, expected.ticks);
  }
}

void independent_reset_intervals() {
  for (const auto &row : vectors) {
    for (unsigned mode = 0; mode < 4; ++mode) {
      NandOperationClock clock(observed(true), row[0], contract);
      no_event(clock, started);
      if (mode == 1) { begin_read(clock); }
      if (mode >= 2) { begin_write(clock, mode == 3); }
      clock.command(reset_command);
      const auto duration = row[mode < 2 ? 2 : mode + 1];
      const NandCompletionEvent expected{.ticks = started + duration,
                                         .operation = NandPending::reset};
      require(clock.next_event() == expected, "Reset duration follows the interrupted operation");
      no_event(clock, expected.ticks - 1);
      require(clock.device().ready() == false, "Reset remains busy until its independent deadline");
      require(clock.advance(expected.ticks) == expected && clock.device().ready() == true,
              "Clock completes reset without completing an interrupted write");
      if (mode >= 2) {
        require(clock.device().cell({.page = 0, .column = 0}).known == 0,
                "Reset of active program/erase leaves interrupted contents unknown");
      }
    }
  }
}

void cancellation_and_power() {
  NandOperationClock clock(observed(), ghz, contract);
  begin_read(clock);
  no_event(clock, started + read_ticks - 1);
  clock.command(reset_command);
  const auto reset_end = started + read_ticks - 1 + reset_ticks;
  no_event(clock, started + read_ticks);
  require(clock.device().ready() == false, "Superseded read deadline cannot finish reset");
  require(clock.advance(reset_end) ==
              NandCompletionEvent{.ticks = reset_end, .operation = NandPending::reset},
          "Only the replacement reset event survives");
  begin_read(clock, reset_end);
  clock.power_loss();
  require(!clock.next_event() && !clock.device().ready().has_value(),
          "Power loss cancels the current event and withdraws Ready knowledge");
  no_event(clock, reset_end + read_ticks);
  require(!clock.device().ready().has_value(), "No stale completion can power a device back on");
  require(clock.device().cell({.page = 0, .column = 0}).value == payload,
          "Power loss during a read preserves independently observed nonvolatile data");
  clock.observe_power_ready();
  begin_read(clock, clock.ticks());
  const auto event = clock.next_event();
  if (!event) { throw Fault("Missing fresh read deadline after power recovery"); }
  const auto deadline = event->ticks;
  require(clock.advance(deadline).has_value() && clock.read() == payload,
          "Fresh power readiness and addressing read the retained byte");
}

void observations_and_unknown_contents() {
  for (const bool erase : {false, true}) {
    NandOperationClock clock(observed(true), ghz, contract);
    begin_write(clock, erase);
    no_event(clock, erase_reset_ticks);
    require(clock.device().ready() == false &&
                clock.device().cell({.page = 0, .column = 0}).known == 0,
            "Elapsed time alone supplies neither write completion nor known cells");
    clock.observe_completion(NandOutcome::pass);
    require(clock.device().ready() == true &&
                clock.device().cell({.page = 0, .column = 0}).known == Nand::byte_mask,
            "Explicit external write outcome remains supported");
  }
  Nand unknown;
  NandOperationClock clock(std::move(unknown), ghz, contract);
  clock.observe_power_ready();
  clock.observe_wp(false);
  begin_read(clock);
  require(clock.advance(started + read_ticks).has_value(),
          "Unknown page still finishes its transfer");
  auto saved = clock;
  reject([&] { static_cast<void>(clock.read()); }, "unknown contents");
  require(clock.faulted() && clock.device().column() == 0 &&
              clock.device().cell({.page = 0, .column = 0}).known == 0,
          "Unknown payload fails before byte consumption and poisons the owner");
  reject([&] { no_event(clock, started + read_ticks); }, "coordinated recovery");
  clock = saved;
  require(!clock.faulted() && clock.device().column() == 0,
          "A complete trusted owner checkpoint restores clock and protocol together");
}

void rejected_contracts_and_limits() {
  reject([] { const NandOperationClock bad(Nand{}, ghz, NandOperationContract::unqualified); },
         "explicit maximum-duration");
  reject([] { const NandOperationClock bad(Nand{}, 0, contract); }, "zero reference");
  Nand pending = observed();
  pending.command(0);
  for (unsigned index = 0; index < 4; ++index) { pending.address(0); }
  reject([&] { const NandOperationClock bad(pending, ghz, contract); }, "no start time");
  NandOperationClock cold(Nand{}, ghz, contract);
  reject([&] { cold.command(reset_command); }, "unknown power readiness");
  NandOperationClock reset(observed(), ghz, contract);
  reset.command(reset_command);
  reject([&] { reset.command(reset_command); }, "repeated reset");
  NandOperationClock backwards(observed(), ghz, contract);
  no_event(backwards, started);
  reject([&] { no_event(backwards, started - 1); }, "backwards");
  constexpr auto last_tick = std::numeric_limits<std::uint64_t>::max();
  NandOperationClock exact(observed(), ghz, contract);
  begin_read(exact, last_tick - read_ticks);
  require(exact.advance(last_tick) ==
              NandCompletionEvent{.ticks = last_tick, .operation = NandPending::read},
          "Exact last-counter deadline does not overflow");
  NandOperationClock overflow(observed(), ghz, contract);
  reject([&] { begin_read(overflow, last_tick - read_ticks + 1); },
         "exceeds the reference counter");
  require(overflow.faulted(), "A partially admitted operation cannot run after deadline overflow");
}
}

int main() {
  try {
    independent_read_intervals();
    independent_reset_intervals();
    cancellation_and_power();
    observations_and_unknown_contents();
    write_busy_ce_pause();
    rejected_contracts_and_limits();
    std::cout
        << "NAND operation clock: independent read/reset deadlines, polling invariance, "
           "cancellation, unknown contents, write observations and joint checkpoints verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
