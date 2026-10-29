// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_operation_clock.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

namespace {
using namespace blofeld;
constexpr auto profile = NandReadContinuation::k9f1208_p;
constexpr auto contract = NandOperationContract::reference_maximum;
constexpr unsigned ghz = 1000000000;
constexpr std::uint64_t read_ticks = 15000;
constexpr unsigned last_column = 527;
constexpr unsigned page_bytes = 528;
constexpr unsigned next_block_row = 32;
constexpr unsigned last_page = 131071;
constexpr std::uint8_t spare = 0x50;
constexpr std::uint8_t final_spare_column = 15;
constexpr std::uint8_t ordinary = 0x31, final_byte = 0xa5, following = 0x3c;
constexpr std::uint8_t status_command = 0x70, ready_mask = 0x40, reset_command = 0xff;
constexpr unsigned byte_mask = 0xff;
enum class NextContents : std::uint8_t { known, missing };

struct Selection {
  unsigned row = 0;
  std::uint8_t command = spare;
  std::uint8_t column = final_spare_column;
};

void require(const bool value, const std::string &reason) {
  if (!value) { throw Fault(reason); }
}

void reject(const auto &operation, const std::string &reason) {
  try {
    operation();
  } catch (const Fault &fault) {
    require(std::string(fault.what()).contains(reason),
            "Wrong NAND end fault: " + std::string(fault.what()));
    return;
  }
  throw Fault("Missing NAND end fault: " + reason);
}

std::uint64_t read_deadline(const NandOperationClock &clock) {
  const auto event = clock.next_event();
  if (!event || event->operation != NandPending::read) {
    throw Fault("Expected an explicitly scheduled NAND read event");
  }
  return event->ticks;
}

void no_event(NandOperationClock &clock, const std::uint64_t ticks) {
  require(!clock.advance(ticks), "No read-end event is due at this observation");
}

NandOperationClock make_clock(const Selection selection = {},
                              const unsigned hz = ghz,
                              const NextContents next = NextContents::known) {
  Nand device(profile);
  std::array<std::uint8_t, page_bytes> page{};
  page.fill(ordinary);
  page.back() = final_byte;
  device.observe_bytes(selection.row, 0, page);
  if (next == NextContents::known && selection.row < last_page) {
    device.observe_cell({.page = selection.row + 1, .column = 0},
                        {.value = following, .known = byte_mask});
    device.observe_cell({.page = selection.row + 1, .column = 512},
                        {.value = following, .known = byte_mask});
  }
  device.observe_power_ready();
  NandOperationClock clock(std::move(device), hz, contract);
  clock.command(selection.command);
  clock.address(selection.column);
  clock.address(static_cast<std::uint8_t>(selection.row & byte_mask));
  clock.address(static_cast<std::uint8_t>((selection.row >> 8) & byte_mask));
  clock.address(static_cast<std::uint8_t>(selection.row >> 16));
  return clock;
}

struct PageCase {
  Selection select;
  unsigned first;
  unsigned count;
  unsigned next_row;
  unsigned next_column;
  std::uint64_t released;
  bool boundary = false;
};

// Independent Samsung pp8,26,29 geometry and literal 1GHz timelines.
constexpr std::array<PageCase, 7> page_cases{{
    {.select = {.row = 0, .command = 0, .column = 0},
     .first = 0,
     .count = 528,
     .next_row = 1,
     .next_column = 0,
     .released = 67850},
    {.select = {.row = 30, .command = 1, .column = 255},
     .first = 511,
     .count = 17,
     .next_row = 31,
     .next_column = 0,
     .released = 16750},
    {.select = {.row = 31, .command = 0x50, .column = 15},
     .first = 527,
     .count = 1,
     .next_row = 31,
     .next_column = 528,
     .released = 15150,
     .boundary = true},
    {.select = {.row = 32, .command = 0x50, .column = 0},
     .first = 512,
     .count = 16,
     .next_row = 33,
     .next_column = 512,
     .released = 16650},
    {.select = {.row = 256, .command = 1, .column = 0},
     .first = 256,
     .count = 272,
     .next_row = 257,
     .next_column = 0,
     .released = 42250},
    {.select = {.row = 131070, .command = 1, .column = 255},
     .first = 511,
     .count = 17,
     .next_row = 131071,
     .next_column = 0,
     .released = 16750},
    {.select = {.row = 131071, .command = 1, .column = 255},
     .first = 511,
     .count = 17,
     .next_row = 131071,
     .next_column = 528,
     .released = 16750,
     .boundary = true},
}};

void page_geometry_and_release() {
  constexpr unsigned cycle = 100;
  constexpr unsigned first_read = 15100;
  constexpr unsigned half_cycle = 50;
  for (const auto &row : page_cases) {
    auto clock = make_clock(row.select);
    require(clock.advance(read_ticks).has_value(), "Initial addressed transfer completes");
    require(clock.device().page() == row.select.row && clock.device().column() == row.first,
            "Independent Read1/Read2 column and 17-bit page address; row=" +
                std::to_string(row.select.row) +
                " actual-column=" + std::to_string(clock.device().column()));
    for (unsigned index = 0; index < row.count; ++index) {
      const auto read_at = first_read + (static_cast<std::uint64_t>(index) * cycle);
      no_event(clock, read_at);
      const auto expected = index + 1 == row.count ? final_byte : ordinary;
      require(clock.read() == expected && clock.device().column() == row.first + index + 1,
              "Every selected byte advances exactly one column");
      no_event(clock, read_at + half_cycle);
      require(clock.device().ready() == true && !clock.next_event(),
              "Last data consumption alone cannot start next-row busy time");
      clock.observe_read_end(NandReadEnd::re_high);
      if (index + 1 < row.count) {
        require(clock.device().ready() == true && !clock.next_event(),
                "Ordinary byte RE release cannot start another array read");
      }
    }
    require(clock.ticks() == row.released && clock.device().page() == row.next_row &&
                clock.device().column() == row.next_column,
            "Only final RE high selects the independently expected next row/area");
    if (row.boundary) {
      require(clock.device().ready() == true && !clock.next_event(),
              "No sequential event may wrap into another block or row zero");
      auto rejected = clock;
      reject([&] { rejected.command(0); }, "block boundary requires CE-high");
      require(rejected.faulted(), "Unterminated block command poisons its clock owner");
      clock.observe_read_end(NandReadEnd::ce_high);
      clock.command(0);
      clock.address(0);
      clock.address(next_block_row);
      clock.address(0);
      clock.address(0);
      require(clock.device().page() == next_block_row && clock.next_event(),
              "CE termination permits a fresh explicit address, never an automatic block wrap");
    } else {
      const NandCompletionEvent event{.ticks = row.released + read_ticks,
                                      .operation = NandPending::read};
      require(clock.device().ready() == false && clock.next_event() == event,
              "Final RE high schedules an actual within-block busy transfer");
      clock.observe_read_end(NandReadEnd::re_high);
      require(clock.next_event() == event,
              "Repeated edge observation cannot restart an active transfer");
      no_event(clock, event.ticks - 1);
      require(clock.device().ready() == false,
              "One tick before sequential completion remains busy");
      require(clock.advance(event.ticks) == event && clock.device().ready() == true,
              "Device event completes once at its exact deadline");
      require(clock.read() == following,
              "Next-page data comes from the independently populated cell");
    }
  }
}

void clock_conversions_and_abort() {
  // Fixed p13 conversions for 15us sequential tR and 5us CE-abort tCRY.
  constexpr std::array<std::array<unsigned, 3>, 5> cases{{
      {{1000000000, 15000, 5000}},
      {{1080000000, 16200, 5400}},
      {{540000000, 8100, 2700}},
      {{216000000, 3240, 1080}},
      {{1000001, 16, 6}},
  }};
  constexpr std::uint64_t consume_at = 100000;
  constexpr std::uint64_t release_at = 100050;
  constexpr std::uint64_t abort_at = 100051;
  for (const auto &row : cases) {
    auto clock = make_clock({}, row[0]);
    require(clock.advance(consume_at).has_value() && clock.read() == final_byte,
            "Explicitly complete and consume the sole last-column byte");
    no_event(clock, release_at);
    auto saved = clock;
    clock.observe_read_end(NandReadEnd::re_high);
    saved.observe_read_end(NandReadEnd::re_high);
    const NandCompletionEvent read{.ticks = release_at + row[1], .operation = NandPending::read};
    require(clock.next_event() == read && saved.next_event() == read,
            "A saved final-byte output retains the part profile and pending release");
    no_event(clock, abort_at);
    clock.observe_read_end(NandReadEnd::ce_high);
    const NandCompletionEvent abort{.ticks = abort_at + row[2], .operation = NandPending::read};
    require(clock.next_event() == abort && clock.device().ready() == false,
            "CE abort replaces the read deadline without inventing immediate Ready");
    auto busy_copy = clock;
    no_event(clock, abort.ticks - 1);
    require(clock.advance(abort.ticks) == abort && busy_copy.advance(abort.ticks + 1) == abort,
            "Fine/coarse advances and busy-state copy agree on exact CE-abort time");
    auto invalid_read = clock;
    reject([&] { static_cast<void>(invalid_read.read()); }, "no completed page transfer");
    require(clock.device().ready() == true && !clock.next_event() &&
                clock.device().cell({.page = 0, .column = last_column}).value == final_byte,
            "Abort returns Ready but provides no valid array buffer and preserves cells");
    clock.command(0);
    for (unsigned index = 0; index < 4; ++index) { clock.address(0); }
    require(read_deadline(clock) == abort.ticks + row[1],
            "Aborted read requires a new command/address and its own transfer time");
    saved.command(status_command);
    require(saved.read(ready_mask) == 0, "Real status exposes sequential busy");
    saved.observe_read_end(NandReadEnd::re_high);
    require(saved.next_event() == read && saved.advance(read.ticks) == read &&
                saved.read(ready_mask) == ready_mask,
            "Status reads/releases cannot restart or complete the array operation");
  }
}

NandOperationClock released_last_byte() {
  auto clock = make_clock();
  require(clock.advance(read_ticks).has_value() && clock.read() == final_byte,
          "Prepared last-column output uses actual protocol consumption");
  return clock;
}

void termination_recovery_and_unknown_cells() {
  auto terminated = released_last_byte();
  terminated.observe_read_end(NandReadEnd::ce_high);
  require(terminated.device().ready() == true && !terminated.next_event(),
          "CE deassertion at final-byte output terminates before any sequential transfer");
  terminated.command(0);
  for (unsigned index = 0; index < 4; ++index) { terminated.address(0); }
  require(read_deadline(terminated) == 2 * read_ticks,
          "Fresh command after termination retains normal addressed-read timing");

  auto reset = released_last_byte();
  reset.observe_read_end(NandReadEnd::re_high);
  constexpr unsigned reset_at = 16000;
  constexpr unsigned reset_done = 21000;
  no_event(reset, reset_at);
  reset.command(reset_command);
  require(reset.next_event() ==
              NandCompletionEvent{.ticks = reset_done, .operation = NandPending::reset},
          "Reset cancels the sequential event and uses its independent 5us bound");
  require(reset.advance(reset_done).has_value(), "Read-interruption reset completes");
  no_event(reset, 2 * read_ticks);
  require(!reset.next_event() && reset.device().ready() == true,
          "Cancelled sequential event cannot reappear after reset");

  auto lost = released_last_byte();
  lost.observe_read_end(NandReadEnd::re_high);
  lost.power_loss();
  require(!lost.next_event() && !lost.device().ready().has_value(),
          "Power loss cancels time and removes Ready knowledge");
  lost.observe_power_ready();
  lost.command(spare);
  lost.address(final_spare_column);
  for (unsigned index = 0; index < 3; ++index) { lost.address(0); }
  require(lost.advance(2 * read_ticks).has_value() && lost.read() == final_byte,
          "Cells and static part profile survive a read-only power cycle");
  lost.observe_read_end(NandReadEnd::re_high);
  require(read_deadline(lost) == 3 * read_ticks, "Power cycle retains explicit P-part behavior");

  auto unknown = make_clock({}, ghz, NextContents::missing);
  require(unknown.advance(read_ticks).has_value() && unknown.read() == final_byte,
          "Current page is known while the next one remains unobserved");
  unknown.observe_read_end(NandReadEnd::re_high);
  require(unknown.advance(2 * read_ticks).has_value() && unknown.device().ready() == true,
          "A transfer can complete without inventing cell contents");
  reject([&] { static_cast<void>(unknown.read()); }, "page=0x000001 column=0x200");
  require(unknown.faulted(), "Unknown next-page data quarantines the coupled owner");
}

void defaults_and_guards() {
  Nand unspecified;
  unspecified.observe_bytes(0, last_column, std::array{final_byte});
  unspecified.observe_power_ready();
  unspecified.command(spare);
  unspecified.address(final_spare_column);
  for (unsigned index = 0; index < 3; ++index) { unspecified.address(0); }
  unspecified.observe_completion();
  require(unspecified.read() == final_byte && !unspecified.ready(),
          "Unspecified-part default keeps the pre-existing unknown post-page state");
  reject([&] { unspecified.observe_read_end(NandReadEnd::re_high); }, "explicit P-package");
  reject([&] { unspecified.command(0); }, "unqualified readiness/post-page");
  reject(
      [] { static_cast<void>(Nand(std::bit_cast<NandReadContinuation>(std::uint8_t{byte_mask}))); },
      "invalid part profile");

  auto invalid = released_last_byte();
  const auto saved = invalid;
  reject([&] { invalid.observe_read_end(std::bit_cast<NandReadEnd>(std::uint8_t{byte_mask})); },
         "invalid observed edge");
  require(invalid.faulted(), "Invalid read-end input poisons the owner");
  reject([&] { static_cast<void>(invalid.advance(2 * read_ticks)); }, "coordinated recovery");
  auto restored = saved;
  restored.observe_read_end(NandReadEnd::re_high);
  require(read_deadline(restored) == 2 * read_ticks,
          "Trusted copy restores state and profile together");

  auto early_ce = make_clock({.command = 0, .column = 0});
  require(early_ce.advance(read_ticks).has_value(),
          "Prepared complete page before any byte output");
  early_ce.observe_read_end(NandReadEnd::ce_high);
  require(!early_ce.faulted() && early_ce.ticks() == read_ticks && !early_ce.next_event() &&
              early_ce.device().ready() == true && early_ce.device().column() == 0,
          "Samsung p18/Fig6 CE pause preserves a completed buffered page without a new operation");
  require(early_ce.read() == ordinary && early_ce.device().column() == 1,
          "A following timing-admitted selected read starts at the retained next column");

  Nand persistent(profile);
  persistent.observe_cell({.page = 0, .column = last_column},
                          {.value = final_byte, .known = byte_mask});
  Nand from_image(persistent.persistent_image(), profile);
  from_image.observe_power_ready();
  from_image.command(spare);
  from_image.address(final_spare_column);
  for (unsigned index = 0; index < 3; ++index) { from_image.address(0); }
  from_image.observe_completion();
  require(from_image.read() == final_byte,
          "Image constructor retains actual persistent cell knowledge");
  from_image.observe_read_end(NandReadEnd::re_high);
  require(from_image.pending() == NandPending::read,
          "Image profile is explicit, not inferred from image bytes");
}

void late_abort_and_counter_exhaustion() {
  auto late = make_clock();
  no_event(late, read_ticks - 1);
  late.observe_read_end(NandReadEnd::ce_high);
  constexpr unsigned aborted = 19999;
  require(read_deadline(late) == aborted, "CE one tick before completion replaces the old event");
  no_event(late, read_ticks);
  require(late.device().ready() == false,
          "Old random-read deadline cannot complete an aborted transfer");
  late.command(status_command);
  require(late.read(ready_mask) == 0, "Status stays busy throughout CE-abort recovery");
  late.observe_read_end(NandReadEnd::re_high);
  require(read_deadline(late) == aborted && late.advance(aborted).has_value() &&
              late.read(ready_mask) == ready_mask,
          "Status output release neither restarts nor fabricates abort completion");

  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  auto exact = released_last_byte();
  no_event(exact, maximum - read_ticks);
  exact.observe_read_end(NandReadEnd::re_high);
  require(read_deadline(exact) == maximum, "Final representable sequential deadline is legal");
  no_event(exact, maximum - 1);
  require(exact.advance(maximum).has_value() && exact.read() == following,
          "Counter limit completes the actual next-page transfer without wrapping");

  auto exhausted = released_last_byte();
  no_event(exhausted, maximum - read_ticks + 1);
  reject([&] { exhausted.observe_read_end(NandReadEnd::re_high); }, "deadline exceeds");
  require(exhausted.faulted(), "Sequential counter exhaustion quarantines the owner");

  auto abort = released_last_byte();
  no_event(abort, maximum - read_ticks);
  abort.observe_read_end(NandReadEnd::re_high);
  constexpr unsigned abort_before_limit = 4999;
  no_event(abort, maximum - abort_before_limit);
  reject([&] { abort.observe_read_end(NandReadEnd::ce_high); }, "deadline exceeds");
  require(abort.faulted(),
          "Abort counter exhaustion cannot wrap into an immediate successful event");
}
}

int main() {
  try {
    page_geometry_and_release();
    clock_conversions_and_abort();
    termination_recovery_and_unknown_cells();
    defaults_and_guards();
    late_abort_and_counter_exhaustion();
    std::cout << "NAND P-part:7 page/area/block vectors,5 clock/abort "
                 "vectors,checkpoint/recovery/strict guards passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
