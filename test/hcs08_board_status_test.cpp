// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_wire.hpp"
#include "hcs08_board_fixture.hpp"
#include "hcs08_board_wire_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::board_wire;
using blofeld::test::board::own_image;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr unsigned entry = 0xe000;
constexpr unsigned ram = 0x80;
constexpr unsigned data = 0x5a;
constexpr unsigned enabled = 0x80;
constexpr unsigned active_status = 0xc0;
// Own LDA #$5A;STA $80;BRA *. Independent budgets:2/3/3 periods.
constexpr std::array<std::uint8_t, 6> program{0xa6, data, 0xb7, ram, 0x20, 0xfe};

void status_at_every_budget_cut() {
  const auto image = own_image(program);
  for (unsigned cut = 0; cut <= 5; ++cut) {
    Hcs08BoardWire board(Hcs08Board::power_on_checkpoint(image, contract, normal),
                         BLOFELD_HCS08_SLA,
                         {},
                         wire_inputs);
    for (unsigned period = 0; period < cut; ++period) { board.advance_reference_period(); }
    const auto cpu = board.cpu_state();
    const auto clock = board.run_state();
    const auto periods = board.memory().periods();
    const auto revision = board.memory().revision();
    const auto saved = board.checkpoint();
    command(board, {hcs08_bdc::read_status});
    require(receive(board) == 0 && board.cpu_state() == cpu && board.run_state() == clock &&
                board.memory().periods() == periods && board.memory().revision() == revision &&
                board.memory().inspect_ram(ram).known == (cut < 5 ? 0 : mask),
            "GT15.4.1 status is outside CPU memory;actual pulses read it without a CPU/bus grant");
    Hcs08BoardWire fresh(saved, BLOFELD_HCS08_SLA);
    command(fresh, {hcs08_bdc::read_status});
    require(
        receive(fresh) == 0 && fresh.wire_state() == board.wire_state() &&
            fresh.bdc_state() == board.bdc_state(),
        "A trusted partial-instruction restore preserves the same independently serviced status");
    for (unsigned period = cut; period < 5; ++period) {
      board.advance_reference_period();
      fresh.advance_reference_period();
    }
    require(
        board.cpu_state().pc == entry + 4 && board.cpu_state().a == data &&
            board.cpu_state().a_known == mask && board.run_state().retired_instructions == 2 &&
            board.run_state().remaining_periods == 0 &&
            board.memory().inspect_ram(ram).value == data &&
            board.memory().inspect_ram(ram).known == mask &&
            board.memory().cop_state().elapsed == 5 && board.memory().flash_image() == image &&
            fresh.cpu_state() == board.cpu_state() && fresh.run_state() == board.run_state(),
        "Read-only BDC service leaves the actual2+3period LDA/STA retirement and RAM write intact");
  }
}

void latched_reply_across_native_bgnd() {
  constexpr std::array<std::uint8_t, 2> code{0x82, 0x9d}; // Own BGND;NOP.
  Hcs08BoardWire board(Hcs08Board::power_on_checkpoint(own_image(code), contract, normal),
                       BLOFELD_HCS08_SLA,
                       {},
                       wire_inputs);
  command(board, {hcs08_bdc::write_control, enabled});
  board.advance_reference_period();
  require(board.run_state().remaining_periods == 4 && !board.bdc_state().active,
          "Actual enabled BGND owns four more native periods before background entry");
  command(board, {hcs08_bdc::read_status});
  Hcs08BoardWire fresh(board.checkpoint(), BLOFELD_HCS08_SLA);
  for (unsigned period = 0; period < 4; ++period) {
    board.advance_reference_period();
    fresh.advance_reference_period();
  }
  require(board.bdc_state().active && board.cpu_state().pc == entry + 1 &&
              board.run_state().retired_instructions == 1 &&
              board.memory().cop_state().elapsed == 5,
          "Independent five-period native BGND changes BDMACT only on its actual retirement");
  require(
      receive(board) == enabled && receive(fresh) == enabled &&
          board.wire_state() == fresh.wire_state(),
      "The existing reference service latches a complete status byte before the later CPU event");
  command(board, {hcs08_bdc::read_status});
  require(
      receive(board) == active_status,
      "A later actual E4 command observes the new native BDMACT value,without a torn first reply");
}

void control_changes_remain_strict() {
  constexpr std::array<std::uint8_t, 2> code{0x20, 0xfe};
  Hcs08BoardWire board(Hcs08Board::power_on_checkpoint(own_image(code), contract, normal),
                       BLOFELD_HCS08_SLA,
                       {},
                       wire_inputs);
  board.advance_reference_period();
  const auto before = board.cpu_state();
  const auto failure = rejected([&] { command(board, {hcs08_bdc::write_control, enabled}); });
  require(
      failure.contains("instruction budget is pending") && failure.contains("agent=BDC") &&
          board.bdc_state().control == 0 && board.bdc_state().serviced == 0 &&
          board.run_state().remaining_periods == 2 && board.cpu_state() == before &&
          board.faulted(),
      "Read-only status admission does not qualify control-clock changes or steal CPU ownership");
}
} // namespace

int main() {
  try {
    status_at_every_budget_cut();
    latched_reply_across_native_bgnd();
    control_changes_remain_strict();
    std::cout
        << "Verified independent BDC status during native CPU budgets and latched mode replies.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
