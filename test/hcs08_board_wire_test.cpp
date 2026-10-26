// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_wire.hpp"
#include "hcs08_board_fixture.hpp"
#include "hcs08_board_wire_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"

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
constexpr unsigned ram = 0x200;
constexpr unsigned byte_values = 256;
constexpr unsigned status = 0xc8;

void initial_entry() {
  const auto image = blank();
  const auto token = Hcs08Board::power_on_checkpoint(image, contract, active);
  Hcs08BoardWire board(token, BLOFELD_HCS08_SLA, {}, wire_inputs);
  constexpr unsigned flags = 0x68;
  constexpr unsigned known_h = 0xff00;
  require(board.cpu_state() == Hcs08State{.sp = mask,
                                          .ccr_known = flags,
                                          .ccr = flags,
                                          .pc_known = false,
                                          .a_known = 0,
                                          .hx_known = known_h} &&
              board.bdc_state().active && board.memory().flash_image() == image &&
              board.run_state().retired_instructions == 0,
          "First BKGD-low power-on needs no manufactured PC or known reset-vector bytes");
  command(board, {hcs08_bdc::read_status});
  require(receive(board) == status && board.wire_state().received_bytes == 1 &&
              board.wire_state().transmitted_bytes == 1 &&
              board.board_state().completed_periods == 0,
          "Actual sixteen-cycle target pulses return C8 without CPU/device budget progress");
  const auto normal_token = Hcs08Board::power_on_checkpoint(image, contract, normal);
  require(rejected([&] {
            const Hcs08BoardWire refused(normal_token, BLOFELD_HCS08_SLA, {}, wire_inputs);
          }).contains("unknown"),
          "The same unknown vector refuses normal power-on rather than selecting a host entry");
  require(
      rejected([&] { board.advance_reference_period(); }).contains("background/debug") &&
          board.memory().periods().attempted == 0 && board.faulted(),
      "The wire wrapper preserves strict active-background BUSCLK refusal before a device edge");
}

void all_bytes() {
  Hcs08BoardWire board(Hcs08Board::power_on_checkpoint(blank(), contract, active),
                       BLOFELD_HCS08_SLA,
                       {},
                       wire_inputs);
  const auto before = board.cpu_state();
  for (unsigned value = 0; value < byte_values; ++value) {
    const auto byte = static_cast<std::uint8_t>(value);
    command(board, {hcs08_bdc::write_byte, 2, 0, byte});
    command(board, {hcs08_bdc::read_byte, 2, 0});
    require(
        receive(board) == byte && board.memory().inspect_ram(ram).value == byte &&
            board.memory().inspect_ram(ram).known == mask,
        "Every independent byte is written by native wire C0 and read as eight actual E0 pulses");
  }
  require(board.bdc_state().serviced == std::uint64_t{byte_values} * 2 &&
              board.wire_state().received_bytes == std::uint64_t{byte_values} * 7 &&
              board.wire_state().transmitted_bytes == byte_values && board.cpu_state() == before &&
              board.board_state().completed_periods == 0 && board.memory().cop_state().elapsed == 0,
          "Serial edges supply no invented CPU,watchdog clock or target instruction");
}

void partial_bit_and_reply_copies() {
  Hcs08BoardWire board(Hcs08Board::power_on_checkpoint(blank(), contract, active),
                       BLOFELD_HCS08_SLA,
                       {},
                       wire_inputs);
  board.host_drive(low, {.pc = host_pc});
  clocks(board, 3);
  const auto partial = board.checkpoint();
  Hcs08BoardWire fresh(partial, BLOFELD_HCS08_SLA);
  const auto suffix = [](Hcs08BoardWire &target) {
    target.host_drive(high, {.pc = host_pc});
    clocks(target, 14);
    // Remaining seven bits of E4 after its first actual one bit.
    constexpr std::array bits{true, true, false, false, true, false, false};
    for (const auto one : bits) {
      target.host_drive(low, {.pc = host_pc});
      clocks(target, 4);
      if (one) { target.host_drive(high, {.pc = host_pc}); }
      clocks(target, 9);
      target.host_drive(high, {.pc = host_pc});
      clocks(target, 4);
    }
    clocks(target, 16);
  };
  suffix(board);
  suffix(fresh);
  require(board.wire_state() == fresh.wire_state() && board.bdc_state() == fresh.bdc_state() &&
              board.bdc_state().serviced == 1,
          "Fresh whole-owner construction resumes the same partial native receive bit");
  const auto reply = board.checkpoint();
  Hcs08BoardWire before_reply(reply, BLOFELD_HCS08_SLA);
  require(receive(before_reply) == status && receive(board) == status,
          "Restoring a pending reply never consumes a constructor-time BDC response byte");
  require(before_reply.wire_state() == board.wire_state(),
          "Fresh reply timing and all pulse bookkeeping remain identical");
}

void no_mid_instruction_service() {
  constexpr std::array<std::uint8_t, 2> code{0x20, 0xfe}; // Own three-period BRA *.
  Hcs08BoardWire board(Hcs08Board::power_on_checkpoint(own_image(code), contract, normal),
                       BLOFELD_HCS08_SLA,
                       {},
                       wire_inputs);
  command(board, {hcs08_bdc::write_control, 0x80});
  board.advance_reference_period();
  require(board.run_state().remaining_periods == 2, "A real native BRA owns its unfinished budget");
  // Last byte sampling precedes the service deadline;pause one edge before it.
  send(board, hcs08_bdc::background);
  while (true) {
    const auto pending = board.wire_state();
    if (!pending.service_due || pending.next_clock_ticks >= *pending.service_due) { break; }
    board.clock_bdc();
  }
  const auto before = board.wire_state();
  const auto failure = rejected([&] { board.clock_bdc(); });
  require(failure.contains("instruction budget is pending") && failure.contains("agent=BDC") &&
              failure.contains("address=BKGD") && failure.contains("pc=0xe000") &&
              board.wire_state() == before && board.bdc_state().serviced == 1 &&
              !board.bdc_state().active && board.run_state().remaining_periods == 2 &&
              board.faulted(),
          "A wire service deadline cannot silently consume a mid-instruction target grant");
  require(rejected([&] { static_cast<void>(board.checkpoint()); }).contains("coordinated recovery"),
          "Wire-stage refusal quarantines successful joint snapshots");
}

void running_memory_and_security() {
  constexpr std::array<std::uint8_t, 2> code{0x20, 0xfe};
  Hcs08BoardWire running(Hcs08Board::power_on_checkpoint(own_image(code), contract, normal),
                         BLOFELD_HCS08_SLA,
                         {},
                         wire_inputs);
  require(rejected([&] {
            command(running, {hcs08_bdc::write_byte, 2, 0, 1});
          }).contains("arbitration is unqualified") &&
              running.memory().inspect_ram(ram).known == 0 && running.faulted(),
          "A complete instruction boundary alone fabricates no running-memory BDC bus grant");
  Hcs08BoardWire secure(Hcs08Board::power_on_checkpoint(blank(3), contract, active),
                        BLOFELD_HCS08_SLA,
                        {},
                        wire_inputs);
  command(secure, {hcs08_bdc::write_byte, 2, 0, 1});
  command(secure, {hcs08_bdc::read_byte, 2, 0});
  require(receive(secure) == 0 && secure.memory().inspect_ram(ram).known == 0,
          "The native GT4.5 secure wire path ignores RAM writes and returns architectural zero");
  auto unknown_image = blank();
  unknown_image.known.reset(hcs08_flash::nv_options - hcs08_flash::first);
  Hcs08BoardWire unknown(Hcs08Board::power_on_checkpoint(unknown_image, contract, active),
                         BLOFELD_HCS08_SLA,
                         {},
                         wire_inputs);
  require(rejected([&] {
            command(unknown, {hcs08_bdc::write_byte, 2, 0, 1});
          }).contains("unknown reset-latched") &&
              unknown.memory().inspect_ram(ram).known == 0 && unknown.faulted(),
          "Forced BKGD entry does not fabricate invalidated security bits or RAM write success");
}

void native_go_and_pending_cpu_copy() {
  Hcs08BoardWire board(Hcs08Board::power_on_checkpoint(blank(), contract, active),
                       BLOFELD_HCS08_SLA,
                       {},
                       wire_inputs);
  constexpr unsigned nop_opcode = 0x9d;
  constexpr unsigned branch_offset = 0xfe;
  // Own RAM NOP;BRA * uploaded entirely as actual serialized BDC commands.
  command(board, {hcs08_bdc::write_byte, 2, 0, nop_opcode});
  command(board, {hcs08_bdc::write_byte, 2, 1, 0x20});
  command(board, {hcs08_bdc::write_byte, 2, 2, branch_offset});
  command(board, {hcs08_bdc::write_pc, 2, 0});
  command(board, {hcs08_bdc::go});
  require(
      !board.bdc_state().active && board.cpu_state().pc == ram &&
          board.run_state().retired_instructions == 0 && board.memory().cop_state().elapsed == 0,
      "Actual GO resumes the wire-written PC but supplies no target instruction or device time");
  board.advance_reference_period();
  board.advance_reference_period();
  require(board.cpu_state().pc == ram + 1 && board.run_state().remaining_periods == 2 &&
              board.run_state().retired_instructions == 1,
          "Independent one-period NOP followed by a pending three-period BRA uses native RAM code");
  Hcs08BoardWire fresh(board.checkpoint(), BLOFELD_HCS08_SLA);
  for (unsigned period = 0; period < 2; ++period) {
    fresh.advance_reference_period();
    board.advance_reference_period();
  }
  require(fresh.cpu_state() == board.cpu_state() && fresh.run_state() == board.run_state() &&
              fresh.board_state() == board.board_state() &&
              fresh.wire_state() == board.wire_state() && fresh.bdc_state() == board.bdc_state() &&
              board.cpu_state().pc == ram + 1 && board.run_state().retired_instructions == 2 &&
              board.memory().cop_state().elapsed == 4,
          "Joint wire/native restoration retains and finishes the same actual reserved branch");
}

void timeout_and_resume() {
  Hcs08BoardWire board(Hcs08Board::power_on_checkpoint(blank(), contract, active),
                       BLOFELD_HCS08_SLA,
                       {},
                       wire_inputs);
  command(board, {hcs08_bdc::write_a, 1});
  send(board, hcs08_bdc::write_byte);
  clocks(board, 512);
  require(board.wire_state().timeouts == 1 && board.bdc_state().received == 0 &&
              board.cpu_state().a == 1 && board.cpu_state().a_known == mask,
          "A real wire timeout drops only incomplete communication and retains committed native A");
  const auto reset = board.reset_checkpoint({.cause = Hcs08ResetCause::warm, .entry = active});
  const Hcs08BoardWire restarted(reset, BLOFELD_HCS08_SLA, {}, wire_inputs);
  require(restarted.cpu_state().a == 1 && restarted.wire_state().cycles == 0 &&
              restarted.bdc_state().serviced == 0 && !restarted.cpu_state().pc_known,
          "A coordinated warm reset discards wire time/frame ownership and retains real native A");
}

void detached_native_gpio() {
  constexpr unsigned load_gpio = 0xb6;
  constexpr unsigned port_e_data = 0x10;
  Hcs08BoardWire board(Hcs08Board::power_on_checkpoint(blank(), contract, active),
                       BLOFELD_HCS08_SLA,
                       {},
                       wire_inputs);
  command(board, {hcs08_bdc::write_byte, 2, 0, load_gpio});
  command(board, {hcs08_bdc::write_byte, 2, 1, port_e_data});
  command(board, {hcs08_bdc::write_pc, 2, 0});
  command(board, {hcs08_bdc::go});
  unsigned samples = 0;
  board.attach_pad_source(Hcs08GpioPort::e, [&](const unsigned selected) {
    ++samples;
    return GpioPadSample{.known = selected, .levels = selected & 8U};
  });
  board.advance_reference_period();
  const auto saved = board.checkpoint();
  board.advance_reference_period();
  board.advance_reference_period();
  require(board.cpu_state().a == 8 && samples == 1 && board.cpu_state().pc == ram + 2,
          "An uploaded native three-period LDA samples the actual bound Port E input");
  Hcs08BoardWire detached(saved, BLOFELD_HCS08_SLA);
  detached.advance_reference_period();
  require(
      rejected([&] { detached.advance_reference_period(); }).contains("Detached resolved GPIO") &&
          samples == 1 && detached.faulted(),
      "Fresh wire/native snapshots never retain callbacks into another live board");
  Hcs08BoardWire rebound(saved, BLOFELD_HCS08_SLA);
  rebound.attach_pad_source(Hcs08GpioPort::e, [](const unsigned selected) {
    return GpioPadSample{.known = selected, .levels = selected & 8U};
  });
  rebound.advance_reference_period();
  rebound.advance_reference_period();
  require(rebound.cpu_state() == board.cpu_state() && rebound.run_state() == board.run_state() &&
              rebound.wire_state() == board.wire_state(),
          "Explicit pad rebinding reproduces the same native and transport suffix");
}

void invalid_inputs_and_opposing_drivers() {
  const auto image = blank();
  const auto reset = Hcs08Board::power_on_checkpoint(image, contract, active);
  require(rejected([&] {
            static_cast<void>(
                Hcs08Board::power_on_checkpoint(image, Hcs08BoardContract::unqualified, active));
          }).contains("explicit board/reset") &&
              rejected([&] {
                static_cast<void>(
                    Hcs08Board::power_on_checkpoint(image, contract, Hcs08ResetEntry::unqualified));
              }).contains("explicit board/reset") &&
              rejected([&] {
                const Hcs08BoardWire refused(reset, BLOFELD_HCS08_SLA, {}, {});
              }).contains("command-service delay"),
          "Unknown architectural entry or wire delay/clock inputs cannot become a default device");
  Hcs08BoardWire opposed(reset, BLOFELD_HCS08_SLA, {}, wire_inputs);
  command(opposed, {hcs08_bdc::read_status});
  opposed.host_drive(low, {.pc = host_pc});
  const auto failure = rejected([&] { clocks(opposed, 8); });
  require(failure.contains("opposing active host/target drivers") &&
              failure.contains("agent=BDC") && failure.contains("address=BKGD") &&
              failure.contains("pc=unknown pc-known=0") && opposed.faulted() &&
              opposed.memory().flash_image() == image,
          "Host-low against the real first-one target high pulse quarantines the whole owner");
  const auto failed = opposed.wire_state();
  require(rejected([&] { opposed.clock_bdc(); }).contains("coordinated recovery") &&
              opposed.wire_state() == failed,
          "A driver collision cannot consume further wire time on a failed owner");
  Hcs08BoardWire recovered(
      opposed.reset_checkpoint({.cause = Hcs08ResetCause::warm, .entry = active}),
      BLOFELD_HCS08_SLA,
      {},
      wire_inputs);
  command(recovered, {hcs08_bdc::read_status});
  require(
      receive(recovered) == status && opposed.faulted(),
      "An explicit fresh coordinated reset drops the old failed wire and returns native status");
}
} // namespace

int main() {
  try {
    initial_entry();
    all_bytes();
    partial_bit_and_reply_copies();
    no_mid_instruction_service();
    running_memory_and_security();
    native_go_and_pending_cpu_copy();
    timeout_and_resume();
    detached_native_gpio();
    invalid_inputs_and_opposing_drivers();
    std::cout << "Verified production board BKGD pulses,unknown-vector entry,grants and joint wire "
                 "snapshots.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
