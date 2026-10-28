// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "hcs08_board_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::board;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr unsigned data = 0x5a, flags = 0x68, known_after_load = 0xee, known_high_h = 0xff00;
constexpr std::uint8_t write_byte = 0xc0, read_byte = 0xe0, write_control = 0xc4, background = 0x90;
// Own LDA #$5A;STA $80;NOP;BRA *. GT instruction table:2,3,1,3 periods.
constexpr std::array<std::uint8_t, 7> code{0xa6, data, 0xb7, ram, 0x9d, 0x20, 0xfe};

void native_flash_error_recovery() {
  constexpr unsigned fault_status = 0xd0;
  constexpr unsigned recovered_status = 0xc0;
  constexpr unsigned negative_flags = 0x6c;
  constexpr unsigned error_period = 12;
  constexpr unsigned final_period = 32;
  constexpr unsigned final_offset = 25;
  constexpr unsigned retired = 10;
  // Own LDA/STA/ORA program; Family Rev2 table6-13 gives2/4/2/4/4/2/4/3/4/3 periods.
  // It creates an invalid-command error, acknowledges it by actual native RMW,
  // then stores the source A and a new FSTAT read to two otherwise unknown RAM bytes.
  constexpr std::array<std::uint8_t, 27> program{
      0xa6, 30,   0xc7, 0x18, 0x20, 0xa6, 0x26, 0xc7, 0x18, 0x26, 0xc6, 0x18, 0x25, 0xaa,
      0x10, 0xc7, 0x18, 0x25, 0xb7, 0x80, 0xc6, 0x18, 0x25, 0xb7, 0x81, 0x20, 0xfe};
  const auto image = own_image(program);
  Hcs08Board board(image, BLOFELD_HCS08_SLA, reference, {});
  to(board, error_period);
  const auto flash = board.memory().flash_state();
  require(
      flash.errors == hcs08_flash::access_error && flash.buffer == Hcs08FlashBuffer::empty &&
          !flash.active && board.cpu_state().pc == entry + 10 &&
          board.run_state().retired_instructions == 4 &&
          board.memory().inspect_ram(ram).known == 0 &&
          board.memory().inspect_ram(ram + 1).known == 0,
      "Four actual reset instructions establish the error before any acknowledgement or RAM seed");
  const auto saved = board.checkpoint();
  to(board, final_period);
  auto expected_flash = flash;
  expected_flash.errors = 0;
  require(board.cpu_state() == Hcs08State{.pc = entry + final_offset,
                                          .sp = byte_mask,
                                          .a = recovered_status,
                                          .ccr_known = known_after_load,
                                          .ccr = negative_flags,
                                          .hx_known = known_high_h} &&
              board.run_state().retired_instructions == retired &&
              board.run_state().remaining_periods == 0 &&
              board.memory().flash_state() == expected_flash &&
              board.memory().flash_image() == image,
          "Native LDA/ORA/STA acknowledges only FACCERR, preserving PC/flags/NVM and idle command "
          "state");
  require(
      board.memory().inspect_ram(ram).value == fault_status &&
          board.memory().inspect_ram(ram).known == byte_mask &&
          board.memory().inspect_ram(ram + 1).value == recovered_status &&
          board.memory().inspect_ram(ram + 1).known == byte_mask,
      "Only actual native stores publish the original A and subsequent hardware status readback");
  for (unsigned replay = 0; replay < 2; ++replay) {
    Hcs08Board restored(saved, BLOFELD_HCS08_SLA);
    to(restored, final_period);
    require(
        restored.cpu_state() == board.cpu_state() && restored.state() == board.state() &&
            restored.run_state() == board.run_state(),
        "Fresh native owners replay the same error acknowledgement and independent period budget");
    same_memory(restored.memory(), board.memory());
  }
}

void native_costs_and_copy() {
  const auto image = own_image(code);
  Hcs08Board board(image, BLOFELD_HCS08_SLA, reference, {});
  require(board.cpu_state() == Hcs08State{.pc = entry,
                                          .sp = byte_mask,
                                          .ccr_known = flags,
                                          .ccr = flags,
                                          .a_known = 0,
                                          .hx_known = known_high_h},
          "A real normal reset vector,not a supplied PC,establishes the native initial state");
  to(board, 4);
  require(
      board.cpu_state() == Hcs08State{.pc = entry + 2,
                                      .sp = byte_mask,
                                      .a = data,
                                      .ccr_known = known_after_load,
                                      .ccr = flags,
                                      .hx_known = known_high_h} &&
          board.run_state().retired_instructions == 1 && board.run_state().remaining_periods == 1 &&
          board.memory().inspect_ram(ram).known == 0 && board.memory().cop_state().elapsed == 4,
      "Independent LDA flags and2+2offered periods leave the actual STA budget and RAM untouched");
  const auto saved = board.checkpoint();
  to(board, 6);
  Hcs08Board fresh(saved, BLOFELD_HCS08_SLA);
  to(fresh, 6);
  require(
      board.cpu_state().pc == entry + 5 && board.cpu_state().a == data &&
          board.run_state().retired_instructions == 3 &&
          board.memory().inspect_ram(ram).value == data &&
          board.memory().inspect_ram(ram).known == byte_mask &&
          board.memory().cop_state().elapsed == 6 && fresh.cpu_state() == board.cpu_state() &&
          fresh.state() == board.state() && fresh.run_state() == board.run_state(),
      "Literal2+3+1retirement costs and real RAM stores survive a fresh partial-STA restoration");
  same_memory(board.memory(), fresh.memory());
  require(board.memory().flash_image() == image,
          "Ordinary original-core execution never patches NVM");
}

void copied_pad_obligation() {
  constexpr std::array<std::uint8_t, 4> program{0xb6, 0x10, 0x20, 0xfe}; // Own LDA PTED;BRA *.
  Hcs08Board board(own_image(program), BLOFELD_HCS08_SLA, reference, {});
  unsigned samples = 0;
  board.attach_pad_source(Hcs08GpioPort::e, [&](const unsigned mask) {
    ++samples;
    return GpioPadSample{.known = mask, .levels = mask & 8U};
  });
  to(board, 1);
  const auto saved = board.checkpoint();
  to(board, 3);
  require(board.cpu_state().a == 8 && board.cpu_state().pc == entry + 2 && samples == 1,
          "Only a real bound Port E sample supplies the native three-period LDA operand");
  Hcs08Board detached(saved, BLOFELD_HCS08_SLA);
  require(rejected([&] { to(detached, 3); }).contains("Detached resolved GPIO") &&
              detached.faulted() && samples == 1 && detached.state().completed_periods == 2,
          "A fresh restore cannot consume the source board's pad callback");
  Hcs08Board rebound(saved, BLOFELD_HCS08_SLA);
  rebound.attach_pad_source(Hcs08GpioPort::e, [](const unsigned mask) {
    return GpioPadSample{.known = mask, .levels = mask & 8U};
  });
  to(rebound, 3);
  require(rebound.cpu_state() == board.cpu_state(),
          "Explicitly reattached observations restore the suffix");
  same_memory(board.memory(), rebound.memory());
}

void cop_quarantine() {
  constexpr unsigned short_timeout = 8192;
  constexpr unsigned short_options = 0x93;
  constexpr unsigned loop_pc = entry + 5;
  constexpr unsigned retired_at_timeout = 2730;
  // Own LDA #$93;STA $1802;BRA *. Prefix costs2+4;BRA costs3;COP is never serviced.
  constexpr std::array<std::uint8_t, 7> program{0xa6, short_options, 0xc7, 0x18, 2, 0x20, 0xfe};
  const auto image = own_image(program);
  Hcs08Board board(image, BLOFELD_HCS08_SLA, reference, {});
  to(board, short_timeout - 1);
  require(board.run_state().retired_instructions == retired_at_timeout &&
              board.run_state().remaining_periods == 2 && board.cpu_state().pc == loop_pc,
          "Independent six-period prefix plus8185branch periods leaves a two-period native budget");
  const auto failure = rejected([&] { board.advance_reference_period(); });
  require(failure.contains("pending COP reset") &&
              board.state().attempted_periods == short_timeout &&
              board.state().completed_periods == short_timeout - 1 &&
              board.state().phase == Hcs08BoardPhase::devices &&
              board.run_state().offered_periods == short_timeout - 1 &&
              board.memory().cop_state().elapsed == short_timeout &&
              board.memory().cop_state().reset_requested && board.faulted(),
          "COP expires on its real8192nd edge,before another CPU budget period or reset-vector "
          "shortcut");
  const auto frozen = board.memory().periods();
  require(rejected([&] { board.advance_reference_period(); }).contains("coordinated recovery") &&
              rejected([&] { static_cast<void>(board.checkpoint()); }).contains("checkpoint") &&
              board.memory().periods() == frozen && board.memory().flash_image() == image,
          "A device-stage failure quarantines whole-board progress and successful snapshots");
}

void bdc_mapping_and_grants() {
  Hcs08Board board(own_image(code), BLOFELD_HCS08_SLA, reference, {});
  command(board, {write_byte, 0, ram, data});
  command(board, {read_byte, 0, ram});
  require(
      board.read_bdc_reply() == data && board.memory().inspect_ram(ram).value == data &&
          board.cpu_state().pc == entry && board.state().completed_periods == 0,
      "Real native byte frames use the same board RAM without invented CPU execution or wire time");
  to(board, 1);
  for (const auto byte : std::array<std::uint8_t, 4>{write_byte, 0, ram, 1}) {
    board.receive_bdc(byte);
  }
  require(rejected([&] {
            board.service_bdc(Hcs08BdcGrant::reference_instruction_boundary);
          }).contains("complete instruction boundary") &&
              board.memory().inspect_ram(ram).value == data &&
              board.run_state().remaining_periods == 1 && board.faulted(),
          "A complete BDC frame cannot steal an uncommitted native instruction's bus grant");
  Hcs08Board secure(own_image(code, 3), BLOFELD_HCS08_SLA, reference, {});
  command(secure, {write_byte, 0, ram, data});
  command(secure, {read_byte, 0, ram});
  require(secure.read_bdc_reply() == 0 && secure.memory().inspect_ram(ram).known == 0,
          "GT4.5 secure BDC RAM writes are ignored and reads return architectural zero");
  command(secure, {write_control, 0x80});
  command(secure, {background});
  require(!secure.bdc_state().active,
          "Secure normal mode cannot enter background through a command");
}

void bdc_privilege_and_unknown_security() {
  Hcs08BoardMemory target(own_image(code), Hcs08BoardMemoryContract::candidate_gt8a_untimed);
  constexpr ExecutionContext context{.pc = entry, .ccr = flags};
  target.begin_instruction(context);
  constexpr unsigned protection = 0xfc;
  target.write({.address = hcs08_flash::protection, .value = protection}, context);
  target.write(hcs08_flash::protection, byte_mask);
  require(target.read(hcs08_flash::protection) == protection,
          "A genuine BDC protection write never leaks background privilege into a later CPU write");
  auto image = own_image(code);
  image.known.reset(hcs08_flash::nv_options - hcs08_flash::first);
  Hcs08Board unknown(image, BLOFELD_HCS08_SLA, reference, {});
  require(
      rejected([&] {
        command(unknown, {write_byte, 0, ram, data});
      }).contains("unknown reset-latched") &&
          unknown.memory().inspect_ram(ram).known == 0 && unknown.faulted(),
      "Unknown nonvolatile SEC bits cannot be replaced with a guessed successful debugger write");
}

void bgnd_and_resume() {
  constexpr std::array<std::uint8_t, 4> program{0x82, 0x9d, 0x20, 0xfe}; // Own BGND;NOP;BRA *.
  Hcs08Board board(own_image(program), BLOFELD_HCS08_SLA, reference, {});
  command(board, {write_control, 0x80});
  to(board, 5);
  require(board.bdc_state().active && board.cpu_state().pc == entry + 1 &&
              board.run_state().retired_instructions == 1,
          "Documented five-period native BGND enters the actual enabled,unsecured controller");
  Hcs08Board resumed(board.checkpoint(), BLOFELD_HCS08_SLA);
  command(resumed, {0x08});
  to(resumed, 6);
  require(!resumed.bdc_state().active && resumed.cpu_state().pc == entry + 2 &&
              resumed.run_state().retired_instructions == 2,
          "A real GO frame resumes the original next PC and native NOP from a fresh joint state");
  require(rejected([&] { board.advance_reference_period(); }).contains("background/debug") &&
              board.state().attempted_periods == 5 && board.memory().cop_state().elapsed == 5,
          "Unqualified active-debug clock gating refuses before any additional device edge");
}
} // namespace

int main() {
  try {
    native_flash_error_recovery();
    native_costs_and_copy();
    copied_pad_obligation();
    cop_quarantine();
    bdc_mapping_and_grants();
    bdc_privilege_and_unknown_security();
    bgnd_and_resume();
    std::cout << "Verified native board periods,watchdog,BDC security/grants and joint restores.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
