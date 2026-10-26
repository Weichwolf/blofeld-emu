// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
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
#include <optional>

namespace {
using namespace blofeld;
using namespace blofeld::test::board;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr unsigned data = 0x5a;
constexpr unsigned flags = 0x68;
constexpr unsigned known_high_h = 0xff00;
constexpr unsigned power_on_status = 0x80;
constexpr unsigned default_options = 0xd3;
constexpr unsigned debug_control = 0x88;
constexpr unsigned debug_status = 0xc8;
constexpr auto normal = Hcs08ResetEntry::normal;
constexpr auto debug = Hcs08ResetEntry::active_background;
constexpr auto warm = Hcs08ResetCause::warm;
constexpr auto cold = Hcs08ResetCause::power_on;
// Own LDA #$5A;STA $80;NOP;BRA *. Independent periods:2,3,1,3.
constexpr std::array<std::uint8_t, 7> code{0xa6, data, 0xb7, ram, 0x9d, 0x20, 0xfe};

void reset_state(const Hcs08Board &board, const std::optional<std::uint8_t> sources) {
  require(!board.faulted() && board.state() == Hcs08BoardState{} &&
              board.run_state().remaining_periods == 0 && board.run_state().offered_periods == 0 &&
              board.run_state().retired_instructions == 0 && board.bdc_state() == Hcs08BdcState{} &&
              board.memory().cop_state().reset_status == sources &&
              board.memory().cop_state().elapsed == 0 &&
              !board.memory().cop_state().reset_requested &&
              !board.memory().cop_state().options_written &&
              board.memory().cop_state().options == default_options,
          "A logical normal reset starts new CPU,BDC,clock and watchdog ownership");
}

void native_reset_cuts() {
  const auto image = own_image(code);
  for (unsigned cut = 0; cut <= 6; ++cut) {
    Hcs08Board original(image, BLOFELD_HCS08_SLA, reference, {});
    to(original, cut);
    const auto previous_cpu = original.cpu_state();
    const auto previous_clock = original.run_state();
    const auto previous_board = original.state();
    const auto previous_bdc = original.bdc_state();
    const auto request = Hcs08BoardResetRequest{.cause = warm, .entry = normal};
    Hcs08Board restarted(original.reset_checkpoint(request), BLOFELD_HCS08_SLA, {});
    reset_state(restarted, std::nullopt);
    const auto expected_a = static_cast<std::uint8_t>(cut < 2 ? 0 : data);
    const auto expected_known = static_cast<std::uint8_t>(cut < 2 ? 0 : byte_mask);
    require(restarted.cpu_state() == Hcs08State{.pc = entry,
                                                .sp = byte_mask,
                                                .a = expected_a,
                                                .ccr_known = flags,
                                                .ccr = flags,
                                                .a_known = expected_known,
                                                .hx_known = known_high_h} &&
                restarted.memory().inspect_ram(ram).known == (cut < 5 ? 0 : byte_mask) &&
                restarted.memory().inspect_ram(ram).value == (cut < 5 ? 0 : data),
            "Each actual instruction-budget cut retains committed A/RAM only and refetches FFFE");
    to(restarted, 5);
    require(restarted.cpu_state().pc == entry + 4 &&
                restarted.run_state().retired_instructions == 2 &&
                restarted.memory().inspect_ram(ram).value == data &&
                restarted.memory().flash_image() == image,
            "New reset execution spends a full2+3periods and never resumes the discarded budget");
    const Hcs08Board power_on(
        original.reset_checkpoint(
            {.cause = cold, .entry = normal, .reset_sources = power_on_status}),
        BLOFELD_HCS08_SLA,
        {});
    reset_state(power_on, power_on_status);
    require(power_on.cpu_state().a_known == 0 && power_on.cpu_state().hx_known == known_high_h &&
                power_on.memory().flash_image() == image,
            "Power-on discards CPU A/X certainty without changing settled NVM");
    for (unsigned address = Hcs08BoardMemory::ram_first; address < Hcs08BoardMemory::ram_end;
         ++address) {
      require(power_on.memory().inspect_ram(static_cast<std::uint16_t>(address)).known == 0,
              "Power-on never preserves any one of the1024SRAM byte masks");
    }
    require(original.cpu_state() == previous_cpu && original.run_state() == previous_clock &&
                original.state() == previous_board && original.bdc_state() == previous_bdc,
            "Preparing an independent reset checkpoint never rewinds or mutates its source");
  }
}

void watchdog_recovery() {
  constexpr unsigned timeout = 8192;
  constexpr unsigned short_options = 0x93;
  constexpr unsigned cop_status = 0x20;
  // Own short COP loop. The expiry interrupts a pending3period BRA,not an instruction write.
  constexpr std::array<std::uint8_t, 7> program{0xa6, short_options, 0xc7, 0x18, 2, 0x20, 0xfe};
  Hcs08Board expired(own_image(program), BLOFELD_HCS08_SLA, reference, {});
  command(expired, {hcs08_bdc::write_byte, 0, ram, data});
  to(expired, timeout - 1);
  static_cast<void>(rejected([&] { expired.advance_reference_period(); }));
  require(expired.faulted() && expired.state().completed_periods == timeout - 1 &&
              expired.memory().cop_state().reset_requested,
          "Recovery starts at the actual watchdog failure,with no additional CPU period");
  Hcs08Board restarted(
      expired.reset_checkpoint({.cause = warm, .entry = normal, .reset_sources = cop_status}),
      BLOFELD_HCS08_SLA,
      {});
  reset_state(restarted, cop_status);
  require(restarted.cpu_state().pc == entry && restarted.cpu_state().a == short_options &&
              restarted.memory().inspect_ram(ram).value == data,
          "Warm coordinated recovery preserves the last healthy A and committed RAM");
  to(restarted, 6);
  require(restarted.cpu_state().pc == entry + 5 &&
              restarted.run_state().retired_instructions == 2 &&
              restarted.memory().cop_state().options == short_options &&
              restarted.memory().cop_state().options_written && expired.faulted(),
          "SOPT's write-once latch genuinely resets;the new native prefix executes independently");
}

void bdc_reset_boundaries() {
  for (unsigned cut = 0; cut <= 4; ++cut) {
    Hcs08Board original(own_image(code), BLOFELD_HCS08_SLA, reference, {});
    constexpr std::array<std::uint8_t, 4> frame{hcs08_bdc::write_byte, 0, ram, data};
    for (unsigned byte = 0; byte < cut; ++byte) { original.receive_bdc(frame[byte]); }
    const Hcs08Board restarted(
        original.reset_checkpoint({.cause = warm, .entry = normal}), BLOFELD_HCS08_SLA, {});
    reset_state(restarted, std::nullopt);
    require(restarted.memory().inspect_ram(ram).known == 0 && original.bdc_state().received == cut,
            "A reset drops partial and complete unserviced native BDC frames before any RAM write");
  }
  Hcs08Board replied(own_image(code), BLOFELD_HCS08_SLA, reference, {});
  command(replied, {hcs08_bdc::read_status});
  Hcs08Board restarted(
      replied.reset_checkpoint({.cause = warm, .entry = normal}), BLOFELD_HCS08_SLA, {});
  reset_state(restarted, std::nullopt);
  require(rejected([&] { static_cast<void>(restarted.read_bdc_reply()); }).contains("no serviced"),
          "A reset cannot replay a pending pre-reset debugger reply");
}

void failed_cpu_and_detached_pads() {
  constexpr std::array<std::uint8_t, 4> program{0xb6, 0x10, 0x20, 0xfe}; // Own LDA PTED;BRA *.
  Hcs08Board original(own_image(program), BLOFELD_HCS08_SLA, reference, {});
  unsigned reads = 0;
  original.attach_pad_source(Hcs08GpioPort::e, [&](const unsigned mask) {
    ++reads;
    return GpioPadSample{.known = mask, .levels = mask & 8U};
  });
  Hcs08Board detached(
      original.reset_checkpoint({.cause = warm, .entry = normal}), BLOFELD_HCS08_SLA, {});
  require(rejected([&] { to(detached, 3); }).contains("Detached resolved GPIO") && reads == 0,
          "Reset snapshots cannot call their source's external pin provider");
  require(rejected([&] {
            static_cast<void>(detached.reset_checkpoint({.cause = warm, .entry = normal}));
          }).contains("healthy completed native boundary"),
          "Warm reset refuses to invent retained A/X after a partially failed CPU transaction");
  Hcs08Board cold_recovered(
      detached.reset_checkpoint({.cause = cold, .entry = normal}), BLOFELD_HCS08_SLA, {});
  cold_recovered.attach_pad_source(Hcs08GpioPort::e, [](const unsigned mask) {
    return GpioPadSample{.known = mask, .levels = mask & 8U};
  });
  to(cold_recovered, 3);
  require(cold_recovered.cpu_state().a == 8 && cold_recovered.cpu_state().pc == entry + 2 &&
              cold_recovered.run_state().retired_instructions == 1 && reads == 0,
          "Cold recovery rebinds explicit pads and executes the original three-period native LDA");
}

void nvm_launch_cuts() {
  constexpr unsigned divider = 0x3d;
  constexpr unsigned target = entry + 0x100;
  constexpr unsigned high = 8;
  constexpr unsigned low = 255;
  const auto image = own_image(code);
  for (unsigned cut = 0; cut <= 4; ++cut) {
    Hcs08Board original(image, BLOFELD_HCS08_SLA, reference, {});
    const std::array<std::array<std::uint8_t, 4>, 4> frames{{
        {hcs08_bdc::write_byte, 0x18, 0x20, divider},
        {hcs08_bdc::write_byte, target >> high, target & low, data},
        {hcs08_bdc::write_byte, 0x18, 0x26, hcs08_flash::program},
        {hcs08_bdc::write_byte, 0x18, 0x25, hcs08_flash::empty},
    }};
    for (unsigned frame = 0; frame < cut; ++frame) {
      for (const auto byte : frames[frame]) { original.receive_bdc(byte); }
      original.service_bdc(Hcs08BdcGrant::reference_instruction_boundary);
    }
    const auto request = Hcs08BoardResetRequest{.cause = cold, .entry = normal};
    const auto snapshot = original.reset_checkpoint(request);
    if (cut < 4) {
      const Hcs08Board settled(snapshot, BLOFELD_HCS08_SLA, {});
      require(settled.memory().flash_image() == image && settled.cpu_state().pc == entry &&
                  settled.memory().flash_state().buffer == Hcs08FlashBuffer::empty &&
                  !settled.memory().flash_state().divider_loaded,
              "Unlaunched staging is volatile;settled FLASH and its actual reset vector survive");
      continue;
    }
    require(original.memory().flash_state().buffer == Hcs08FlashBuffer::launched &&
                original.memory().flash_image() == image &&
                rejected([&] {
                  const Hcs08Board refused(snapshot, BLOFELD_HCS08_SLA, {});
                }).contains("unknown"),
            "A launched interrupted command invalidates NVM certainty before any normal vector");
    Hcs08Board recovery(
        original.reset_checkpoint({.cause = cold, .entry = debug}), BLOFELD_HCS08_SLA, {});
    require(recovery.bdc_state().active && recovery.bdc_state().control == debug_control &&
                !recovery.cpu_state().pc_known && recovery.cpu_state().a_known == 0 &&
                recovery.memory().flash_image().data == image.data &&
                recovery.memory().flash_image().known.none() &&
                recovery.memory().flash_image().fresh.none(),
            "Observed BKGD-low entry needs no invented vector;unknown retention remains unknown");
    Hcs08Board unknown_security(
        original.reset_checkpoint({.cause = cold, .entry = debug}), BLOFELD_HCS08_SLA, {});
    require(rejected([&] {
              command(unknown_security, {hcs08_bdc::write_byte, 0, ram, data});
            }).contains("unknown reset-latched") &&
                unknown_security.memory().inspect_ram(ram).known == 0 && unknown_security.faulted(),
            "Forced debug entry does not guess invalidated SEC bits or grant RAM write success");
    command(recovery, {hcs08_bdc::read_status});
    require(recovery.read_bdc_reply() == debug_status &&
                rejected([&] { command(recovery, {hcs08_bdc::go}); }).contains("unknown CPU PC"),
            "A native recovery status reply grants neither a known launch address nor GO success");
  }
}

void invalid_reset_requests() {
  Hcs08Board board(own_image(code), BLOFELD_HCS08_SLA, reference, {});
  const auto before = board.cpu_state();
  require(rejected([&] {
            static_cast<void>(
                board.reset_checkpoint({.cause = Hcs08ResetCause::unqualified, .entry = normal}));
          }).contains("architectural reset cause") &&
              rejected([&] {
                static_cast<void>(
                    board.reset_checkpoint({.cause = warm, .entry = Hcs08ResetEntry::unqualified}));
              }).contains("sampled reset entry") &&
              rejected([&] {
                static_cast<void>(
                    board.reset_checkpoint({.cause = warm, .entry = normal, .reset_sources = 1}));
              }).contains("SRS bit0") &&
              !board.faulted() && board.cpu_state() == before,
          "Invalid reset contracts fail before mutating the source board");
}
} // namespace

int main() {
  try {
    native_reset_cuts();
    watchdog_recovery();
    bdc_reset_boundaries();
    failed_cpu_and_detached_pads();
    nvm_launch_cuts();
    invalid_reset_requests();
    std::cout << "Verified whole-board warm/cold reset,watchdog recovery,BDC cuts and strict NVM "
                 "uncertainty.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
