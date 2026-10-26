// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_cop.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_icg.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_tpm.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr auto contract = Hcs08BoardMemoryContract::candidate_gt8a_untimed;
constexpr unsigned ram = 0x80, last_ram = 0x47f, first_hole = 0x480;
constexpr unsigned byte = 0xa5, partial = 0xa0, high_bits = 0xf0, live_trim = 0x5a;
constexpr unsigned nop = 0x9d, port_e_mask = 0x3f, master_enable = 0x50, adc_format = 0xc2;
constexpr unsigned last_address = 0xffff;
constexpr ExecutionContext context{.pc = 0xe123, .ccr = 0x69};

Hcs08BoardMemory memory() {
  Hcs08BoardMemory result(Hcs08FlashImage{}, contract);
  result.begin_instruction(context);
  return result;
}

void ram_and_reset() {
  auto board = memory();
  for (unsigned address = ram; address <= last_ram; ++address) {
    const auto value = board.read_known_byte(static_cast<std::uint16_t>(address));
    require(value.known == 0 && value.value == 0,
            "Every physical SRAM byte starts unknown; canonical payload is not a reset value");
  }
  board.write(ram, byte);
  board.write_known_byte({.address = last_ram, .value = byte}, high_bits);
  require(board.read(ram) == byte && board.read_known_byte(last_ram).known == high_bits &&
              board.read_known_byte(last_ram).value == partial,
          "Mapped first/last RAM stores retain literal values and exact partial masks");
  board.write(hcs08_icg::trim, live_trim);
  board.reset_peripherals(Hcs08IcgReset::warm, std::nullopt);
  require(board.read(ram) == byte && board.read_known_byte(last_ram).known == high_bits &&
              board.icg_state().trim == live_trim && !board.cop_state().reset_status,
          "Warm reset preserves actual SRAM/trim without supplying an unknown reset source");
  board.reset_peripherals(Hcs08IcgReset::power_on, 0x80);
  require(board.read_known_byte(ram).known == 0 && board.read_known_byte(last_ram).known == 0 &&
              board.icg_state().trim == 0x80 && board.cop_state().reset_status == 0x80,
          "Power-on invalidates RAM and uses documented live trim and explicit SRS");
  auto fresh = memory();
  require(rejected([&] { static_cast<void>(fresh.read(ram)); }).contains("unknown SRAM bits") &&
              fresh.faulted(),
          "Ordinary data reads never turn unknown SRAM into zero");
}

void code_permissions() {
  auto board = memory();
  const auto before = board.code_revision();
  board.write(ram, byte);
  require(board.code_revision() == before, "Untranslated data writes retain the code revision");
  board.note_code_byte(ram);
  board.write(ram, nop); // Own native NOP, not manufacturer bytes.
  require(board.code_revision() == before + 1 && board.peek(ram) == nop,
          "A write to actual decoder lookahead invalidates the native translation identity");
  board.check_execute({.address = ram, .length = 1});
  board.write_known_byte({.address = ram, .value = byte}, high_bits);
  require(board.peek(ram) == 0 && rejected([&] {
                                    board.check_execute({.address = ram, .length = 1});
                                  }).contains("unknown SRAM opcode bits"),
          "Decoder zero lookahead cannot become a consumed partially known opcode");
  require(rejected([&] {
            board.check_execute({.address = last_address, .length = 2});
          }).contains("wrapping instruction"),
          "Execution permissions refuse a real wrapping instruction span");
}

void holes_and_masks() {
  constexpr std::array holes{0x20U, 0x7fU, first_hole, 0x1801U, 0x1822U, 0xdfffU};
  for (const auto address : holes) {
    for (const bool writing : {false, true}) {
      auto board = memory();
      const auto access = static_cast<std::uint16_t>(address);
      const auto failure = rejected([&] {
        if (writing) {
          board.write(access, byte);
        } else {
          static_cast<void>(board.read_known_byte(access));
        }
      });
      require(failure.contains("address=" + hex(address)) && failure.contains("pc=0xe123") &&
                  failure.contains("ccr=0x0069") && board.faulted(),
              "Unsupported physical locations fail with actual operation/address/CPU context");
      require(rejected([&] { board.write(ram, byte); }).contains("coordinated recovery"),
              "A failed board cannot resume on a different apparently valid access");
    }
  }
  auto board = memory();
  board.write_known_byte({.address = Hcs08Cop::status, .value = byte}, 0);
  require(board.cop_state().reset_status == 0x80 && board.cop_state().elapsed == 0,
          "Actual SRS service ignores every payload bit without inventing its value");
  require(rejected([&] {
            board.write_known_byte({.address = hcs08_adc::control, .value = byte}, high_bits);
          }).contains("unknown peripheral operand bits") &&
              board.adc_state().control == 0,
          "Other unknown peripheral operands fail before device state changes");
}

void strict_live_devices() {
  auto timer = memory();
  timer.write(Hcs08Tpm::status, 0xf);
  require(rejected([&] {
            static_cast<void>(timer.read(Hcs08Tpm::count_high));
          }).contains("live TPM requires elapsed bus periods"),
          "No stopped fake counter substitutes for a missing timer clock");
  auto adc = memory();
  adc.write(hcs08_adc::control, adc_format);
  adc.write(hcs08_adc::status, 0);
  require(adc.adc_state().active && rejected([&] {
                                      static_cast<void>(adc.read(hcs08_adc::status));
                                    }).contains("live ADC requires elapsed periods") &&
              adc.adc_state().conversions == 0 && !adc.adc_state().result,
          "An untimed ADC poll cannot assert a frozen incomplete or successful conversion");
  auto spi = memory();
  spi.write(hcs08_spi::control1, master_enable);
  static_cast<void>(spi.read(hcs08_spi::status));
  spi.write(hcs08_spi::data, byte);
  require(rejected([&] {
            static_cast<void>(spi.read(hcs08_spi::status));
          }).contains("live SPI requires qualified transfer"),
          "An untimed queued SPI byte never becomes a fabricated ready response");
  auto flash = memory();
  const auto image = flash.flash_image();
  require(rejected([&] {
            flash.write(hcs08_flash::status, Hcs08KnownByte::all);
          }).contains("unqualified idle FCBEF launch") &&
              flash.flash_image() == image,
          "Original idle FSTATFF remains strict with all NVM bits unchanged");
}

void live_flash() {
  Hcs08FlashImage image;
  image.data.fill(Hcs08KnownByte::all);
  image.known.set();
  image.fresh.set();
  image.data[hcs08_flash::nv_options - hcs08_flash::first] = 2;
  image.fresh.reset(hcs08_flash::nv_options - hcs08_flash::first);
  Hcs08BoardMemory board(image, contract);
  board.begin_instruction(context);
  board.write(ram, byte);
  constexpr unsigned divider = 0x3d;
  board.write(hcs08_flash::divider, divider);
  board.write(hcs08_flash::first, byte);
  board.write(hcs08_flash::command, hcs08_flash::program);
  board.write(hcs08_flash::status, hcs08_flash::empty);
  require(rejected([&] {
            static_cast<void>(board.read(hcs08_flash::status));
          }).contains("live FLASH requires a qualified command clock") &&
              board.faulted() && board.flash_image() == image,
          "An accepted command launch cannot become an indefinitely frozen or completed NVM job");
  board.reset_peripherals(Hcs08IcgReset::warm, std::nullopt);
  require(
      !board.faulted() && board.read(ram) == byte && board.flash_image().data == image.data &&
          board.flash_image().known.none() && board.flash_image().fresh.none() &&
          !board.cop_state().reset_status,
      "Coordinated warm reset drops pending NVM certainty,retains SRAM and supplies no SRS cause");
}

void pad_copies_and_irq_context() {
  auto original = memory();
  unsigned samples = 0;
  original.attach_pad_source(Hcs08GpioPort::e, [&](const unsigned mask) {
    ++samples;
    return GpioPadSample{.known = mask, .levels = mask};
  });
  auto fresh = original;
  constexpr unsigned port_e = 0x10; // Candidate GT8A register map: PTED,not PTCD ($0008).
  require(original.read(port_e) == port_e_mask && samples == 1,
          "Only actual bound external observations establish implemented Port E input bits");
  static_cast<void>(rejected([&] { static_cast<void>(fresh.read(port_e)); }));
  require(samples == 1 && fresh.faulted(), "A copied owner cannot call the source board's pads");
  auto board = memory();
  constexpr Hcs08State query{.pc = 0x8765, .ccr = 0x60};
  const auto failure = rejected([&] { static_cast<void>(board.interrupt_request(query)); });
  require(failure.contains("pc=0x8765") && failure.contains("IRQ/reset selection"),
          "Uncertain native source selection reports actual query context,not an old bus PC");
  constexpr Hcs08State unknown{.ccr_known = 0xf7, .ccr = 0x60, .pc_known = false};
  const auto unknown_failure =
      rejected([&] { static_cast<void>(board.interrupt_request(unknown)); });
  require(
      unknown_failure.starts_with("HCS08 board IRQ query address=0x004a pc=unknown pc-known=0") &&
          unknown_failure.contains("ccr-known=0x00f7"),
      "The source diagnostic distinguishes unknown PC/I from canonical zero payload bits");
}
} // namespace

int main() {
  try {
    ram_and_reset();
    code_permissions();
    holes_and_masks();
    strict_live_devices();
    live_flash();
    pad_copies_and_irq_context();
    std::cout
        << "Verified candidate board RAM,permissions,reset,pad copies and strict device gates.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
