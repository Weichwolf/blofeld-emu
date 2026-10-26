// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "hcs08_board_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_icg.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_system_options.hpp"
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
namespace fixture = blofeld::test::board;
constexpr unsigned timer_vector = 0xffee, spi_vector = 0xffe0, adc_vector = 0xffd0;
constexpr unsigned timer_bit = 4, spi_bit = 2, adc_bit = 1, all_sources = 7;
constexpr unsigned timer_run = 8, irq_enable = 0x40, timer_wrap = 65536;
constexpr unsigned spi_enabled = 0x74, spi_quiet = 0x54, adc_format = 0xc2;
constexpr unsigned cop_disabled = 0x11, analog_uv = 1650000, supply_uv = 3300000;
constexpr unsigned bus_hz = 12000000, adc_result = 512, unknown_i = 0xf7;
constexpr std::uint8_t nop = 0x9d;
constexpr auto observed = Hcs08BoardClockObservation::no_fault;
constexpr ExecutionContext context{.pc = 0xe055, .ccr = 0x60};
constexpr Hcs08State unmasked{.pc = 0xe055, .ccr = 0x60};
constexpr Hcs08State masked{.pc = 0xe055, .ccr = 0x68};
constexpr Hcs08State uncertain_i{.pc = 0xe055, .ccr_known = unknown_i, .ccr = 0x60};

std::optional<InterruptRequest> expected(const unsigned sources) {
  // Independent literal vector/priority expectations from GT Table5-1,p68.
  if ((sources & timer_bit) != 0) {
    return InterruptRequest{.vector = timer_vector, .kind = InterruptKind::maskable};
  }
  if ((sources & spi_bit) != 0) {
    return InterruptRequest{.vector = spi_vector, .kind = InterruptKind::maskable};
  }
  if ((sources & adc_bit) != 0) {
    return InterruptRequest{.vector = adc_vector, .kind = InterruptKind::maskable};
  }
  return {};
}

bool same_request(const std::optional<InterruptRequest> &first,
                  const std::optional<InterruptRequest> &second) {
  return first.has_value() == second.has_value() &&
         (!first || (first->vector == second->vector && first->kind == second->kind));
}

Hcs08BoardMemory pending(const unsigned sources) {
  constexpr std::array code{nop};
  Hcs08BoardMemory board(fixture::own_image(code),
                         Hcs08BoardMemoryContract::candidate_gt8a_reference_bus);
  board.advance_reference_bus(context, observed);
  board.write(Hcs08SystemOptions::address, cop_disabled);
  board.observe_timer_phase(0);
  board.observe_adc_reference(
      {.bus_hz = bus_hz, .supply_uv = supply_uv, .low_uv = 0, .high_uv = supply_uv});
  board.observe_adc_input(0, analog_uv);
  board.write(hcs08_adc::pins, 1);
  board.write(hcs08_adc::control, adc_format);
  board.write(hcs08_adc::status, (sources & adc_bit) != 0 ? irq_enable : 0);
  board.write(hcs08_spi::control1, (sources & spi_bit) != 0 ? spi_enabled : spi_quiet);
  board.write(Hcs08Tpm::status, timer_run | ((sources & timer_bit) != 0 ? irq_enable : 0));
  for (unsigned period = 0; period < timer_wrap; ++period) {
    board.advance_reference_bus(context, observed);
  }
  require(board.timer_state().counter == 0 && board.timer_state().overflow &&
              board.adc_state().complete && board.adc_state().conversions == 1 &&
              board.adc_state().result == adc_result && !board.spi_state().transmit &&
              !board.spi_state().active,
          "Only real 65536 timer edges and one actual stable-input conversion establish events");
  return board;
}

void source_clear(Hcs08BoardMemory &board, const unsigned vector) {
  if (vector == timer_vector) {
    board.write(Hcs08Tpm::status, timer_run | irq_enable);
    require(board.timer_state().overflow && !board.timer_state().overflow_read,
            "TOF cannot clear without its actual preceding status read");
    static_cast<void>(board.read(Hcs08Tpm::status));
    board.write(Hcs08Tpm::status, timer_run | irq_enable);
    require(!board.timer_state().overflow, "Real status-read/write-zero sequence clears TOF");
  } else if (vector == spi_vector) {
    board.write(hcs08_spi::control1, spi_quiet);
  } else {
    require(vector == adc_vector && board.read(hcs08_adc::high) == 2 &&
                !board.adc_state().complete && board.read(hcs08_adc::low) == 0,
            "Actual result reads acknowledge literal right-aligned ADC512 without injected data");
  }
}

void all_priorities() {
  for (unsigned sources = 0; sources <= all_sources; ++sources) {
    auto board = pending(sources);
    const auto original = board;
    require(same_request(board.interrupt_request(unmasked), expected(sources)) &&
                same_request(board.interrupt_request(uncertain_i), expected(sources)) &&
                !board.interrupt_request(masked),
            "All eight real flag/enable combinations obey literal TPM1>SPI>ADC and known CPU I");
    fixture::same_memory(board, original);
    auto copied = board;
    require(same_request(copied.interrupt_request(unmasked), expected(sources)),
            "Copied simultaneous pending events retain priority without a source callback");
    unsigned remaining = sources;
    while (const auto request = expected(remaining)) {
      require(same_request(board.interrupt_request(unmasked), request),
              "Clearing the selected source exposes the next genuine peripheral request");
      const auto before = board;
      board.begin_interrupt(*request, false);
      board.acknowledge_interrupt(request->vector);
      fixture::same_memory(board, before);
      require(same_request(board.interrupt_request(unmasked), request),
              "CPU vector/frame acceptance never clears peripheral IRQ flags or arms a read");
      source_clear(board, request->vector);
      if (request->vector == timer_vector) {
        remaining &= ~timer_bit;
      } else if (request->vector == spi_vector) {
        remaining &= ~spi_bit;
      } else {
        remaining &= ~adc_bit;
      }
    }
    require(!board.interrupt_request(unmasked) && board.flash_image() == original.flash_image(),
            "Only each actual device acknowledgement removes its request;NVM stays exact");
  }
}

void strict_controls() {
  auto board = pending(all_sources);
  constexpr unsigned invalid_vector = 0xffe2;
  const InterruptRequest invalid{.vector = invalid_vector, .kind = InterruptKind::maskable};
  require(rejected([&] { board.begin_interrupt(invalid, false); }).contains("unsupported source") &&
              rejected([&] {
                board.acknowledge_interrupt(invalid_vector);
              }).contains("source is absent") &&
              rejected([&] {
                board.begin_interrupt(*expected(all_sources), true);
              }).contains("frame reuse"),
          "Unimplemented TPM2 vector and stack reuse still fail before device acknowledgements");
  for (const auto vector : {timer_vector, spi_vector, adc_vector}) {
    auto absent = pending(0);
    const InterruptRequest request{.vector = static_cast<std::uint16_t>(vector),
                                   .kind = InterruptKind::maskable};
    require(
        rejected([&] { absent.begin_interrupt(request, false); }).contains("unsupported source") &&
            rejected([&] {
              absent.acknowledge_interrupt(request.vector);
            }).contains("source is absent"),
        "Correct vector addresses never create absent or disabled source events");
  }
  board.advance_reference_bus(context, Hcs08BoardClockObservation::loss_of_lock);
  const auto failure = rejected([&] { static_cast<void>(board.interrupt_request(unmasked)); });
  require(
      failure.contains("coordinated service required") && failure.contains("pc=0xe055") &&
          !board.interrupt_request(masked),
      "ICG IRQ remains a separate visible unsupported owner,not hidden by lower-priority events");
  auto unknown = pending(all_sources);
  unknown.begin_instruction(context);
  require(rejected([&] {
            static_cast<void>(unknown.interrupt_request(unmasked));
          }).contains("IRQ/reset selection"),
          "Unobserved ICG fault histories still refuse native source selection");
  for (const auto cause : {Hcs08IcgReset::warm, Hcs08IcgReset::power_on}) {
    auto reset = pending(all_sources);
    reset.write(fixture::ram, adc_result >> 8U);
    const auto image = reset.flash_image();
    reset.reset_peripherals(cause, std::nullopt);
    require(!reset.interrupt_request(unmasked) && !reset.timer_state().overflow &&
                !reset.adc_state().complete && reset.flash_image() == image &&
                reset.inspect_ram(fixture::ram).known ==
                    (cause == Hcs08IcgReset::warm ? fixture::byte_mask : 0),
            "Logical warm/cold reset removes actual pending flags while preserving exact NVM and "
            "RAM reset rules");
  }
}
} // namespace

int main() {
  try {
    all_priorities();
    strict_controls();
    std::cout << "Verified actual ADC/TPM1/SPI flags,enables,literal priority and strict source "
                 "guards.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
