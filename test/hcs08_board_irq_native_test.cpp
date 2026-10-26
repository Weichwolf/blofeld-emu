// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "board/hcs08_board_serial.hpp"
#include "common/bus.hpp"
#include "hcs08_board_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"
#include "keyboard/hcs08_tpm.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::hcs08::require;
namespace fixture = blofeld::test::board;
constexpr unsigned timer_isr = 0xe100, spi_isr = 0xe110, adc_isr = 0xe120;
constexpr unsigned timer_vector = 0xffee, spi_vector = 0xffe0, adc_vector = 0xffd0;
constexpr unsigned marker_high = 0x80, marker_low = 0x81, byte_mask = 0xff;
constexpr unsigned spi_enabled = 0x74, spi_quiet = 0x54, adc_format = 0xc2;
constexpr unsigned timer_enabled = 0x48, timer_stopped = 0x40, cop_disabled = 0x11;
constexpr unsigned supply_uv = 3300000, analog_uv = 1650000, bus_hz = 12000000;
constexpr unsigned stack = 0x47f, cold_h_known = 0xff00, flags_known = 0xef, caller_flags = 0x61;
constexpr unsigned boot_bytes = 36, foreground_end = 65578, endpoint = 65663, irq_periods = 33;
constexpr unsigned program_periods = 52, foreground_retired = 13124, loop_pc = 0xe022;
constexpr unsigned timer_counter = 23, adc_code = 512;
constexpr auto observed = Hcs08BoardClockObservation::no_fault;
constexpr std::array<unsigned, 18> cuts{0,
                                        1,
                                        35,
                                        36,
                                        201,
                                        202,
                                        65570,
                                        65571,
                                        65572,
                                        65576,
                                        65577,
                                        65578,
                                        65579,
                                        65589,
                                        65595,
                                        65606,
                                        65617,
                                        65642};

namespace op {
constexpr std::uint8_t ldhx = 0x45, txs = 0x94, lda = 0xa6, store_ext = 0xc7;
constexpr std::uint8_t store = 0xb7, load = 0xb6, brclr7 = 0x0f, cli = 0x9a;
constexpr std::uint8_t nop = 0x9d, bra = 0x20, rti = 0x80, back_three = 0xfd, back_two = 0xfe;
constexpr std::uint8_t options_high = 0x18;
}

Hcs08FlashImage own_program() {
  // Own code only. Native reset/setup runs through the real first timer wrap.
  // 36 setup periods + 13108*5 BRCLR periods + CLI/NOP = 65578 foreground periods.
  constexpr std::array<std::uint8_t, boot_bytes> boot{op::ldhx,
                                                      4,
                                                      0x80,
                                                      op::txs,
                                                      op::lda,
                                                      cop_disabled,
                                                      op::store_ext,
                                                      op::options_high,
                                                      2,
                                                      op::lda,
                                                      1,
                                                      op::store,
                                                      hcs08_adc::pins,
                                                      op::lda,
                                                      adc_format,
                                                      op::store,
                                                      hcs08_adc::control,
                                                      op::lda,
                                                      timer_stopped,
                                                      op::store,
                                                      hcs08_adc::status,
                                                      op::lda,
                                                      spi_enabled,
                                                      op::store,
                                                      hcs08_spi::control1,
                                                      op::lda,
                                                      timer_enabled,
                                                      op::store,
                                                      Hcs08Tpm::status,
                                                      op::brclr7,
                                                      Hcs08Tpm::status,
                                                      op::back_three,
                                                      op::cli,
                                                      op::nop,
                                                      op::bra,
                                                      op::back_two};
  auto image = fixture::own_image(boot);
  const auto put = [&](const unsigned address, const std::uint8_t value) {
    const auto index = address - hcs08_flash::first;
    image.data.at(index) = value;
    image.fresh.reset(index);
  };
  const auto install = [&](const unsigned address, const auto &bytes) {
    for (unsigned index = 0; index < bytes.size(); ++index) { put(address + index, bytes[index]); }
  };
  // Nominal ISR budgets:3+2+3+9=17;2+3+9=14;3+3+3+3+9=21.
  constexpr std::array timer{op::load,
                             std::uint8_t{Hcs08Tpm::status},
                             op::lda,
                             std::uint8_t{timer_stopped},
                             op::store,
                             std::uint8_t{Hcs08Tpm::status},
                             op::rti,
                             op::nop,
                             op::nop};
  constexpr std::array spi{op::lda,
                           std::uint8_t{spi_quiet},
                           op::store,
                           std::uint8_t{hcs08_spi::control1},
                           op::rti,
                           op::nop,
                           op::nop};
  constexpr std::array adc{op::load,
                           std::uint8_t{hcs08_adc::high},
                           op::store,
                           std::uint8_t{marker_high},
                           op::load,
                           std::uint8_t{hcs08_adc::low},
                           op::store,
                           std::uint8_t{marker_low},
                           op::rti,
                           op::nop,
                           op::nop};
  install(timer_isr, timer);
  install(spi_isr, spi);
  install(adc_isr, adc);
  for (const auto pair : {std::array{timer_vector, timer_isr},
                          std::array{spi_vector, spi_isr},
                          std::array{adc_vector, adc_isr}}) {
    put(pair[0], static_cast<std::uint8_t>(pair[1] >> 8U));
    put(pair[0] + 1, static_cast<std::uint8_t>(pair[1] & byte_mask));
  }
  return image;
}

Hcs08BoardCheckpoint reset(const Hcs08FlashImage &image) {
  Hcs08BoardInputs inputs;
  inputs.adc_reference = Hcs08AdcReference{
      .bus_hz = bus_hz, .supply_uv = supply_uv, .low_uv = 0, .high_uv = supply_uv};
  inputs.analog[0] = analog_uv;
  inputs.timer_phase = 0;
  inputs.reset_observation = observed;
  const Hcs08Board board(image,
                         BLOFELD_HCS08_SLA,
                         fixture::reference,
                         inputs,
                         Hcs08RtiIrqContract::reference_immediate);
  require(board.cpu_state().pc == fixture::entry && board.cpu_state().a_known == 0 &&
              board.cpu_state().hx_known == cold_h_known && board.cpu_state().sp == byte_mask &&
              board.run_state().retired_instructions == 0 && board.state().completed_periods == 0,
          "Only real cold vector/H/SP/I reset precedes the own native setup;A/X remain unknown");
  return board.checkpoint();
}

struct Entry {
  unsigned period, pc;
  bool operator==(const Entry &) const = default;
};

constexpr std::array<Entry, 3> entries{{{.period = 65589, .pc = timer_isr},
                                        {.period = 65617, .pc = spi_isr},
                                        {.period = 65642, .pc = adc_isr}}};

struct Machine {
  std::optional<Hcs08BoardSerial> board;
  std::vector<Entry> entries;
};

void to(Machine &machine, const unsigned period) {
  if (!machine.board) { throw Fault("Native test owner is absent"); }
  auto &board = machine.board.value();
  while (board.board_state().completed_periods < period) {
    const auto before = board.run_state().irq_periods;
    require(!board.advance_reference_period(std::nullopt, observed),
            "Empty TX-buffer IRQ supplies no invented serial edge or MISO sample");
    const auto after = board.run_state();
    constexpr unsigned vector_budget = 11;
    if (after.irq_periods != before && after.irq_periods % vector_budget == 0) {
      machine.entries.push_back(
          {.period = static_cast<unsigned>(after.completed_periods), .pc = board.cpu_state().pc});
    }
  }
}

void endpoint_checks(const Machine &machine, const Hcs08FlashImage &image) {
  if (!machine.board) { throw Fault("Native test owner is absent"); }
  const auto &board = machine.board.value();
  require(machine.entries == std::vector<Entry>(entries.begin(), entries.end()),
          "Literal real vector-read completion periods establish TPM1,SPI,ADC in that order");
  require(board.cpu_state() == Hcs08State{.pc = loop_pc,
                                          .hx = stack + 1,
                                          .sp = stack,
                                          .a = timer_enabled,
                                          .ccr_known = flags_known,
                                          .ccr = caller_flags} &&
              board.run_state().foreground_periods == foreground_end &&
              board.foreground_state().retired_instructions == foreground_retired &&
              board.run_state().irq_periods == irq_periods &&
              board.run_state().program_periods == program_periods &&
              board.run_state().completed_periods == endpoint &&
              board.board_state().completed_periods == endpoint,
          "Independent reset/setup,BRCLR,three11-period entries,ISR costs and actual RTIs restore "
          "the caller");
  const auto timer = board.memory().timer_state();
  const auto adc = board.memory().adc_state();
  require(timer.control == timer_stopped && !timer.overflow && timer.counter == timer_counter &&
              !timer.prescaler_phase && adc.result == adc_code && adc.conversions == 1 &&
              !adc.complete && !adc.active && board.memory().spi_state().control1 == spi_quiet &&
              board.memory().inspect_ram(marker_high).value == 2 &&
              board.memory().inspect_ram(marker_high).known == byte_mask &&
              board.memory().inspect_ram(marker_low).value == 0 &&
              board.memory().inspect_ram(marker_low).known == byte_mask &&
              board.clock_state().generated_edges == 0 && board.memory().flash_image() == image,
          "Only native ISR read/write sequences clear flags,stop TPM and publish literal "
          "ADC512;NVM stays exact");
}

struct Snapshot {
  Hcs08BoardSerialCheckpoint board;
  std::vector<Entry> entries;
};

void run(const Hcs08SpiRunContract order) {
  const auto image = own_program();
  Machine original{
      .board = std::optional<Hcs08BoardSerial>(
          std::in_place,
          reset(image),
          BLOFELD_HCS08_SLA,
          Hcs08BoardSerialContract{
              .order = order,
              .launch = Hcs08SpiLaunchContract::reference_one_period,
              .program = Hcs08PrefetchContract::reference_subroutine_program_cycles,
              .foreground = Hcs08SpiForegroundContract::reference_instruction_retirement}),
      .entries = {}};
  std::vector<Snapshot> saved;
  for (const auto cut : cuts) {
    to(original, cut);
    saved.push_back({.board = original.board.value().checkpoint(), .entries = original.entries});
  }
  to(original, endpoint);
  endpoint_checks(original, image);
  const auto end = original.board.value().checkpoint();
  original.board.reset();
  const Hcs08BoardSerial expected(end, BLOFELD_HCS08_SLA);
  for (const auto &snapshot : saved) {
    for (unsigned copy = 0; copy < 2; ++copy) {
      Machine fresh{.board = std::optional<Hcs08BoardSerial>(
                        std::in_place, snapshot.board, BLOFELD_HCS08_SLA),
                    .entries = snapshot.entries};
      to(fresh, endpoint);
      endpoint_checks(fresh, image);
      require(fresh.board.value().cpu_state() == expected.cpu_state() &&
                  fresh.board.value().board_state() == expected.board_state() &&
                  fresh.board.value().run_state() == expected.run_state() &&
                  fresh.board.value().foreground_state() == expected.foreground_state() &&
                  fresh.board.value().bdc_state() == expected.bdc_state() &&
                  fresh.board.value().clock_state() == expected.clock_state(),
              "Destroyed-owner fresh suffixes preserve all CPU/clock/IRQ/program/BDC owner states");
      fixture::same_memory(fresh.board.value().memory(), expected.memory());
    }
  }
}
} // namespace

int main() {
  try {
    run(Hcs08SpiRunContract::reference_cpu_before_spi);
    run(Hcs08SpiRunContract::reference_spi_before_cpu);
    std::cout << "Verified native cold ADC/TPM1/SPI priority,real ISR acknowledgements and fresh "
                 "joint suffixes.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
