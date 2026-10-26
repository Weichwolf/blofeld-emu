// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "board/hcs08_board_serial.hpp"
#include "common/bus.hpp"
#include "hcs08_board_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_clock.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
namespace fixture = blofeld::test::board;
namespace reg = hcs08_spi;
constexpr unsigned entry = 0xe000, foreground = entry + 10, handler = 0xe100, vector = 0xffe0;
constexpr unsigned enabled = 0x74, quiet = 0x54, reply = 0x5a, pattern = 0xa5;
constexpr unsigned write_period = 17, return_period = 32, final_edge = 16, irq_budget = 11;
constexpr auto observed = Hcs08BoardClockObservation::no_fault;
constexpr unsigned end_period = 38, stack = 0x47f;
// Own LDHX #$0480;TXS;LDA #$74;STA SPIC1;CLI;NOP;BRA *.
constexpr std::array<std::uint8_t, 12> boot{
    0x45, 4, 0x80, 0x94, 0xa6, enabled, 0xb7, reg::control1, 0x9a, 0x9d, 0x20, 0xfe};
constexpr std::array<unsigned, 8> cuts{0, 10, 11, 17, 18, 21, 31, 33};

Hcs08FlashImage image(const unsigned byte, const std::uint8_t after = quiet) {
  auto result = fixture::own_image(boot);
  // Own LDA SPIS;LDA #byte;STA SPID;MOV #$54,SPIC1;RTI;NOP;NOP.
  const std::array<std::uint8_t, 12> code{0xb6,
                                          reg::status,
                                          0xa6,
                                          static_cast<std::uint8_t>(byte),
                                          0xb7,
                                          reg::data,
                                          0x6e,
                                          after,
                                          reg::control1,
                                          0x80,
                                          0x9d,
                                          0x9d};
  const auto put = [&](const unsigned address, const std::uint8_t value) {
    result.data.at(address - hcs08_flash::first) = value;
    result.fresh.reset(address - hcs08_flash::first);
  };
  for (unsigned n = 0; n < code.size(); ++n) { put(handler + n, code[n]); }
  put(vector, handler >> 8U);
  put(vector + 1, handler & fixture::byte_mask);
  return result;
}

Hcs08BoardCheckpoint
boundary(const Hcs08FlashImage &program,
         const Hcs08RtiIrqContract rti = Hcs08RtiIrqContract::reference_immediate,
         const bool unknown_first = false,
         const bool unknown_reset = false) {
  Hcs08BoardInputs inputs;
  inputs.reset_observation = unknown_reset ? Hcs08BoardClockObservation::unobserved : observed;
  Hcs08Board board(program, BLOFELD_HCS08_SLA, fixture::reference, inputs, rti);
  for (unsigned n = 0; board.cpu_state().pc != foreground; ++n) {
    require(n < 32, "Own native reset/setup reaches its pending SPI IRQ boundary");
    board.advance_reference_period(unknown_first && n == 0 ? Hcs08BoardClockObservation::unobserved
                                                           : observed);
  }
  require(board.run_state().remaining_periods == 0 && board.cpu_state().sp == stack &&
              board.cpu_state().hx == stack + 1 && board.cpu_state().a == enabled &&
              board.memory().spi_state().control1 == enabled &&
              !board.memory().spi_state().transmit &&
              board.memory().periods().completed == board.state().completed_periods &&
              board.memory().cop_state().elapsed == board.state().completed_periods &&
              board.memory().flash_image() == program,
          "Real reset and six own instructions initialize CPU/stack/SPI without target setters");
  return board.checkpoint();
}

struct Event {
  unsigned period;
  Hcs08SpiPins pins;
  bool operator==(const Event &) const = default;
};

struct Peer {
  unsigned samples = 0, transmitted = 0;
  std::vector<Event> events;

  void tick(Hcs08BoardSerial &board) {
    const auto bit = samples < 8 ? 7 - samples : 0;
    const bool input = ((reply >> bit) & 1U) != 0;
    const auto pins = board.advance_reference_period(input, observed);
    if (!pins) { return; }
    events.push_back(
        {.period = static_cast<unsigned>(board.run_state().completed_periods), .pins = *pins});
    if (pins->clock) { return; }
    if (!pins->mosi.has_value()) { throw Fault("Missing real MOSI at a trailing sample edge"); }
    transmitted = (transmitted << 1U) | static_cast<unsigned>(pins->mosi.value());
    ++samples;
  }
};

unsigned delay(const Hcs08BoardSerialContract contract) {
  return contract.launch == Hcs08SpiLaunchContract::reference_one_period ? 1U : 2U;
}

void check_period(const Hcs08BoardSerial &board,
                  const Hcs08BoardSerialContract contract,
                  const Hcs08State initial,
                  const std::uint64_t base) {
  const auto period = board.run_state().completed_periods;
  const auto launch = write_period + delay(contract);
  const auto elapsed = period >= launch ? period - launch + 1 : 0;
  const auto edges = std::min<std::uint64_t>(elapsed, final_edge);
  require(board.board_state().completed_periods == base + period &&
              board.board_state().attempted_periods == base + period &&
              board.memory().periods().completed == base + period &&
              board.memory().cop_state().elapsed == base + period &&
              board.run_state().irq_periods == std::min<std::uint64_t>(period, irq_budget) &&
              board.run_state().spi_periods == period &&
              board.clock_state().offered_periods == period &&
              board.clock_state().generated_edges == edges &&
              board.run_state().launches == static_cast<unsigned>(period >= launch) &&
              board.clock_state().completed_transfers ==
                  static_cast<unsigned>(elapsed >= final_edge) &&
              board.memory().spi_state().receive_full == (elapsed >= final_edge),
          "Literal IRQ11,store17,one/two-stage launch and16 divide-two edges share real device "
          "periods");
  if (period >= return_period) {
    require(board.cpu_state() == initial,
            "Actual native RTI restores the reset-derived complete caller knowledge");
  }
}

void run_case(const Hcs08BoardSerialContract contract, const unsigned byte) {
  const auto program = image(byte);
  const auto source = boundary(program);
  const Hcs08Board state(source, BLOFELD_HCS08_SLA);
  const auto initial = state.cpu_state();
  const auto base = state.state().completed_periods;
  Hcs08BoardSerial board(source, BLOFELD_HCS08_SLA, contract);
  Peer peer;
  for (unsigned n = 0; n < end_period; ++n) {
    peer.tick(board);
    check_period(board, contract, initial, base);
  }
  require(peer.transmitted == byte && peer.samples == 8 && peer.events.size() == final_edge &&
              board.memory().spi_state().receive == reply &&
              board.memory().spi_state().control1 == quiet &&
              board.memory().flash_image() == program,
          "Actual CPU SPID writes and16 pad transitions form both independently known full bytes");
}

void continuations(const Hcs08BoardSerialContract contract) {
  constexpr unsigned byte = 0xa5;
  const auto program = image(byte);
  const auto source = boundary(program);
  const Hcs08Board initial_board(source, BLOFELD_HCS08_SLA);
  const auto initial = initial_board.cpu_state();
  const auto base = initial_board.state().completed_periods;
  for (const auto cut : cuts) {
    auto original = std::make_unique<Hcs08BoardSerial>(source, BLOFELD_HCS08_SLA, contract);
    Peer prefix;
    for (unsigned n = 0; n < cut; ++n) { prefix.tick(*original); }
    const auto saved = original->checkpoint();
    auto expected = prefix;
    for (unsigned n = cut; n < end_period; ++n) { expected.tick(*original); }
    const auto cpu = original->cpu_state();
    const auto run = original->run_state();
    const auto clock = original->clock_state();
    const auto memory = original->memory();
    original.reset();
    for (unsigned replay = 0; replay < 2; ++replay) {
      Hcs08BoardSerial fresh(saved, BLOFELD_HCS08_SLA);
      auto actual = prefix;
      for (unsigned n = cut; n < end_period; ++n) {
        actual.tick(fresh);
        check_period(fresh, contract, initial, base);
      }
      require(actual.events == expected.events && actual.samples == expected.samples &&
                  actual.transmitted == byte && fresh.cpu_state() == cpu &&
                  fresh.run_state() == run && fresh.clock_state() == clock,
              "Destroyed original owners and two fresh whole-board copies reproduce every actual "
              "edge and CPU state");
      fixture::same_memory(fresh.memory(), memory);
    }
  }
}

void unknown_input(const Hcs08BoardSerialContract contract) {
  Hcs08BoardSerial board(boundary(image(pattern)), BLOFELD_HCS08_SLA, contract);
  const auto base = board.board_state().completed_periods;
  const auto failure_period = write_period + delay(contract) + 1;
  for (unsigned n = 1; n < failure_period; ++n) {
    static_cast<void>(board.advance_reference_period(false, observed));
  }
  const auto reason =
      rejected([&] { static_cast<void>(board.advance_reference_period(std::nullopt, observed)); });
  require(reason.contains("unknown resolved input") && reason.contains("agent=SPI") &&
              board.faulted() && board.run_state().completed_periods == failure_period - 1 &&
              board.board_state().completed_periods == base + failure_period - 1 &&
              board.memory().periods().completed == base + failure_period &&
              board.memory().cop_state().elapsed == base + failure_period &&
              board.clock_state().generated_edges == 1 && board.memory().spi_state().samples == 0 &&
              !board.memory().spi_state().receive_full,
          "Unknown MISO fails at the first actual sample and retains earlier real device/CPU/edge "
          "prefixes");
  const auto before = board.board_state();
  require(rejected([&] {
            static_cast<void>(board.advance_reference_period(false, observed));
          }).contains("coordinated recovery") &&
              rejected([&] {
                static_cast<void>(board.checkpoint());
              }).contains("coordinated recovery") &&
              board.board_state() == before,
          "Failed coupled owners cannot advance or create successful snapshots");
}

void absent_or_real_clock_fault(const Hcs08BoardSerialContract contract) {
  for (const auto event :
       {Hcs08BoardClockObservation::unobserved, Hcs08BoardClockObservation::loss_of_clock}) {
    Hcs08BoardSerial board(boundary(image(pattern)), BLOFELD_HCS08_SLA, contract);
    const auto base = board.board_state().completed_periods;
    const auto reason =
        rejected([&] { static_cast<void>(board.advance_reference_period(false, event)); });
    require(
        reason.contains(event == Hcs08BoardClockObservation::unobserved
                            ? "unobserved event history"
                            : "coordinated service required") &&
            board.faulted() && board.run_state().completed_periods == 0 &&
            board.clock_state().generated_edges == 0 &&
            board.memory().periods().completed == base + 1 &&
            board.board_state().completed_periods == base,
        "Neither missing clock history nor a real ICG source is replaced by a fabricated SPI IRQ");
  }
}

void past_clock_uncertainty_is_retained(const Hcs08BoardSerialContract contract) {
  for (const bool reset : {false, true}) {
    Hcs08BoardSerial board(
        boundary(image(pattern), Hcs08RtiIrqContract::reference_immediate, !reset, reset),
        BLOFELD_HCS08_SLA,
        contract);
    const auto reason =
        rejected([&] { static_cast<void>(board.advance_reference_period(false, observed)); });
    require(
        reason.contains("unobserved event history") && board.faulted() &&
            board.run_state().completed_periods == 0 && board.clock_state().generated_edges == 0,
        "Later no-fault input cannot erase an unknown reset-entry or earlier period fault history");
  }
}

void resets_abort_live_serial_owners(const Hcs08BoardSerialContract contract) {
  constexpr unsigned cut = 20;
  const auto program = image(pattern, enabled);
  Hcs08BoardSerial original(boundary(program), BLOFELD_HCS08_SLA, contract);
  Peer prefix;
  for (unsigned n = 0; n < cut; ++n) { prefix.tick(original); }
  const auto old_cpu = original.cpu_state();
  const auto old_spi = original.memory().spi_state();
  const auto old_run = original.run_state();
  require(old_spi.active && original.clock_state().generated_edges > 0,
          "Reset cut follows real SPID promotion and actual serial edges");
  for (const auto cause : {Hcs08ResetCause::warm, Hcs08ResetCause::power_on}) {
    const auto saved =
        original.reset_checkpoint({.cause = cause, .entry = Hcs08ResetEntry::normal});
    Hcs08BoardInputs inputs;
    inputs.reset_observation = observed;
    Hcs08Board restarted(saved, BLOFELD_HCS08_SLA, inputs);
    require(restarted.cpu_state().pc == entry &&
                restarted.memory().spi_state() == Hcs08SpiState{} &&
                restarted.memory().periods().completed == 0 &&
                restarted.memory().flash_image() == program,
            "Logical reset discards the real SPI shifter/queue and elapsed device state,retaining "
            "exact NVM");
    for (unsigned n = 0; restarted.cpu_state().pc != foreground; ++n) {
      require(n < 32, "Own reset setup reaches genuine SPI IRQ eligibility again");
      restarted.advance_reference_period(observed);
    }
    Hcs08BoardSerial fresh(restarted.checkpoint(), BLOFELD_HCS08_SLA, contract);
    Peer actual;
    for (unsigned n = 0; n < end_period; ++n) { actual.tick(fresh); }
    require(actual.transmitted == pattern && actual.samples == 8 &&
                fresh.memory().spi_state().receive == reply &&
                fresh.clock_state().completed_transfers == 1 &&
                fresh.run_state().irq_periods == irq_budget + end_period - return_period &&
                fresh.memory().flash_image() == program && original.cpu_state() == old_cpu &&
                original.memory().spi_state() == old_spi && original.run_state() == old_run,
            "Fresh reset owners preserve explicit RTI reentry policy and actual bytes;source "
            "owners stay unchanged");
  }
}

void policy_is_not_invented(const Hcs08BoardSerialContract contract) {
  Hcs08BoardSerial board(boundary(image(pattern, enabled), Hcs08RtiIrqContract::unqualified),
                         BLOFELD_HCS08_SLA,
                         contract);
  const auto reason = rejected([&] {
    for (unsigned n = 0; n < end_period; ++n) {
      static_cast<void>(board.advance_reference_period(false, observed));
    }
  });
  const bool prior_edge = contract.order == Hcs08SpiRunContract::reference_spi_before_cpu;
  const auto edges =
      return_period + 1 - write_period - delay(contract) + static_cast<unsigned>(prior_edge);
  require(reason.contains("RTI IRQ timing contract") && board.faulted() &&
              board.run_state().completed_periods == return_period &&
              board.run_state().attempted_periods == return_period + 1 &&
              board.run_state().spi_periods == return_period + static_cast<unsigned>(prior_edge) &&
              board.run_state().last_pins.has_value() == prior_edge &&
              board.clock_state().generated_edges == edges &&
              board.memory().spi_state().receive_full == (edges == final_edge),
          "An eligible IRQ after real RTI retains the default refusal and any earlier actual SPI "
          "edge/completion");
}
} // namespace

int main() {
  try {
    for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                             Hcs08SpiRunContract::reference_spi_before_cpu}) {
      for (const auto launch : {Hcs08SpiLaunchContract::reference_one_period,
                                Hcs08SpiLaunchContract::reference_two_periods}) {
        const Hcs08BoardSerialContract contract{
            .order = order,
            .launch = launch,
            .program = Hcs08PrefetchContract::reference_peripheral_program_cycles};
        for (const auto byte : {0U, 0x5aU, 0xa5U, 0xffU}) { run_case(contract, byte); }
        continuations(contract);
        unknown_input(contract);
        policy_is_not_invented(contract);
        absent_or_real_clock_fault(contract);
        past_clock_uncertainty_is_retained(contract);
        resets_abort_live_serial_owners(contract);
      }
    }
    std::cout << "Verified production-board native SPI IRQ,real pin bytes,reset knowledge and "
                 "joint serial continuations.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
