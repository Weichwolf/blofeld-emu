// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "board/hcs08_board_serial.hpp"
#include "common/bus.hpp"
#include "hcs08_board_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
namespace fixture = blofeld::test::board;
namespace reg = hcs08_spi;
constexpr unsigned entry = 0xe000, handler = 0xe100, vector = 0xffe0;
constexpr unsigned tx_irq = 0x74, rx_irq = 0xd4, quiet = 0x54, reply = 0x5a, stack = 0x47f;
// Family Rev2 p292:TXS costs two periods(fp);the six reset/setup instructions total12.
constexpr unsigned setup = 12, irq_budget = 11, tx_store = 29, rx_store = 20, edge_count = 16;
constexpr unsigned early_rx_foreground = 36, late_rx_foreground = 39;
constexpr unsigned tx_handler_periods = 21, rx_handler_periods = 19;
constexpr unsigned tx_loop_offset = 10, rx_loop_offset = 17;
constexpr unsigned reset_h_known = 0xff00, reset_sp = 0xff, loaded_ccr_known = 0xee;
constexpr unsigned fixed_ccr = 0x60, negative_ccr = 4, zero_ccr = 2;
constexpr std::uint8_t ldhx_immediate = 0x45, txs = 0x94, lda_immediate = 0xa6;
constexpr std::uint8_t sta_direct = 0xb7, cli = 0x9a, nop = 0x9d, bra = 0x20;
constexpr std::uint8_t lda_direct = 0xb6, mov_immediate = 0x6e, rti = 0x80, back_two = 0xfe;
constexpr auto observed = Hcs08BoardClockObservation::no_fault;
constexpr auto retirement = Hcs08SpiForegroundContract::reference_instruction_retirement;
constexpr std::array<unsigned, 29> cuts{0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 19, 20,
                                        21, 22, 28, 29, 34, 35, 36, 37, 38, 39, 40, 43, 44, 45};

struct Case {
  Hcs08BoardSerialContract contract;
  bool receive_irq;
  unsigned byte;
  std::uint16_t initial_hx_known = reset_h_known;
  std::uint8_t initial_a_known = 0;
};

unsigned latency(const Case input) {
  return input.contract.launch == Hcs08SpiLaunchContract::reference_one_period ? 1U : 2U;
}

unsigned irq_start(const Case input) {
  if (!input.receive_irq) { return setup + 1; }
  const bool late =
      latency(input) == 2 && input.contract.order == Hcs08SpiRunContract::reference_cpu_before_spi;
  return (late ? late_rx_foreground : early_rx_foreground) + 1;
}

unsigned returned(const Case input) {
  return input.receive_irq ? irq_start(input) + irq_budget + rx_handler_periods - 1
                           : setup + irq_budget + tx_handler_periods;
}

Hcs08FlashImage image(const Case input) {
  // Own reset instructions establish HX/SP/SPI,then CLI's protected NOP.
  std::vector<std::uint8_t> main{ldhx_immediate,
                                 4,
                                 0x80,
                                 txs,
                                 lda_immediate,
                                 static_cast<std::uint8_t>(input.receive_irq ? rx_irq : tx_irq),
                                 sta_direct,
                                 reg::control1,
                                 cli,
                                 nop};
  if (input.receive_irq) {
    const std::array<std::uint8_t, 7> queue{lda_direct,
                                            reg::status,
                                            lda_immediate,
                                            static_cast<std::uint8_t>(input.byte),
                                            sta_direct,
                                            reg::data,
                                            nop};
    main.insert(main.end(), queue.begin(), queue.end());
  }
  main.insert(main.end(), {bra, back_two}); // Own foreground BRA *.
  auto result = fixture::own_image(main);
  std::vector<std::uint8_t> isr{lda_direct, reg::status};
  if (input.receive_irq) {
    isr.insert(isr.end(), {lda_direct, reg::data});
  } else {
    isr.insert(isr.end(),
               {lda_immediate, static_cast<std::uint8_t>(input.byte), sta_direct, reg::data});
  }
  isr.insert(isr.end(), {mov_immediate, quiet, reg::control1, rti, nop, nop});
  const auto put = [&](const unsigned address, const std::uint8_t byte) {
    result.data.at(address - hcs08_flash::first) = byte;
    result.fresh.reset(address - hcs08_flash::first);
  };
  for (unsigned n = 0; n < isr.size(); ++n) { put(handler + n, isr[n]); }
  put(vector, handler >> 8U);
  put(vector + 1, handler & fixture::byte_mask);
  return result;
}

Hcs08BoardCheckpoint reset(const Hcs08FlashImage &program) {
  Hcs08BoardInputs inputs;
  inputs.reset_observation = observed;
  const Hcs08Board board(program,
                         BLOFELD_HCS08_SLA,
                         fixture::reference,
                         inputs,
                         Hcs08RtiIrqContract::reference_immediate);
  require(board.state().completed_periods == 0 && board.run_state().retired_instructions == 0 &&
              board.cpu_state().pc == entry && board.cpu_state().a_known == 0 &&
              board.cpu_state().hx_known == reset_h_known && board.cpu_state().sp == reset_sp,
          "Native cold vector/reset boundary precedes all own setup instructions");
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
    const unsigned bit = samples < 8 ? 7 - samples : 0;
    const auto pins = board.advance_reference_period(((reply >> bit) & 1U) != 0, observed);
    if (!pins) { return; }
    events.push_back(
        {.period = static_cast<unsigned>(board.run_state().completed_periods), .pins = *pins});
    if (!pins->clock) {
      if (!pins->mosi) { throw Fault("Missing actual MOSI at the native trailing sample edge"); }
      transmitted = (transmitted << 1U) | static_cast<unsigned>(*pins->mosi);
      ++samples;
    }
  }
};

Hcs08State caller(const Case input) {
  const auto value = input.receive_irq ? input.byte : tx_irq;
  unsigned flags = fixed_ccr;
  if (input.receive_irq) {
    flags |= (input.byte & 0x80U) != 0 ? negative_ccr : 0U;
    flags |= input.byte == 0 ? zero_ccr : 0U;
  }
  return {.pc = static_cast<std::uint16_t>(entry +
                                           (input.receive_irq ? rx_loop_offset : tx_loop_offset)),
          .hx = stack + 1,
          .sp = stack,
          .a = static_cast<std::uint8_t>(value),
          .ccr_known = loaded_ccr_known,
          .ccr = static_cast<std::uint8_t>(flags)};
}

void check(const Hcs08BoardSerial &board, const Case input) {
  const auto p = board.run_state().completed_periods;
  const auto foreground = std::min<std::uint64_t>(p, irq_start(input) - 1);
  const auto irq = std::min<std::uint64_t>(p - foreground, irq_budget);
  const auto store = input.receive_irq ? rx_store : tx_store;
  const auto launch = store + latency(input);
  const auto edges = p >= launch ? std::min<std::uint64_t>(p - launch + 1, edge_count) : 0;
  const auto complete = launch + edge_count - 1;
  const auto read = irq_start(input) + 14;
  require(board.board_state().completed_periods == p && board.memory().periods().completed == p &&
              board.memory().cop_state().elapsed == p &&
              board.run_state().foreground_periods == foreground &&
              board.foreground_state().offered_periods == foreground &&
              board.foreground_state().executing_periods == foreground &&
              board.run_state().irq_periods == irq &&
              board.run_state().program_periods == p - foreground - irq &&
              board.run_state().spi_periods == p && board.clock_state().offered_periods == p &&
              board.clock_state().generated_edges == edges &&
              board.memory().spi_state().receive_full ==
                  (p >= complete && (!input.receive_irq || p < read)),
          "Literal native setup,budget/IRQ boundaries,SPID store,edge and actual RX-ack periods");
  if (p == returned(input)) {
    require(board.cpu_state() == caller(input),
            "Native RTI restores the literal reset-derived caller and knowledge");
  }
  if (p < 3) {
    require(board.cpu_state().pc == entry && board.cpu_state().hx_known == input.initial_hx_known &&
                board.cpu_state().a_known == input.initial_a_known &&
                board.foreground_state().remaining_periods == 3 - p,
            "Uncommitted LDHX retains the actual cold/warm X and A knowledge until its complete "
            "native budget");
  }
  if (p == 4) {
    require(board.cpu_state().sp == reset_sp && board.foreground_state().remaining_periods == 1,
            "TXS retains the reset stack pointer during its first of two documented periods");
  }
  if (p == 5) {
    require(board.cpu_state().sp == stack && board.foreground_state().remaining_periods == 0,
            "Native TXS publishes HX-1 only at its complete two-period retirement boundary");
  }
  if (input.receive_irq && latency(input) == 2 &&
      input.contract.order == Hcs08SpiRunContract::reference_cpu_before_spi &&
      (p > early_rx_foreground && p < late_rx_foreground)) {
    require(board.memory().spi_state().receive_full && board.run_state().irq_periods == 0 &&
                board.foreground_state().remaining_periods == late_rx_foreground - p,
            "Real receive IRQ waits for the already reserved BRA budget at periods37/38");
  }
}

void until(Hcs08BoardSerial &board, Peer &peer, const Case input, const unsigned end) {
  while (board.run_state().completed_periods < end) {
    peer.tick(board);
    check(board, input);
  }
}

void run_case(const Case input) {
  const auto program = image(input);
  Hcs08BoardSerial board(reset(program), BLOFELD_HCS08_SLA, input.contract);
  Peer peer;
  until(board, peer, input, returned(input) + 6);
  require(peer.samples == 8 && peer.transmitted == input.byte && peer.events.size() == edge_count &&
              board.memory().spi_state().receive == reply &&
              board.clock_state().completed_transfers == 1 &&
              board.memory().spi_state().control1 == quiet &&
              board.memory().flash_image() == program,
          "Native reset-to-foreground/IRQ/SPI path emits real bytes without a target state setter");
}

void continuations(const Case input) {
  const auto program = image(input);
  const auto source = reset(program);
  const auto end = returned(input) + 6;
  for (const auto cut : cuts) {
    auto original = std::make_unique<Hcs08BoardSerial>(source, BLOFELD_HCS08_SLA, input.contract);
    Peer prefix;
    until(*original, prefix, input, cut);
    const auto saved = original->checkpoint();
    auto expected = prefix;
    until(*original, expected, input, end);
    const auto cpu = original->cpu_state();
    const auto run = original->run_state();
    const auto foreground = original->foreground_state();
    const auto clock = original->clock_state();
    const auto memory = original->memory();
    original.reset();
    for (unsigned n = 0; n < 2; ++n) {
      Hcs08BoardSerial fresh(saved, BLOFELD_HCS08_SLA);
      auto actual = prefix;
      until(fresh, actual, input, end);
      require(actual.events == expected.events && actual.transmitted == input.byte &&
                  fresh.cpu_state() == cpu && fresh.run_state() == run &&
                  fresh.foreground_state() == foreground && fresh.clock_state() == clock,
              "Destroyed original owners and fresh partial-budget/IRQ/shifter owners reproduce "
              "every suffix");
      fixture::same_memory(fresh.memory(), memory);
    }
  }
}

void reset_active_owners(const Case input) {
  const auto program = image(input);
  for (const auto cut : {1U, 7U, 20U, 36U}) {
    Hcs08BoardSerial source(reset(program), BLOFELD_HCS08_SLA, input.contract);
    Peer prefix;
    until(source, prefix, input, cut);
    const auto before_cpu = source.cpu_state();
    const auto before_run = source.run_state();
    const auto before_foreground = source.foreground_state();
    for (const auto cause : {Hcs08ResetCause::warm, Hcs08ResetCause::power_on}) {
      const auto saved =
          source.reset_checkpoint({.cause = cause, .entry = Hcs08ResetEntry::normal});
      Hcs08BoardInputs inputs;
      inputs.reset_observation = observed;
      const Hcs08Board restarted(saved, BLOFELD_HCS08_SLA, inputs);
      require(restarted.memory().spi_state() == Hcs08SpiState{} &&
                  restarted.run_state().remaining_periods == 0 &&
                  restarted.cpu_state().pc == entry && restarted.memory().flash_image() == program,
              "Logical reset aborts an actual partial foreground/IRQ/shifter owner without an NVM "
              "change");
      auto expected = input;
      expected.initial_hx_known =
          cause == Hcs08ResetCause::warm ? before_cpu.hx_known : reset_h_known;
      expected.initial_a_known = cause == Hcs08ResetCause::warm ? before_cpu.a_known : 0;
      require(restarted.cpu_state().hx_known == expected.initial_hx_known &&
                  restarted.cpu_state().a_known == expected.initial_a_known,
              "Warm retains actual X/A knowledge;power-on does not invent it");
      Hcs08BoardSerial fresh(restarted.checkpoint(), BLOFELD_HCS08_SLA, input.contract);
      Peer actual;
      until(fresh, actual, expected, returned(input) + 6);
      require(actual.transmitted == input.byte && actual.samples == 8 &&
                  source.cpu_state() == before_cpu && source.run_state() == before_run &&
                  source.foreground_state() == before_foreground,
              "Fresh warm/cold reset owners execute setup and the real byte again;source owners "
              "remain intact");
    }
  }
}

void missing_receive_pin(const Case input) {
  if (!input.receive_irq) { return; }
  Hcs08BoardSerial board(reset(image(input)), BLOFELD_HCS08_SLA, input.contract);
  Peer prefix;
  const auto sample = rx_store + latency(input) + 1;
  until(board, prefix, input, sample - 1);
  const auto before = board.foreground_state();
  const auto reason =
      rejected([&] { static_cast<void>(board.advance_reference_period(std::nullopt, observed)); });
  const auto committed =
      static_cast<unsigned>(input.contract.order == Hcs08SpiRunContract::reference_cpu_before_spi);
  require(reason.contains("unknown resolved input") && board.faulted() &&
              board.run_state().completed_periods == sample - 1 &&
              board.board_state().completed_periods == sample - 1 &&
              board.memory().periods().completed == sample &&
              board.clock_state().generated_edges == 1 && board.memory().spi_state().samples == 0 &&
              board.foreground_state().offered_periods == before.offered_periods + committed,
          "Missing MISO preserves the actual partial CPU budget according to the declared CPU/SPI "
          "order");
  const auto periods = board.memory().periods();
  require(rejected([&] {
            static_cast<void>(board.advance_reference_period(false, observed));
          }).contains("coordinated recovery") &&
              rejected([&] {
                static_cast<void>(board.checkpoint());
              }).contains("coordinated recovery") &&
              board.memory().periods() == periods,
          "Failed foreground/serial owners cannot spend another period or create a successful "
          "checkpoint");
}

void native_background_entry(const Hcs08BoardSerialContract contract) {
  constexpr std::uint8_t bgnd = 0x82;
  constexpr unsigned periods = 5;
  constexpr std::array<std::uint8_t, 1> own{bgnd};
  const auto program = fixture::own_image(own);
  Hcs08BoardInputs inputs;
  inputs.reset_observation = observed;
  Hcs08Board initial(program, BLOFELD_HCS08_SLA, fixture::reference, inputs);
  constexpr std::uint8_t enable_bdm = 0x80;
  // Explicit existing byte-service reference input,not a CPU/RAM/debug-state setter.
  fixture::command(initial, {hcs08_bdc::write_control, enable_bdm});
  require(initial.bdc_state().control == enable_bdm && !initial.bdc_state().active &&
              initial.run_state().offered_periods == 0 && initial.cpu_state().pc == entry,
          "Actual WRITE_CONTROL enables BGND before any own instruction;normal reset alone leaves "
          "ENBDM clear");
  Hcs08BoardSerial board(initial.checkpoint(), BLOFELD_HCS08_SLA, contract);
  for (unsigned n = 1; n < periods; ++n) {
    static_cast<void>(board.advance_reference_period(std::nullopt, observed));
  }
  const bool before_spi = contract.order == Hcs08SpiRunContract::reference_cpu_before_spi;
  if (before_spi) {
    const auto reason = rejected(
        [&] { static_cast<void>(board.advance_reference_period(std::nullopt, observed)); });
    require(reason.contains("active background"), "CPU-first BGND clock refusal: " + reason);
  } else {
    static_cast<void>(board.advance_reference_period(std::nullopt, observed));
    require(board.board_state().completed_periods == periods && board.bdc_state().active,
            "SPI-first retains its already completed normal-running stage and actual BGND entry");
    require(rejected([&] {
              static_cast<void>(board.advance_reference_period(std::nullopt, observed));
            }).contains("active background"),
            "A subsequent halted CPU/device/SPI period requires its separate qualified owner");
  }
  require(
      board.faulted() && board.bdc_state().active &&
          board.foreground_state().retired_instructions == 1 &&
          board.foreground_state().offered_periods == periods &&
          board.memory().periods().completed == periods &&
          board.run_state().foreground_periods == periods && board.cpu_state().pc == entry + 1 &&
          board.clock_state().offered_periods == periods - static_cast<unsigned>(before_spi) &&
          board.board_state().completed_periods == periods - static_cast<unsigned>(before_spi) &&
          board.memory().flash_image() == program,
      "Native BGND prefixes remain visible;no extra halted device or SPI clock is invented");
}

void default_stays_strict(const Case input) {
  auto contract = input.contract;
  contract.foreground = Hcs08SpiForegroundContract::unqualified;
  Hcs08BoardSerial board(reset(image(input)), BLOFELD_HCS08_SLA, contract);
  const auto cpu = board.cpu_state();
  require(rejected([&] {
            static_cast<void>(board.advance_reference_period(false, observed));
          }).contains("unowned foreground") &&
              board.faulted() && board.cpu_state() == cpu &&
              board.foreground_state().offered_periods == 0,
          "The default cannot invent ordinary CPU ownership or a successful idle period");
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
            .program = Hcs08PrefetchContract::reference_peripheral_program_cycles,
            .foreground = retirement};
        native_background_entry(contract);
        for (const bool rx : {false, true}) {
          for (const auto byte : {0U, 0x5aU, 0xa5U, 0xffU}) {
            run_case({.contract = contract, .receive_irq = rx, .byte = byte});
          }
          const Case input{.contract = contract, .receive_irq = rx, .byte = 0xa5};
          continuations(input);
          reset_active_owners(input);
          missing_receive_pin(input);
          default_stays_strict(input);
        }
      }
    }
    std::cout << "Verified native reset/foreground budgets,real SPI IRQ deferral,pin bytes and "
                 "fresh whole-owner continuations.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
