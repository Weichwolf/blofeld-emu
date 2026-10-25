// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_serial_clock_fixture.hpp"
#include "hcs08_spi_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_clock.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"
#include "storage/private_file.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <sstream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
namespace app = blofeld::test::hcs08_clock_startup;
namespace hp = blofeld::test::hcs08_programmer;
namespace wire = blofeld::test::hcs08_spi_test;
namespace shared = blofeld::test::hcs08_serial_clock;
using shared::Event;
using shared::Snapshot;
using shared::startup_ram;
constexpr unsigned partial = 0xef;
constexpr unsigned write_period = 45, conversion_periods = 176, half_period = 8;
constexpr unsigned retired = 29, reads = 78, ram_accesses = 34, second_irq_end = 70,
                   irq_periods = 22;
constexpr unsigned stop_period = 123;
constexpr unsigned scanner_stop = 171, scanner_word_store = 0xe252, firmware_stack = app::stack - 8;
constexpr unsigned scanner_program_reads = 35, word_operand = 0xe225;
constexpr unsigned scanner_edges = 15, scanner_samples = 7, scanner_tx = 0x48, scanner_rx = 0x12;
constexpr unsigned contact_high = 0x240, contact_low = 0x241, frame_high = firmware_stack + 1,
                   frame_offset = firmware_stack + 2;
constexpr unsigned current_instructions = 15;
constexpr std::array<unsigned, 37> cpu_cuts{
    0,  11, 18, 26, 29, 37, 42, 44,  45,  46,  47,  49,  50,  55,  56,  59,  60,  65, 66,
    67, 70, 71, 77, 85, 88, 96, 102, 103, 104, 105, 109, 114, 115, 118, 119, 120, 123};

struct Input {
  Hcs08SpiRunContract order;
  Hcs08SpiLaunchContract launch;
  unsigned seed;
};

unsigned delay(const Input input) {
  return input.launch == Hcs08SpiLaunchContract::reference_one_period ? 1U : 2U;
}

std::set<unsigned> cuts(const Input input) {
  std::set<unsigned> result(cpu_cuts.begin(), cpu_cuts.end());
  for (unsigned edge = 1; edge <= scanner_edges; ++edge) {
    const unsigned period = write_period + delay(input) + (edge * half_period) - 1;
    result.insert(period - 1);
    result.insert(period);
  }
  result.insert(scanner_stop);
  for (unsigned n = stop_period + 1; n < scanner_stop; ++n) { result.insert(n); }
  return result;
}

shared::Setup reference_setup(const Input input) {
  return {.order = input.order,
          .launch = input.launch,
          .seed = input.seed,
          .program = Hcs08PrefetchContract::reference_scanner_word_cycles,
          .scanner = true,
          .pads = std::nullopt};
}

struct Machine : shared::Machine {
  Machine(const std::span<const std::uint8_t> pages,
          const Input input,
          const app::ApplicationMemory *startup = nullptr)
      : shared::Machine(pages, reference_setup(input), startup) {}
};

void scanner_accesses(const Machine &m, const Input input) {
  const unsigned destination = m.memory.flash.image().data.at(word_operand - hcs08_flash::first);
  const std::array<Event, 10> expected{{
      {.period = 138,
       .access = {.address = contact_low + (2 * input.seed), .value = 0, .write = false}},
      {.period = 141, .access = {.address = firmware_stack, .value = 0, .write = true}},
      {.period = 144,
       .access = {.address = contact_high + (2 * input.seed), .value = 0, .write = false}},
      {.period = 148, .access = {.address = frame_high, .value = 0, .write = true}},
      {.period = 153, .access = {.address = firmware_stack - 1, .value = 0, .write = true}},
      {.period = 156, .access = {.address = frame_offset, .value = 2 * input.seed, .write = true}},
      {.period = 159, .access = {.address = firmware_stack - 1, .value = 0, .write = false}},
      {.period = 162, .access = {.address = firmware_stack, .value = 0, .write = false}},
      {.period = 165, .access = {.address = destination, .value = 0, .write = true}},
      {.period = 166, .access = {.address = destination + 1, .value = 0, .write = true}},
  }};
  require(m.ram_events.size() >= expected.size() &&
              std::ranges::equal(std::span(m.ram_events).last(expected.size()), expected),
          "Actual native scanner RAM reads/stores/pulls occur once at independent addresses and "
          "periods");
}

void scanner_endpoint(const Machine &m, const Input input, std::ostream &trace) {
  scanner_accesses(m, input);
  const auto state = m.cpu.state();
  const auto s = m.clock->state();
  const auto q = m.cpu.prefetch_state();
  const auto serial = m.spi->state();
  const auto peripheral = m.memory.spi.state();
  if (!q || !peripheral.transmit) { throw Fault("Missing genuine word/serial state"); }
  const unsigned destination = m.memory.flash.image().data.at(word_operand - hcs08_flash::first);
  require(destination >= app::ram_first && destination + 1 < app::ram_end &&
              m.memory.bytes[destination] == 0 && m.memory.bytes[destination + 1] == 0 &&
              m.memory.known[destination] && m.memory.known[destination + 1],
          "The two native STHX writes preserve the independently identified original RAM word");
  require(s.completed_periods == scanner_stop && s.attempted_periods == scanner_stop &&
              s.spi_periods == scanner_stop && s.irq_periods == irq_periods &&
              s.program_periods == scanner_stop - irq_periods && s.launches == 1 &&
              state.pc == scanner_word_store && state.sp == firmware_stack && state.a == 0 &&
              state.hx == 0 && state.ccr == (flags_fixed | flag_z) && state.ccr_known == partial &&
              m.memory.retired == retired + 16 &&
              m.memory.flash_reads.size() == reads + scanner_program_reads &&
              m.memory.accesses.size() == ram_accesses + 11 && m.events.size() == 5 &&
              m.memory.interrupt_entries == 2 && m.memory.interrupt_acknowledgements == 2 &&
              q->contract == Hcs08PrefetchContract::reference_scanner_word_cycles &&
              q->head == scanner_word_store && q->next_read == scanner_word_store + 3 &&
              q->queued_bytes == 3 && q->ready_bytes == 2 && q->program_byte_in_flight &&
              q->completed_periods == scanner_stop - second_irq_end &&
              q->retired_instructions == current_instructions + 16 && q->operand == Hcs08DataRead{},
          "Native word and taken BEQ complete without a host instruction or PC substitution");
  require(serial.generated_edges == scanner_edges && serial.completed_transfers == 0 &&
              serial.active_periods == scanner_stop - write_period - delay(input) + 1 &&
              serial.remaining_periods == delay(input) + 1 && peripheral.active &&
              peripheral.samples == scanner_samples && peripheral.shift_in == scanner_rx &&
              peripheral.transmit == wire::event_bytes[1] && !peripheral.receive_full &&
              m.transmitted == scanner_tx && m.samples == scanner_samples &&
              m.memory.adc.state().active &&
              m.memory.adc.state().remaining == conversion_periods - scanner_stop,
          "Real CPU word phases retain concurrent SPI/ADC progress and incomplete wire data");
  trace << "WORD seed=" << input.seed << " order=" << static_cast<unsigned>(input.order)
        << " delay=" << delay(input) << " periods=" << scanner_stop << " pc=" << state.pc
        << " sp=" << state.sp << " a=" << static_cast<unsigned>(state.a) << " hx=" << state.hx
        << " ccr=" << static_cast<unsigned>(state.ccr)
        << " known=" << static_cast<unsigned>(state.ccr_known) << " retired=" << m.memory.retired
        << " edges=" << serial.generated_edges << " active_periods=" << serial.active_periods
        << " remaining=" << serial.remaining_periods << " tx=" << m.transmitted
        << " samples=" << m.samples << " shift_in=" << static_cast<unsigned>(peripheral.shift_in)
        << " queued=" << static_cast<unsigned>(*peripheral.transmit)
        << " adc_remaining=" << m.memory.adc.state().remaining
        << " reads=" << m.memory.flash_reads.size() << " data_accesses=" << m.memory.accesses.size()
        << " events=" << m.events.size() << " word_address=" << destination
        << " ready=" << q->ready_bytes << " inflight=" << q->program_byte_in_flight
        << " next=" << q->next_read << " head=" << q->head << '\n';
}

void frontier(Machine &m, const Input input, std::ostream &trace) {
  const auto accesses = m.memory.accesses;
  const auto reads_before = m.memory.flash_reads;
  const auto state = m.cpu.state();
  const auto error = rejected([&] { m.tick(); });
  const bool prior_spi = input.order == Hcs08SpiRunContract::reference_spi_before_cpu;
  const auto s = m.clock->state();
  const unsigned stop = scanner_stop;
  constexpr unsigned instructions = retired + 16;
  const bool diagnostic = error.contains("opcode=0x004e") && error.contains("pc=0xe252");
  constexpr unsigned expected_edges = scanner_edges;
  require(diagnostic && m.clock->faulted() && m.cpu.faulted() && m.cpu.state() == state &&
              m.memory.accesses == accesses && m.memory.flash_reads == reads_before &&
              m.memory.retired == instructions && s.attempted_periods == stop + 1 &&
              s.completed_periods == stop && s.irq_periods == irq_periods &&
              s.program_periods == stop - irq_periods &&
              s.spi_periods == stop + static_cast<unsigned>(prior_spi) &&
              m.memory.spi.state().edges == expected_edges &&
              m.memory.adc.state().remaining == conversion_periods - stop - 1,
          "Unqualified original foreground cycles fail before execution while retaining genuine "
          "ADC/SPI "
          "prefixes");
  const auto q = m.cpu.prefetch_state();
  require(q && q->ready_bytes == 3 && !q->program_byte_in_flight,
          "The previous actual program-byte arrival survives the refused instruction");
  require(!s.last_pins, "The refused MOV read has no preceding serial edge");
  trace << "FRONTIER order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
        << " pc=" << state.pc << " attempted=" << s.attempted_periods
        << " completed=" << s.completed_periods << " spi_periods=" << s.spi_periods
        << " adc_remaining=" << m.memory.adc.state().remaining;
  trace << " edges=" << m.memory.spi.state().edges << " pins=" << s.last_pins.has_value();
  trace << '\n';
}

Snapshot capture(const std::span<const std::uint8_t> pages,
                 const Input input,
                 const unsigned cut,
                 const app::ApplicationMemory *startup) {
  Machine original(pages, input, startup);
  for (unsigned n = 0; n < cut; ++n) { original.tick(); }
  return shared::capture(original);
}

void run(const std::span<const std::uint8_t> pages, std::ostream &trace) {
  const auto startup = startup_ram(pages, trace);
  const auto *initial = &startup;
  unsigned periods = 0;
  unsigned replays = 0;
  unsigned cut_count = 0;
  for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                           Hcs08SpiRunContract::reference_spi_before_cpu}) {
    for (const auto launch : {Hcs08SpiLaunchContract::reference_one_period,
                              Hcs08SpiLaunchContract::reference_two_periods}) {
      for (unsigned seed = 0; seed < 8; ++seed) {
        const Input input{.order = order, .launch = launch, .seed = seed};
        Machine m(pages, input, initial);
        for (unsigned n = 0; n < scanner_stop; ++n) {
          m.tick();
          ++periods;
        }
        scanner_endpoint(m, input, trace);
        frontier(m, input, trace);
      }
      const Input input{.order = order, .launch = launch, .seed = 7};
      for (const auto cut : cuts(input)) {
        const auto saved = capture(pages, input, cut, initial);
        periods += cut;
        ++cut_count;
        for (unsigned replay = 0; replay < 2; ++replay) {
          Machine fresh(pages, input, initial);
          restore(fresh, saved);
          for (unsigned n = cut; n < scanner_stop; ++n) {
            fresh.tick();
            ++periods;
          }
          trace << "REPLAY order=" << static_cast<unsigned>(order) << " delay=" << delay(input)
                << " cut=" << cut << " index=" << replay << '\n';
          scanner_endpoint(fresh, input, trace);
          frontier(fresh, input, trace);
          ++replays;
        }
      }
    }
  }
  trace << "SUMMARY periods=" << periods << " contexts=32 cuts=" << cut_count
        << " replays=" << replays << '\n';
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned pages and private word-clock trace");
    const auto pages = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    run(pages, trace);
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout << "Original CPU/SPI/ADC periods preserve genuine serial prefixes and the strict "
                 "foreground frontier.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
