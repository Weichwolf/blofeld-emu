// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_serial_clock_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_clock.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"

#include <algorithm>
#include <array>
#include <ostream>
#include <sstream>
#include <string>

namespace blofeld::test::modify_receiver {
using namespace blofeld::test::hcs08;
namespace shared = blofeld::test::hcs08_serial_clock;
namespace app = blofeld::test::hcs08_clock_startup;
constexpr unsigned prefix_end = 176, store_cycle = 214, receive_end = 223, finish_period = 268;
constexpr unsigned rx_buffer = 0x200, rx_write = 0x99, rx_latest = 0x9a, rx_previous = 0x9b;
constexpr unsigned received_byte = 0x12, next_byte = 0x64, partial_flags = 0xef;
constexpr unsigned store_pc = 0xea39, receive_next = 0xea42, frontier_pc = 0xe258;
constexpr unsigned frame_sp = app::stack - 8;
// Fixed expectations from the pinned original instruction boundaries and ISA bus sequences.
constexpr unsigned irq_periods = 33, receive_retired = 59, receive_ram = 59, receive_reads = 147;
constexpr unsigned final_retired = 72, final_ram = 70, final_reads = 177;
constexpr unsigned final_owned_periods = 81, final_owned_retired = 26;
constexpr unsigned final_edges = 27, max_active_periods = 225, final_transmitted = 0x1207;
constexpr unsigned previous_contract_retired = 54, previous_contract_end = 212;
constexpr auto contract = Hcs08PrefetchContract::reference_modify_program_cycles;

struct Input {
  Hcs08SpiRunContract order;
  Hcs08SpiLaunchContract launch;
  unsigned pads;
};

inline unsigned delay(const Input input) {
  return input.launch == Hcs08SpiLaunchContract::reference_one_period ? 1U : 2U;
}

struct Machine : shared::Machine {
  Machine(const Input input,
          const app::ApplicationMemory &startup,
          const Hcs08PrefetchContract selected = contract)
      : shared::Machine({},
                        {.order = input.order,
                         .launch = input.launch,
                         .seed = 7,
                         .program = selected,
                         .scanner = true,
                         .reconnect_gpio = true,
                         .pads = input.pads},
                        &startup) {}
};

inline std::string diagnostic(const Machine &m) {
  const auto cpu = m.cpu.state();
  const auto clock = m.clock->state();
  const auto serial = m.spi->state();
  const auto peripheral = m.memory.spi.state();
  std::ostringstream out;
  out << " [pc=" << cpu.pc << " sp=" << cpu.sp << " hx=" << cpu.hx
      << " a=" << static_cast<unsigned>(cpu.a) << " ccr=" << static_cast<unsigned>(cpu.ccr)
      << " known=" << static_cast<unsigned>(cpu.ccr_known) << " period=" << clock.completed_periods
      << " irq_periods=" << clock.irq_periods << " launches=" << clock.launches
      << " retired=" << m.memory.retired << " ram=" << m.memory.accesses.size()
      << " flash=" << m.memory.flash_reads.size() << " peripheral=" << m.events.size()
      << " entries=" << m.memory.interrupt_entries
      << " acks=" << m.memory.interrupt_acknowledgements << " edges=" << serial.generated_edges
      << " active=" << serial.active_periods << " remaining=" << serial.remaining_periods
      << " samples=" << peripheral.samples << " mosi_samples=" << m.samples
      << " tx=" << m.transmitted << ']';
  return out.str();
}

inline void receive_endpoint(const Machine &m) {
  const auto state = m.cpu.state();
  const auto clock = m.clock->state();
  require(
      clock.completed_periods == receive_end && clock.irq_periods == irq_periods &&
          clock.program_periods == receive_end - irq_periods && clock.launches == 2 &&
          state.pc == receive_next && state.sp == frame_sp - 6 && state.hx == 1 && state.a == 1 &&
          state.ccr == (flags_fixed | flag_i | flag_c) && state.ccr_known == partial_flags &&
          m.memory.retired == receive_retired && m.memory.accesses.size() == receive_ram &&
          m.memory.flash_reads.size() == receive_reads && m.events.size() == 8 &&
          m.memory.interrupt_entries == 3 && m.memory.interrupt_acknowledgements == 3,
      "Independent original RX instruction/cycle/frame counts reach the TX check" + diagnostic(m));
  require(m.memory.bytes[rx_buffer] == received_byte && m.memory.bytes[rx_write] == 1 &&
              m.memory.bytes[rx_latest] == received_byte && m.memory.bytes[rx_previous] == 0,
          "Unmodified original IRQ records received data and increments its real ring cursor");
  constexpr std::array<shared::Event, 4> expected{{
      {.period = 197, .access = {.address = rx_previous, .value = 0, .write = true}},
      {.period = 203, .access = {.address = rx_latest, .value = received_byte, .write = true}},
      {.period = store_cycle,
       .access = {.address = rx_buffer, .value = received_byte, .write = true}},
      {.period = 221, .access = {.address = rx_write, .value = 1, .write = true}},
  }};
  for (const auto event : expected) {
    require(std::ranges::count(m.ram_events, event) == 1,
            "Real native RX writes occur once at independently derived w phases");
  }
}

inline void endpoint(const Machine &m, const Input input, std::ostream &trace) {
  const auto state = m.cpu.state();
  const auto clock = m.clock->state();
  const auto serial = m.spi->state();
  const auto peripheral = m.memory.spi.state();
  const auto atd = m.memory.adc.state();
  const auto queue = m.cpu.prefetch_state();
  require(clock.completed_periods == finish_period && clock.attempted_periods == finish_period &&
              clock.spi_periods == finish_period && clock.irq_periods == irq_periods &&
              clock.program_periods == finish_period - irq_periods && clock.launches == 2 &&
              state.pc == frontier_pc && state.sp == frame_sp && state.hx == 0 && state.a == 0 &&
              state.ccr == flags_fixed && state.ccr_known == partial_flags &&
              m.memory.retired == final_retired && m.memory.accesses.size() == final_ram &&
              m.memory.flash_reads.size() == final_reads && m.events.size() == 10 && queue &&
              queue->contract == contract && queue->head == frontier_pc &&
              queue->completed_periods == final_owned_periods &&
              queue->retired_instructions == final_owned_retired,
          "Original RX/TX/RTI and foreground MOV reach the independent pre-CLR boundary" +
              diagnostic(m));
  require(serial.generated_edges == final_edges && serial.completed_transfers == 1 &&
              serial.active_periods == max_active_periods - 2 * delay(input) && serial.active &&
              serial.remaining_periods == 2 * delay(input) - 1 && peripheral.active &&
              peripheral.samples == 5 && peripheral.edges == 11 &&
              peripheral.transmit == next_byte && !peripheral.receive_full &&
              m.transmitted == final_transmitted && m.samples == 13 && !atd.active &&
              atd.complete && atd.remaining == 0 && atd.result == 512 && atd.conversions == 1,
          "Queued third byte does not fabricate another completion/IRQ while byte two is active" +
              diagnostic(m));
  require(m.memory.bytes[rx_buffer] == received_byte && m.memory.bytes[rx_write] == 1 &&
              m.memory.bytes[rx_latest] == received_byte && m.memory.bytes[rx_previous] == 0,
          "Real receive ring survives the original return and foreground output");
  trace << "END order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
        << " pads=" << input.pads << " periods=" << finish_period << " pc=" << state.pc
        << " retired=" << m.memory.retired << " reads=" << m.memory.flash_reads.size()
        << " data=" << m.memory.accesses.size() << " edges=" << serial.generated_edges
        << " rx=" << static_cast<unsigned>(m.memory.bytes[rx_buffer])
        << " rx_write=" << static_cast<unsigned>(m.memory.bytes[rx_write]) << '\n';
}
}
