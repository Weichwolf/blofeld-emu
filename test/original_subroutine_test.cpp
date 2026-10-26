// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_serial_clock_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_clock.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"
#include "modify_receiver_fixture.hpp"
#include "storage/private_file.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <ostream>
#include <span>
#include <sstream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::modify_receiver;
namespace hp = blofeld::test::hcs08_programmer;
constexpr auto call_contract = Hcs08PrefetchContract::reference_subroutine_program_cycles;
constexpr unsigned modify_end = 288, next_pc = 0xe2a1, call_end = 294;
constexpr unsigned row_cell = 0xa6, expiry_cell = 0xa8, next_expiry = 8;
constexpr unsigned final_count = 78, final_fetches = 190, final_accesses = 75;
constexpr unsigned owned_periods = 101, owned_retired = 32, edge_count = 30;
constexpr unsigned return_pc = 0xe2a4, final_pc = 0xe2a7;
constexpr unsigned final_end = 415, final_retired = 124, final_reads = 283, final_data = 101;
constexpr unsigned final_irq_periods = 44, final_entries = 4, final_launches = 3;
constexpr unsigned sample_cell = 0x9d;
constexpr unsigned completed_wire_word = 0x903c, completed_wire_bits = 16, third_launch_base = 299;
constexpr unsigned final_active_base = 256, edge_period = 8;
constexpr unsigned return_high_read = 405, return_low_read = 406, gpio_write = 413;
constexpr unsigned transmitted = 0x481e, max_active = 245, wire_samples = 15;

void advance(Machine &m, const unsigned end) {
  while (m.clock->state().completed_periods < end) {
    m.tick();
    if (m.clock->state().completed_periods == receive_end) { receive_endpoint(m); }
  }
}

void counters(const Machine &m, const Input input) {
  const auto state = m.cpu.state();
  const auto queue = m.cpu.prefetch_state();
  const auto serial = m.spi->state();
  const auto peripheral = m.memory.spi.state();
  const auto clock = m.clock->state();
  require(clock.completed_periods == modify_end && clock.irq_periods == irq_periods &&
              clock.program_periods == modify_end - irq_periods &&
              clock.spi_periods == modify_end && clock.launches == 2 && state.pc == next_pc &&
              state.sp == frame_sp && state.hx == 0 && state.a == 5 &&
              state.ccr == (flags_fixed | flag_c) && state.ccr_known == partial_flags &&
              m.memory.retired == final_count && m.memory.flash_reads.size() == final_fetches &&
              m.memory.accesses.size() == final_accesses && m.memory.bytes[row_cell] == 0 &&
              m.memory.bytes[expiry_cell] == next_expiry && queue &&
              queue->completed_periods == owned_periods &&
              queue->retired_instructions == owned_retired && m.memory.interrupt_entries == 3 &&
              m.memory.interrupt_acknowledgements == 3,
          "Independent original CLR/INC/LDA/CMP/taken-BCS/LDA5 endpoint" + diagnostic(m));
  require(serial.generated_edges == edge_count && serial.completed_transfers == 1 &&
              serial.active && serial.active_periods == max_active - 2 * delay(input) &&
              serial.remaining_periods == 2 * delay(input) + 3 && peripheral.active &&
              peripheral.edges == 14 && peripheral.samples == 7 &&
              peripheral.transmit == next_byte && !peripheral.receive_full &&
              m.samples == wire_samples && m.transmitted == transmitted &&
              m.memory.adc.state().conversions == 1 && m.memory.adc.state().complete,
          "Actual second-byte edges and independent 90/3c first15 transmitted bits remain live" +
              diagnostic(m));
}

unsigned first_irq(const Input input) {
  constexpr unsigned base = 299;
  return base + (3 * delay(input)) +
         static_cast<unsigned>(input.order == Hcs08SpiRunContract::reference_cpu_before_spi);
}

void checked(Machine &m, const Input input, std::ostream &trace) {
  const auto state = m.cpu.state();
  const auto queue = m.cpu.prefetch_state();
  const auto clock = m.clock->state();
  const auto serial = m.spi->state();
  const auto peripheral = m.memory.spi.state();
  const unsigned active = final_end - (third_launch_base + (3 * delay(input))) + 1;
  const unsigned final_edges = 32 + (active / edge_period);
  const unsigned payload_bits = (active / edge_period) / 2;
  const unsigned final_samples = completed_wire_bits + payload_bits;
  const unsigned final_word =
      (completed_wire_word << payload_bits) | (next_byte >> (8U - payload_bits));
  constexpr unsigned paused_control = 0x54;
  constexpr unsigned sign = 0x80;
  const unsigned expected_gpio_flags = flags_fixed | flag_c | (input.pads == 0 ? flag_z : 0U) |
                                       ((input.pads & sign) != 0 ? flag_n : 0U);
  require(
      clock.completed_periods == final_end && clock.irq_periods == final_irq_periods &&
          clock.program_periods == final_end - final_irq_periods &&
          clock.spi_periods == final_end && clock.launches == final_launches &&
          state.pc == final_pc && state.sp == frame_sp && state.hx == 0 && state.a == byte_mask &&
          state.ccr == expected_gpio_flags && state.ccr_known == partial_flags &&
          m.memory.retired == final_retired && m.memory.flash_reads.size() == final_reads &&
          m.memory.accesses.size() == final_data && m.memory.interrupt_entries == final_entries &&
          m.memory.interrupt_acknowledgements == final_entries && queue &&
          queue->head == final_pc && queue->queued_bytes == 3 && queue->ready_bytes == 2 &&
          queue->program_byte_in_flight,
      "Independent original six-body delay/genuine RX-empty-TX IRQ/RTS/GPIO endpoint" +
          diagnostic(m));
  require(serial.completed_transfers == 2 && serial.generated_edges == final_edges &&
              serial.active && serial.active_periods == final_active_base + active &&
              serial.remaining_periods == edge_period - (active % edge_period) &&
              peripheral.control1 == paused_control && peripheral.active &&
              peripheral.edges == final_edges - 32 && peripheral.samples == payload_bits &&
              !peripheral.receive_full && !peripheral.transmit && m.samples == final_samples &&
              m.transmitted == final_word && m.memory.bytes[rx_write] == 2 &&
              m.memory.bytes[rx_latest] == received_byte &&
              m.memory.bytes[rx_previous] == received_byte &&
              m.memory.bytes[rx_buffer + 1] == received_byte &&
              m.memory.bytes[sample_cell] == input.pads,
          "Real RX/empty-TX IRQ and declared GPIO sample preserve native wire state" +
              diagnostic(m));
  const std::array<shared::Event, 3> expected{
      {{.period = return_high_read,
        .access = {.address = frame_sp - 1, .value = return_pc >> 8U, .write = false}},
       {.period = return_low_read,
        .access = {.address = frame_sp, .value = return_pc & byte_mask, .write = false}},
       {.period = gpio_write,
        .access = {.address = sample_cell, .value = input.pads, .write = true}}}};
  require(m.ram_events.size() >= expected.size() &&
              std::ranges::equal(std::span(m.ram_events).last(expected.size()), expected),
          "Real return PCH/PCL reads and native MOV of independently declared GPIO levels");
  trace << "DELAY order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
        << " pads=" << input.pads << " periods=" << final_end << " pc=" << final_pc
        << " sp=" << frame_sp << " retired=" << final_retired << " reads=" << final_reads
        << " data=" << final_data << " irq=" << final_entries
        << " irq_periods=" << final_irq_periods << " launches=" << final_launches
        << " edges=" << final_edges << " samples=" << final_samples << " tx=" << final_word
        << " active=" << final_active_base + active
        << " remaining=" << edge_period - (active % edge_period) << " gpio=" << input.pads
        << " ccr=" << expected_gpio_flags << '\n';
}

void replay(const Input input, const app::ApplicationMemory &startup, std::ostream &trace) {
  Machine parent(input, startup, call_contract);
  advance(parent, call_end);
  const unsigned irq = first_irq(input);
  constexpr unsigned rx_empty_irq_periods = 74;
  std::vector<unsigned> cuts{call_end,
                             irq - 1,
                             irq,
                             irq + 4,
                             irq + 6,
                             irq + 10,
                             irq + rx_empty_irq_periods - 10,
                             irq + rx_empty_irq_periods - 4,
                             irq + rx_empty_irq_periods - 1};
  for (unsigned period = return_high_read; period <= final_end; ++period) {
    cuts.push_back(period);
  }
  for (const unsigned cut : cuts) {
    advance(parent, cut);
    const auto saved = shared::capture(parent);
    for (unsigned index = 0; index < 2; ++index) {
      Machine fresh(input, startup, call_contract);
      shared::restore(fresh, saved);
      advance(fresh, final_end);
      require(parent.memory.flash.image() == fresh.memory.flash.image(),
              "Original FLASH unchanged");
      trace << "REPLAY order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
            << " cut=" << cut << " index=" << index << '\n';
      checked(fresh, input, trace);
    }
  }
}

void old_contract(const Input input, const app::ApplicationMemory &startup, std::ostream &trace) {
  Machine m(input, startup, Hcs08PrefetchContract::reference_call_program_cycles);
  constexpr unsigned old_end = 295;
  constexpr unsigned old_pc = 0xea9f;
  advance(m, old_end);
  const auto before = m.cpu.state();
  const auto accesses = m.memory.accesses;
  const auto error = rejected([&] { m.tick(); });
  require(error.contains("pc=0xea9f") && error.contains("opcode=0x004a") &&
              error.contains("unqualified") && m.cpu.faulted() && m.clock->faulted() &&
              m.cpu.state() == before && m.memory.accesses == accesses &&
              m.clock->state().completed_periods == old_end,
          "Prior call-only contract refuses the actual original DECA before bus effects");
  trace << "OLD order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
        << " attempted=296 completed=295 pc=" << old_pc << '\n';
}

void run(const std::span<const std::uint8_t> pages, std::ostream &trace) {
  const auto startup = shared::startup_ram(pages, trace);
  for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                           Hcs08SpiRunContract::reference_spi_before_cpu}) {
    for (const auto launch : {Hcs08SpiLaunchContract::reference_one_period,
                              Hcs08SpiLaunchContract::reference_two_periods}) {
      for (const unsigned pads : {0U, 0x55U, 0x80U, 0xffU}) {
        const Input input{.order = order, .launch = launch, .pads = pads};
        Machine m(input, startup, call_contract);
        advance(m, modify_end);
        counters(m, input);
        advance(m, final_end);
        checked(m, input, trace);
      }
      const Input input{.order = order, .launch = launch, .pads = 0x80};
      old_contract(input, startup, trace);
      replay(input, startup, trace);
    }
  }
  trace << "SUMMARY contexts=16 cuts=80 replays=160 old=4\n";
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned original pages and private subroutine trace");
    const auto pages = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    run(pages, trace);
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout << "Original native delay:16 contexts,80 cuts,160 fresh suffixes,4 strict controls. "
                 "Declared cycle/clock references; full boot and physical phase remain open.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
