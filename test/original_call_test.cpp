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

namespace {
using namespace blofeld;
using namespace blofeld::test::modify_receiver;
namespace hp = blofeld::test::hcs08_programmer;
constexpr auto call_contract = Hcs08PrefetchContract::reference_call_program_cycles;
constexpr unsigned modify_end = 288, next_pc = 0xe2a1, call_end = 294;
constexpr unsigned row_cell = 0xa6, expiry_cell = 0xa8, next_expiry = 8;
constexpr unsigned final_count = 78, final_fetches = 190, final_accesses = 75;
constexpr unsigned owned_periods = 101, owned_retired = 32, edge_count = 30;
constexpr unsigned call_target = 0xea9e, return_pc = 0xe2a4;
constexpr unsigned call_count = 79, call_fetches = 194, call_accesses = 77;
constexpr unsigned call_owned_periods = 107, call_owned_retired = 33;
constexpr unsigned low_period = 290, high_period = 291;
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

void checked(Machine &m, const Input input, std::ostream &trace) {
  const auto state = m.cpu.state();
  const auto queue = m.cpu.prefetch_state();
  const auto clock = m.clock->state();
  require(
      clock.completed_periods == call_end && clock.irq_periods == irq_periods &&
          clock.program_periods == call_end - irq_periods && clock.spi_periods == call_end &&
          state.pc == call_target && state.sp == frame_sp - 2 && state.hx == 0 && state.a == 5 &&
          state.ccr == (flags_fixed | flag_c) && state.ccr_known == partial_flags &&
          m.memory.retired == call_count && m.memory.flash_reads.size() == call_fetches &&
          m.memory.accesses.size() == call_accesses && queue &&
          queue->completed_periods == call_owned_periods &&
          queue->retired_instructions == call_owned_retired && queue->head == call_target &&
          queue->queued_bytes == 3 && queue->ready_bytes == 2 && queue->program_byte_in_flight &&
          m.memory.bytes[frame_sp] == (return_pc & byte_mask) &&
          m.memory.bytes[frame_sp - 1] == (return_pc >> 8U),
      "Independent native original JSR endpoint and actual return stack" + diagnostic(m));
  const std::array<shared::Event, 2> expected{
      {{.period = low_period,
        .access = {.address = frame_sp, .value = return_pc & byte_mask, .write = true}},
       {.period = high_period,
        .access = {.address = frame_sp - 1, .value = return_pc >> 8U, .write = true}}}};
  require(m.ram_events.size() >= expected.size() &&
              std::ranges::equal(std::span(m.ram_events).last(expected.size()), expected),
          "Genuine native PCL at s290 and PCH at s291, no host return or target injection");
  trace << "CALL order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
        << " pads=" << input.pads << " periods=294 pc=60062 sp=" << frame_sp - 2
        << " retired=79 reads=194 data=77 low_period=290 high_period=291 return_pc=58020\n";
}

void replay(const Input input, const app::ApplicationMemory &startup, std::ostream &trace) {
  Machine parent(input, startup, call_contract);
  advance(parent, modify_end);
  for (unsigned cut = modify_end; cut <= call_end; ++cut) {
    advance(parent, cut);
    const auto saved = shared::capture(parent);
    for (unsigned index = 0; index < 2; ++index) {
      Machine fresh(input, startup, call_contract);
      shared::restore(fresh, saved);
      advance(fresh, call_end);
      require(parent.memory.flash.image() == fresh.memory.flash.image(),
              "Original FLASH unchanged");
      trace << "REPLAY order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
            << " cut=" << cut << " index=" << index << '\n';
      checked(fresh, input, trace);
    }
  }
}

void old_contract(const Input input, const app::ApplicationMemory &startup, std::ostream &trace) {
  Machine m(input, startup, Hcs08PrefetchContract::reference_modify_program_cycles);
  advance(m, modify_end);
  counters(m, input);
  const auto before = m.cpu.state();
  const auto accesses = m.memory.accesses;
  const auto error = rejected([&] { m.tick(); });
  require(error.contains("pc=0xe2a1") && error.contains("opcode=0x00cd") &&
              error.contains("unqualified") && m.cpu.faulted() && m.clock->faulted() &&
              m.cpu.state() == before && m.memory.accesses == accesses &&
              m.clock->state().completed_periods == modify_end,
          "Prior RMW contract retains original JSR refusal without stack effects");
  trace << "OLD order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
        << " attempted=289 completed=288 pc=" << before.pc << '\n';
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
        advance(m, call_end);
        checked(m, input, trace);
      }
      const Input input{.order = order, .launch = launch, .pads = 0x80};
      old_contract(input, startup, trace);
      replay(input, startup, trace);
    }
  }
  trace << "SUMMARY contexts=16 cuts=28 replays=56 old=4\n";
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned original pages and private call trace");
    const auto pages = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    run(pages, trace);
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout << "Original native JSR:16 contexts,28 cuts,56 fresh suffixes,4 strict controls. "
                 "Declared cycle/clock references; full boot and physical phase remain open.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
