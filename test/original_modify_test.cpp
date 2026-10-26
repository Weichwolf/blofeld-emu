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
constexpr unsigned clear_end = 273, modify_end = 288, next_pc = 0xe2a1;
constexpr unsigned row_cell = 0xa6, expiry_cell = 0xa8, caller_seed = 7, next_expiry = 8;
constexpr unsigned final_count = 78, final_fetches = 190, final_accesses = 75;
constexpr unsigned owned_periods = 101, owned_retired = 32, edge_count = 30;
constexpr unsigned transmitted = 0x481e, max_active = 245, wire_samples = 15;
constexpr unsigned read_period = 274, write_period = 276, load_period = 279;

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
  counters(m, input);
  constexpr std::array<shared::Event, 3> expected{
      {{.period = read_period,
        .access = {.address = expiry_cell, .value = caller_seed, .write = false}},
       {.period = write_period,
        .access = {.address = expiry_cell, .value = next_expiry, .write = true}},
       {.period = load_period,
        .access = {.address = expiry_cell, .value = next_expiry, .write = false}}}};
  require(m.ram_events.size() >= expected.size(), "Original counter event suffix exists");
  const auto actual = std::span(m.ram_events).last(expected.size());
  require(
      std::ranges::equal(expected, actual),
      "Genuine native r274/w276 and original dependent r279; no host counter or result injection");
  trace << "MODIFY order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
        << " pads=" << input.pads << " periods=" << modify_end << " pc=" << next_pc
        << " retired=" << final_count << " reads=" << final_fetches << " data=" << final_accesses
        << " edges=" << edge_count << " expiry=" << next_expiry << " transmitted=" << transmitted
        << '\n';
  const auto before = m.cpu.state();
  const auto bytes = m.memory.bytes;
  const auto accesses = m.memory.accesses;
  const auto error = rejected([&] { m.tick(); });
  require(error.contains("pc=0xe2a1") && error.contains("opcode=0x00cd") &&
              error.contains("unqualified") && m.cpu.faulted() && m.clock->faulted() &&
              m.cpu.state() == before && m.memory.bytes == bytes && m.memory.accesses == accesses,
          "Next original JSR retains its explicit unsupported-cycle frontier: " + error);
  trace << "FRONTIER attempted=289 completed=288 pc=" << next_pc << '\n';
}

void replay(const Input input, const app::ApplicationMemory &startup, std::ostream &trace) {
  Machine parent(input, startup);
  advance(parent, clear_end);
  for (unsigned cut = clear_end; cut <= modify_end; ++cut) {
    advance(parent, cut);
    const auto saved = shared::capture(parent);
    for (unsigned index = 0; index < 2; ++index) {
      Machine fresh(input, startup);
      shared::restore(fresh, saved);
      advance(fresh, modify_end);
      require(parent.memory.flash.image() == fresh.memory.flash.image(),
              "Original FLASH unchanged");
      trace << "REPLAY order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
            << " cut=" << cut << " index=" << index << '\n';
      checked(fresh, input, trace);
    }
  }
}

void old_contract(const Input input, const app::ApplicationMemory &startup, std::ostream &trace) {
  Machine m(input, startup, Hcs08PrefetchContract::reference_clear_program_cycles);
  advance(m, clear_end);
  const auto before = m.cpu.state();
  const auto accesses = m.memory.accesses;
  require(m.memory.bytes[expiry_cell] == caller_seed, "Declared seed7 remains before native INC");
  const auto error = rejected([&] { m.tick(); });
  require(error.contains("pc=0xe25a") && error.contains("opcode=0x003c") &&
              error.contains("unqualified") && m.cpu.faulted() && m.clock->faulted() &&
              m.cpu.state() == before && m.memory.accesses == accesses &&
              m.memory.bytes[expiry_cell] == caller_seed &&
              m.clock->state().completed_periods == clear_end,
          "Prior CLR contract still rejects actual original INC before its r period");
  trace << "OLD order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
        << " attempted=274 completed=273 pc=" << before.pc << '\n';
}

void run(const std::span<const std::uint8_t> pages, std::ostream &trace) {
  const auto startup = shared::startup_ram(pages, trace);
  for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                           Hcs08SpiRunContract::reference_spi_before_cpu}) {
    for (const auto launch : {Hcs08SpiLaunchContract::reference_one_period,
                              Hcs08SpiLaunchContract::reference_two_periods}) {
      for (const unsigned pads : {0U, 0x55U, 0x80U, 0xffU}) {
        const Input input{.order = order, .launch = launch, .pads = pads};
        Machine m(input, startup);
        advance(m, finish_period);
        endpoint(m, input, trace);
        advance(m, modify_end);
        checked(m, input, trace);
      }
      const Input input{.order = order, .launch = launch, .pads = 0x80};
      old_contract(input, startup, trace);
      replay(input, startup, trace);
    }
  }
  trace << "SUMMARY contexts=16 cuts=64 replays=128 old=4\n";
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned original pages and private RMW trace");
    const auto pages = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    run(pages, trace);
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout << "Original native INC:16 contexts,64 cuts,128 fresh suffixes,4 strict controls. "
                 "Declared cycle/clock references; full boot and physical phase remain open.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
