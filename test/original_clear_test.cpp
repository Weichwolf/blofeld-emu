// SPDX-License-Identifier: GPL-3.0-only
#include "clear_receiver_fixture.hpp"
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_serial_clock_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi_clock.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"
#include "storage/private_file.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <ostream>
#include <span>
#include <sstream>

namespace {
using namespace blofeld;
using namespace blofeld::test::clear_receiver;
namespace hp = blofeld::test::hcs08_programmer;
constexpr unsigned clear_end = 273, next_pc = 0xe25a, row_cell = 0xa6;
constexpr unsigned clear_retired = 73, clear_reads = 179, clear_ram = 72;
constexpr unsigned clear_owned = 86, clear_owned_retired = 27, clear_edges = 28;
constexpr unsigned read_period = 269, write_period = 271;

void advance(Machine &m, const unsigned end) {
  while (m.clock->state().completed_periods < end) {
    m.tick();
    if (m.clock->state().completed_periods == receive_end) { receive_endpoint(m); }
  }
}

void checked(Machine &m, const Input input, std::ostream &trace) {
  const auto state = m.cpu.state();
  const auto queue = m.cpu.prefetch_state();
  require(m.clock->state().completed_periods == clear_end &&
              m.clock->state().irq_periods == irq_periods && state.pc == next_pc &&
              state.sp == frame_sp && state.hx == 0 && state.a == 0 &&
              state.ccr == (flags_fixed | flag_z) && state.ccr_known == partial_flags &&
              m.memory.retired == clear_retired && m.memory.flash_reads.size() == clear_reads &&
              m.memory.accesses.size() == clear_ram && m.memory.bytes[row_cell] == 0 && queue &&
              queue->completed_periods == clear_owned &&
              queue->retired_instructions == clear_owned_retired &&
              m.spi->state().generated_edges == clear_edges &&
              m.spi->state().completed_transfers == 1 && m.spi->state().active,
          "Original CLR r269/f270/w271/p272/p273 and independent final state" + diagnostic(m));
  const auto size = m.ram_events.size();
  require(size >= 2 && m.ram_events[size - 2].period == read_period &&
              m.ram_events[size - 2].access.address == row_cell &&
              !m.ram_events[size - 2].access.write &&
              m.ram_events[size - 1].period == write_period &&
              m.ram_events[size - 1].access.address == row_cell &&
              m.ram_events[size - 1].access.write && m.ram_events[size - 1].access.value == 0,
          "The original direct operand generates its real read and native write");
  trace << "CLEAR order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
        << " pads=" << input.pads << " periods=" << clear_end << " pc=" << next_pc
        << " retired=" << clear_retired << " reads=" << clear_reads << " data=" << clear_ram
        << " edges=" << clear_edges << " row=0\n";
  const auto before = m.cpu.state();
  const auto bytes = m.memory.bytes;
  const auto accesses = m.memory.accesses;
  const auto error = rejected([&] { m.tick(); });
  require(error.contains("pc=0xe25a") && error.contains("opcode=0x003c") &&
              error.contains("unqualified") && m.cpu.faulted() && m.clock->faulted() &&
              m.cpu.state() == before && m.memory.bytes == bytes && m.memory.accesses == accesses,
          "The next original INC preserves its visible strict unsupported-cycle boundary: " +
              error);
  trace << "FRONTIER attempted=274 completed=273 pc=" << next_pc << '\n';
}

void replay(const Input input, const app::ApplicationMemory &startup, std::ostream &trace) {
  Machine parent(input, startup);
  advance(parent, finish_period);
  for (unsigned cut = finish_period; cut <= clear_end; ++cut) {
    advance(parent, cut);
    const auto saved = shared::capture(parent);
    for (unsigned index = 0; index < 2; ++index) {
      Machine fresh(input, startup);
      shared::restore(fresh, saved);
      advance(fresh, clear_end);
      require(parent.memory.flash.image() == fresh.memory.flash.image(),
              "Original FLASH unchanged");
      trace << "REPLAY order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
            << " cut=" << cut << " index=" << index << '\n';
      checked(fresh, input, trace);
    }
  }
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
        advance(m, clear_end);
        checked(m, input, trace);
      }
      replay({.order = order, .launch = launch, .pads = 0x80}, startup, trace);
    }
  }
  trace << "SUMMARY contexts=16 cuts=24 replays=48\n";
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned original pages and private CLR trace");
    const auto pages = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    run(pages, trace);
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout << "Original native CLR:16 contexts,24 cuts,48 fresh suffixes. "
                 "Declared cycle/clock references; full boot and physical phase remain open.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
