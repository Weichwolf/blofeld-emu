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
#include "receiver_fixture.hpp"
#include "storage/private_file.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <ostream>
#include <span>
#include <sstream>

namespace {
using namespace blofeld;
using namespace blofeld::test::receiver;
namespace hp = blofeld::test::hcs08_programmer;
namespace hw = blofeld::test::hcs08;

void advance(Machine &m, const unsigned end) {
  while (m.clock->state().completed_periods < end) {
    m.tick();
    if (m.clock->state().completed_periods == receive_end) { receive_endpoint(m); }
  }
}

void same(const Machine &parent, const Machine &fresh) {
  hw::require(
      parent.cpu.state() == fresh.cpu.state() &&
          parent.cpu.prefetch_state() == fresh.cpu.prefetch_state() &&
          parent.clock->state() == fresh.clock->state() &&
          parent.spi->state() == fresh.spi->state() &&
          parent.memory.spi.state() == fresh.memory.spi.state() &&
          parent.memory.adc.state() == fresh.memory.adc.state() &&
          parent.memory.bytes == fresh.memory.bytes && parent.memory.known == fresh.memory.known &&
          parent.memory.partial_known == fresh.memory.partial_known &&
          parent.memory.accesses == fresh.memory.accesses &&
          parent.memory.flash_reads == fresh.memory.flash_reads && parent.events == fresh.events &&
          parent.ram_events == fresh.ram_events && parent.memory.retired == fresh.memory.retired &&
          parent.transmitted == fresh.transmitted && parent.samples == fresh.samples,
      "Joint fresh ownership preserves every captured CPU/SPI/ADC/RAM event");
}

void frontier(Machine &m, std::ostream &trace) {
  const auto before = m.cpu.state();
  const auto bytes = m.memory.bytes;
  const auto accesses = m.memory.accesses;
  const auto reads = m.memory.flash_reads;
  const auto error = hw::rejected([&] { m.tick(); });
  hw::require(error.contains("pc=0xe258") && error.contains("opcode=0x003f") &&
                  error.contains("unqualified") && m.cpu.faulted() && m.clock->faulted() &&
                  m.cpu.state() == before && m.memory.bytes == bytes &&
                  m.memory.accesses == accesses && m.memory.flash_reads == reads &&
                  m.clock->state().attempted_periods == finish_period + 1 &&
                  m.clock->state().completed_periods == finish_period,
              "The next original CLR retains its visible unsupported cycle frontier: " + error);
  static_cast<void>(hw::rejected([&] { m.tick(); }));
  trace << "FRONTIER attempted=269 completed=268 pc=" << frontier_pc << '\n';
}

void checked(Machine &m,
             const Input input,
             const app::ApplicationMemory &startup,
             std::ostream &trace) {
  endpoint(m, input, trace);
  hw::require(m.memory.flash.image() == startup.flash.image(),
              "Original code/pages and NVM history remain unchanged throughout RX/TX/RTI");
  frontier(m, trace);
}

void old_contract(const Input input, const app::ApplicationMemory &startup, std::ostream &trace) {
  Machine m(input, startup, Hcs08PrefetchContract::reference_scanner_move_cycles);
  advance(m, store_cycle - 2);
  const auto before = m.cpu.state();
  const auto bytes = m.memory.bytes;
  const auto accesses = m.memory.accesses;
  const auto error = hw::rejected([&] { m.tick(); });
  hw::require(error.contains("pc=0xea39") && error.contains("opcode=0x00d7") &&
                  error.contains("unqualified") && m.cpu.state() == before &&
                  before.pc == store_pc && m.memory.bytes == bytes &&
                  m.memory.accesses == accesses && m.memory.bytes[rx_buffer] == 0 &&
                  m.memory.bytes[rx_write] == 0 && m.memory.bytes[rx_latest] == received_byte &&
                  m.memory.retired == previous_contract_retired &&
                  m.clock->state().completed_periods == previous_contract_end,
              "Prior contracts preserve their actual original indexed-store refusal: " + error);
  trace << "OLD order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
        << " attempted=213 completed=212 pc=" << store_pc << '\n';
}

void replay(const Input input, const app::ApplicationMemory &startup, std::ostream &trace) {
  Machine parent(input, startup);
  advance(parent, prefix_end);
  for (unsigned cut = prefix_end; cut <= finish_period; ++cut) {
    advance(parent, cut);
    const auto saved = shared::capture(parent);
    for (unsigned index = 0; index < 2; ++index) {
      Machine fresh(input, startup);
      shared::restore(fresh, saved);
      same(parent, fresh);
      advance(fresh, finish_period);
      trace << "REPLAY order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
            << " cut=" << cut << " index=" << index << '\n';
      checked(fresh, input, startup, trace);
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
        checked(m, input, startup, trace);
      }
      const Input input{.order = order, .launch = launch, .pads = 0x80};
      old_contract(input, startup, trace);
      replay(input, startup, trace);
    }
  }
  trace << "SUMMARY contexts=16 cuts=372 replays=744 old=4\n";
}
}

int main(const int count, char **arguments) {
  try {
    hw::require(count == 3, "Expected pinned original pages and private RX-clock trace");
    const auto pages = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    run(pages, trace);
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout << "Original native RX ring store/TX/RTI:16 contexts, 372 cuts, 744 fresh suffixes. "
                 "Declared cycle/clock references; full boot and physical phase remain open.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
