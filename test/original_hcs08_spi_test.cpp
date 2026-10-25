// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_shi.hpp"
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_spi_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "storage/private_file.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <sstream>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_spi_test;
namespace app = blofeld::test::hcs08_clock_startup;
namespace hp = blofeld::test::hcs08_programmer;
constexpr unsigned handler = 0xea29, return_pc = 0xe000, frame_sp = 0x47a, original_sp = 0x47f;
constexpr unsigned frame_address = frame_sp + 1, return_high = 0xe0, flags = 0x71,
                   entry_flags = 0x79;
constexpr unsigned tx_read = 0x97, tx_write = 0x98, rx_write = 0x99, rx_latest = 0x9a;
constexpr unsigned rx_previous = 0x9b, tx_buffer = 0x100, rx_buffer = 0x200, full_status = 0xa0;
constexpr unsigned byte_shift = 8, step_bound = 64, retired_total = 83, packaged_trim = 0x80;

void interrupt_entry(app::ApplicationMemory &memory, Hcs08 &cpu, const Hcs08State foreground) {
  // Own explicitly declared IRQ frame for a bounded direct original ISR test.
  // No actual interrupt admission or interrupt-entry bus timing is supplied.
  const std::array<std::uint8_t, 5> frame{
      flags, foreground.a, static_cast<std::uint8_t>(foreground.hx & byte_mask), return_high, 0};
  memory.install(frame_address, frame);
  auto entry = foreground;
  entry.pc = handler;
  entry.sp = frame_sp;
  entry.ccr = entry_flags;
  cpu.set_state(entry);
}

void handler_call(app::ApplicationMemory &memory,
                  Hcs08 &cpu,
                  const Hcs08State foreground,
                  const unsigned call,
                  std::ostream &trace) {
  constexpr std::array<unsigned, 4> costs{14, 25, 25, 19};
  interrupt_entry(memory, cpu, foreground);
  const auto before = memory.retired;
  while (cpu.state().pc != return_pc) {
    require(memory.retired - before < step_bound, "Original SPI ISR must return within its body");
    cpu.step();
  }
  require(cpu.state() == foreground && memory.retired - before == costs.at(call) &&
              !cpu.background_requested() && !cpu.faulted(),
          "Original PSHH/PULH/native RTI restore the full own frame and retained H");
  const auto expected_tx = call < event_bytes.size() ? call + 1 : event_bytes.size();
  require(memory.bytes[tx_read] == expected_tx && memory.bytes[tx_write] == event_bytes.size() &&
              memory.bytes[rx_write] == call &&
              memory.bytes[rx_latest] == (call == 0 ? 0 : reply_bytes[call - 1]) &&
              memory.bytes[rx_previous] == (call < 2 ? 0 : reply_bytes[call - 2]),
          "Actual original ISR updates exact TX/RX cursors and previous/current peer data");
  for (unsigned n = 0; n < call; ++n) {
    require(memory.known[rx_buffer + n] && memory.bytes[rx_buffer + n] == reply_bytes[n],
            "Original receive path stores only the actual serialized peer bytes");
  }
  if (call < event_bytes.size()) {
    require(
        memory.spi.state().transmit == event_bytes[call] && !memory.spi.interrupt_requested(),
        "Original SPIS read/actual SPID store fills the real transmit buffer and clears its IRQ");
  } else {
    require(memory.spi.read(spi::control1) == quiet_control && !memory.spi.interrupt_requested() &&
                !memory.spi.state().transmit,
            "Original empty-ring branch writes54 to disable transmit IRQ without fake output");
  }
  trace << "ISR call=" << call << " instructions=" << memory.retired - before
        << " tx=" << static_cast<unsigned>(memory.bytes[tx_read])
        << " rx=" << static_cast<unsigned>(memory.bytes[rx_write])
        << " latest=" << static_cast<unsigned>(memory.bytes[rx_latest])
        << " previous=" << static_cast<unsigned>(memory.bytes[rx_previous])
        << " control=" << static_cast<unsigned>(memory.spi.read(spi::control1)) << '\n';
}

void unchanged_supplied_pages(const app::ApplicationMemory &memory,
                              const std::span<const std::uint8_t> records) {
  const auto &image = memory.flash.image();
  for (unsigned page = 0; page < app::pages.size(); ++page) {
    for (unsigned n = 0; n < app::hf::page_bytes; ++n) {
      const auto offset = app::pages[page] - app::hf::first + n;
      require(image.known[offset] &&
                  image.data[offset] == records[(page * hp::record_bytes) + 2 + n],
              "Every supplied original instruction/data/calibration byte remains unchanged");
    }
  }
}

void run(const std::span<const std::uint8_t> records, const unsigned seed, std::ostream &trace) {
  app::ApplicationMemory memory(records, {.seed = 0, .trim = packaged_trim, .observation = -1});
  memory.warm_reset();
  const auto image = memory.flash.image();
  unchanged_supplied_pages(memory, records);
  constexpr std::array<std::uint8_t, 5> cursors{0, 3, 0, 0, 0};
  memory.install(tx_read, cursors);
  memory.install(tx_buffer, event_bytes);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  const Hcs08State foreground{
      .pc = return_pc,
      .hx = static_cast<std::uint16_t>((seed << byte_shift) | (byte_mask - seed)),
      .sp = original_sp,
      .a = static_cast<std::uint8_t>(seed ^ byte_mask),
      .ccr = flags};
  configure(memory.spi);
  DspShi peer;
  select(peer);
  trace << "CASE seed=" << seed << " hx=" << foreground.hx
        << " a=" << static_cast<unsigned>(foreground.a) << '\n';
  for (unsigned call = 0; call < 4; ++call) {
    require(memory.spi.interrupt_requested(),
            "Actual peripheral request precedes every bounded direct original ISR entry");
    handler_call(memory, cpu, foreground, call, trace);
    if (call >= event_bytes.size()) { continue; }
    memory.spi.transfer_to_shifter();
    for (unsigned n = 0; n < edges; ++n) { coupled_edge(memory.spi, peer); }
    memory.spi.complete_transfer();
    require(memory.spi.read(spi::status) == full_status &&
                memory.spi.state().receive == reply_bytes[call],
            "All sixteen actual edges receive the exact peer byte before the next native ISR");
    trace << "BYTE index=" << call << " edges=" << memory.spi.state().edges
          << " samples=" << memory.spi.state().samples
          << " tx=" << static_cast<unsigned>(event_bytes[call])
          << " rx=" << static_cast<unsigned>(memory.spi.state().receive) << '\n';
  }
  require(
      peer.read(hrx) == event && memory.retired == retired_total && memory.flash.image() == image &&
          !memory.options.written(),
      "Original SPI ISR plus real serial wires creates exactly one DSP word without NVM changes");
  trace << "END retired=" << memory.retired << " dsp=" << event << " return=" << cpu.state().pc
        << " sp=" << cpu.state().sp << " ccr=" << static_cast<unsigned>(cpu.state().ccr) << '\n';
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned supplied pages and private original-SPI trace path");
    const auto records = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    for (unsigned seed = 0; seed < 8; ++seed) { run(records, seed, trace); }
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout << "Original HCS08 SPI ISR and master edges deliver actual DSP words and receive "
                 "peer data.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
