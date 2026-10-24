// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_flash.hpp"
#include "blofeld/hcs08_known_memory.hpp"
#include "blofeld/hcs08_spi.hpp"
#include "blofeld/private_file.hpp"
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_spi_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <sstream>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_spi_test;
namespace app = blofeld::test::hcs08_clock_startup;
namespace hp = blofeld::test::hcs08_programmer;
using blofeld::test::hcs08::Access;
constexpr unsigned handler = 0xea29, return_pc = 0xe000, original_sp = 0x47f;
constexpr unsigned vector = 0xffe0, saved_ccr = 0x47b, ccr_mask = 0xef, flags = 0x61;
constexpr unsigned tx_read = 0x97, tx_write = 0x98, rx_write = 0x99, rx_latest = 0x9a;
constexpr unsigned rx_previous = 0x9b, tx_buffer = 0x100, rx_buffer = 0x200;
constexpr unsigned byte_shift = 8, step_bound = 64, packaged_trim = 0x80;
constexpr unsigned entered_flags = 0x69, return_high = 0xe0, handler_high = 0xea,
                   handler_low = 0x29;
constexpr unsigned serial_retirements = 83, pipeline_retirements = 72;

struct Result {
  unsigned instructions, tx, rx, latest, previous, control;
};

constexpr std::array<Result, 4> after_byte{
    {{.instructions = 14, .tx = 1, .rx = 0, .latest = 0, .previous = 0, .control = 0x74},
     {.instructions = 25, .tx = 2, .rx = 1, .latest = 0x12, .previous = 0, .control = 0x74},
     {.instructions = 25, .tx = 3, .rx = 2, .latest = 0x34, .previous = 0x12, .control = 0x74},
     {.instructions = 19, .tx = 3, .rx = 3, .latest = 0x56, .previous = 0x34, .control = 0x54}}};
constexpr std::array<Result, 4> prefetch{
    {{.instructions = 14, .tx = 1, .rx = 0, .latest = 0, .previous = 0, .control = 0x74},
     {.instructions = 14, .tx = 2, .rx = 0, .latest = 0, .previous = 0, .control = 0x74},
     {.instructions = 25, .tx = 3, .rx = 1, .latest = 0x12, .previous = 0, .control = 0x74},
     {.instructions = 19, .tx = 3, .rx = 2, .latest = 0x34, .previous = 0x12, .control = 0x54}}};

void enter(app::ApplicationMemory &memory, Hcs08 &cpu, const Hcs08State foreground) {
  const auto accesses = memory.accesses.size();
  const auto reads = memory.flash_reads.size();
  const auto retired = memory.retired;
  const auto entries = memory.interrupt_entries;
  const auto spi_state = memory.spi.state();
  require(cpu.state() == foreground && cpu.pending_interrupt() == vector && cpu.service_interrupt(),
          "Actual original SPI source selects the actual original vector at a CPU boundary");
  auto expected = foreground;
  expected.pc = handler;
  expected.sp = original_sp - 5;
  expected.ccr = entered_flags;
  const std::vector<Access> frame{
      {.address = original_sp, .value = 0, .write = true},
      {.address = original_sp - 1, .value = return_high, .write = true},
      {.address = original_sp - 2, .value = foreground.hx & byte_mask, .write = true},
      {.address = original_sp - 3, .value = foreground.a, .write = true},
      {.address = saved_ccr, .value = flags, .write = true, .known = ccr_mask}};
  require(cpu.state() == expected && memory.retired == retired &&
              memory.interrupt_entries == entries + 1 &&
              memory.interrupt_acknowledgements == entries + 1 && memory.spi.state() == spi_state &&
              memory.accesses.size() == accesses + frame.size() &&
              std::vector<Access>(memory.accesses.begin() + static_cast<std::ptrdiff_t>(accesses),
                                  memory.accesses.end()) == frame &&
              memory.flash_reads.size() == reads + 2 &&
              memory.flash_reads[reads] ==
                  Access{.address = vector, .value = handler_high, .write = false} &&
              memory.flash_reads[reads + 1] ==
                  Access{.address = vector + 1, .value = handler_low, .write = false} &&
              !memory.known[saved_ccr] && memory.partial_known[saved_ccr] == ccr_mask,
          "Actual IRQ writes the five-byte hardware frame, retains unknown H flag and reads "
          "EA29 from FLASH without retiring an instruction or clearing peripheral flags");
}

void service(app::ApplicationMemory &memory,
             Hcs08 &cpu,
             const Hcs08State foreground,
             const Result result,
             std::ostream &trace) {
  enter(memory, cpu, foreground);
  const auto before = memory.retired;
  while (cpu.state().pc != return_pc) {
    require(memory.retired - before < step_bound,
            "Original IRQ must return inside its native body");
    cpu.step();
  }
  require(cpu.state() == foreground && memory.retired - before == result.instructions &&
              !cpu.background_requested() && !cpu.faulted() && memory.bytes[tx_read] == result.tx &&
              memory.bytes[tx_write] == 3 && memory.bytes[rx_write] == result.rx &&
              memory.bytes[rx_latest] == result.latest &&
              memory.bytes[rx_previous] == result.previous &&
              memory.spi.read(spi::control1) == result.control && !cpu.pending_interrupt(),
          "Native original handler restores its real IRQ frame and independently expected rings");
  for (unsigned n = 0; n < result.rx; ++n) {
    require(memory.known[rx_buffer + n] && memory.bytes[rx_buffer + n] == reply_bytes[n],
            "Original RX stores only actual completed peer bytes");
  }
  trace << "IRQ call=" << memory.interrupt_entries - 1 << " instructions=" << result.instructions
        << " tx=" << result.tx << " rx=" << result.rx << " latest=" << result.latest
        << " previous=" << result.previous << " control=" << result.control
        << " ccr=" << static_cast<unsigned>(cpu.state().ccr)
        << " known=" << static_cast<unsigned>(cpu.state().ccr_known) << '\n';
}

void shift(app::ApplicationMemory &memory,
           DspShi &peer,
           const unsigned index,
           std::ostream &trace) {
  for (unsigned edge = 0; edge < edges; ++edge) { coupled_edge(memory.spi, peer); }
  memory.spi.complete_transfer();
  const auto s = memory.spi.state();
  require(s.receive_full && s.receive == reply_bytes[index] && s.edges == 16 && s.samples == 8,
          "Actual DSP MISO pins produce the independently expected received byte");
  trace << "BYTE index=" << index << " edges=" << s.edges << " samples=" << s.samples
        << " tx=" << static_cast<unsigned>(event_bytes[index])
        << " rx=" << static_cast<unsigned>(s.receive) << '\n';
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

void reset_storage_policy(const std::span<const std::uint8_t> records) {
  app::ApplicationMemory memory(records, {.seed = 0, .trim = packaged_trim, .observation = -1});
  memory.warm_reset();
  const std::array<std::uint8_t, 1> own_byte{flags};
  memory.install(saved_ccr, own_byte);
  memory.begin_reset({.pc = return_pc, .ccr = flags});
  for (unsigned operation = 0; operation < 3; ++operation) {
    const auto error = rejected([&] {
      if (operation == 0) {
        static_cast<void>(memory.read_known_byte(saved_ccr));
      } else if (operation == 1) {
        memory.write_known_byte({.address = saved_ccr, .value = flags}, ccr_mask);
      } else {
        memory.write(saved_ccr, flags);
      }
    });
    require(error.contains("address=0x047b pc=0xe000 ccr=0x0061") &&
                error.contains("reset transaction") && memory.accesses.empty() &&
                memory.known[saved_ccr] && memory.bytes[saved_ccr] == flags,
            "Qualified application masked reads/writes and ordinary writes cannot bypass reset "
            "transaction ownership or commit a RAM prefix");
  }
  memory.end_reset();
  require(memory.read_known_byte(saved_ccr).known == byte_mask,
          "The real owner resumes fully known RAM reads after reset completes");
  memory.write_known_byte({.address = saved_ccr, .value = flags}, ccr_mask);
  require(!memory.known[saved_ccr] && memory.partial_known[saved_ccr] == ccr_mask &&
              rejected([&] { (void)memory.read(saved_ccr); }).contains("address=0x047b"),
          "Explicit masked storage remains available outside reset without ordinary zero fill");
}

Hcs08State bus_foreground(const unsigned seed) {
  return {.pc = return_pc,
          .hx = static_cast<std::uint16_t>((seed << byte_shift) | (byte_mask - seed)),
          .sp = original_sp,
          .a = static_cast<std::uint8_t>(seed ^ byte_mask),
          .ccr_known = ccr_mask,
          .ccr = flags};
}

void prepare_bus(app::ApplicationMemory &memory, Hcs08 &cpu, const unsigned seed) {
  memory.warm_reset();
  constexpr std::array<std::uint8_t, 5> cursors{0, 3, 0, 0, 0};
  memory.install(tx_read, cursors);
  memory.install(tx_buffer, event_bytes);
  cpu.set_state(bus_foreground(seed)); // Own foreground only, never an IRQ handler/frame.
  configure(memory.spi);
  require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
          "Actual original SPI source admits the explicit11-period bus sequence");
}

void bus_cycles(Hcs08 &cpu, const unsigned count, unsigned &periods) {
  for (unsigned n = 0; n < count; ++n) {
    cpu.advance_irq_bus();
    ++periods;
  }
}

void bus_endpoint(const app::ApplicationMemory &memory,
                  const Hcs08 &cpu,
                  const unsigned seed,
                  std::ostream &trace) {
  auto expected = bus_foreground(seed);
  expected.pc = handler;
  expected.sp = original_sp - 5;
  expected.ccr = entered_flags;
  const auto state = cpu.irq_bus_state();
  if (!state) { throw Fault("Original IRQ must retain its owned program queue"); }
  const auto &image = memory.flash.image();
  constexpr unsigned cycles = 11;
  require(cpu.state() == expected && state->completed_cycles == cycles &&
              state->attempted_cycles == cycles && state->vector == vector &&
              state->target == handler && state->ready_program_bytes == 2 &&
              state->program_byte_in_flight && state->prefetched_bytes == 3 &&
              state->acknowledged && memory.retired == 0 && memory.interrupt_entries == 1 &&
              memory.interrupt_acknowledgements == 1 && memory.spi.interrupt_requested() &&
              !memory.known[saved_ccr] && memory.partial_known[saved_ccr] == ccr_mask &&
              !memory.options.written(),
          "Original FLASH vector and program reads retain the genuine frame/queue without an ISR "
          "opcode");
  constexpr std::array<unsigned, 5> addresses{
      vector, vector + 1, handler, handler + 1, handler + 2};
  require(memory.flash_reads.size() == addresses.size(),
          "Exactly five actual original FLASH reads");
  for (unsigned n = 0; n < addresses.size(); ++n) {
    const auto address = addresses[n];
    require(memory.flash_reads[n] == Access{.address = address,
                                            .value = image.data[address - app::hf::first],
                                            .write = false},
            "Independent vector/program address order matches the pinned original bytes");
    if (n >= 2) {
      require(state->prefetched[n - 2] == image.data[address - app::hf::first],
              "Actual captured bytes retain the original immutable program image");
    }
  }
  trace << "END seed=" << seed << " cycles=" << state->completed_cycles << " pc=" << cpu.state().pc
        << " sp=" << cpu.state().sp << " ccr=" << static_cast<unsigned>(cpu.state().ccr)
        << " known=" << static_cast<unsigned>(cpu.state().ccr_known)
        << " ready=" << state->ready_program_bytes << " inflight=" << state->program_byte_in_flight
        << " q0=" << static_cast<unsigned>(state->prefetched[0])
        << " q1=" << static_cast<unsigned>(state->prefetched[1])
        << " q2=" << static_cast<unsigned>(state->prefetched[2]) << " retired=" << memory.retired
        << '\n';
}

void bus_replays(const std::span<const std::uint8_t> records,
                 std::ostream &trace,
                 unsigned &periods) {
  constexpr unsigned seed = 7;
  constexpr unsigned cycles = 11;
  app::ApplicationMemory initial(records, {.seed = 0, .trim = packaged_trim, .observation = -1});
  Hcs08 first(initial, BLOFELD_HCS08_SLA);
  prepare_bus(initial, first, seed);
  const auto zero = first.checkpoint();
  std::optional<app::ApplicationMemory> endpoint;
  for (unsigned cut = 0; cut <= cycles; ++cut) {
    auto partial = initial;
    Hcs08 cut_cpu(partial, BLOFELD_HCS08_SLA);
    cut_cpu.restore_checkpoint(zero);
    bus_cycles(cut_cpu, cut, periods);
    const auto saved = cut_cpu.checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      auto restored = partial;
      Hcs08 other(restored, BLOFELD_HCS08_SLA);
      other.restore_checkpoint(saved);
      require(other.state() == cut_cpu.state() && other.irq_bus_state() == cut_cpu.irq_bus_state(),
              "Actual partial original frame/vector/program queue survives a fresh joint restore");
      bus_cycles(other, cycles - cut, periods);
      trace << "REPLAY cut=" << cut << " index=" << replay << '\n';
      bus_endpoint(restored, other, seed, trace);
      unchanged_supplied_pages(restored, records);
      if (endpoint) {
        app::same_endpoint(restored, *endpoint);
      } else {
        endpoint = restored;
      }
    }
  }
}

void run_bus(const std::span<const std::uint8_t> records, std::ostream &trace) {
  constexpr unsigned cycles = 11;
  unsigned periods = 0;
  for (unsigned seed = 0; seed < 8; ++seed) {
    app::ApplicationMemory memory(records, {.seed = 0, .trim = packaged_trim, .observation = -1});
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
    prepare_bus(memory, cpu, seed);
    bus_cycles(cpu, cycles, periods);
    bus_endpoint(memory, cpu, seed, trace);
    unchanged_supplied_pages(memory, records);
  }
  bus_replays(records, trace, periods);
  trace << "SUMMARY periods=" << periods << " baseline=8 replays=24 retired=0\n";
}

void run(const std::span<const std::uint8_t> records,
         const unsigned seed,
         const bool pipeline,
         std::ostream &trace) {
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
      .ccr_known = ccr_mask,
      .ccr = flags};
  cpu.set_state(
      foreground); // Own foreground once; all ISR PCs/frames come from real IRQ admission.
  configure(memory.spi);
  DspShi peer;
  select(peer);
  trace << "CASE seed=" << seed << " pipeline=" << pipeline << " hx=" << foreground.hx
        << " a=" << static_cast<unsigned>(foreground.a) << '\n';
  const auto &results = pipeline ? prefetch : after_byte;
  service(memory, cpu, foreground, results[0], trace);
  for (unsigned index = 0; index < event_bytes.size(); ++index) {
    require(memory.spi.state().transmit == event_bytes[index],
            "Only the original native SPID store supplies each pending byte");
    memory.spi.transfer_to_shifter();
    if (pipeline) { service(memory, cpu, foreground, results[index + 1], trace); }
    shift(memory, peer, index, trace);
    if (!pipeline) { service(memory, cpu, foreground, results[index + 1], trace); }
  }
  const auto state = memory.spi.state();
  require(peer.read(hrx) == event &&
              memory.retired == (pipeline ? pipeline_retirements : serial_retirements) &&
              memory.interrupt_entries == 4 && memory.interrupt_acknowledgements == 4 &&
              memory.flash.image() == image && !memory.options.written() &&
              !cpu.pending_interrupt() && state.receive_full == pipeline &&
              state.receive == reply_bytes.back() && !state.transmit,
          "Both explicit untimed schedules deliver actual DSP data; prefetch leaves the last "
          "RX byte unread because the original empty-ring path disables SPI interrupts");
  unchanged_supplied_pages(memory, records);
  trace << "END retired=" << memory.retired << " entries=" << memory.interrupt_entries
        << " ack=" << memory.interrupt_acknowledgements << " dsp=" << event
        << " pending_rx=" << state.receive_full
        << " receive=" << static_cast<unsigned>(state.receive) << " return=" << cpu.state().pc
        << " sp=" << cpu.state().sp << '\n';
}
}

int main(const int count, char **arguments) {
  try {
    const bool bus_mode = count == 4 && std::string_view(arguments[3]) == "--irq-bus";
    require(count == 3 || bus_mode,
            "Expected pinned supplied pages, private trace and optional IRQ-bus mode");
    const auto records = hp::read_input(arguments[1], app::records_bytes);
    reset_storage_policy(records);
    std::ostringstream trace;
    if (bus_mode) {
      run_bus(records, trace);
    } else {
      for (unsigned seed = 0; seed < 8; ++seed) {
        for (const bool pipeline : {false, true}) { run(records, seed, pipeline, trace); }
      }
    }
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    if (bus_mode) {
      std::cout
          << "Original HCS08 IRQ bus sequence retains actual program bytes and joint state.\n";
    } else {
      std::cout
          << "Original HCS08 SPI interrupts preserve partial CCR through actual IRQ and RTI.\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
