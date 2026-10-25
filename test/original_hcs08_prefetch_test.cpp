// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_spi_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "storage/private_file.hpp"

#include <array>
#include <bit>
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
using namespace blofeld::test::hcs08;
namespace app = blofeld::test::hcs08_clock_startup;
namespace hp = blofeld::test::hcs08_programmer;
namespace wire = blofeld::test::hcs08_spi_test;
constexpr unsigned handler = 0xea29, stack = 0x47f, flags = 0x61, partial = 0xef;
constexpr unsigned foreground_pc = 0xe000, entered_flags = 0x69;
constexpr unsigned cursor = 0x97, tx = 0x100, irq_cycles = 11;
constexpr unsigned spi_vector = 0xffe0;
constexpr auto program_contract = Hcs08PrefetchContract::reference_linear_program_cycles;

Hcs08PrefetchState owned_program(const Hcs08 &cpu) {
  const auto q = cpu.prefetch_state();
  if (!q) { throw Fault("Expected owned prefetch queue"); }
  return *q;
}

void prepare(app::ApplicationMemory &memory,
             Hcs08 &cpu,
             const unsigned seed,
             unsigned &periods,
             const Hcs08PrefetchContract mode = program_contract,
             const unsigned queued = 3) {
  memory.warm_reset();
  require(queued <= 3, "Declared original TX fixture is empty or contains at most three bytes");
  const std::array<std::uint8_t, 5> cursors{0, static_cast<std::uint8_t>(queued), 0, 0, 0};
  memory.install(cursor, cursors);
  memory.install(tx, wire::event_bytes);
  cpu.set_state({.pc = foreground_pc,
                 .hx = static_cast<std::uint16_t>((seed << 8U) | (byte_mask - seed)),
                 .sp = stack,
                 .a = static_cast<std::uint8_t>(seed ^ byte_mask),
                 .ccr_known = partial,
                 .ccr = flags}); // Declared foreground; no handler PC/frame is supplied.
  wire::configure(memory.spi);
  require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
          "Genuine original SPI request admits its actual vector/frame/program reads");
  for (unsigned n = 0; n < irq_cycles; ++n) {
    cpu.advance_irq_bus();
    ++periods;
  }
  cpu.start_irq_program(mode);
}

void advance(Hcs08 &cpu, const unsigned count, unsigned &periods) {
  for (unsigned n = 0; n < count; ++n) {
    cpu.advance_irq_program();
    ++periods;
  }
}

void endpoint(const app::ApplicationMemory &memory,
              const Hcs08 &cpu,
              const unsigned seed,
              std::ostream &trace) {
  const auto q = owned_program(cpu);
  const auto &image = memory.flash.image();
  require(
      !cpu.irq_bus_state() && cpu.state().pc == handler + 1 && cpu.state().sp == stack - 6 &&
          cpu.state().ccr == entered_flags && cpu.state().ccr_known == partial &&
          memory.retired == 1 && memory.bytes[stack - 5] == seed && memory.interrupt_entries == 1 &&
          memory.interrupt_acknowledgements == 1 && memory.spi.interrupt_requested() &&
          q.retired_instructions == 1 && q.completed_periods == 2 && q.ready_bytes == 2 &&
          q.queued_bytes == 3 && q.program_byte_in_flight && q.head == handler + 1 &&
          q.next_read == handler + 4 && memory.flash_reads.size() == 6 &&
          memory.accesses.size() == 6 &&
          memory.accesses.back() == Access{.address = stack - 5, .value = seed, .write = true},
      "Original queued PSHH executes one real s write then one real p read, preserving IRQ/flags");
  const auto &last = memory.flash_reads.back();
  require(last.address == handler + 3 && !last.write &&
              last.value == image.data[handler + 3 - app::hf::first],
          "Only the fourth original program byte replenishes the consumed queue");
  for (unsigned n = 0; n < q.queued_bytes; ++n) {
    require(q.queued[n] == image.data[handler + 1 + n - app::hf::first],
            "Original BRCLR bytes remain actually captured, never reconstructed or dropped");
  }
  trace << "END seed=" << seed << " pc=" << cpu.state().pc << " sp=" << cpu.state().sp
        << " ccr=" << static_cast<unsigned>(cpu.state().ccr)
        << " known=" << static_cast<unsigned>(cpu.state().ccr_known)
        << " pushed=" << static_cast<unsigned>(memory.bytes[stack - 5])
        << " retired=" << memory.retired << " ready=" << q.ready_bytes
        << " inflight=" << q.program_byte_in_flight << " next=" << q.next_read
        << " q0=" << static_cast<unsigned>(q.queued[0])
        << " q1=" << static_cast<unsigned>(q.queued[1])
        << " q2=" << static_cast<unsigned>(q.queued[2]) << '\n';
}

struct Snapshot {
  app::ApplicationMemory memory;
  Hcs08Checkpoint cpu;
};

Snapshot capture(const std::span<const std::uint8_t> records,
                 const unsigned cut,
                 unsigned &periods,
                 const Hcs08PrefetchContract mode = program_contract,
                 const unsigned queued = 3) {
  app::ApplicationMemory memory(records, {.seed = 0, .trim = 0x80, .observation = -1});
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  prepare(memory, cpu, 7, periods, mode, queued);
  advance(cpu, cut, periods);
  return {.memory = memory, .cpu = cpu.checkpoint()};
}

void run(const std::span<const std::uint8_t> records, std::ostream &trace) {
  unsigned periods = 0;
  for (unsigned seed = 0; seed < 8; ++seed) {
    app::ApplicationMemory memory(records, {.seed = 0, .trim = 0x80, .observation = -1});
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
    prepare(memory, cpu, seed, periods);
    advance(cpu, 2, periods);
    endpoint(memory, cpu, seed, trace);
    const auto image = memory.flash.image();
    const auto accesses = memory.accesses;
    const auto reads = memory.flash_reads;
    require(
        rejected([&] { cpu.advance_irq_program(); }).contains("pc=0xea2a") && cpu.faulted() &&
            memory.accesses == accesses && memory.flash_reads == reads &&
            owned_program(cpu).ready_bytes == 3 && memory.flash.image().data == image.data,
        "The actual next original BRCLR refuses its unqualified data/flow cycles without access");
  }
  for (unsigned cut = 0; cut <= 2; ++cut) {
    const auto saved = capture(records, cut, periods); // Original owner is destroyed.
    for (unsigned replay = 0; replay < 2; ++replay) {
      auto memory = saved.memory;
      Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
      cpu.restore_checkpoint(saved.cpu);
      advance(cpu, 2 - cut, periods);
      trace << "REPLAY cut=" << cut << " index=" << replay << '\n';
      endpoint(memory, cpu, 7, trace);
    }
  }
  trace << "SUMMARY periods=" << periods << " baselines=8 replays=6 retired=1\n";
}

unsigned data_target(const app::ApplicationMemory &memory) {
  const auto &image = memory.flash.image().data;
  const auto offset = std::bit_cast<std::int8_t>(image[handler + 3 - app::hf::first]);
  return static_cast<std::uint16_t>(static_cast<int>(handler) + 4 + offset);
}

void data_endpoint(const app::ApplicationMemory &memory,
                   const Hcs08 &cpu,
                   const unsigned seed,
                   std::ostream &trace) {
  constexpr unsigned cycles = 15;
  constexpr unsigned compared_flags = 0x68;
  constexpr unsigned status = 0x2b;
  constexpr unsigned empty_tx = 0x20;
  const auto q = owned_program(cpu);
  if (!q.discarded || !q.operand.value) {
    throw Fault("Missing actual data/speculative observations");
  }
  const auto target = data_target(memory);
  const auto pc = target + 6;
  const auto &image = memory.flash.image().data;
  const auto actual = cpu.state();
  require(actual.pc == pc && actual.sp == stack - 6 && actual.a == 3 &&
              actual.hx == ((seed << 8U) | (byte_mask - seed)) && actual.ccr == compared_flags &&
              actual.ccr_known == partial && memory.retired == 4 && memory.interrupt_entries == 1 &&
              memory.interrupt_acknowledgements == 1 && memory.spi.interrupt_requested() &&
              memory.spi.state().transmit_armed && !memory.spi.state().active &&
              !memory.spi.state().transmit && memory.spi.state().edges == 0 &&
              q.retired_instructions == 4 && q.completed_periods == cycles && q.head == pc &&
              q.next_read == pc + 3 && q.ready_bytes == 2 && q.queued_bytes == 3 &&
              q.program_byte_in_flight && q.operand.address == cursor && q.operand.value == 0 &&
              q.operand.consumed && !q.operand.in_flight,
          "Original PSHH/BRCLR/LDA/CMP retain native PC/registers, actual data and owned queue");
  require(memory.accesses.size() == 8 &&
              memory.accesses[5] == Access{.address = stack - 5, .value = seed, .write = true} &&
              memory.accesses[6] == Access{.address = cursor + 1, .value = 3, .write = false} &&
              memory.accesses[7] == Access{.address = cursor, .value = 0, .write = false} &&
              memory.peripheral_accesses.size() == 1 &&
              memory.peripheral_accesses[0] ==
                  Access{.address = status, .value = empty_tx, .write = false},
          "Native data is consumed from one real SPIS read and one read of each TX cursor");
  constexpr unsigned flash_count = 16;
  const std::array<unsigned, flash_count> addresses{spi_vector,
                                                    spi_vector + 1,
                                                    handler,
                                                    handler + 1,
                                                    handler + 2,
                                                    handler + 3,
                                                    handler + 4,
                                                    target,
                                                    target + 1,
                                                    target + 2,
                                                    target + 3,
                                                    target + 4,
                                                    target + 5,
                                                    target + 6,
                                                    target + 7,
                                                    target + 8};
  require(memory.flash_reads.size() == addresses.size() && q.discarded.has_value(),
          "Every original vector/speculative/refill program read is present");
  for (unsigned n = 0; n < addresses.size(); ++n) {
    require(memory.flash_reads[n] == Access{.address = addresses[n],
                                            .value = image[addresses[n] - app::hf::first],
                                            .write = false},
            "Explicit reference preserves ordered original speculative/refill reads");
  }
  require(q.discarded->address == handler + 4 &&
              q.discarded->value == image[handler + 4 - app::hf::first] && !q.discarded->in_flight,
          "Actual sequential speculation survives as a discarded completed observation");
  for (unsigned n = 0; n < q.queued_bytes; ++n) {
    require(q.queued[n] == image[pc + n - app::hf::first],
            "Actual native-selected successor bytes populate the final queue");
  }
  trace << "DATA seed=" << seed << " pc=" << actual.pc << " sp=" << actual.sp
        << " a=" << static_cast<unsigned>(actual.a) << " hx=" << actual.hx
        << " ccr=" << static_cast<unsigned>(actual.ccr)
        << " known=" << static_cast<unsigned>(actual.ccr_known)
        << " pushed=" << static_cast<unsigned>(memory.bytes[stack - 5])
        << " retired=" << memory.retired << " ready=" << q.ready_bytes
        << " inflight=" << q.program_byte_in_flight << " next=" << q.next_read
        << " q0=" << static_cast<unsigned>(q.queued[0])
        << " q1=" << static_cast<unsigned>(q.queued[1])
        << " q2=" << static_cast<unsigned>(q.queued[2]) << " operand=" << q.operand.address
        << " value=" << static_cast<unsigned>(*q.operand.value)
        << " discarded=" << q.discarded->address
        << " speculative=" << static_cast<unsigned>(q.discarded->value)
        << " reads=" << memory.flash_reads.size() << " status=" << empty_tx << '\n';
}

void run_data(const std::span<const std::uint8_t> records, std::ostream &trace) {
  constexpr auto mode = Hcs08PrefetchContract::reference_data_branch_cycles;
  constexpr unsigned endpoint_cycles = 15;
  unsigned periods = 0;
  for (unsigned seed = 0; seed < 8; ++seed) {
    app::ApplicationMemory memory(records, {.seed = 0, .trim = 0x80, .observation = -1});
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
    prepare(memory, cpu, seed, periods, mode);
    advance(cpu, endpoint_cycles, periods);
    data_endpoint(memory, cpu, seed, trace);
    const auto image = memory.flash.image();
    const auto accesses = memory.accesses;
    const auto peripherals = memory.peripheral_accesses;
    const auto reads = memory.flash_reads;
    require(rejected([&] { cpu.advance_irq_program(); }).contains("opcode=0x0027") &&
                cpu.faulted() && memory.accesses == accesses &&
                memory.peripheral_accesses == peripherals && memory.flash_reads == reads &&
                owned_program(cpu).ready_bytes == 3 && memory.flash.image().data == image.data &&
                memory.flash.image().known == image.known,
            "Next original BEQ remains a strict unsupported flow boundary without another access");
  }
  for (unsigned cut = 0; cut <= endpoint_cycles; ++cut) {
    const auto saved = capture(records, cut, periods, mode);
    for (unsigned replay = 0; replay < 2; ++replay) {
      auto memory = saved.memory;
      Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
      cpu.restore_checkpoint(saved.cpu);
      advance(cpu, endpoint_cycles - cut, periods);
      trace << "DREPLAY cut=" << cut << " index=" << replay << '\n';
      data_endpoint(memory, cpu, 7, trace);
    }
  }
  trace << "SUMMARY periods=" << periods << " baselines=8 replays=32 retired=4\n";
}

struct ShortInput {
  unsigned seed;
  bool empty;
};

constexpr unsigned empty_periods = 18, send_periods = 24;

unsigned short_cycles(const bool empty) {
  return empty ? empty_periods : send_periods;
}

unsigned short_selected(const app::ApplicationMemory &memory, const bool empty) {
  const auto branch = data_target(memory) + 6;
  if (!empty) { return branch + 2; }
  const auto offset =
      std::bit_cast<std::int8_t>(memory.flash.image().data[branch + 1 - app::hf::first]);
  return static_cast<std::uint16_t>(static_cast<int>(branch) + 2 + offset);
}

std::vector<unsigned> short_addresses(const app::ApplicationMemory &memory, const bool empty) {
  const auto data = data_target(memory);
  const auto branch = data + 6;
  std::vector<unsigned> addresses{
      spi_vector, spi_vector + 1, handler, handler + 1, handler + 2, handler + 3, handler + 4};
  for (unsigned n = 0; n < 9; ++n) { addresses.push_back(data + n); }
  for (unsigned n = 0; n < 3; ++n) { addresses.push_back(short_selected(memory, empty) + n); }
  if (!empty) {
    for (unsigned n = 5; n < 10; ++n) { addresses.push_back(branch + n); }
  }
  return addresses;
}

void short_reads(const app::ApplicationMemory &memory, const bool empty) {
  const auto addresses = short_addresses(memory, empty);
  require(memory.flash_reads.size() == addresses.size(),
          "Every actual original program read is present");
  const auto &image = memory.flash.image().data;
  for (unsigned n = 0; n < addresses.size(); ++n) {
    require(memory.flash_reads[n] == Access{.address = addresses[n],
                                            .value = image[addresses[n] - app::hf::first],
                                            .write = false},
            "Original vector, bit-branch, data, BEQ and native successor reads preserve order");
  }
}

void short_endpoint(const app::ApplicationMemory &memory,
                    const Hcs08 &cpu,
                    const ShortInput input,
                    std::ostream &trace) {
  constexpr unsigned compared_flags = 0x68;
  const auto q = owned_program(cpu);
  if (!q.discarded) { throw Fault("Missing actual original BEQ successor observation"); }
  const auto branch = data_target(memory) + 6;
  const auto pc = short_selected(memory, input.empty) + (input.empty ? 0U : 5U);
  const unsigned retired = input.empty ? 5 : 8;
  const auto actual = cpu.state();
  require(
      actual.pc == pc && actual.sp == stack - 6 && actual.a == (input.empty ? 0U : 1U) &&
          actual.hx == ((input.seed << 8U) | (input.empty ? byte_mask - input.seed : 0U)) &&
          actual.ccr == (compared_flags | (input.empty ? flag_z : 0U)) &&
          actual.ccr_known == partial && memory.retired == retired &&
          q.retired_instructions == retired && memory.interrupt_entries == 1 &&
          memory.interrupt_acknowledgements == 1 &&
          q.completed_periods == short_cycles(input.empty) && q.head == pc &&
          q.next_read == pc + 3 && q.queued_bytes == 3 && q.ready_bytes == 2 &&
          q.program_byte_in_flight && !q.operand.value && memory.bytes[stack - 5] == input.seed &&
          memory.spi.interrupt_requested() && memory.spi.state().transmit_armed &&
          !memory.spi.state().active && !memory.spi.state().transmit &&
          memory.spi.state().edges == 0,
      "Original BEQ chooses empty/send paths through native flags and actual original successors");
  const auto &image = memory.flash.image().data;
  require(q.discarded->address == branch + 2 &&
              q.discarded->value == image[branch + 2 - app::hf::first] && !q.discarded->in_flight &&
              memory.accesses.size() == (input.empty ? 8U : 9U) &&
              memory.accesses[6] ==
                  Access{.address = cursor + 1, .value = input.empty ? 0U : 3U, .write = false} &&
              memory.accesses[7] == Access{.address = cursor, .value = 0, .write = false} &&
              memory.peripheral_accesses == std::vector<Access>{{.address = hcs08_spi::status,
                                                                 .value = hcs08_spi::transmit_empty,
                                                                 .write = false}},
          "BEQ discards the actual mature successor; only genuine SPI/status/cursor reads occur");
  if (!input.empty) {
    require(memory.accesses[8] == Access{.address = cursor, .value = 0, .write = false},
            "The native send successor performs its own one actual TX cursor read");
  }
  for (unsigned n = 0; n < q.queued_bytes; ++n) {
    require(q.queued[n] == image[pc + n - app::hf::first],
            "Final original instruction bytes remain captured");
  }
  short_reads(memory, input.empty);
  trace << "SHORT empty=" << input.empty << " seed=" << input.seed << " pc=" << actual.pc
        << " sp=" << actual.sp << " a=" << static_cast<unsigned>(actual.a) << " hx=" << actual.hx
        << " ccr=" << static_cast<unsigned>(actual.ccr)
        << " known=" << static_cast<unsigned>(actual.ccr_known)
        << " pushed=" << static_cast<unsigned>(memory.bytes[stack - 5])
        << " retired=" << memory.retired << " ready=" << q.ready_bytes
        << " inflight=" << q.program_byte_in_flight << " next=" << q.next_read
        << " q0=" << static_cast<unsigned>(q.queued[0])
        << " q1=" << static_cast<unsigned>(q.queued[1])
        << " q2=" << static_cast<unsigned>(q.queued[2]) << " discarded=" << q.discarded->address
        << " speculative=" << static_cast<unsigned>(q.discarded->value)
        << " reads=" << memory.flash_reads.size() << " data_reads=" << (input.empty ? 3U : 4U)
        << '\n';
}

void run_short(const std::span<const std::uint8_t> records, std::ostream &trace) {
  constexpr auto mode = Hcs08PrefetchContract::reference_short_branch_refill;
  unsigned periods = 0;
  for (unsigned seed = 0; seed < 8; ++seed) {
    for (const bool empty : {false, true}) {
      app::ApplicationMemory memory(records, {.seed = 0, .trim = 0x80, .observation = -1});
      Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
      prepare(memory, cpu, seed, periods, mode, empty ? 0U : 3U);
      advance(cpu, short_cycles(empty), periods);
      short_endpoint(memory, cpu, {.seed = seed, .empty = empty}, trace);
      const auto image = memory.flash.image();
      const auto accesses = memory.accesses;
      const auto peripherals = memory.peripheral_accesses;
      const auto reads = memory.flash_reads;
      constexpr unsigned mov = 0x6e;
      constexpr unsigned sta = 0xc7;
      require(
          rejected([&] {
            cpu.advance_irq_program();
          }).contains("opcode=" + hex(empty ? mov : sta)) &&
              cpu.faulted() && memory.accesses == accesses &&
              memory.peripheral_accesses == peripherals && memory.flash_reads == reads &&
              memory.flash.image().data == image.data &&
              memory.flash.image().known == image.known && owned_program(cpu).ready_bytes == 3,
          "Next original MOV/STA remains a visible strict write-cycle boundary without a store");
    }
  }
  for (const bool empty : {false, true}) {
    for (unsigned cut = 0; cut <= short_cycles(empty); ++cut) {
      const auto saved = capture(records, cut, periods, mode, empty ? 0U : 3U);
      for (unsigned replay = 0; replay < 2; ++replay) {
        auto memory = saved.memory;
        Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
        cpu.restore_checkpoint(saved.cpu);
        advance(cpu, short_cycles(empty) - cut, periods);
        trace << "SREPLAY empty=" << empty << " cut=" << cut << " index=" << replay << '\n';
        short_endpoint(memory, cpu, {.seed = 7, .empty = empty}, trace);
      }
    }
  }
  trace << "SUMMARY periods=" << periods
        << " baselines=16 replays=88 retired_empty=5 retired_send=8\n";
}

constexpr unsigned empty_peripheral_periods = 25, send_peripheral_periods = 39;

unsigned peripheral_cycles(const bool empty) {
  return empty ? empty_peripheral_periods : send_peripheral_periods;
}

void peripheral_reads(const app::ApplicationMemory &memory,
                      const bool empty,
                      const bool returned = false) {
  auto addresses = short_addresses(memory, empty);
  const unsigned first = empty ? 3 : 8;
  const unsigned end = empty ? 7 : 18;
  for (unsigned n = first; n < end; ++n) { addresses.push_back(short_selected(memory, empty) + n); }
  if (returned) {
    for (unsigned n = 0; n < 3; ++n) { addresses.push_back(foreground_pc + n); }
  }
  require(memory.flash_reads.size() == addresses.size(),
          "All original peripheral-path p reads occur once");
  const auto &image = memory.flash.image().data;
  for (unsigned n = 0; n < addresses.size(); ++n) {
    require(memory.flash_reads[n] == Access{.address = addresses[n],
                                            .value = image[addresses[n] - app::hf::first],
                                            .write = false},
            "Original peripheral p cycles retain exact vector/speculation/refill/read order");
  }
}

void peripheral_endpoint(const app::ApplicationMemory &memory,
                         const Hcs08 &cpu,
                         const ShortInput input,
                         std::ostream &trace) {
  constexpr unsigned empty_flags = 0x68;
  constexpr unsigned send_flags = 0x6c;
  const auto q = owned_program(cpu);
  if (!q.operand.value) { throw Fault("Missing actual original PULH read"); }
  const auto pc = short_selected(memory, input.empty) + (input.empty ? 4U : 15U);
  const unsigned retired = input.empty ? 7 : 13;
  const unsigned a = input.empty ? 0 : wire::event_bytes[0];
  const unsigned control = input.empty ? wire::quiet_control : wire::control;
  const auto actual = cpu.state();
  const auto spi = memory.spi.state();
  require(actual.pc == pc && actual.sp == stack - 5 && actual.a == a &&
              actual.hx == ((input.seed << 8U) | (input.empty ? byte_mask - input.seed : 0U)) &&
              actual.ccr == (input.empty ? empty_flags : send_flags) &&
              actual.ccr_known == partial && memory.retired == retired &&
              q.retired_instructions == retired &&
              q.completed_periods == peripheral_cycles(input.empty) && q.head == pc &&
              q.next_read == pc + 3 && q.queued_bytes == 3 && q.ready_bytes == 2 &&
              q.program_byte_in_flight && q.operand.address == stack - 5 &&
              *q.operand.value == input.seed && q.operand.consumed && !q.operand.in_flight &&
              !memory.spi.interrupt_requested() && spi.control1 == control && !spi.active &&
              spi.edges == 0 && spi.transmit_armed == input.empty &&
              spi.transmit == (input.empty ? std::nullopt : std::optional(wire::event_bytes[0])) &&
              memory.bytes[cursor] == (input.empty ? 0U : 1U) && memory.interrupt_entries == 1 &&
              memory.interrupt_acknowledgements == 1,
          "Original native MOV/STA/indexed LDA/PULH expose real control/queue/stack effects before "
          "RTI");
  require(memory.accesses.size() == (input.empty ? 9U : 12U) &&
              memory.accesses.back() ==
                  Access{.address = stack - 5, .value = input.seed, .write = false},
          "Original PULH reads the actually saved H byte once and restores SP");
  if (!input.empty) {
    require(memory.accesses[9] == Access{.address = cursor, .value = 1, .write = true} &&
                memory.accesses[10] ==
                    Access{.address = tx, .value = wire::event_bytes[0], .write = false},
            "Actual native cursor increment precedes the actual indexed TX buffer load");
  }
  require(
      memory.peripheral_accesses ==
          std::vector<Access>{
              {.address = hcs08_spi::status, .value = hcs08_spi::transmit_empty, .write = false},
              {.address = input.empty ? hcs08_spi::control1 : hcs08_spi::data,
               .value = input.empty ? wire::quiet_control : wire::event_bytes[0],
               .write = true}},
      "One real SPIS read arms the actual later control/data write; no host byte or completion is "
      "supplied");
  const auto &image = memory.flash.image().data;
  for (unsigned n = 0; n < q.queued_bytes; ++n) {
    require(q.queued[n] == image[pc + n - app::hf::first],
            "Actual original RTI and successors remain queued");
  }
  peripheral_reads(memory, input.empty);
  trace << "PERIPHERAL empty=" << input.empty << " seed=" << input.seed << " pc=" << actual.pc
        << " sp=" << actual.sp << " a=" << static_cast<unsigned>(actual.a) << " hx=" << actual.hx
        << " ccr=" << static_cast<unsigned>(actual.ccr)
        << " known=" << static_cast<unsigned>(actual.ccr_known) << " retired=" << memory.retired
        << " ready=" << q.ready_bytes << " inflight=" << q.program_byte_in_flight
        << " next=" << q.next_read << " q0=" << static_cast<unsigned>(q.queued[0])
        << " q1=" << static_cast<unsigned>(q.queued[1])
        << " q2=" << static_cast<unsigned>(q.queued[2])
        << " control=" << static_cast<unsigned>(spi.control1)
        << " queued=" << (spi.transmit ? static_cast<int>(*spi.transmit) : -1)
        << " cursor=" << static_cast<unsigned>(memory.bytes[cursor])
        << " read_addr=" << q.operand.address
        << " read_value=" << static_cast<unsigned>(*q.operand.value)
        << " reads=" << memory.flash_reads.size() << " data_reads=" << (input.empty ? 4U : 6U)
        << '\n';
}

void run_peripheral(const std::span<const std::uint8_t> records, std::ostream &trace) {
  constexpr auto mode = Hcs08PrefetchContract::reference_peripheral_program_cycles;
  unsigned periods = 0;
  for (unsigned seed = 0; seed < 8; ++seed) {
    for (const bool empty : {false, true}) {
      app::ApplicationMemory memory(records, {.seed = 0, .trim = 0x80, .observation = -1});
      Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
      prepare(memory, cpu, seed, periods, mode, empty ? 0U : 3U);
      advance(cpu, peripheral_cycles(empty), periods);
      peripheral_endpoint(memory, cpu, {.seed = seed, .empty = empty}, trace);
    }
  }
  for (const bool empty : {false, true}) {
    for (unsigned cut = 0; cut <= peripheral_cycles(empty); ++cut) {
      const auto saved = capture(records, cut, periods, mode, empty ? 0U : 3U);
      for (unsigned replay = 0; replay < 2; ++replay) {
        auto memory = saved.memory;
        Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
        cpu.restore_checkpoint(saved.cpu);
        advance(cpu, peripheral_cycles(empty) - cut, periods);
        trace << "PREPLAY empty=" << empty << " cut=" << cut << " index=" << replay << '\n';
        peripheral_endpoint(memory, cpu, {.seed = 7, .empty = empty}, trace);
      }
    }
  }
  trace << "SUMMARY periods=" << periods
        << " baselines=16 replays=132 retired_empty=7 retired_send=13\n";
}

constexpr unsigned return_periods = 9;

void return_endpoint(const app::ApplicationMemory &memory,
                     const Hcs08 &cpu,
                     const ShortInput input,
                     std::ostream &trace) {
  const auto q = owned_program(cpu);
  if (!q.discarded || !q.return_discarded) {
    throw Fault("Missing actual original RTI successor discards");
  }
  const auto actual = cpu.state();
  const auto spi = memory.spi.state();
  const unsigned low = byte_mask - input.seed;
  const unsigned retired = input.empty ? 8U : 14U;
  const unsigned pc = short_selected(memory, input.empty) + (input.empty ? 4U : 15U);
  const auto &image = memory.flash.image().data;
  require(actual.pc == foreground_pc && actual.sp == stack && actual.a == low &&
              actual.hx == ((input.seed << 8U) | low) && actual.ccr == flags &&
              actual.ccr_known == partial && actual.a_known == byte_mask &&
              actual.hx_known == word_mask && actual.pc_known &&
              cpu.irq_delay() == Hcs08IrqDelay::unqualified_return &&
              q.completed_periods == peripheral_cycles(input.empty) + return_periods &&
              q.retired_instructions == retired && memory.retired == retired &&
              q.head == foreground_pc && q.next_read == foreground_pc + 3 && q.queued_bytes == 3 &&
              q.ready_bytes == 2 && q.program_byte_in_flight && q.native_executed &&
              memory.interrupt_entries == 1 && memory.interrupt_acknowledgements == 1 &&
              !memory.spi.interrupt_requested() && !spi.active && spi.edges == 0 &&
              spi.transmit_armed == input.empty &&
              spi.control1 == (input.empty ? wire::quiet_control : wire::control) &&
              spi.transmit == (input.empty ? std::nullopt : std::optional(wire::event_bytes[0])) &&
              memory.bytes[cursor] == (input.empty ? 0U : 1U),
          "Original RTI restores the actual IRQ frame and saved H without supplying SPI edges or "
          "an IRQ-delay policy");
  require(q.discarded == Hcs08DiscardedProgramRead{.address = static_cast<std::uint16_t>(pc + 1),
                                                   .value = image[pc + 1 - app::hf::first],
                                                   .in_flight = false} &&
              q.return_discarded ==
                  Hcs08DiscardedProgramRead{.address = static_cast<std::uint16_t>(pc + 2),
                                            .value = image[pc + 2 - app::hf::first],
                                            .in_flight = false},
          "Both actually fetched original RTI successors remain observable as discarded reads");
  const unsigned prefix = input.empty ? 9 : 12;
  require(memory.accesses.size() == prefix + q.return_reads.size(),
          "Original RTI has exactly five actual stack accesses");
  const std::array<unsigned, 5> frame{
      flags, low, low, foreground_pc >> 8U, foreground_pc & byte_mask};
  for (unsigned n = 0; n < frame.size(); ++n) {
    const auto &read = q.return_reads[n];
    const auto mask = static_cast<std::uint8_t>(n == 0 ? partial : byte_mask);
    require(
        read.address == stack - 4 + n && read.value == frame[n] && read.known == mask &&
            read.consumed && !read.in_flight &&
            memory.accesses[prefix + n] ==
                Access{.address = stack - 4 + n, .value = frame[n], .write = false, .known = mask},
        "Native RTI consumes the real CCR/A/X/PCH/PCL reads once, retaining unknown H flag");
  }
  for (unsigned n = 0; n < q.queued_bytes; ++n) {
    require(q.queued[n] == image[foreground_pc + n - app::hf::first],
            "RTI refills genuine original foreground bytes from the native restored PC");
  }
  peripheral_reads(memory, input.empty, true);
  trace << "RETURN empty=" << input.empty << " seed=" << input.seed << " pc=" << actual.pc
        << " sp=" << actual.sp << " a=" << static_cast<unsigned>(actual.a) << " hx=" << actual.hx
        << " ccr=" << static_cast<unsigned>(actual.ccr)
        << " known=" << static_cast<unsigned>(actual.ccr_known) << " retired=" << retired
        << " ready=" << q.ready_bytes << " inflight=" << q.program_byte_in_flight
        << " next=" << q.next_read << " q0=" << static_cast<unsigned>(q.queued[0])
        << " q1=" << static_cast<unsigned>(q.queued[1])
        << " q2=" << static_cast<unsigned>(q.queued[2])
        << " control=" << static_cast<unsigned>(spi.control1)
        << " queued=" << (spi.transmit ? static_cast<int>(*spi.transmit) : -1)
        << " cursor=" << static_cast<unsigned>(memory.bytes[cursor])
        << " reads=" << memory.flash_reads.size() << " data_reads=" << (input.empty ? 9U : 11U)
        << " disc0_addr=" << q.discarded->address
        << " disc0_value=" << static_cast<unsigned>(q.discarded->value)
        << " disc1_addr=" << q.return_discarded->address
        << " disc1_value=" << static_cast<unsigned>(q.return_discarded->value);
  for (unsigned n = 0; n < frame.size(); ++n) { trace << " frame" << n << '=' << frame[n]; }
  trace << '\n';
}

void run_return(const std::span<const std::uint8_t> records, std::ostream &trace) {
  constexpr auto mode = Hcs08PrefetchContract::reference_peripheral_program_cycles;
  unsigned periods = 0;
  for (unsigned seed = 0; seed < 8; ++seed) {
    for (const bool empty : {false, true}) {
      app::ApplicationMemory memory(records, {.seed = 0, .trim = 0x80, .observation = -1});
      Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
      prepare(memory, cpu, seed, periods, mode, empty ? 0U : 3U);
      advance(cpu, peripheral_cycles(empty) + return_periods, periods);
      return_endpoint(memory, cpu, {.seed = seed, .empty = empty}, trace);
    }
  }
  for (const bool empty : {false, true}) {
    const unsigned length = peripheral_cycles(empty) + return_periods;
    for (unsigned cut = 0; cut <= length; ++cut) {
      const auto saved = capture(records, cut, periods, mode, empty ? 0U : 3U);
      for (unsigned replay = 0; replay < 2; ++replay) {
        auto memory = saved.memory;
        Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
        cpu.restore_checkpoint(saved.cpu);
        advance(cpu, length - cut, periods);
        trace << "RREPLAY empty=" << empty << " cut=" << cut << " index=" << replay << '\n';
        return_endpoint(memory, cpu, {.seed = 7, .empty = empty}, trace);
      }
    }
  }
  trace << "SUMMARY periods=" << periods
        << " baselines=16 replays=168 retired_empty=8 retired_send=14\n";
}

constexpr unsigned handoff_periods = irq_cycles + send_peripheral_periods + return_periods;
constexpr unsigned handoff_retired = 28, handoff_data_accesses = 34, handoff_program_reads = 74;
constexpr auto handoff_contract = Hcs08IrqBusContract::reference_owned_program_handoff;
constexpr auto immediate_irq = Hcs08RtiIrqContract::reference_immediate;

Hcs08PrefetchState prepare_handoff(app::ApplicationMemory &memory,
                                   Hcs08 &cpu,
                                   const unsigned seed,
                                   unsigned &periods) {
  constexpr auto mode = Hcs08PrefetchContract::reference_peripheral_program_cycles;
  prepare(memory, cpu, seed, periods, mode);
  advance(cpu, send_peripheral_periods + return_periods, periods);
  const auto archived = owned_program(cpu);
  require(cpu.state().pc == foreground_pc && cpu.state().sp == stack &&
              cpu.irq_delay() == Hcs08IrqDelay::none && memory.retired == 14 &&
              memory.spi.state().transmit == wire::event_bytes[0] &&
              !memory.spi.interrupt_requested(),
          "First original ISR returns through its real frame with a genuinely queued SPI byte");
  memory.spi.context({.pc = cpu.state().pc, .ccr = cpu.state().ccr});
  memory.spi.transfer_to_shifter(); // Explicit promotion event; no physical launch-phase claim.
  require(memory.spi.interrupt_requested() && memory.spi.state().active &&
              !memory.spi.state().transmit && memory.spi.state().shift_out == wire::event_bytes[0],
          "Promotion of the actual native queued byte raises the genuine next SPI source");
  const auto accesses = memory.accesses;
  const auto reads = memory.flash_reads;
  require(cpu.start_irq_bus(handoff_contract) && !cpu.prefetch_state() &&
              memory.accesses == accesses && memory.flash_reads == reads,
          "Original queue-to-IRQ admission neither rereads FLASH nor writes a supplied frame");
  const auto irq = cpu.irq_bus_state();
  require(irq && irq->discarded_program == archived && irq->entry.pc == foreground_pc &&
              irq->entry.sp == stack && irq->admitted_vector == spi_vector,
          "Second original IRQ retains the entire actual preceding program queue");
  return archived;
}

void advance_handoff(Hcs08 &cpu,
                     const unsigned index,
                     unsigned &periods,
                     Hcs08PrefetchState archived) {
  if (index < irq_cycles) {
    cpu.advance_irq_bus();
    const auto irq = cpu.irq_bus_state();
    archived.ready_bytes = 3;
    archived.program_byte_in_flight = false;
    require(irq && irq->discarded_program == archived,
            "Every second IRQ prefix retains all prior RTI masks, values, discards and arrivals");
  } else {
    if (index == irq_cycles) {
      cpu.start_irq_program(Hcs08PrefetchContract::reference_peripheral_program_cycles);
    }
    cpu.advance_irq_program();
  }
  ++periods;
}

void handoff_endpoint(const app::ApplicationMemory &memory,
                      const Hcs08 &cpu,
                      const unsigned seed,
                      const Hcs08PrefetchState &archived,
                      std::ostream &trace) {
  const auto state = cpu.state();
  const auto q = owned_program(cpu);
  const auto spi = memory.spi.state();
  if (!spi.transmit) { throw Fault("Expected second native queued SPI byte"); }
  const unsigned low = byte_mask - seed;
  require(
      state.pc == foreground_pc && state.sp == stack && state.a == low &&
          state.hx == ((seed << 8U) | low) && state.ccr == flags && state.ccr_known == partial &&
          state.a_known == byte_mask && state.hx_known == word_mask &&
          cpu.irq_delay() == Hcs08IrqDelay::none && memory.retired == handoff_retired &&
          memory.interrupt_entries == 2 && memory.interrupt_acknowledgements == 2 &&
          q.completed_periods == send_peripheral_periods + return_periods &&
          q.retired_instructions == 14 && q.ready_bytes == 2 && q.queued_bytes == 3 &&
          q.program_byte_in_flight && q.head == foreground_pc && q.next_read == foreground_pc + 3 &&
          spi.active && spi.edges == 0 && spi.samples == 0 &&
          spi.shift_out == wire::event_bytes[0] && spi.transmit == wire::event_bytes[1] &&
          spi.control1 == wire::control && !memory.spi.interrupt_requested() &&
          memory.bytes[cursor] == 2 && memory.accesses.size() == handoff_data_accesses &&
          memory.flash_reads.size() == handoff_program_reads,
      "Two original ISR/frame/RTI paths refill the real SPI buffer while the first byte is in "
      "the shifter; no edge, RX byte or new handler state is supplied");
  require(
      memory.peripheral_accesses ==
          std::vector<Access>{
              {.address = hcs08_spi::status, .value = hcs08_spi::transmit_empty, .write = false},
              {.address = hcs08_spi::data, .value = wire::event_bytes[0], .write = true},
              {.address = hcs08_spi::status, .value = hcs08_spi::transmit_empty, .write = false},
              {.address = hcs08_spi::data, .value = wire::event_bytes[1], .write = true}},
      "Only original SPIS reads arm original SPID writes for both queued bytes");
  const auto &image = memory.flash.image().data;
  require(archived.head == foreground_pc && archived.next_read == foreground_pc + 3 &&
              archived.ready_bytes == 2 && archived.program_byte_in_flight &&
              archived.retired_instructions == 14 &&
              archived.completed_periods == send_peripheral_periods + return_periods,
          "The observed incoming queue still describes its genuine pending third read");
  for (unsigned n = 0; n < 3; ++n) {
    require(q.queued[n] == image[foreground_pc + n - app::hf::first] &&
                archived.queued[n] == image[foreground_pc + n - app::hf::first],
            "Both IRQ returns refill actual original foreground bytes");
  }
  trace << "HANDOFF seed=" << seed << " pc=" << state.pc << " sp=" << state.sp
        << " a=" << static_cast<unsigned>(state.a) << " hx=" << state.hx
        << " ccr=" << static_cast<unsigned>(state.ccr)
        << " known=" << static_cast<unsigned>(state.ccr_known) << " retired=" << memory.retired
        << " irq=" << memory.interrupt_entries << " ack=" << memory.interrupt_acknowledgements
        << " ready=" << q.ready_bytes << " inflight=" << q.program_byte_in_flight
        << " next=" << q.next_read << " q0=" << static_cast<unsigned>(q.queued[0])
        << " q1=" << static_cast<unsigned>(q.queued[1])
        << " q2=" << static_cast<unsigned>(q.queued[2])
        << " tx=" << static_cast<unsigned>(spi.shift_out)
        << " queued=" << static_cast<unsigned>(*spi.transmit) << " active=" << spi.active
        << " edges=" << spi.edges << " cursor=" << static_cast<unsigned>(memory.bytes[cursor])
        << " reads=" << memory.flash_reads.size() << " data_accesses=" << memory.accesses.size()
        << " old_pc=" << archived.head << " old_next=" << archived.next_read
        << " old_ready=" << archived.ready_bytes
        << " old_inflight=" << archived.program_byte_in_flight
        << " old_retired=" << archived.retired_instructions
        << " old_periods=" << archived.completed_periods << '\n';
}

struct HandoffSnapshot {
  app::ApplicationMemory memory;
  Hcs08Checkpoint cpu;
  Hcs08PrefetchState archived;
};

HandoffSnapshot capture_handoff(const std::span<const std::uint8_t> records,
                                const unsigned cut,
                                unsigned &periods) {
  app::ApplicationMemory memory(records, {.seed = 0, .trim = 0x80, .observation = -1});
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA, immediate_irq);
  const auto archived = prepare_handoff(memory, cpu, 7, periods);
  for (unsigned n = 0; n < cut; ++n) { advance_handoff(cpu, n, periods, archived); }
  return {.memory = memory, .cpu = cpu.checkpoint(), .archived = archived};
}

void run_handoff(const std::span<const std::uint8_t> records, std::ostream &trace) {
  unsigned periods = 0;
  for (unsigned seed = 0; seed < 8; ++seed) {
    app::ApplicationMemory memory(records, {.seed = 0, .trim = 0x80, .observation = -1});
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA, immediate_irq);
    const auto archived = prepare_handoff(memory, cpu, seed, periods);
    for (unsigned n = 0; n < handoff_periods; ++n) { advance_handoff(cpu, n, periods, archived); }
    handoff_endpoint(memory, cpu, seed, archived, trace);
  }
  for (unsigned cut = 0; cut <= handoff_periods; ++cut) {
    const auto saved = capture_handoff(records, cut, periods);
    for (unsigned replay = 0; replay < 2; ++replay) {
      auto memory = saved.memory;
      Hcs08 cpu(memory, BLOFELD_HCS08_SLA, immediate_irq);
      cpu.restore_checkpoint(saved.cpu);
      for (unsigned n = cut; n < handoff_periods; ++n) {
        advance_handoff(cpu, n, periods, saved.archived);
      }
      trace << "HREPLAY cut=" << cut << " index=" << replay << '\n';
      handoff_endpoint(memory, cpu, 7, saved.archived, trace);
    }
  }
  trace << "SUMMARY periods=" << periods << " baselines=8 replays=120 retired=28\n";
}
}

int main(const int count, char **arguments) {
  try {
    const bool data_mode = count == 4 && std::string_view(arguments[3]) == "--data-branch";
    const bool short_mode = count == 4 && std::string_view(arguments[3]) == "--short-branch";
    const bool peripheral_mode = count == 4 && std::string_view(arguments[3]) == "--peripheral";
    const bool return_mode = count == 4 && std::string_view(arguments[3]) == "--return";
    const bool handoff_mode = count == 4 && std::string_view(arguments[3]) == "--handoff";
    require(count == 3 || data_mode || short_mode || peripheral_mode || return_mode || handoff_mode,
            "Expected pinned pages, trace and supported optional mode");
    const auto records = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    if (handoff_mode) {
      run_handoff(records, trace);
    } else if (return_mode) {
      run_return(records, trace);
    } else if (peripheral_mode) {
      run_peripheral(records, trace);
    } else if (short_mode) {
      run_short(records, trace);
    } else if (data_mode) {
      run_data(records, trace);
    } else {
      run(records, trace);
    }
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    if (handoff_mode) {
      std::cout << "Original queued SPI byte raises the next IRQ through retained native queues "
                   "and frames.\n";
      return 0;
    }
    if (return_mode) {
      std::cout
          << "Original RTI cycles consume the actual IRQ frame and refill native foreground.\n";
      return 0;
    }
    if (peripheral_mode) {
      std::cout
          << "Original peripheral cycles preserve actual native stores, indexed loads and PULH.\n";
      return 0;
    }
    if (short_mode) {
      std::cout
          << "Original BEQ paths preserve native successors and actual short-branch refill.\n";
      return 0;
    }
    std::cout << (data_mode ? "Original prefetched data/branch cycles preserve actual reads and "
                              "native decisions.\n"
                            : "Original IRQ-prefetched PSHH executes native stack/program cycles "
                              "and fresh continuations.\n");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
