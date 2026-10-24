// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_bdc.hpp"
#include "blofeld/hcs08_flash.hpp"
#include "blofeld/hcs08_run_clock.hpp"
#include "hcs08_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned vector = 0xffe0, high_vector = 0xfffa, handler = 0x9000, other_handler = 0x9100;
constexpr unsigned stack = 0x240, saved_pc = 0x1234, total_cycles = 11, first_program = 9;
constexpr unsigned entry_ccr = 0x61, nop = 0x9d, page_high = 0x90, other_page = 0x91;
constexpr unsigned partial_ccr = 0xef, partial_a = 0x55, partial_x = 0xaa;
constexpr auto contract = Hcs08IrqBusContract::reference_unstretched_sequence;
constexpr Hcs08State foreground{.pc = saved_pc,
                                .hx = 0xbc34,
                                .sp = stack,
                                .a = 0x5a,
                                .ccr_known = partial_ccr,
                                .ccr = entry_ccr,
                                .a_known = partial_a,
                                .hx_known = 0xff00 | partial_x};

struct Record {
  unsigned period;
  Access access;
  bool operator==(const Record &) const = default;
};

class SequenceMemory final : public MaskedMemory {
public:
  unsigned period = 0, entries = 0, acknowledgements = 0;
  unsigned entry_period = 0, ack_period = 0;
  bool deny_begin = false, deny_ack = false;
  std::vector<Record> records;
  std::vector<unsigned> widths;

  SequenceMemory() {
    writable.set();
    constexpr std::array<std::uint8_t, 2> target{page_high, 0};
    constexpr std::array<std::uint8_t, 2> other{other_page, 0};
    constexpr std::array<std::uint8_t, 3> program{nop, nop, nop};
    install(vector, target);
    install(high_vector, other);
    install(handler, program);
    install(other_handler, program);
    interrupt = {.vector = vector};
  }

  std::uint8_t read(const std::uint16_t address) override {
    const auto value = Memory::read(address);
    records.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = false}});
    return value;
  }

  std::uint64_t read_memory(const MemoryRead access) override {
    widths.push_back(access.size);
    return Bus::read_memory(access);
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    Memory::write(address, value);
    records.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = true}});
  }

  void write_known_byte(const ByteWrite access, const std::uint8_t mask) override {
    require(mask != byte_mask, "Only a genuine partial frame uses masked storage");
    Memory::write_known_byte(access, mask);
    records.push_back({.period = period,
                       .access = {.address = access.address,
                                  .value = static_cast<unsigned>(access.value & mask),
                                  .write = true,
                                  .known = mask}});
  }

  void begin_interrupt(const InterruptRequest request, const bool reuse) override {
    require(request.kind == InterruptKind::maskable && !reuse && request.vector == vector,
            "The initial source and normal five-byte stack frame are explicit");
    if (deny_begin) { fail(request.vector, "directed IRQ admission"); }
    ++entries;
    entry_period = period;
  }

  void acknowledge_interrupt(const std::uint16_t address) override {
    require(address == vector || address == high_vector,
            "Acknowledge the source selected at vector time");
    if (deny_ack) { fail(address, "directed IRQ acknowledgement"); }
    ++acknowledgements;
    ack_period = period;
  }
};

Hcs08IrqBusState progress(const Hcs08 &cpu) {
  const auto result = cpu.irq_bus_state();
  if (!result) { throw Fault("Expected a retained IRQ bus owner"); }
  return *result;
}

void advance(SequenceMemory &memory, Hcs08 &cpu) {
  ++memory.period;
  cpu.advance_irq_bus();
}

std::vector<Record> expected_records(const Hcs08State s) {
  return {{.period = 1, .access = {.address = s.sp, .value = s.pc & byte_mask, .write = true}},
          {.period = 2,
           .access = {.address = static_cast<std::uint16_t>(s.sp + space_bytes - 1U),
                      .value = static_cast<unsigned>(s.pc) >> 8U,
                      .write = true}},
          {.period = 3,
           .access = {.address = static_cast<std::uint16_t>(s.sp + space_bytes - 2U),
                      .value = s.hx & byte_mask,
                      .write = true,
                      .known = static_cast<std::uint8_t>(s.hx_known)}},
          {.period = 4,
           .access = {.address = static_cast<std::uint16_t>(s.sp + space_bytes - 3U),
                      .value = s.a,
                      .write = true,
                      .known = s.a_known}},
          {.period = 5,
           .access = {.address = static_cast<std::uint16_t>(s.sp + space_bytes - 4U),
                      .value = s.ccr,
                      .write = true,
                      .known = s.ccr_known}},
          {.period = 6, .access = {.address = vector, .value = page_high, .write = false}},
          {.period = 7, .access = {.address = vector + 1, .value = 0, .write = false}},
          {.period = 9, .access = {.address = handler, .value = nop, .write = false}},
          {.period = 10, .access = {.address = handler + 1, .value = nop, .write = false}},
          {.period = 11, .access = {.address = handler + 2, .value = nop, .write = false}}};
}

void endpoint(const SequenceMemory &memory, const Hcs08 &cpu, const Hcs08State before) {
  auto expected = before;
  expected.sp = static_cast<std::uint16_t>(before.sp + space_bytes - 5U);
  expected.ccr |= flag_i;
  expected.pc = handler;
  const auto state = progress(cpu);
  require(cpu.state() == expected && state.completed_cycles == total_cycles &&
              state.attempted_cycles == total_cycles && state.vector == vector &&
              state.target_known && state.target == handler && state.vector_reads == 2 &&
              state.ready_program_bytes == 2 && state.program_byte_in_flight &&
              state.prefetched_bytes == 3 &&
              state.prefetched == std::array<std::uint8_t, 3>{nop, nop, nop} &&
              state.acknowledged && memory.retired == 0 && memory.entries == 1 &&
              memory.acknowledgements == 1 && memory.entry_period == 1 && memory.ack_period == 7 &&
              memory.records == expected_records(before) &&
              memory.widths == std::vector<unsigned>(5, 1),
          "Independent11-cycle access sequence, real frame/target/prefetch and no retired opcode");
}

void full_sequence(SequenceMemory &memory, Hcs08 &cpu, const Hcs08State before) {
  const auto all = expected_records(before);
  for (unsigned period = 1; period <= total_cycles; ++period) {
    advance(memory, cpu);
    const auto s = progress(cpu);
    auto expected = before;
    expected.sp = static_cast<std::uint16_t>(before.sp + space_bytes - std::min(period, 5U));
    if (period >= 5) { expected.ccr |= flag_i; }
    if (period >= 8) { expected.pc = handler; }
    const auto fetched = period < first_program ? 0U : period - 8;
    std::vector<Record> prefix;
    for (const auto &record : all) {
      if (record.period <= period) { prefix.push_back(record); }
    }
    require(cpu.state() == expected && s.entry == before && s.completed_cycles == period &&
                s.attempted_cycles == period && s.prefetched_bytes == fetched &&
                s.ready_program_bytes == (fetched == 0 ? 0U : fetched - 1) &&
                s.program_byte_in_flight == (fetched != 0) &&
                s.vector_reads == (period < 6 ? 0U : std::min(period - 5, 2U)) &&
                s.target_known == (period >= 8) && s.acknowledged == (period >= 7) &&
                memory.records == prefix && memory.retired == 0,
            "Every cycle retains exact independent state and access prefix; f performs no read");
  }
  endpoint(memory, cpu, before);
}

void masks_and_wrap() {
  constexpr std::array<unsigned, 8> stacks{0, 1, 2, 3, 4, 5, stack, word_mask};
  constexpr std::array<unsigned, 3> a_masks{0, partial_a, byte_mask};
  for (const auto sp : stacks) {
    for (const auto a_mask : a_masks) {
      SequenceMemory memory;
      Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
      auto supplied = foreground;
      supplied.sp = static_cast<std::uint16_t>(sp);
      supplied.a_known = static_cast<std::uint8_t>(a_mask);
      cpu.set_state(supplied);
      const auto before = cpu.state();
      require(cpu.start_irq_bus(contract) && memory.records.empty() && memory.entries == 0 &&
                  cpu.state() == before && progress(cpu).completed_cycles == 0,
              "Admission establishes ownership without an early stack, vector or prefetch access");
      full_sequence(memory, cpu, before);
    }
  }
}

void copies() {
  for (unsigned cut = 0; cut <= total_cycles; ++cut) {
    SequenceMemory memory;
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
    cpu.set_state(foreground);
    const auto before = cpu.state();
    require(cpu.start_irq_bus(contract), "Real IRQ precedes each joint checkpoint");
    for (unsigned n = 0; n < cut; ++n) { advance(memory, cpu); }
    const auto saved = cpu.checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      auto restored = memory;
      Hcs08 other(restored, BLOFELD_HCS08_SLA);
      other.restore_checkpoint(saved);
      require(other.state() == cpu.state() && other.irq_bus_state() == cpu.irq_bus_state(),
              "Fresh joint restore retains raw registers, masks, phase, vector and partial queue");
      for (unsigned n = cut; n < total_cycles; ++n) { advance(restored, other); }
      endpoint(restored, other, before);
    }
  }
}

void failures() {
  constexpr std::array<unsigned, 10> cuts{1, 2, 3, 4, 5, 6, 7, 9, 10, 11};
  for (const auto cut : cuts) {
    SequenceMemory memory;
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
    cpu.set_state(foreground);
    require(cpu.start_irq_bus(contract), "Real source before each failed access");
    for (unsigned n = 1; n < cut; ++n) { advance(memory, cpu); }
    if (cut <= 5) {
      memory.writable.reset(stack - cut + 1);
    } else if (cut <= 7) {
      memory.known.reset(vector + cut - 6);
    } else {
      memory.known.reset(handler + cut - first_program);
    }
    const auto before = cpu.state();
    const auto records = memory.records;
    const auto error = rejected([&] { advance(memory, cpu); });
    const auto s = progress(cpu);
    require(error.contains("address=") && error.contains("cycle=" + std::to_string(cut)) &&
                cpu.faulted() && cpu.state() == before && s.completed_cycles == cut - 1 &&
                s.attempted_cycles == cut && memory.records == records && memory.retired == 0 &&
                s.ready_program_bytes == (cut < 10 ? 0U : cut - 9) && !s.program_byte_in_flight,
            "Each genuine stack/vector/program access failure retains its exact successful prefix");
    require(rejected([&] { cpu.advance_irq_bus(); }).contains("coordinated recovery") &&
                memory.records == records,
            "A failed phase cannot retry a stack or read side effect");
  }
  for (const bool ack : {false, true}) {
    SequenceMemory memory;
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
    cpu.set_state(foreground);
    require(cpu.start_irq_bus(contract), "Real source before hook failure");
    const unsigned cut = ack ? 7 : 1;
    for (unsigned n = 1; n < cut; ++n) { advance(memory, cpu); }
    memory.deny_begin = !ack;
    memory.deny_ack = ack;
    require(rejected([&] { advance(memory, cpu); }).contains("directed IRQ") && cpu.faulted() &&
                progress(cpu).completed_cycles == cut - 1 &&
                progress(cpu).attempted_cycles == cut &&
                progress(cpu).vector_reads == (ack ? 2U : 0U) && !progress(cpu).acknowledged,
            "Hook failure retains the completed vector bytes without fabricating acknowledgement");
  }
}

void source_selection() {
  SequenceMemory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(foreground);
  require(cpu.start_irq_bus(contract), "Initial source selects IRQ entry");
  for (unsigned n = 0; n < 5; ++n) { advance(memory, cpu); }
  memory.interrupt = {.vector = high_vector};
  advance(memory, cpu); // Actual vector selection after the five pushes.
  require(progress(cpu).vector == high_vector && progress(cpu).vector_bytes[0] == other_page,
          "The local highest-priority pending source is selected at the first actual vector read");
  memory.interrupt = {.vector = vector}; // A later source cannot mix another low vector byte.
  for (unsigned n = 6; n < total_cycles; ++n) { advance(memory, cpu); }
  require(progress(cpu).vector == high_vector && progress(cpu).target == other_handler &&
              cpu.state().pc == other_handler && memory.acknowledgements == 1,
          "Vector high/low remain coherent after selection even if local pending input changes");
  cpu.set_state(foreground);
  memory.interrupt = {.vector = vector};
  require(cpu.start_irq_bus(contract), "Fresh explicit debug context for withdrawal refusal");
  for (unsigned n = 0; n < 5; ++n) { advance(memory, cpu); }
  memory.interrupt.reset();
  const auto records = memory.records;
  require(rejected([&] { cpu.advance_irq_bus(); }).contains("withdrawal") && cpu.faulted() &&
              progress(cpu).completed_cycles == 5 && memory.records == records,
          "Unknown withdrawal/latch behavior cannot become a remembered successful source");
}

void prefetch_wrap() {
  SequenceMemory memory;
  constexpr std::array<std::uint8_t, 2> target{0xff, 0xff};
  constexpr std::array<std::uint8_t, 1> last{nop};
  constexpr std::array<std::uint8_t, 2> first{0x82, 0x9b};
  memory.install(vector, target);
  memory.install(word_mask, last);
  memory.install(0, first);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(foreground);
  require(cpu.start_irq_bus(contract), "Real IRQ before a16-bit program-fetch wrap");
  for (unsigned n = 0; n < total_cycles; ++n) { advance(memory, cpu); }
  const auto state = progress(cpu);
  require(
      cpu.state().pc == word_mask && state.prefetched[0] == last[0] &&
          state.prefetched[1] == first[0] && state.prefetched[2] == first[1] &&
          memory.records[7].access.address == word_mask && memory.records[8].access.address == 0 &&
          memory.records[9].access.address == 1 && memory.retired == 0,
      "Actual program requests wrap at16 bits and preserve each captured byte without execution");
}

void queue_ownership() {
  SequenceMemory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(foreground);
  require(cpu.start_irq_bus(contract), "Real source precedes ownership tests");
  const auto initial = cpu.checkpoint();
  require(rejected([&] { cpu.step(); }).contains("still has an owner") && memory.records.empty(),
          "Instruction execution cannot overlap an incomplete IRQ sequence");
  cpu.restore_checkpoint(initial);
  for (unsigned n = 0; n < total_cycles; ++n) { advance(memory, cpu); }
  const auto saved = cpu.checkpoint();
  const auto queue = progress(cpu).prefetched;
  memory.write(handler, 0); // Actual memory can change while prefetched bytes remain captured.
  require(progress(cpu).prefetched == queue &&
              rejected([&] { cpu.step(); }).contains("queue consumption") && memory.retired == 0,
          "Ordinary native execution cannot silently discard captured bytes or run changed code");
  cpu.restore_checkpoint(saved);
  require(rejected([&] { (void)cpu.service_interrupt(); }).contains("still has an owner"),
          "Untimed IRQ entry cannot bypass the owned program queue");
  cpu.set_state(foreground);
  require(!cpu.irq_bus_state(), "Explicit debug state replacement discards bus/queue ownership");
  require(rejected([&] {
            (void)cpu.start_irq_bus(Hcs08IrqBusContract::unqualified);
          }).contains("explicit") &&
              !cpu.faulted(),
          "An absent bus-sequence contract cannot mutate a healthy CPU");
}

class NoBdcMemory final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(ExecutionContext /*context*/) const override { return false; }

  std::uint8_t read(std::uint16_t /*address*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected BDC read");
  }

  void write(ByteWrite /*access*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected BDC write");
  }
};

void clock_ownership() {
  SequenceMemory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  NoBdcMemory target;
  cpu.set_state(foreground);
  Hcs08Bdc bdc(cpu, target, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::bkgd_held_low);
  require(!bdc.start_irq_bus(contract) && !cpu.irq_bus_state() && memory.records.empty(),
          "Active BDC holds IRQ admission before the first access");
  bdc.receive(hcs08_bdc::go);
  bdc.service();
  Hcs08RunClock instruction_clock(memory, bdc, Hcs08RunContract::reference_instruction_retirement);
  require(bdc.start_irq_bus(contract), "Actual BDC GO permits separate IRQ bus ownership");
  require(
      rejected([&] { instruction_clock.advance(); }).contains("separate qualified clock owner") &&
          instruction_clock.state().offered_periods == 0 &&
          instruction_clock.state().pending == cpu.state() && !cpu.faulted() &&
          memory.records.empty(),
      "Instruction retirement timing cannot charge or fetch an owned IRQ as an opcode");
  for (unsigned n = 0; n < total_cycles; ++n) {
    ++memory.period;
    bdc.advance_irq_bus();
  }
  require(progress(cpu).prefetched_bytes == 3 && memory.retired == 0,
          "The actual BDC wrapper advances all real IRQ accesses without retiring an instruction");
}
}

int main() {
  try {
    masks_and_wrap();
    copies();
    failures();
    source_selection();
    prefetch_wrap();
    queue_ownership();
    clock_ownership();
    std::cout << "HCS08 eleven-cycle IRQ bus prefixes and retained program queues pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
