// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_cop.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned source = 0x4a, destination = 0x40, cop = 0x1800, stack = 0x47c;
constexpr unsigned high_h = 0x4200, high_known = 0xff00, reset_ccr = 0x68;
constexpr unsigned recovery = 0x5a, affected = 0x86, nz = 6;
constexpr unsigned status_mask = 0xfe;
constexpr unsigned vectors_per_form = 65536, expected_vectors = 2 * vectors_per_form;
unsigned vectors = 0;

class Memory final : public MaskedMemory {
public:
  Hcs08KnownByte sample;
  unsigned reads = 0, cop_writes = 0;
  Hcs08Cop watchdog{Hcs08CopContract::candidate_gt8a_run_bus_edges};

  void begin_instruction(const ExecutionContext value) override {
    MaskedMemory::begin_instruction(value);
    watchdog.context(value);
  }

  Hcs08KnownByte read_known_byte(const std::uint16_t address) override {
    if (address != source) { return MaskedMemory::read_known_byte(address); }
    ++reads;
    accesses.push_back(
        {.address = address, .value = sample.value, .write = false, .known = sample.known});
    return sample;
  }

  std::uint8_t read(const std::uint16_t address) override {
    if (address != source) { return MaskedMemory::read(address); }
    if (sample.known != byte_mask) { fail(address, "ordinary unknown source read"); }
    return read_known_byte(address).value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (address != cop) {
      MaskedMemory::write(address, value);
      return;
    }
    write_known_byte({.address = address, .value = value}, byte_mask);
  }

  void write_known_byte(const ByteWrite access, const std::uint8_t mask) override {
    if (access.address != cop) {
      MaskedMemory::write_known_byte(access, mask);
      return;
    }
    // GT5.7.2: SRS data is ignored. The real COP owner consumes the actual store.
    watchdog.write(access);
    ++cop_writes;
    accesses.push_back({.address = cop, .value = access.value, .write = true, .known = mask});
  }

  void acknowledge_interrupt(const std::uint16_t vector) override {
    require(interrupt && interrupt->vector == vector,
            "Only the actual own pending IRQ acknowledges");
    interrupt.reset();
  }
};

struct Flags {
  unsigned value, known;
};

Flags expected_flags(const Hcs08State &before, const Hcs08KnownByte byte) {
  // Independent finite concretization: flags are known only if every matching
  // physical byte agrees. No implementation transfer equation is reused.
  unsigned always = nz;
  unsigned ever = 0;
  for (unsigned candidate = 0; candidate < byte_values; ++candidate) {
    if ((candidate & byte.known) != (byte.value & byte.known)) { continue; }
    const unsigned flags = (candidate >= 128 ? flag_n : 0U) | (candidate == 0 ? flag_z : 0U);
    always &= flags;
    ever |= flags;
  }
  const unsigned known = (before.ccr_known & ~affected) | flag_v | ((~(always ^ ever)) & nz);
  return {.value = ((before.ccr & ~affected) | always) & known, .known = known};
}

Hcs08State initial(const unsigned mask) {
  return {.pc = entry,
          .hx = high_h,
          .sp = stack,
          .a = byte_mask,
          .ccr_known = static_cast<std::uint8_t>(mask % 2 == 0 ? reset_ccr : byte_mask),
          .ccr = byte_mask,
          .a_known = 0,
          .hx_known = high_known};
}

void check_state(const Hcs08State &actual,
                 Hcs08State expected,
                 const Hcs08KnownByte byte,
                 const unsigned pc) {
  const auto flags = expected_flags(expected, byte);
  expected.pc = static_cast<std::uint16_t>(pc);
  expected.a = static_cast<std::uint8_t>(byte.value & byte.known);
  expected.a_known = byte.known;
  expected.ccr = static_cast<std::uint8_t>(flags.value);
  expected.ccr_known = static_cast<std::uint8_t>(flags.known);
  require(actual == expected,
          "Actual native A/HX/SP/PC/CCR and knownness match all concrete bytes");
}

void matrix_form(const bool extended) {
  Memory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  const std::vector<std::uint8_t> program =
      extended ? std::vector<std::uint8_t>{0xc6,
                                           0,
                                           source,
                                           0xb7,
                                           destination,
                                           0xc7,
                                           0x18,
                                           0,
                                           0xa6,
                                           recovery}
               : std::vector<std::uint8_t>{
                     0xb6, source, 0xb7, destination, 0xc7, 0x18, 0, 0xa6, recovery};
  memory.install(entry, program);
  memory.writable.set(destination);
  const unsigned loaded_pc = entry + (extended ? 3U : 2U);
  for (unsigned mask = 0; mask < byte_values; ++mask) {
    for (unsigned value = 0; value < byte_values; ++value) {
      memory.sample = {.value = static_cast<std::uint8_t>(value),
                       .known = static_cast<std::uint8_t>(mask)};
      memory.accesses.clear();
      memory.reads = 0;
      memory.cop_writes = 0;
      cpu.set_state(initial(mask));
      const auto before = cpu.state();
      require(memory.watchdog.advance_bus(7) == 7 && memory.watchdog.state().elapsed == 7,
              "Declared bus edges advance the actual COP before the real native service");
      cpu.step();
      check_state(cpu.state(), before, memory.sample, loaded_pc);
      cpu.step();
      check_state(cpu.state(), before, memory.sample, loaded_pc + 2);
      require(memory.bytes[destination] == (value & mask) &&
                  memory.known[destination] == (mask == byte_mask) &&
                  (mask == byte_mask || memory.partial_known[destination] == mask),
              "Real native direct STA stores the value mask, never a known representative");
      cpu.step();
      check_state(cpu.state(), before, memory.sample, loaded_pc + 5);
      require(memory.reads == 1 && memory.cop_writes == 1 && memory.accesses.size() == 3 &&
                  memory.watchdog.state().elapsed == 0 &&
                  !memory.watchdog.state().reset_requested && memory.accesses[1].known == mask &&
                  memory.accesses[2].known == mask && memory.accesses[2].value == (value & mask),
              "One actual source read and each real native store side effect occur once");
      cpu.step();
      require(cpu.state().a == recovery && cpu.state().a_known == byte_mask &&
                  (cpu.state().ccr_known & affected) == affected,
              "A subsequent real known LDA recovers A and V/N/Z without repairing other registers");
      ++vectors;
    }
  }
  require(memory.retired == 4 * vectors_per_form,
          "All four real native instructions retire once per vector");
}

void unknown_consumers_and_restore() {
  Memory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  constexpr std::array<std::uint8_t, 5> branch{0xb6, source, 0x27, 0, 0x9d};
  memory.install(entry, branch);
  memory.sample = {.value = 0, .known = status_mask};
  cpu.set_state(initial(0));
  const auto reset = cpu.checkpoint();
  const auto reset_memory = memory;
  cpu.step();
  const auto after_load = cpu.state();
  require((after_load.ccr_known & flag_z) == 0 && after_load.a_known == status_mask,
          "An unknown status bit zero keeps Z and that A bit unknown");
  require(rejected([&] { cpu.step(); }).contains("unknown flags") && cpu.faulted() &&
              memory.reads == 1 && memory.cop_writes == 0,
          "A dependent real branch refuses before using an unknown representative");
  require(rejected([&] { static_cast<void>(cpu.checkpoint()); }).contains("healthy"),
          "A faulted continuation cannot publish a checkpoint");
  memory = reset_memory;
  cpu.restore_checkpoint(reset);
  memory.sample = {.value = 1, .known = byte_mask};
  cpu.step();
  cpu.step();
  require(cpu.state().pc == entry + 4 && cpu.state().a == 1 && cpu.state().a_known == byte_mask,
          "Joint recovery with a complete changed future executes the real not-taken branch");
  constexpr std::array<std::uint8_t, 4> comparison{0xb6, source, 0xa1, 0};
  memory.install(entry, comparison);
  memory.sample = {.value = byte_mask, .known = 0};
  cpu.set_state(initial(0));
  cpu.step();
  require(rejected([&] { cpu.step(); }).contains("unknown operand") && cpu.faulted(),
          "Unqualified arithmetic consumers remain strict");
}

void write_refusals() {
  Memory denied;
  Hcs08 cpu(denied, BLOFELD_HCS08_SLA);
  constexpr std::array<std::uint8_t, 2> store{0xb7, destination};
  denied.install(entry, store);
  cpu.set_state(initial(0));
  require(rejected([&] { cpu.step(); }).contains("masked RAM write") && cpu.faulted() &&
              denied.retired == 0 && denied.accesses.empty(),
          "Unknown data never bypasses real destination permissions or retires a rejected write");
  Memory expired;
  Hcs08 cop_cpu(expired, BLOFELD_HCS08_SLA);
  constexpr unsigned timeout = 262144;
  constexpr std::array<std::uint8_t, 3> service{0xc7, 0x18, 0};
  expired.install(entry, service);
  require(expired.watchdog.advance_bus(timeout) == timeout &&
              expired.watchdog.state().reset_requested,
          "The real COP reaches its exact declared timeout before the service attempt");
  const auto before = expired.watchdog.state();
  cop_cpu.set_state(initial(0));
  require(
      rejected([&] { cop_cpu.step(); }).contains("pending COP reset") && cop_cpu.faulted() &&
          expired.retired == 0 && expired.cop_writes == 0 && expired.watchdog.state() == before,
      "A masked native store cannot clear a pending hardware reset or fabricate service success");
  Memory operand;
  Hcs08 x_cpu(operand, BLOFELD_HCS08_SLA);
  constexpr std::array<std::uint8_t, 2> load_x{0xbe, source};
  operand.install(entry, load_x);
  operand.sample = {.value = byte_mask, .known = 0};
  x_cpu.set_state(initial(0));
  require(rejected([&] { x_cpu.step(); }).contains("ordinary unknown source") && x_cpu.faulted() &&
              operand.retired == 0 && operand.reads == 0,
          "The raw LDA capability does not admit other native unknown-memory consumers");
}

void irq_roundtrip() {
  Memory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA, Hcs08RtiIrqContract::reference_next_instruction);
  constexpr unsigned vector = 0xffe0;
  constexpr unsigned handler = 0x8100;
  constexpr std::array<std::uint8_t, 3> load{0xb6, source, 0x9d};
  constexpr std::array<std::uint8_t, 2> target{0x81, 0};
  constexpr std::array<std::uint8_t, 1> rti{0x80};
  memory.install(entry, load);
  memory.install(vector, target);
  memory.install(handler, rti);
  for (unsigned n = 0; n < 5; ++n) { memory.writable.set(stack - n); }
  memory.sample = {.value = 1, .known = status_mask};
  auto state = initial(0);
  state.ccr = flags_fixed;
  state.ccr_known = byte_mask;
  cpu.set_state(state);
  cpu.step();
  const auto before_irq = cpu.state();
  const auto checkpoint = cpu.checkpoint();
  const auto saved_memory = memory;
  for (unsigned replay = 0; replay < 3; ++replay) {
    memory = saved_memory;
    cpu.restore_checkpoint(checkpoint);
    memory.interrupt = InterruptRequest{.vector = vector, .kind = InterruptKind::maskable};
    require(cpu.service_interrupt() && cpu.state().pc == handler &&
                memory.partial_known[stack - 3] == status_mask &&
                memory.partial_known[stack - 2] == 0,
            "Real IRQ pushes unknown A/X bits through qualified raw stack storage");
    cpu.step();
    require(cpu.state() == before_irq && !cpu.faulted(),
            "Real RTI and repeated joint restore preserve the exact loaded A/CCR masks");
  }
}
}

int main() {
  try {
    matrix_form(false);
    matrix_form(true);
    require(vectors == expected_vectors, "Every own data/mask word ran for both LDA forms");
    unknown_consumers_and_restore();
    write_refusals();
    irq_roundtrip();
    std::cout << "Native known LDA/STA: " << vectors
              << " vectors; exhaustive concrete flag expectations, strict consumers and IRQ "
                 "restore passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
