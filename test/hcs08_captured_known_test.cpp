// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_cop.hpp"
#include "keyboard/hcs08_icg_events.hpp"
#include "keyboard/hcs08_icg_status.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x240;
constexpr unsigned source = 0x4a, destination = 0x40, cop = 0x1800;
constexpr unsigned lda_direct = 0xb6, lda_extended = 0xc6, sta_direct = 0xb7, sta_extended = 0xc7;
constexpr unsigned lda_immediate = 0xa6, cmp_direct = 0xb1, ldx_direct = 0xbe;
constexpr unsigned nop = 0x9d, recover = 0x55, accumulator = 0x5c, index = 0x1234;
constexpr unsigned initial_flags = 0x61, partial_flags = 0xef, vnz = 0x86, nz = 6;
constexpr auto contract = Hcs08PrefetchContract::reference_peripheral_program_cycles;
unsigned directed_cases = 0, restored_cases = 0;

class Memory final : public MaskedMemory {
public:
  Hcs08KnownByte sample;
  Hcs08IcgStatus status{Hcs08IcgStatusContract::candidate_gt8a_observed_bits};
  bool use_status = false;
  unsigned reads = 0, writes = 0;
  bool deny_read = false, fail_after_read = false, deny_write = false, fail_after_write = false;
  Hcs08Cop watchdog{Hcs08CopContract::candidate_gt8a_run_bus_edges};

  void begin_instruction(const ExecutionContext value) override {
    MaskedMemory::begin_instruction(value);
    watchdog.context(value);
    status.context(value);
  }

  Hcs08KnownByte read_known_byte(const std::uint16_t address) override {
    if (address != source) { return MaskedMemory::read_known_byte(address); }
    if (deny_read) { fail(address, "own denied source read"); }
    const auto observed = use_status ? status.read_known(address) : sample;
    ++reads;
    accesses.push_back(
        {.address = address, .value = observed.value, .write = false, .known = observed.known});
    // Own changing-register fixture: the captured read must survive this change.
    sample = {.value = static_cast<std::uint8_t>(observed.value ^ byte_mask)};
    if (fail_after_read) { fail(address, "own failure after source side effect"); }
    return observed;
  }

  std::uint8_t read(const std::uint16_t address) override {
    if (address != source) { return MaskedMemory::read(address); }
    if (sample.known != byte_mask) { fail(address, "ordinary unknown source read"); }
    return read_known_byte(address).value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (address == destination || address == cop) {
      write_known_byte({.address = address, .value = value}, byte_mask);
    } else {
      MaskedMemory::write(address, value);
    }
  }

  void write_known_byte(const ByteWrite access, const std::uint8_t known) override {
    if (access.address != destination && access.address != cop) {
      MaskedMemory::write_known_byte(access, known);
      return;
    }
    if (deny_write) { fail(access.address, "own denied masked write"); }
    if (access.address == cop) {
      watchdog.write(access); // Candidate GT5.7.2: data is ignored by the actual COP owner.
      accesses.push_back({.address = cop, .value = access.value, .write = true, .known = known});
    } else if (known == byte_mask) {
      blofeld::test::hcs08::Memory::write(access.address, access.value);
    } else {
      MaskedMemory::write_known_byte(access, known);
    }
    ++writes;
    if (fail_after_write) { fail(access.address, "own failure after masked write"); }
  }

  void acknowledge_interrupt(const std::uint16_t selected) override {
    require(interrupt && interrupt->vector == selected,
            "Only the real pending source acknowledges");
    interrupt.reset();
  }
};

struct Input {
  std::uint8_t value = 0, known = 0;
  bool load_extended = false, store_extended = false;
};

unsigned load_bytes(const Input input) {
  return input.load_extended ? 3U : 2U;
}

unsigned store_bytes(const Input input) {
  return input.store_extended ? 3U : 2U;
}

Hcs08PrefetchState program(const Hcs08 &cpu) {
  const auto current = cpu.prefetch_state();
  if (!current) { throw Fault("Expected actual owned captured-data program"); }
  return *current;
}

struct Machine {
  Memory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};

  void prepare(const Input input, const unsigned operation = 0) {
    memory = Memory{};
    memory.writable.set();
    const std::array<std::uint8_t, 2> target{handler >> 8U, 0};
    const unsigned load = input.load_extended ? lda_extended : lda_direct;
    std::vector<std::uint8_t> code{static_cast<std::uint8_t>(operation != 0 ? operation : load)};
    if (input.load_extended) { code.push_back(0); }
    code.push_back(source);
    code.push_back(static_cast<std::uint8_t>(input.store_extended ? sta_extended : sta_direct));
    if (input.store_extended) { code.push_back(cop >> 8U); }
    code.push_back(static_cast<std::uint8_t>(input.store_extended ? 0 : destination));
    code.push_back(lda_immediate); // LDA immediate recovers A after the partial-byte transfer.
    code.push_back(recover);
    code.insert(code.end(), 8, nop);
    memory.install(vector, target);
    memory.install(handler, code);
    cpu.set_state({.pc = entry,
                   .hx = index,
                   .sp = stack,
                   .a = accumulator,
                   .ccr_known = partial_flags,
                   .ccr = initial_flags});
    memory.interrupt = {.vector = vector};
    require(cpu.start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
            "Actual eleven-period IRQ entry provides the program queue");
    for (unsigned n = 0; n < 11; ++n) { cpu.advance_irq_bus(); }
    cpu.start_irq_program(contract);
    memory.sample = {.value = input.value, .known = input.known};
    require(memory.watchdog.advance_bus(7) == 7, "Real COP has elapsed declared bus edges");
  }
};

Hcs08State expected(Hcs08State before, const Input input, const unsigned pc) {
  // Independent ISA oracle: enumerate every complete byte allowed by the observation.
  unsigned always = nz;
  unsigned ever = 0;
  for (unsigned concrete = 0; concrete < byte_values; ++concrete) {
    if ((concrete & input.known) != (input.value & input.known)) { continue; }
    const unsigned flags = (concrete >= 128 ? flag_n : 0U) | (concrete == 0 ? flag_z : 0U);
    always &= flags;
    ever |= flags;
  }
  before.a = static_cast<std::uint8_t>(input.value & input.known);
  before.a_known = input.known;
  before.pc = static_cast<std::uint16_t>(pc);
  before.ccr_known =
      static_cast<std::uint8_t>((before.ccr_known & ~vnz) | flag_v | ((~(always ^ ever)) & nz));
  before.ccr = static_cast<std::uint8_t>(((before.ccr & ~vnz) | always) & before.ccr_known);
  return before;
}

void check_load(Machine &machine, const Input input) {
  const auto before = machine.cpu.state();
  const auto result = expected(before, input, handler + load_bytes(input));
  const auto from = machine.memory.accesses.size();
  const unsigned read_phase = input.load_extended ? 2U : 1U;
  unsigned next_program = handler + 3;
  for (unsigned phase = 1; phase <= load_bytes(input) + 1; ++phase) {
    machine.cpu.advance_irq_program();
    const auto q = program(machine.cpu);
    const bool sampled = phase >= read_phase;
    const bool executed = phase > read_phase;
    require(machine.cpu.state() == (executed ? result : before) &&
                machine.memory.reads == static_cast<unsigned>(sampled) &&
                q.operand.in_flight == (phase == read_phase) && q.operand.consumed == executed &&
                q.native_executed == executed && q.completed_periods == phase &&
                q.attempted_periods == phase &&
                machine.memory.retired == static_cast<unsigned>(phase == load_bytes(input) + 1),
            "Independent rpp/prpp phase, arrival, native state and retirement match");
    if (sampled) {
      require(q.operand.value == (input.value & input.known) && q.operand.known == input.known &&
                  q.operand.address == source,
              "The real byte latch retains canonical value and exact mask after source changes");
    }
    const bool data = phase == read_phase;
    const unsigned address = data ? source : next_program++;
    const Access access{.address = address,
                        .value = data ? input.value : machine.memory.bytes[address],
                        .write = false,
                        .known = data ? input.known : static_cast<std::uint8_t>(byte_mask)};
    require(machine.memory.accesses.size() == from + phase &&
                machine.memory.accesses.back() == access,
            "One real read per period uses the independently specified address and byte");
  }
}

void check_store(Machine &machine, const Input input) {
  const auto before = machine.cpu.state();
  const unsigned write_phase = input.store_extended ? 2U : 1U;
  const unsigned pc = handler + load_bytes(input) + store_bytes(input);
  const auto result = expected(before, input, pc);
  const auto from = machine.memory.accesses.size();
  unsigned next_program = handler + load_bytes(input) + 3;
  const unsigned target = input.store_extended ? cop : destination;
  for (unsigned phase = 1; phase <= store_bytes(input) + 1; ++phase) {
    machine.cpu.advance_irq_program();
    const bool executed = phase >= write_phase;
    require(machine.cpu.state() == (executed ? result : before) && machine.memory.reads == 1 &&
                machine.memory.writes == static_cast<unsigned>(executed) &&
                machine.memory.retired ==
                    1U + static_cast<unsigned>(phase == store_bytes(input) + 1),
            "Independent wpp/pwpp state and retirement preserve the actual partial A");
    const bool data = phase == write_phase;
    const unsigned address = data ? target : next_program++;
    const Access access{.address = address,
                        .value = data ? static_cast<unsigned>(input.value & input.known)
                                      : machine.memory.bytes[address],
                        .write = data,
                        .known = data ? input.known : static_cast<std::uint8_t>(byte_mask)};
    require(machine.memory.accesses.size() == from + phase &&
                machine.memory.accesses.back() == access,
            "One real write at w carries the source mask through the actual bus owner");
    if (input.store_extended) {
      require(machine.memory.watchdog.state().elapsed == (executed ? 0U : 7U),
              "The original COP service side effect occurs at w, never at retirement or replay");
    }
  }
}

void check_case(Machine &machine, const Input input) {
  machine.prepare(input);
  check_load(machine, input);
  check_store(machine, input);
  for (unsigned n = 0; n < 2; ++n) { machine.cpu.advance_irq_program(); }
  const auto state = machine.cpu.state();
  require(state.pc == handler + load_bytes(input) + store_bytes(input) + 2 && state.a == recover &&
              state.a_known == byte_mask && (state.ccr_known & vnz) == vnz &&
              machine.memory.reads == 1 && machine.memory.writes == 1 &&
              machine.memory.retired == 3,
          "The next known native LDA recovers A/VNZ without re-reading or repeating the store");
  ++directed_cases;
}

struct Snapshot {
  Memory memory;
  Hcs08Checkpoint cpu;
};

Snapshot capture(const Input input, const unsigned cut) {
  Machine original;
  original.prepare(input);
  for (unsigned n = 0; n < cut; ++n) { original.cpu.advance_irq_program(); }
  return {.memory = original.memory, .cpu = original.cpu.checkpoint()};
}

void continuations() {
  constexpr std::array<Input, 4> inputs{
      {{.value = 0xff, .known = 0x15},
       {.value = 0, .known = 0xfe, .load_extended = true},
       {.value = 0x80, .known = 0x80, .store_extended = true},
       {.value = 0xff, .known = 0, .load_extended = true, .store_extended = true}}};
  for (const auto input : inputs) {
    Machine baseline;
    check_case(baseline, input);
    const unsigned periods = load_bytes(input) + store_bytes(input) + 4;
    for (unsigned cut = 0; cut <= periods; ++cut) {
      const auto saved = capture(input, cut);
      Machine fresh;
      fresh.memory = saved.memory;
      fresh.cpu.restore_checkpoint(saved.cpu);
      for (unsigned n = cut; n < periods; ++n) { fresh.cpu.advance_irq_program(); }
      require(fresh.cpu.state() == baseline.cpu.state() &&
                  fresh.cpu.prefetch_state() == baseline.cpu.prefetch_state() &&
                  fresh.memory.accesses == baseline.memory.accesses &&
                  fresh.memory.bytes == baseline.memory.bytes &&
                  fresh.memory.partial_known == baseline.memory.partial_known &&
                  fresh.memory.sample.value == baseline.memory.sample.value &&
                  fresh.memory.sample.known == baseline.memory.sample.known &&
                  fresh.memory.reads == 1 && fresh.memory.writes == 1 &&
                  fresh.memory.retired == 3 &&
                  fresh.memory.watchdog.state() == baseline.memory.watchdog.state(),
              "Fresh joint restores preserve every r/p/w arrival, native copy and side effect");
      ++restored_cases;
    }
  }
}

void actual_status_read_effect() {
  constexpr Input input{.value = 0x11, .known = 0x15, .store_extended = true};
  Machine machine;
  machine.prepare(input);
  machine.memory.use_status = true;
  machine.memory.status.signal(Hcs08IcgFault::loss_of_lock);
  require(!machine.memory.status.state().events.interrupt_read_armed,
          "An established candidate fault starts with its real read-ack latch unarmed");
  const auto before = machine.cpu.state();
  machine.cpu.advance_irq_program();
  require(machine.cpu.state() == before && machine.memory.reads == 1 &&
              machine.memory.status.state().events.interrupt_read_armed &&
              program(machine.cpu).operand.known == input.known &&
              program(machine.cpu).operand.value == input.value &&
              program(machine.cpu).operand.in_flight,
          "Actual ICGS1 read arms read-ack at r, before native A/CCR retirement");
  machine.memory.status.signal(Hcs08IcgFault::loss_of_clock);
  require(!machine.memory.status.state().events.interrupt_read_armed,
          "A newly established detector event invalidates the earlier read-ack sequence");
  machine.cpu.advance_irq_program();
  machine.cpu.advance_irq_program();
  require(machine.cpu.state() == expected(before, input, handler + 2) &&
              machine.memory.reads == 1 &&
              !machine.memory.status.state().events.interrupt_read_armed,
          "Native LDA copies the older real latch without re-reading or re-arming changed ICGS1");
  check_store(machine, input);
  require(machine.memory.watchdog.state().elapsed == 0,
          "The captured status value reaches the real data-ignoring COP store");
}

void strict_consumers_and_fetch() {
  constexpr Input input{.value = 0xff, .known = 0x15};
  for (const unsigned opcode :
       {cmp_direct, ldx_direct}) { // CMP and LDX do not own raw-byte semantics.
    Machine machine;
    machine.prepare(input, opcode);
    require(rejected([&] { machine.cpu.advance_irq_program(); }).contains("ordinary unknown") &&
                machine.cpu.faulted() && machine.memory.reads == 0 && machine.memory.retired == 0 &&
                !program(machine.cpu).operand.value,
            "Unqualified native consumers still refuse unknown data at the actual r period");
  }
  Machine machine;
  machine.prepare(input);
  machine.cpu.advance_irq_program();
  const auto saved_memory = machine.memory;
  const auto saved_cpu = machine.cpu.checkpoint();
  machine.memory.known.reset(handler + 3);
  machine.memory.partial_known[handler + 3] = input.known;
  require(rejected([&] { machine.cpu.advance_irq_program(); }).contains("address=0x9003") &&
              machine.cpu.faulted() && machine.memory.reads == 1 && machine.memory.retired == 0 &&
              program(machine.cpu).operand.known == input.known &&
              program(machine.cpu).operand.value == (input.value & input.known) &&
              !program(machine.cpu).operand.in_flight && !program(machine.cpu).operand.consumed,
          "Unknown program bytes remain strict; a failed p preserves the arrived raw data latch");
  machine.memory = saved_memory;
  machine.cpu.restore_checkpoint(saved_cpu);
  machine.cpu.advance_irq_program();
  machine.cpu.advance_irq_program();
  const auto loaded = machine.cpu.state();
  require(loaded.a_known == input.known && machine.memory.reads == 1 && machine.memory.retired == 1,
          "Joint recovery completes the actual captured load without repeating its read effect");
  const auto transfer = machine.cpu.checkpoint();
  blofeld::test::hcs08::Memory ordinary;
  ordinary = machine.memory;
  Hcs08 fresh(ordinary, BLOFELD_HCS08_SLA);
  fresh.restore_checkpoint(transfer);
  const auto accesses = ordinary.accesses;
  require(rejected([&] { fresh.advance_irq_program(); }).contains("partial-register") &&
              fresh.faulted() && ordinary.accesses == accesses && ordinary.retired == 1,
          "The actual data bus must own masked stores; the captured code view cannot grant them");
  machine.memory.writable.reset(destination);
  require(rejected([&] { machine.cpu.advance_irq_program(); }).contains("masked RAM write") &&
              machine.cpu.faulted() && machine.memory.writes == 0 && machine.memory.retired == 1,
          "A qualified partial store still enforces the actual board write permission");
}

void failures() {
  constexpr Input input{.value = 0xff, .known = 0xfe};
  for (const bool after : {false, true}) {
    Machine machine;
    machine.prepare(input);
    const auto before = machine.cpu.state();
    machine.memory.deny_read = !after;
    machine.memory.fail_after_read = after;
    require(rejected([&] { machine.cpu.advance_irq_program(); }).contains("address=0x004a") &&
                machine.cpu.faulted() && machine.cpu.state() == before &&
                machine.memory.reads == static_cast<unsigned>(after) &&
                program(machine.cpu).completed_periods == 0 &&
                !program(machine.cpu).operand.value &&
                rejected([&] { machine.cpu.advance_irq_program(); }).contains("recovery"),
            "A refused r keeps its real side-effect prefix and cannot manufacture a captured byte");
  }
  for (const bool after : {false, true}) {
    Machine machine;
    machine.prepare(input);
    check_load(machine, input);
    const auto before = machine.cpu.state();
    machine.memory.deny_write = !after;
    machine.memory.fail_after_write = after;
    require(rejected([&] { machine.cpu.advance_irq_program(); }).contains("address=0x0040") &&
                machine.cpu.faulted() && machine.memory.writes == static_cast<unsigned>(after) &&
                machine.memory.retired == 1 && machine.cpu.state() == before &&
                program(machine.cpu).completed_periods == 3 &&
                rejected([&] { machine.cpu.advance_irq_program(); }).contains("recovery"),
            "A refused w preserves the native prefix, never rolls back or retires a partial write");
  }
}
}

int main() {
  try {
    Machine machine;
    constexpr std::array<std::uint8_t, 6> values{0, 1, 0x15, 0x7f, 0x80, 0xff};
    for (unsigned mask = 0; mask < byte_values; ++mask) {
      for (const auto value : values) {
        for (const bool load_extended : {false, true}) {
          for (const bool store_extended : {false, true}) {
            check_case(machine,
                       {.value = value,
                        .known = static_cast<std::uint8_t>(mask),
                        .load_extended = load_extended,
                        .store_extended = store_extended});
          }
        }
      }
    }
    continuations();
    actual_status_read_effect();
    strict_consumers_and_fetch();
    failures();
    std::cout << "Captured LDA/STA: " << directed_cases << " independent directed phase cases; "
              << restored_cases << " fresh checkpoint continuations; failure prefixes pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
