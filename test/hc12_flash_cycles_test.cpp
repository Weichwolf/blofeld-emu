// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "hc12_cycle_fixture.hpp"
#include "mcu_checkpoint_fixture.hpp"

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace {
using blofeld::test::require;
constexpr unsigned entry = 0x8000;
constexpr unsigned flash_base = 0x9000;
constexpr unsigned source = 0x9001;
constexpr unsigned second_source = 0xa001;
constexpr unsigned ram_source = 0x1201;
constexpr unsigned stack = 0x2000;
constexpr unsigned initial_d = 0x3456;
constexpr unsigned initial_y = 0x4567;
constexpr unsigned initial_ccr = 0xd1;
constexpr unsigned negative_ccr = 0xd9;
constexpr unsigned word = 0x1234;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned owner_ticks = 90;
constexpr unsigned owner_edges = 18;

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) { return fault.what(); }
  throw blofeld::Fault("Expected phased-read rejection");
}

struct Read {
  std::uint16_t address = 0;
  unsigned size = 1;
  std::size_t cycle = 0;
};

struct ReadMemory final : blofeld::Bus {
  explicit ReadMemory(const std::vector<blofeld::Hc12BusCycle> &clocks) : clocks_(clocks) {
    known.set();
  }

  std::uint8_t read(const std::uint16_t address) override {
    if (!known.test(address)) {
      throw blofeld::Fault("Own unknown byte at " + blofeld::hex(address));
    }
    return data.read(address);
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    data.write(address, value);
  }

  std::uint64_t read_memory(const blofeld::MemoryRead transfer) override {
    reads.push_back({.address = transfer.address, .size = transfer.size, .cycle = clocks_.size()});
    return Bus::read_memory(transfer);
  }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    return data.peek(address);
  }

  [[nodiscard]] std::uint64_t revision() const override { return data.revision(); }

  void check_execute(blofeld::InstructionFetch /*fetch*/) const override {}

  blofeld::test::CycleMemory data;
  std::bitset<65536> known;
  std::vector<Read> reads;

  const std::vector<blofeld::Hc12BusCycle> &clocks_;
};

blofeld::Hc12ReadTiming geometry(const blofeld::MemoryRead transfer) {
  return transfer.size == 2 && transfer.address >= flash_base && (transfer.address & 1U) != 0
             ? blofeld::Hc12ReadTiming::byte_cycles
             : blofeld::Hc12ReadTiming::single_cycle;
}

struct Fixture {
  std::vector<blofeld::Hc12BusCycle> clocks;
  ReadMemory memory{clocks};
  std::function<void(blofeld::Hc12BusCycle)> observer;
  blofeld::Hc12CycleBus bus;
  blofeld::Hc12 cpu;

  explicit Fixture(blofeld::Hc12CycleBus::ReadTiming timing = geometry)
      : bus(
            memory,
            [this](const auto cycle) {
              clocks.push_back(cycle);
              if (observer) { observer(cycle); }
            },
            std::move(timing)),
        cpu(bus, BLOFELD_DEFAULT_SLA) {
    cpu.reset();
    memory.reads.clear();
  }

  void install(const std::span<const std::uint8_t> code) {
    memory.data.install(code, entry);
    cpu.set_state({.pc = entry,
                   .d = initial_d,
                   .x = source,
                   .y = initial_y,
                   .sp = stack,
                   .ccr = initial_ccr});
  }

  void word_at(const unsigned address, const unsigned value) {
    memory.data.bytes.at(address) = static_cast<std::uint8_t>(value >> 8U);
    memory.data.bytes.at(address + 1) = static_cast<std::uint8_t>(value & byte_mask);
  }
};

void check_read(const Fixture &fixture,
                const unsigned index,
                const Read expected,
                const blofeld::Hc12WordPart part,
                const char kind = 'R') {
  const auto actual = fixture.memory.reads.at(index);
  const auto clock = fixture.clocks.at(expected.cycle - 1);
  require(actual.address == expected.address && actual.size == expected.size &&
              actual.cycle == expected.cycle && clock.kind == kind &&
              clock.address == expected.address && clock.size == expected.size &&
              clock.word_part == part,
          "Each physical byte must be read after its own literal clock, with native slot metadata");
}

void test_literal_values_and_counts() {
  struct Vector {
    unsigned value;
    unsigned flags;
  };

  constexpr std::array<Vector, 3> vectors{{{.value = 0, .flags = 0xd5},
                                           {.value = 0x8000, .flags = negative_ccr},
                                           {.value = word, .flags = initial_ccr}}};
  constexpr std::array<unsigned, 3> addresses{flash_base, source, ram_source};
  for (const auto address : addresses) {
    for (const auto item : vectors) {
      Fixture fixture;
      const std::array<std::uint8_t, 3> code{0xfc,
                                             static_cast<std::uint8_t>(address >> 8U),
                                             static_cast<std::uint8_t>(address & byte_mask)};
      fixture.install(code);
      fixture.word_at(address, item.value);
      fixture.cpu.step();
      const bool expanded = address == source;
      const auto state = fixture.cpu.state();
      require(fixture.bus.cycles() == (expanded ? 9U : 8U) && state.pc == entry + code.size() &&
                  state.d == item.value && state.x == source && state.y == initial_y &&
                  state.sp == stack && state.ccr == item.flags,
              "LDD matches literal value/flags and manual reset+RPO(+one Flash cycle) costs");
      if (expanded) {
        require(fixture.memory.reads.size() == 2, "One logical word uses exactly two byte reads");
        check_read(fixture, 0, {.address = source, .cycle = 6}, blofeld::Hc12WordPart::high);
        check_read(fixture, 1, {.address = source + 1, .cycle = 7}, blofeld::Hc12WordPart::low);
      } else {
        require(fixture.memory.reads.size() == 1 && fixture.memory.reads.front().size == 2 &&
                    fixture.memory.reads.front().cycle == 6 &&
                    fixture.clocks.at(5).word_part == blofeld::Hc12WordPart::whole,
                "Aligned Flash and misaligned RAM retain one logical word access and one cycle");
      }
    }
  }
}

void test_latch_at_each_phase() {
  Fixture fixture;
  constexpr std::array<std::uint8_t, 3> code{0xfc, 0x90, 1};
  fixture.install(code);
  fixture.word_at(source, word);
  constexpr unsigned changed_word = 0xabcd;
  constexpr unsigned expected = 0x12cd;
  fixture.observer = [&fixture](const auto cycle) {
    if (cycle.word_part == blofeld::Hc12WordPart::low) { fixture.word_at(source, changed_word); }
  };
  fixture.cpu.step();
  require(fixture.cpu.state().d == expected && fixture.cpu.state().ccr == initial_ccr &&
              fixture.bus.cycles() == 9 && fixture.memory.reads.size() == 2,
          "The first byte stays latched; only the second byte observes the later memory state");
}

void test_indirect_pointer_and_operand_phases() {
  Fixture fixture;
  constexpr std::array<std::uint8_t, 4> code{0xec, 0xe3, 0, 0}; // Own LDD [0,X].
  fixture.install(code);
  fixture.word_at(source, second_source);
  constexpr unsigned result = 0x8000;
  fixture.word_at(second_source, result);
  fixture.cpu.step();
  require(fixture.bus.cycles() == 13 && fixture.cpu.state().d == result &&
              fixture.cpu.state().ccr == negative_ccr &&
              fixture.cpu.state().pc == entry + code.size() && fixture.cpu.state().x == source &&
              fixture.memory.reads.size() == 4,
          "The manual fIPRPf path expands both odd Flash words without changing native semantics");
  check_read(fixture, 0, {.address = source, .cycle = 7}, blofeld::Hc12WordPart::high, 'I');
  check_read(fixture, 1, {.address = source + 1, .cycle = 8}, blofeld::Hc12WordPart::low, 'I');
  check_read(fixture, 2, {.address = second_source, .cycle = 10}, blofeld::Hc12WordPart::high);
  check_read(fixture, 3, {.address = second_source + 1, .cycle = 11}, blofeld::Hc12WordPart::low);
}

void check_failed_cpu(const Fixture &fixture) {
  const auto state = fixture.cpu.state();
  require(fixture.bus.faulted() && state.pc == entry && state.d == initial_d && state.x == source &&
              state.y == initial_y && state.sp == stack && state.ccr == initial_ccr,
          "A partial native read must not commit architectural registers or flags");
}

void test_byte_and_clock_failures() {
  constexpr std::array<std::uint8_t, 3> code{0xfc, 0x90, 1};
  for (unsigned byte = 0; byte < 2; ++byte) {
    for (const bool clock_failure : {false, true}) {
      Fixture fixture;
      fixture.install(code);
      fixture.word_at(source, word);
      if (clock_failure) {
        fixture.observer = [byte](const auto cycle) {
          const auto part = byte == 0 ? blofeld::Hc12WordPart::high : blofeld::Hc12WordPart::low;
          if (cycle.word_part == part) { throw blofeld::Fault("Own phase clock failure"); }
        };
      } else {
        fixture.memory.known.reset(source + byte);
      }
      const auto error = rejected([&fixture] { fixture.cpu.step(); });
      const unsigned completed = (clock_failure ? 5U : 6U) + byte;
      const unsigned reads = (clock_failure ? 0U : 1U) + byte;
      check_failed_cpu(fixture);
      require(error.contains("pc=0x8000") && error.contains("ccr=0xd1") &&
                  fixture.bus.cycles() == completed && fixture.memory.reads.size() == reads,
              "Phase failure retains exactly the completed clocks and backing byte reads");
      const auto attempts = fixture.clocks.size();
      static_cast<void>(rejected([&fixture] { fixture.cpu.step(); }));
      require(fixture.clocks.size() == attempts && fixture.memory.reads.size() == reads,
              "A failed read remains latched and cannot execute its remaining phase on retry");
    }
  }
}

blofeld::McuRunContract contract(const blofeld::McuRunMemory memory) {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .initial_oscillator_phase_ticks = 0,
                    .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = memory};
}

blofeld::McuFlashImage image() {
  constexpr std::array<std::uint8_t, 3> code{0xfc, 0x90, 1};
  auto result = blofeld::test::checkpoint_image(code);
  result.bytes.at(source - entry) = word >> 8U;
  result.bytes.at(source - entry + 1) = word & byte_mask;
  return result;
}

void check_owner(const blofeld::McuMachine &machine) {
  require(machine.run_clock().bus_cycles() == 9 &&
              machine.run_clock().elapsed_ticks() == owner_ticks &&
              machine.run_clock().oscillator_cycles() == owner_edges && !machine.timing_faulted() &&
              machine.cpu().state().d == word && machine.cpu().state().pc == entry + 3,
          "The owner counts both Flash phases in the same clock used by trusted checkpoints");
}

void test_owner_recovery_and_contracts() {
  const auto timing = contract(blofeld::McuRunMemory::internal_flash_reads);
  blofeld::McuMachine machine(image(), BLOFELD_DEFAULT_SLA, timing);
  const auto frozen = machine.checkpoint();
  machine.set_cycle_observer([&machine, &frozen](const auto cycle) {
    if (cycle.word_part != blofeld::Hc12WordPart::low) { return; }
    static_cast<void>(rejected([&machine] { static_cast<void>(machine.checkpoint()); }));
    static_cast<void>(rejected([&machine, &frozen] { machine.restore(frozen); }));
    throw blofeld::Fault("Own second-phase observation failure");
  });
  static_cast<void>(rejected([&machine] { machine.cpu().step(); }));
  require(
      machine.timing_faulted() && machine.run_clock().bus_cycles() == 6,
      "Failed second-phase observation leaves one completed data cycle, without a second clock");
  machine.restore(frozen);
  machine.cpu().step();
  check_owner(machine);
  const auto completed = machine.checkpoint();
  machine.cpu().step();
  machine.restore(completed);
  check_owner(machine);
  blofeld::McuMachine unstretched(
      image(), BLOFELD_DEFAULT_SLA, contract(blofeld::McuRunMemory::unstretched_internal));
  static_cast<void>(rejected([&unstretched] { unstretched.cpu().step(); }));
  require(unstretched.timing_faulted() && unstretched.run_clock().bus_cycles() == 5,
          "The original un-stretched contract still rejects odd Flash word timing");
  static_cast<void>(rejected([&unstretched, &frozen] { unstretched.restore(frozen); }));
}

void test_owner_unknown_bytes() {
  const auto timing = contract(blofeld::McuRunMemory::internal_flash_reads);
  const blofeld::McuMachine known(image(), BLOFELD_DEFAULT_SLA, timing);
  const auto healthy = known.checkpoint();
  for (unsigned byte = 0; byte < 2; ++byte) {
    auto unknown = image();
    unknown.invalidate({.offset = static_cast<std::uint16_t>(source - entry + byte), .size = 1});
    blofeld::McuMachine machine(unknown, BLOFELD_DEFAULT_SLA, timing);
    const auto frozen = machine.checkpoint();
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
      machine.restore(frozen);
      const auto error = rejected([&machine] { machine.cpu().step(); });
      require(
          error.contains(blofeld::hex(source + byte)) && machine.timing_faulted() &&
              machine.run_clock().bus_cycles() == 6 + byte &&
              !machine.bus().power_off_flash_image().known.test(source - entry + byte),
          "Unknown first/second Flash bytes fail at their own phase and remain unknown on restore");
    }
    machine.restore(healthy);
    machine.cpu().step();
    check_owner(machine);
  }
}

void test_flash_alias_and_unsupported_accesses() {
  const auto timing = contract(blofeld::McuRunMemory::internal_flash_reads);
  constexpr unsigned alias = 0x5001;
  auto aliased = image();
  aliased.bytes[1] = alias >> 8U;
  aliased.bytes[2] = alias & byte_mask;
  blofeld::McuMachine machine(aliased, BLOFELD_DEFAULT_SLA, timing);
  std::vector<blofeld::Hc12BusCycle> bytes;
  machine.set_cycle_observer([&bytes](const auto cycle) {
    if (cycle.word_part != blofeld::Hc12WordPart::whole) { bytes.push_back(cycle); }
  });
  machine.cpu().step();
  check_owner(machine);
  require(bytes.size() == 2 && bytes[0].address == alias && bytes[1].address == alias + 1,
          "Flash aliases use their logical addresses and the same physical array geometry");
  constexpr std::array<unsigned, 3> unsupported{0x7fff, 0xffff, 0x0249};
  for (const auto address : unsupported) {
    auto program = image();
    program.bytes[1] = static_cast<std::uint8_t>(address >> 8U);
    program.bytes[2] = static_cast<std::uint8_t>(address & byte_mask);
    blofeld::McuMachine rejected_word(program, BLOFELD_DEFAULT_SLA, timing);
    const auto error = rejected([&rejected_word] { rejected_word.cpu().step(); });
    require(error.contains("Unqualified MCU reference R") && rejected_word.timing_faulted() &&
                rejected_word.run_clock().bus_cycles() == 5,
            "Array discontinuities, address wrapping and odd register words stay unqualified");
  }
  auto program = image();
  constexpr unsigned store_d = 0x7c;
  program.bytes[0] = store_d;
  blofeld::McuMachine write(program, BLOFELD_DEFAULT_SLA, timing);
  const auto error = rejected([&write] { write.cpu().step(); });
  // S12CPUV2 STD EXT is PWO: the program fetch completes before the rejected W.
  require(error.contains("Unqualified MCU reference W") && write.run_clock().bus_cycles() == 6 &&
              write.bus().power_off_flash_image().bytes == program.bytes,
          "The read timing contract neither qualifies odd Flash writes nor changes NVM");
}

void test_invalid_geometry() {
  constexpr std::array<std::uint8_t, 3> byte_code{0xb6, 0x90, 1};
  Fixture byte([](const auto transfer) {
    return transfer.address == source ? blofeld::Hc12ReadTiming::byte_cycles
                                      : blofeld::Hc12ReadTiming::single_cycle;
  });
  byte.install(byte_code);
  static_cast<void>(rejected([&byte] { byte.cpu.step(); }));
  require(byte.bus.cycles() == 5 && byte.memory.reads.empty(),
          "Byte operands cannot be misclassified as a split word");
  Fixture invalid([](const auto transfer) {
    return transfer.address == source ? blofeld::Hc12ReadTiming::unqualified
                                      : blofeld::Hc12ReadTiming::single_cycle;
  });
  constexpr std::array<std::uint8_t, 3> word_code{0xfc, 0x90, 1};
  invalid.install(word_code);
  static_cast<void>(rejected([&invalid] { invalid.cpu.step(); }));
  require(invalid.bus.cycles() == 5 && invalid.memory.reads.empty(),
          "Unknown timing selections reject before their first data cycle");
  Fixture wrap;
  constexpr std::array<std::uint8_t, 3> wrap_code{0xfc, 0xff, 0xff};
  wrap.install(wrap_code);
  static_cast<void>(rejected([&wrap] { wrap.cpu.step(); }));
  require(wrap.bus.cycles() == 5 && wrap.memory.reads.empty(),
          "An expanded word cannot silently wrap its second byte to address zero");
}
}

int main() {
  try {
    test_literal_values_and_counts();
    test_latch_at_each_phase();
    test_indirect_pointer_and_operand_phases();
    test_byte_and_clock_failures();
    test_owner_recovery_and_contracts();
    test_owner_unknown_bytes();
    test_flash_alias_and_unsupported_accesses();
    test_invalid_geometry();
    std::cout << "FTS32K word/byte phases, literal timing, native state and coordinated recovery "
                 "passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
