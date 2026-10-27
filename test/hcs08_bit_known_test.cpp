// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned affected = 0x86, source = 0x40, stack = 0x47c;
constexpr unsigned expected_vectors = 1679616;
constexpr unsigned strap_mask = 0x38, beq = 0x27, byte_replication = 257;

struct Flags {
  unsigned value, known;
};

Flags expected_flags(const Hcs08State &before, const unsigned immediate) {
  unsigned always = affected;
  unsigned ever = 0;
  for (unsigned candidate = 0; candidate < byte_values; ++candidate) {
    if ((candidate & before.a_known) != before.a) { continue; }
    const auto result = candidate & immediate;
    const unsigned flags = (result >= 128 ? flag_n : 0U) | (result == 0 ? flag_z : 0U);
    always &= flags;
    ever |= flags;
  }
  const unsigned known = (before.ccr_known & ~affected) | ((~(always ^ ever)) & affected);
  return {.value = ((before.ccr & ~affected) | always) & known, .known = known};
}

Hcs08State initial(const unsigned value, const unsigned known, const unsigned immediate) {
  return {.pc = entry,
          .hx = static_cast<std::uint16_t>((value << 8U) | immediate),
          .sp = stack,
          .a = static_cast<std::uint8_t>(value),
          .ccr_known = static_cast<std::uint8_t>(known ^ immediate),
          .ccr = static_cast<std::uint8_t>(value ^ immediate),
          .a_known = static_cast<std::uint8_t>(known),
          .hx_known = static_cast<std::uint16_t>(known * byte_replication)};
}

Hcs08State expected_state(Hcs08State before, const unsigned immediate) {
  const auto flags = expected_flags(before, immediate);
  before.pc += 2;
  before.ccr = static_cast<std::uint8_t>(flags.value);
  before.ccr_known = static_cast<std::uint8_t>(flags.known);
  return before;
}

void matrix() {
  Memory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  unsigned vectors = 0;
  for (unsigned immediate = 0; immediate < byte_values; ++immediate) {
    const std::array<std::uint8_t, 2> instruction{0xa5, static_cast<std::uint8_t>(immediate)};
    memory.install(entry, instruction);
    for (unsigned known = 0; known < byte_values; ++known) {
      for (unsigned value = 0; value < byte_values; ++value) {
        if ((value & known) != value) { continue; }
        cpu.set_state(initial(value, known, immediate));
        const auto before = cpu.state();
        cpu.step();
        require(cpu.state() == expected_state(before, immediate),
                "Native BIT full state and flag knowledge agree with every concrete input");
        ++vectors;
      }
    }
  }
  require(vectors == expected_vectors && memory.retired == vectors && memory.accesses.empty(),
          "All canonical partial-byte/immediate states execute exactly once without data I/O");
}

class InputMemory final : public MaskedMemory {
public:
  Hcs08KnownByte sample;
  unsigned reads = 0;

  Hcs08KnownByte read_known_byte(const std::uint16_t address) override {
    if (address != source) { return MaskedMemory::read_known_byte(address); }
    ++reads;
    return sample;
  }
};

void guard_and_restore() {
  for (const auto branch : {0x27U, 0x26U}) {
    for (const auto sample : {Hcs08KnownByte{.value = 0, .known = 0x38},
                              Hcs08KnownByte{.value = 8, .known = 8},
                              Hcs08KnownByte{.value = 0, .known = 0}}) {
      InputMemory memory;
      const std::array<std::uint8_t, 7> code{
          0xb6, source, 0xa5, 0x38, static_cast<std::uint8_t>(branch), 2, 0x9d};
      memory.install(entry, code);
      memory.sample = sample;
      Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
      cpu.set_state(initial(0, 0, 0));
      cpu.step();
      const auto after_load = cpu.state();
      const auto saved_cpu = cpu.checkpoint();
      const auto saved_memory = memory;
      for (unsigned replay = 0; replay < 3; ++replay) {
        if (replay != 0) {
          memory = saved_memory;
          cpu.restore_checkpoint(saved_cpu);
        }
        cpu.step();
        const auto after_bit = cpu.state();
        require(after_bit == expected_state(after_load, strap_mask) && memory.reads == 1,
                "Real load/BIT and trusted replay retain the single captured partial-byte read");
        if ((after_bit.ccr_known & flag_z) == 0) {
          require(rejected([&] { cpu.step(); }).contains("unknown flags") && cpu.faulted() &&
                      memory.retired == 2 && memory.reads == 1,
                  "An unknown deciding bit refuses the actual branch without retirement");
        } else {
          cpu.step();
          auto expected = after_bit;
          const bool zero = sample.value == 0;
          expected.pc =
              static_cast<std::uint16_t>(entry + 6U + ((zero == (branch == beq)) ? 2 : 0));
          require(cpu.state() == expected && memory.retired == 3 && memory.reads == 1,
                  "Native BEQ/BNE consumes only established Z and preserves unknown A/HX");
        }
      }
    }
  }
}

class NoDebugMemory final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(ExecutionContext /*context*/) const override {
    throw Fault("Unexpected BDC security query");
  }

  std::uint8_t read(std::uint16_t /*address*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected BDC read");
  }

  void write(ByteWrite /*access*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected BDC write");
  }
};

void retirement_periods() {
  InputMemory memory;
  constexpr std::array<std::uint8_t, 6> code{0xb6, source, 0xa5, 0x38, 0x27, 2};
  memory.install(entry, code);
  memory.sample = {.value = 8, .known = 8};
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(initial(0, 0, 0));
  NoDebugMemory debug;
  Hcs08Bdc bdc(cpu, debug, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::normal);
  Hcs08RunClock clock(memory, bdc, Hcs08RunContract::reference_instruction_retirement);
  for (unsigned period = 0; period < 3; ++period) { clock.advance(); }
  require(memory.reads == 1 && memory.retired == 1,
          "Direct LDA consumes three actual reference periods and one partial operand");
  const auto before = cpu.state();
  clock.advance();
  require(cpu.state() == before && memory.retired == 1,
          "BIT immediate has no native effect before its second documented period");
  clock.advance();
  require(cpu.state() == expected_state(before, strap_mask) && memory.retired == 2,
          "BIT immediate retires after exactly two periods under the existing clock contract");
  for (unsigned period = 0; period < 3; ++period) { clock.advance(); }
  require(cpu.state().pc == entry + 6 && memory.retired == 3 && memory.reads == 1 &&
              clock.state().executing_periods == 8,
          "The following native BEQ consumes its full three periods and takes no false branch");
}

void probe() {
  Memory memory;
  constexpr std::array<std::uint8_t, 2> code{0xa5, 0x38};
  memory.install(entry, code);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(initial(8, strap_mask, 0));
  const auto before = cpu.state();
  cpu.step();
  require(cpu.state() == expected_state(before, strap_mask),
          "Unknown untested bits do not prevent native BIT");
}
} // namespace

int main(const int argc, char **argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "probe") {
      probe();
    } else if (argc == 1) {
      matrix();
      guard_and_restore();
      retirement_periods();
    } else {
      throw Fault("Usage: hcs08-bit-known-test [probe]");
    }
    std::cout << "Native BIT preserves partial registers and consumes only tested bit knowledge.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
