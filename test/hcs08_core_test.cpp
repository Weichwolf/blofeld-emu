// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned stack = 0x47c, high_h = 0x1200, immediate_x = 0x56;
constexpr unsigned load_pc = 0x8002, jsr_pc = 0x8003, subroutine = 0x8200;
constexpr unsigned replacement_value = 0x5a, spi_vector = 0xffe0;

void stack_and_background() {
  Memory bus;
  Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
  require(cpu.faulted() &&
              rejected([&] { static_cast<void>(cpu.state()); }).contains("entry registers"),
          "Unspecified entry registers cannot silently become invented reset values");
  // Own LDX #56; TXS preserves H and replaces the old SP with H:56 - 1.
  constexpr std::array<std::uint8_t, 4> code{0xae, immediate_x, 0x94, 0x82};
  bus.install(entry, code);
  cpu.set_state({.pc = entry, .hx = high_h + 1, .sp = stack, .ccr = 0});
  cpu.step();
  require(
      cpu.state() ==
          Hcs08State{.pc = load_pc, .hx = high_h + immediate_x, .sp = stack, .ccr = flags_fixed},
      "Own LDX preserves H; NZV describe the loaded byte");
  cpu.step();
  require(cpu.state().sp == high_h + immediate_x - 1 && cpu.state().ccr == flags_fixed,
          "TXS copies full H:X minus one, not a repaired low-RAM stack");
  cpu.step();
  require(cpu.background_requested() && !cpu.faulted() && cpu.state().pc == entry + code.size(),
          "BGND surfaces an actual CPU request at its following PC");
  const auto before = cpu.state();
  require(rejected([&] { cpu.step(); }).contains("owning BDC decision") && cpu.state() == before,
          "A BGND request cannot silently continue or fabricate active BDC status");
}

void calls_and_stores() {
  Memory bus;
  Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
  constexpr std::array<std::uint8_t, 4> caller{0xcd, 0x82, 0x00, 0x9d};
  constexpr std::array<std::uint8_t, 1> body{0x81};
  bus.install(entry, caller);
  bus.install(subroutine, body);
  bus.writable.set(stack).set(stack - 1);
  cpu.set_state({.pc = entry, .hx = high_h, .sp = stack, .ccr = flags_fixed + flag_c});
  cpu.step();
  require(cpu.state().pc == subroutine && cpu.state().sp == stack - 2 &&
              bus.bytes[stack] == (jsr_pc & byte_mask) && bus.bytes[stack - 1] == (jsr_pc >> 8U),
          "JSR pushes original return low then high using the HCS08 descending stack");
  cpu.step();
  require(cpu.state().pc == jsr_pc && cpu.state().sp == stack && cpu.state().hx == high_h &&
              cpu.state().ccr == flags_fixed + flag_c,
          "RTS independently pulls high then low without altering flags/H:X");
  cpu.step();
  require(cpu.state().pc == jsr_pc + 1 && bus.retired == 3,
          "Native NOP does not eagerly execute unknown following memory");
}

void failures_and_invalidation() {
  Memory bus;
  Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
  constexpr std::array<std::uint8_t, 3> code{0xc6, 0x18, 0x20}; // Own LDA $1820.
  bus.install(entry, code);
  cpu.set_state({.pc = entry, .hx = high_h, .sp = stack});
  require(cpu.disassemble().starts_with("LDA"), "Native target instruction disassembly");
  const auto error = rejected([&] { cpu.step(); });
  require(error.contains("read address=0x1820") && error.contains("pc=0x8000") && cpu.faulted(),
          "Unmapped flash controller accesses fail with operation/address/PC/context");
  const auto failed = cpu.state();
  require(rejected([&] { cpu.step(); }).contains("coordinated recovery") && cpu.state() == failed &&
              bus.retired == 0,
          "An unretired partial target instruction cannot resume");
  constexpr std::array<std::uint8_t, 2> replacement{0xa6, 0x5a};
  bus.install(entry, replacement);
  cpu.set_state({.pc = entry});
  cpu.step();
  require(cpu.state().a == replacement_value && !cpu.faulted() && cpu.state().pc == entry + 2,
          "Explicit recovery invalidates stale native translations after RAM-code replacement");
  bus.known.reset(entry + 1);
  ++bus.code_generation;
  cpu.set_state({.pc = entry});
  require(rejected([&] { cpu.step(); }).contains("fetch address=0x8001") && cpu.faulted(),
          "Unknown actual operand fetch fails although decoder lookahead is side-effect free");
}

void unsupported_effects() {
  Memory bus;
  Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
  constexpr std::array<std::uint8_t, 1> wait{0x8f};
  bus.install(entry, wait);
  cpu.set_state({.pc = entry});
  require(rejected([&] { cpu.step(); }).contains("Unsupported HCS08 intrinsic: wait") &&
              cpu.faulted() && bus.retired == 0,
          "Unmodeled clock/WAIT behavior cannot become silent success");
  cpu.set_state({.pc = entry});
  bus.interrupt = InterruptRequest{.vector = spi_vector};
  require(rejected([&] { cpu.step(); }).contains("interrupt entry is not qualified") &&
              cpu.faulted() && bus.retired == 0,
          "An offered unsupported target interrupt cannot be silently dropped");
}

void fixed_condition_bits() {
  Memory bus;
  Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
  constexpr std::array<std::uint8_t, 2> code{0x84, 0x85}; // Own TAP; TPA.
  bus.install(entry, code);
  for (unsigned value = 0; value < byte_values; ++value) {
    cpu.set_state({.pc = entry, .a = static_cast<std::uint8_t>(value)});
    cpu.step();
    require(cpu.state().ccr == (value | flags_fixed),
            "S08CPUV2 fixes CCR bits6/5 even when TAP attempts to clear them");
    cpu.step();
    require(cpu.state().a == (value | flags_fixed) && cpu.state().pc == entry + code.size(),
            "Native TPA reads actual fixed bits along with all six writable flags");
  }
}
} // namespace

int main() {
  try {
    stack_and_background();
    calls_and_stores();
    failures_and_invalidation();
    unsupported_effects();
    fixed_condition_bits();
    std::cout << "Native HCS08 stack, background request and strict failure cases passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
