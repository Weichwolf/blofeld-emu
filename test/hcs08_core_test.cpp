// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

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
  const std::vector<Access> writes{{.address = stack, .value = jsr_pc & byte_mask, .write = true},
                                   {.address = stack - 1, .value = jsr_pc >> 8U, .write = true}};
  if (bus.accesses != writes) {
    std::ostringstream actual;
    for (const auto &access : bus.accesses) {
      actual << (access.write ? 'w' : 'r') << ' ' << hex(access.address) << '=' << hex(access.value)
             << ';';
    }
    require(false, "JSR must commit native PCL before PCH; actual: " + actual.str());
  }
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

void signed_branch_conditions() {
  Memory bus;
  Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
  constexpr std::array<unsigned, 2> opcodes{0x90, 0x91};
  constexpr std::array<int, 5> offsets{-128, -1, 0, 1, 127};
  constexpr unsigned greater_equal = 0x90;
  constexpr unsigned low_flags = 0x1f;
  for (const auto opcode : opcodes) {
    for (unsigned bits = 0; bits < 64; ++bits) {
      const auto flags =
          static_cast<std::uint8_t>(flags_fixed | (bits & low_flags) | ((bits & 32U) << 2U));
      const bool unequal = ((flags & flag_n) != 0) != ((flags & flag_v) != 0);
      const bool taken = opcode == greater_equal ? !unequal : unequal;
      for (const auto offset : offsets) {
        const std::array<std::uint8_t, 2> code{static_cast<std::uint8_t>(opcode),
                                               static_cast<std::uint8_t>(offset)};
        bus.install(entry, code);
        bus.accesses.clear();
        cpu.set_state({.pc = entry,
                       .hx = high_h + immediate_x,
                       .sp = stack,
                       .a = replacement_value,
                       .ccr = flags});
        auto expected = cpu.state();
        expected.pc =
            static_cast<std::uint16_t>(static_cast<int>(entry) + 2 + (taken ? offset : 0));
        cpu.step();
        require(cpu.state() == expected && bus.accesses.empty(),
                "Independent BGE N==V and BLT N!=V preserve all CCR/register bits across signed "
                "offsets");
      }
    }
  }
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

struct CallCase {
  std::array<std::uint8_t, 3> code;
  unsigned bytes;
  std::uint16_t hx, target;
};

void ordered_calls() {
  constexpr std::array<CallCase, 6> cases{{
      {.code = {0xbd, 0x80, 0}, .bytes = 2, .hx = 0x1357, .target = 0x0080},
      {.code = {0xcd, 0x91, 0xa5}, .bytes = 3, .hx = 0x1357, .target = 0x91a5},
      {.code = {0xdd, 0x01, 0x23}, .bytes = 3, .hx = 0xff00, .target = 0x0023},
      {.code = {0xed, 0x01, 0}, .bytes = 2, .hx = 0xffff, .target = 0x0000},
      {.code = {0xfd, 0, 0}, .bytes = 1, .hx = 0x9200, .target = 0x9200},
      {.code = {0xad, 0x80, 0}, .bytes = 2, .hx = 0x1357, .target = 0x7f82},
  }};
  constexpr std::array<unsigned, 4> stacks{0, 1, stack, word_mask};
  for (const auto &input : cases) {
    for (const auto sp : stacks) {
      Memory bus;
      bus.install(entry, std::span(input.code).first(input.bytes));
      const auto high = static_cast<std::uint16_t>(sp + word_mask);
      bus.writable.set(sp).set(high);
      Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
      for (unsigned flags = 0; flags < byte_values; ++flags) {
        bus.accesses.clear();
        const auto retired = bus.retired;
        constexpr unsigned accumulator = 0x5a;
        const auto expected_flags = static_cast<std::uint8_t>(flags | flags_fixed);
        cpu.set_state({.pc = entry,
                       .hx = input.hx,
                       .sp = static_cast<std::uint16_t>(sp),
                       .a = accumulator,
                       .ccr = static_cast<std::uint8_t>(flags)});
        cpu.step();
        const unsigned returned = entry + input.bytes;
        const std::vector<Access> expected{
            {.address = sp, .value = returned & byte_mask, .write = true},
            {.address = high, .value = returned >> 8U, .write = true}};
        require(bus.accesses == expected && bus.retired == retired + 1 &&
                    cpu.state() ==
                        Hcs08State{.pc = input.target,
                                   .hx = input.hx,
                                   .sp = static_cast<std::uint16_t>(sp + space_bytes - 2U),
                                   .a = accumulator,
                                   .ccr = expected_flags},
                "Six native call forms push literal return PCL/PCH in descending order, including "
                "wrap");
      }
    }
  }
}

void independent_returns() {
  constexpr std::array<unsigned, 4> stacks{0, stack, word_mask - 1, word_mask};
  constexpr unsigned restored_pc = 0x91a5;
  constexpr unsigned saved_hx = 0x1234;
  constexpr unsigned saved_a = 0x56;
  constexpr std::array<std::uint8_t, 1> code{0x81};
  for (const auto sp : stacks) {
    Memory bus;
    bus.install(entry, code);
    const auto high = static_cast<std::uint16_t>(sp + 1U);
    const auto low = static_cast<std::uint16_t>(sp + 2U);
    bus.bytes[high] = restored_pc >> 8U;
    bus.bytes[low] = restored_pc & byte_mask;
    bus.known.set(high).set(low);
    Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
    for (unsigned flags = 0; flags < byte_values; ++flags) {
      bus.accesses.clear();
      const auto retired = bus.retired;
      cpu.set_state({.pc = entry,
                     .hx = saved_hx,
                     .sp = static_cast<std::uint16_t>(sp),
                     .a = saved_a,
                     .ccr = static_cast<std::uint8_t>(flags)});
      cpu.step();
      const std::vector<Access> expected{
          {.address = high, .value = restored_pc >> 8U, .write = false},
          {.address = low, .value = restored_pc & byte_mask, .write = false}};
      require(
          bus.accesses == expected && bus.retired == retired + 1 &&
              cpu.state() == Hcs08State{.pc = restored_pc,
                                        .hx = saved_hx,
                                        .sp = static_cast<std::uint16_t>(sp + 2U),
                                        .a = saved_a,
                                        .ccr = static_cast<std::uint8_t>(flags | flags_fixed)},
          "RTS consumes independent high/low stack data, including wrap, and preserves every flag");
    }
  }
}

class FailingStackMemory : public Memory {
public:
  std::uint16_t failure = 0;
  bool write_failure = true, after = false;

  std::uint8_t read(const std::uint16_t address) override {
    if (!write_failure && address == failure && !after) { fail(address, "injected stack read"); }
    const auto value = Memory::read(address);
    if (!write_failure && address == failure && after) { fail(address, "completed stack read"); }
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (write_failure && address == failure && !after) { fail(address, "injected stack write"); }
    Memory::write(address, value);
    if (write_failure && address == failure && after) { fail(address, "completed stack write"); }
  }
};

void one_stack_failure(const bool pushing, const unsigned fault) {
  constexpr std::array<unsigned, 4> prefixes{0, 1, 1, 2};
  constexpr unsigned returned = 0x8003;
  constexpr unsigned target = 0x91a5;
  constexpr std::array<std::uint8_t, 3> call{0xcd, 0x91, 0xa5};
  constexpr std::array<std::uint8_t, 1> pop{0x81};
  FailingStackMemory bus;
  if (pushing) {
    bus.install(entry, call);
  } else {
    bus.install(entry, pop);
  }
  const auto first = pushing ? stack : word_mask;
  const auto second = pushing ? stack - 1U : 0U;
  bus.bytes[first] = static_cast<std::uint8_t>(pushing ? 0U : target >> 8U);
  bus.bytes[second] = static_cast<std::uint8_t>(pushing ? 0U : target & byte_mask);
  bus.known.set(first).set(second);
  bus.writable.set(first).set(second);
  bus.failure = static_cast<std::uint16_t>(fault < 2 ? first : second);
  bus.write_failure = pushing;
  bus.after = (fault & 1U) != 0;
  Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = entry,
                 .sp = static_cast<std::uint16_t>(pushing ? stack : word_mask - 1U),
                 .ccr = flags_fixed + flag_c});
  const auto error = rejected([&] { cpu.step(); });
  std::vector<Access> expected{
      {.address = first, .value = pushing ? returned & byte_mask : target >> 8U, .write = pushing},
      {.address = second,
       .value = pushing ? returned >> 8U : target & byte_mask,
       .write = pushing}};
  expected.resize(prefixes[fault]);
  require(cpu.faulted() && cpu.state().pc == entry && bus.retired == 0 &&
              bus.accesses == expected && error.contains("address=" + hex(bus.failure)) &&
              error.contains("pc=0x8000"),
          "Before/after byte failures retain only the actual native call/return prefix");
  const auto stopped = cpu.state();
  require(rejected([&] { cpu.step(); }).contains("coordinated recovery") &&
              cpu.state() == stopped && bus.accesses == expected,
          "A failed native call/return cannot replay an observed byte or manufacture completion");
}

void call_and_return_failures() {
  for (const bool pushing : {false, true}) {
    for (unsigned fault = 0; fault < 4; ++fault) { one_stack_failure(pushing, fault); }
  }
}

void captured_call_and_noncall_word() {
  constexpr unsigned target = 0x91a5;
  constexpr std::array<std::uint8_t, 3> call{0xcd, 0x91, 0xa5};
  for (const unsigned sp : {entry + 1, entry + 2}) {
    Memory bus;
    bus.install(entry, call);
    bus.writable.set(sp).set(sp - 1U);
    Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
    cpu.set_state({.pc = entry, .sp = static_cast<std::uint16_t>(sp)});
    cpu.step();
    require(cpu.state().pc == target && bus.retired == 1 && bus.accesses.size() == 2 &&
                bus.accesses[0].address == sp && bus.accesses[1].address == sp - 1U,
            "Return pushes cannot replace the actual native call target already decoded from code");
  }
  Memory bus;
  bus.install(entry, call);
  constexpr std::array<std::uint8_t, 2> store{0x35, 0x80};
  bus.install(target, store);
  constexpr unsigned value = 0x9234;
  constexpr unsigned destination = 0x80;
  bus.writable.set(stack).set(stack - 1).set(destination).set(destination + 1);
  Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = entry, .hx = value, .sp = stack});
  cpu.step();
  bus.accesses.clear();
  cpu.step();
  const std::vector<Access> expected{
      {.address = destination, .value = value >> 8U, .write = true},
      {.address = destination + 1, .value = value & byte_mask, .write = true}};
  require(bus.accesses == expected && cpu.state().pc == target + store.size() && bus.retired == 2,
          "A following STHX retains normal high-byte-first data stores after an actual call");
}
} // namespace

int main() {
  try {
    stack_and_background();
    calls_and_stores();
    failures_and_invalidation();
    unsupported_effects();
    fixed_condition_bits();
    signed_branch_conditions();
    ordered_calls();
    independent_returns();
    call_and_return_failures();
    captured_call_and_noncall_word();
    std::cout << "Native HCS08 core:6144 call vectors,1024 independent return vectors,8 failure "
                 "prefixes,2 code aliases and1 noncall word control passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
