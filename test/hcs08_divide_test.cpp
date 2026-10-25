// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned divide = 0x52, stack = 0x240, all_flags = 0xff, partial_flags = 0xfd;
constexpr unsigned high_byte_mask = 0xff00;
constexpr unsigned cleared_flags = 0x7b;
constexpr unsigned result_a = 0x5a, recovered_flags = 0x79, maximum_quotient = 255;
constexpr std::array<unsigned, 8> seeds{0x60, 0x61, 0x62, 0x64, 0x68, 0x70, 0xe0, 0xff};
constexpr std::array<std::uint8_t, 8> recovery{divide, 0x25, 1, 0x82, 0xa6, result_a, 0x8c, 0x82};

Hcs08State input(const unsigned dividend, const unsigned divisor, const unsigned flags) {
  return {.pc = entry,
          .hx = static_cast<std::uint16_t>((dividend & high_byte_mask) | divisor),
          .sp = stack,
          .a = static_cast<std::uint8_t>(dividend & byte_mask),
          .ccr = static_cast<std::uint8_t>(flags)};
}

Hcs08State expected(const unsigned dividend, const unsigned divisor, const unsigned flags) {
  auto state = input(dividend, divisor, flags);
  ++state.pc;
  // Independent wide arithmetic, not the implementation's H<X domain predicate.
  const bool defined = divisor != 0 && dividend / divisor <= maximum_quotient;
  state.ccr = static_cast<std::uint8_t>(flags & ~(flag_z | flag_c));
  if (defined) {
    state.a = static_cast<std::uint8_t>(dividend / divisor);
    state.hx = static_cast<std::uint16_t>(((dividend % divisor) << 8U) | divisor);
    if (state.a == 0) { state.ccr |= flag_z; }
  } else {
    state.a = 0; // State views mask unknown bits; this is not a hardware zero result.
    state.a_known = 0;
    state.hx = static_cast<std::uint16_t>(divisor);
    state.hx_known = byte_mask;
    state.ccr_known = partial_flags;
    state.ccr |= flag_c;
  }
  return state;
}

void check(Memory &memory,
           Hcs08 &cpu,
           const unsigned dividend,
           const unsigned divisor,
           const unsigned flags) {
  const auto before = memory.retired;
  cpu.set_state(input(dividend, divisor, flags));
  cpu.step();
  require(cpu.state() == expected(dividend, divisor, flags) && !cpu.faulted() &&
              memory.retired == before + 1 && memory.accesses.empty(),
          "Independent DIV quotient/remainder/X/flags/knownness dividend=" + hex(dividend) +
              " divisor=" + hex(divisor) + " seed=" + hex(flags));
}

void matrix(const std::string &specification) {
  Memory memory;
  constexpr std::array<std::uint8_t, 1> code{divide};
  memory.install(entry, code);
  Hcs08 cpu(memory, specification);
  check(memory,
        cpu,
        word_mask,
        byte_mask,
        all_flags); // Prior implementation invents known overflow data.
  constexpr std::array<unsigned, 5> quotients{0, 1, 127, 255, 256};
  unsigned count = 1;
  for (unsigned divisor = 1; divisor < byte_values; ++divisor) {
    for (const auto flags : seeds) {
      for (const auto quotient : quotients) {
        check(memory, cpu, divisor * quotient, divisor, flags);
        ++count;
        if (divisor != 1) {
          check(memory, cpu, (divisor * quotient) + divisor - 1, divisor, flags);
          ++count;
        }
      }
      check(memory, cpu, word_mask, divisor, flags);
      ++count;
    }
  }
  for (const unsigned dividend : {0U, 1U, 255U, 256U, 65280U, 65535U}) {
    for (const auto flags : seeds) {
      check(memory, cpu, dividend, 0, flags);
      ++count;
    }
  }
  constexpr unsigned cases = 22449;
  require(count == cases, "Finite independent DIV boundary matrix retains its exact extent");
}

void partial_ccr(const std::string &specification) {
  Memory memory;
  constexpr std::array<std::uint8_t, 1> code{divide};
  memory.install(entry, code);
  Hcs08 cpu(memory, specification);
  for (const unsigned mask : {0x60U, 0xefU, 0xfdU}) {
    for (const unsigned divisor : {0U, 2U, 255U}) {
      const unsigned dividend = divisor == 2 ? 1U : word_mask;
      auto state = input(dividend, divisor, all_flags & mask);
      state.ccr_known = static_cast<std::uint8_t>(mask);
      cpu.set_state(state);
      cpu.step();
      auto wanted = expected(dividend, divisor, all_flags & mask);
      const bool defined = divisor != 0 && dividend / divisor <= maximum_quotient;
      wanted.ccr_known =
          static_cast<std::uint8_t>(defined ? mask | flag_c | flag_z : (mask | flag_c) & ~flag_z);
      wanted.ccr &= wanted.ccr_known;
      require(cpu.state() == wanted && !cpu.faulted(),
              "DIV defines only its documented C/Z and retains unrelated unknown CCR bits");
    }
  }
}

void unknown_operands(const std::string &specification) {
  constexpr unsigned operand_bits = 24;
  for (unsigned bit = 0; bit < operand_bits; ++bit) {
    Memory memory;
    constexpr std::array<std::uint8_t, 1> code{divide};
    memory.install(entry, code);
    Hcs08 cpu(memory, specification);
    auto state = input(word_mask, byte_mask, all_flags);
    if (bit < 8) {
      state.a_known = static_cast<std::uint8_t>(byte_mask ^ (1U << bit));
    } else {
      state.hx_known = static_cast<std::uint16_t>(word_mask ^ (1U << (bit - 8)));
    }
    cpu.set_state(state);
    require(rejected([&] { cpu.step(); }).contains("unknown operand") && cpu.faulted() &&
                memory.retired == 0 && memory.accesses.empty(),
            "Every missing H/A/X operand bit refuses before native division or retirement");
  }
}

void consumers(const std::string &specification) {
  constexpr unsigned push_h = 0x8b;
  constexpr unsigned tpa = 0x85;
  constexpr unsigned sta = 0xc7;
  for (const auto opcode : {push_h, tpa, sta}) {
    Memory memory;
    const std::array<std::uint8_t, 4> code{divide, static_cast<std::uint8_t>(opcode), 0, operand};
    memory.install(entry, code);
    memory.bytes[operand] = result_a;
    memory.writable.set(operand);
    Hcs08 cpu(memory, specification);
    cpu.set_state(input(word_mask, byte_mask, all_flags));
    cpu.step();
    const auto state = cpu.state();
    const auto reason = rejected([&] { cpu.step(); });
    require(reason.contains("unknown") && reason.contains("pc=" + hex(entry + 1)) &&
                cpu.state() == state && cpu.faulted() && memory.retired == 1 &&
                memory.bytes[operand] == result_a && memory.accesses.empty(),
            "Undefined A/H/Z cannot leak through RAM stores, stack or CCR-to-A conversion");
  }
}

void recover(Hcs08 &cpu, const unsigned divisor) {
  cpu.step(); // Known C takes BCS over the first BGND; unknown Z/A/H are not consumed.
  require(cpu.state() == Hcs08State{.pc = entry + 4,
                                    .hx = static_cast<std::uint16_t>(divisor),
                                    .sp = stack,
                                    .a = 0,
                                    .ccr_known = partial_flags,
                                    .ccr = partial_flags,
                                    .a_known = 0,
                                    .hx_known = byte_mask},
          "Known error carry permits native BCS with indeterminate quotient/remainder/Z");
  cpu.step();
  require(cpu.state() == Hcs08State{.pc = entry + 6,
                                    .hx = static_cast<std::uint16_t>(divisor),
                                    .sp = stack,
                                    .a = result_a,
                                    .ccr = recovered_flags,
                                    .hx_known = byte_mask},
          "Guest LDA independently restores A and Z knowledge while H remains unknown");
  cpu.step();
  require(cpu.state() == Hcs08State{.pc = entry + 7,
                                    .hx = static_cast<std::uint16_t>(divisor),
                                    .sp = stack,
                                    .a = result_a,
                                    .ccr = cleared_flags},
          "Guest CLRH restores only H without inventing the discarded remainder");
  cpu.step();
  require(cpu.state().pc == entry + 8 && cpu.background_requested() && !cpu.faulted(),
          "Actual guest recovery reaches BGND with the original known carry retained");
}

void replay(const std::string &specification) {
  for (const unsigned divisor : {0U, 255U}) {
    Memory memory;
    memory.install(entry, recovery);
    Hcs08 cpu(memory, specification);
    cpu.set_state(input(word_mask, divisor, all_flags));
    cpu.step();
    const auto checkpoint = cpu.checkpoint();
    const auto saved = memory;
    recover(cpu, divisor);
    for (unsigned n = 0; n < 2; ++n) {
      Memory copy = saved;
      Hcs08 fresh(copy, specification);
      fresh.restore_checkpoint(checkpoint);
      recover(fresh, divisor);
      require(
          fresh.state() == cpu.state() && copy.retired == memory.retired,
          "Fresh snapshots preserve undefined results and independently checked guest recovery");
    }
  }
}

class NoBdcMemory final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(ExecutionContext /*context*/) const override { return false; }

  std::uint8_t read(std::uint16_t /*address*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected DIV test BDC read");
  }

  void write(ByteWrite /*access*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected DIV test BDC write");
  }
};

void go(Hcs08Bdc &bdc) {
  bdc.receive(hcs08_bdc::go);
  bdc.service(Hcs08BdcGrant::reference_instruction_boundary);
}

void clock_suffix(
    Hcs08 &cpu, Hcs08Bdc &bdc, Hcs08RunClock &clock, const unsigned divisor, const unsigned first) {
  constexpr std::array<unsigned, 5> boundaries{6, 9, 11, 12, 17};
  for (unsigned period = first; period <= boundaries.back(); ++period) {
    const auto before = cpu.state();
    clock.advance();
    unsigned retired = 0;
    for (const auto end : boundaries) { retired += static_cast<unsigned>(period >= end); }
    require(clock.state().retired_instructions == retired,
            "Independent6/3/2/1/5-period DIV/BCS/LDA/CLRH/BGND budgets");
    if (retired == 0) {
      require(cpu.state() == input(word_mask, divisor, all_flags), "DIV cannot retire early");
    }
    if (period == 6) {
      require(cpu.state() == expected(word_mask, divisor, all_flags),
              "DIV error effects retire at six");
    }
    if (period < 6 || (period > 6 && period < 9) || period == 10 ||
        (period > 12 && period < boundaries.back())) {
      require(cpu.state() == before,
              "Incomplete native budgets cannot leak register/knowledge effects");
    }
  }
  require(cpu.state() == Hcs08State{.pc = entry + 8,
                                    .hx = static_cast<std::uint16_t>(divisor),
                                    .sp = stack,
                                    .a = result_a,
                                    .ccr = cleared_flags} &&
              bdc.state().active && clock.state().offered_periods == boundaries.back() &&
              !clock.faulted(),
          "Clocked guest recovery has its independently specified endpoint and actual held BDC");
}

void clocked(const std::string &specification) {
  for (const unsigned divisor : {0U, 255U}) {
    Memory memory;
    memory.install(entry, recovery);
    Hcs08 cpu(memory, specification);
    cpu.set_state(input(word_mask, divisor, all_flags));
    NoBdcMemory mapping;
    Hcs08Bdc bdc(cpu, mapping, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::bkgd_held_low);
    Hcs08RunClock clock(memory, bdc, Hcs08RunContract::reference_instruction_retirement);
    go(bdc);
    clock.advance();
    clock.advance();
    require(clock.state().remaining_periods == 4 &&
                cpu.state() == input(word_mask, divisor, all_flags),
            "Joint snapshot is inside an actual six-period DIV before any undefined result");
    const auto saved_memory = memory;
    const auto saved_cpu = cpu.checkpoint();
    const auto saved_bdc = bdc.checkpoint();
    const auto saved_clock = clock.checkpoint();
    clock_suffix(cpu, bdc, clock, divisor, 3);
    for (unsigned n = 0; n < 2; ++n) {
      Memory copy = saved_memory;
      Hcs08 fresh(copy, specification);
      fresh.restore_checkpoint(saved_cpu);
      NoBdcMemory other_mapping;
      Hcs08Bdc other(
          fresh, other_mapping, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::normal);
      other.restore_checkpoint(saved_bdc);
      Hcs08RunClock fresh_clock(copy, other, Hcs08RunContract::reference_instruction_retirement);
      fresh_clock.restore_checkpoint(saved_clock);
      clock_suffix(fresh, other, fresh_clock, divisor, 3);
      require(fresh_clock.state() == clock.state() && copy.retired == memory.retired,
              "Fresh CPU/RAM/BDC/clock owners preserve the exact partial-DIV reservation suffix");
    }
  }
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Expected HCS08 runtime specification path");
    const std::string specification(arguments[1]);
    matrix(specification);
    partial_ccr(specification);
    unknown_operands(specification);
    consumers(specification);
    replay(specification);
    clocked(specification);
    std::cout << "HCS08 DIV:22449 independent boundaries,24 missing operand bits,undefined "
                 "results,guest recovery and joint clock replay verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
