// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned vector = 0xfffc, normal_handler = 0x9001, overwritten_handler = 0xf191;
constexpr unsigned start_stack = 0, setup = 5, raw = 0x91, saved_flags = 0xf1;
constexpr unsigned caller_pc = 0x800a, return_pc = 0x800b, caller_hx = 0x1234;

void install(Memory &memory) {
  // Flat CPU test memory permits all16-bit stack addresses;not a GT8A board claim.
  memory.writable.set();
  constexpr std::array<std::uint8_t, 14> code{
      0x45, 0, 1, 0x94, 0x45, 0x12, 0x34, 0xa6, 0x91, 0x84, 0x83, 0x9d, 0x9d, 0x9d};
  constexpr std::array<std::uint8_t, 2> target{0x90, 1};
  constexpr std::array<std::uint8_t, 2> reset{0x80, 0};
  constexpr std::array<std::uint8_t, 3> handler{0x80, 0x9d, 0x9d};
  memory.install(entry, code);
  memory.install(vector, target);
  memory.install(vector + 2, reset);
  memory.install(normal_handler, handler);
  memory.install(overwritten_handler, handler);
}

void wrapped_flat_vector(Memory &memory) {
  install(memory);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA, Hcs08RtiIrqContract::reference_immediate);
  cpu.reset(Hcs08ResetEntry::normal, Hcs08ResetCause::power_on);
  for (unsigned i = 0; i < setup; ++i) { cpu.step(); }
  Hcs08State expected{
      .pc = caller_pc, .hx = caller_hx, .sp = start_stack, .a = raw, .ccr = saved_flags};
  require(cpu.state() == expected, "Actual TXS reaches zero SP without a state setter");
  memory.accesses.clear();
  cpu.step();
  constexpr std::array<unsigned, 5> addresses{0, 0xffff, 0xfffe, 0xfffd, 0xfffc};
  constexpr unsigned return_high = 0x80;
  constexpr unsigned saved_x = 0x34;
  const std::vector<Access> actual{{.address = addresses[0], .value = 0x0b, .write = true},
                                   {.address = addresses[1], .value = return_high, .write = true},
                                   {.address = addresses[2], .value = saved_x, .write = true},
                                   {.address = addresses[3], .value = raw, .write = true},
                                   {.address = addresses[4], .value = saved_flags, .write = true},
                                   {.address = addresses[4], .value = saved_flags, .write = false},
                                   {.address = addresses[3], .value = raw, .write = false}};
  constexpr unsigned wrapped_sp = 0xfffb;
  expected.pc = overwritten_handler;
  expected.sp = wrapped_sp;
  expected.ccr |= 8U;
  require(cpu.state() == expected && memory.accesses == actual,
          "Real wrapping stack overwrites vector before both reads;branch uses F191,not old9001");
  cpu.step();
  expected.pc = return_pc;
  expected.sp = start_stack;
  expected.ccr = saved_flags;
  require(cpu.state() == expected,
          "Native RTI reads actual wrapping PCH/PCL across FFFF/0000 and restores caller");
}

void missing_mask_capability() {
  Memory memory;
  install(memory);
  constexpr std::array<std::uint8_t, 3> code{0x83, 0x9d, 0x9d};
  memory.install(entry, code);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.reset(Hcs08ResetEntry::normal, Hcs08ResetCause::power_on);
  const auto before = cpu.state();
  const auto generation = memory.generation;
  memory.accesses.clear();
  const auto error = rejected([&] { cpu.step(); });
  require(error.contains("SWI admission") && error.contains("address=0xfffc") &&
              error.contains("pc=" + hex(entry)) && error.contains("agent=CPU") &&
              cpu.state() == before && memory.generation == generation && memory.accesses.empty(),
          "Unknown cold frame cannot silently become known without explicit masked RAM support");
}
} // namespace

int main() {
  try {
    MaskedMemory masked;
    wrapped_flat_vector(masked);
    Memory plain;
    wrapped_flat_vector(plain);
    missing_mask_capability();
    std::cout
        << "Verified actual wrapping SWI/vector aliasing,wrapped RTI and missing-mask refusal.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
