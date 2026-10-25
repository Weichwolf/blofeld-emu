// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_fixture.hpp"
#include "hcs08_prefetch_move_test.hpp"
#include "hcs08_prefetch_word_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::word;
namespace hw = blofeld::test::hcs08;
constexpr unsigned sign = 0x8000;
constexpr unsigned low_handler = 0x80;
constexpr unsigned stack_offset = 0x7f;

Hcs08State expected(const Hcs08State before, const Input input, const unsigned phase) {
  auto result = before;
  if (phase > first_write(input)) {
    result.pc = static_cast<std::uint16_t>(input.pc + size(input));
    result.ccr =
        static_cast<std::uint8_t>((before.ccr & ~changed) | (input.value == 0 ? hw::flag_z : 0U) |
                                  ((input.value & sign) != 0 ? hw::flag_n : 0U));
    result.ccr_known |= changed;
  }
  return result;
}

void check_case(Machine &machine, const Input input) {
  machine.prepare(input);
  const auto before = machine.cpu.state();
  const auto events = machine.memory.events.size();
  unsigned next_program = input.pc + 3;
  unsigned writes = 0;
  for (unsigned phase = 1; phase <= periods(input); ++phase) {
    machine.tick();
    const auto queue = owned(machine.cpu);
    const bool write = phase == first_write(input) || phase == first_write(input) + 1;
    Access access{
        .address = next_program, .value = machine.memory.bytes[next_program], .write = false};
    if (write) {
      constexpr unsigned byte_bits = 8;
      access = {.address = static_cast<std::uint16_t>(target(input) + writes),
                .value = (input.value >> ((1U - writes) * byte_bits)) & hw::byte_mask,
                .write = true};
      ++writes;
    } else {
      ++next_program;
    }
    hw::require(machine.memory.events.size() == events + phase &&
                    machine.memory.events.back() == Event{.period = phase, .access = access},
                "Independent ISA wwpp/pwwpp address, byte order and bus-period oracle");
    hw::require(machine.cpu.state() == expected(before, input, phase) &&
                    queue.native_executed == (phase > first_write(input)) &&
                    queue.completed_periods == phase && queue.attempted_periods == phase,
                "Native PC/flags retain the genuine incomplete prefix until both bytes commit");
    hw::require(machine.memory.write_widths.size() == writes &&
                    machine.memory.native_starts ==
                        static_cast<unsigned>(phase >= first_write(input)) &&
                    machine.memory.retired == static_cast<unsigned>(phase == periods(input)),
                "Native start occurs once, each byte is separate, retirement follows the final p");
  }
  const auto queue = owned(machine.cpu);
  hw::require(queue.retired_instructions == 1 && queue.queued_bytes == 3 &&
                  queue.ready_bytes == 2 && queue.program_byte_in_flight &&
                  queue.head == input.pc + size(input),
              "The completed word store leaves the genuine successor queue intact");
}

void same(const Machine &left, const Machine &right) {
  hw::require(
      left.cpu.state() == right.cpu.state() &&
          left.cpu.prefetch_state() == right.cpu.prefetch_state() &&
          left.memory.bytes == right.memory.bytes && left.memory.known == right.memory.known &&
          left.memory.partial_known == right.memory.partial_known &&
          left.memory.events == right.memory.events &&
          left.memory.accesses == right.memory.accesses &&
          left.memory.write_widths == right.memory.write_widths &&
          left.memory.native_starts == right.memory.native_starts &&
          left.memory.retired == right.memory.retired,
      "Fresh native restoration repeats neither the committed write prefix nor earlier hooks");
}

void replay(Machine &parent, Machine &fresh, const Input input, const bool replace_code) {
  for (unsigned cut = 0; cut <= periods(input); ++cut) {
    parent.prepare(input);
    for (unsigned phase = 0; phase < cut; ++phase) { parent.tick(); }
    if (replace_code && cut == first_write(input)) {
      constexpr std::array<std::uint8_t, 3> replacement{nop, nop, nop};
      parent.memory.install(input.pc, replacement);
    }
    const auto checkpoint = parent.cpu.checkpoint();
    fresh.memory = parent.memory;
    fresh.cpu.restore_checkpoint(checkpoint);
    same(parent, fresh);
    while (owned(parent.cpu).completed_periods < periods(input)) {
      parent.tick();
      fresh.tick();
      same(parent, fresh);
    }
  }
}

void failures(Machine &machine, Machine &fresh, const Input input) {
  for (unsigned byte = 0; byte < 2; ++byte) {
    for (const bool after : {false, true}) {
      machine.prepare(input);
      for (unsigned phase = 1; phase < first_write(input) + byte; ++phase) { machine.tick(); }
      const auto checkpoint = machine.cpu.checkpoint();
      fresh.memory = machine.memory;
      machine.memory.refused = static_cast<std::uint16_t>(target(input) + byte);
      machine.memory.after = after;
      const auto message = hw::rejected([&] { machine.tick(); });
      const unsigned committed = byte + static_cast<unsigned>(after);
      unsigned writes = 0;
      for (const auto &event : machine.memory.events) {
        if (event.period != 0 && event.access.write) { ++writes; }
      }
      hw::require(message.contains("pc=") && message.contains("address=") &&
                      machine.cpu.faulted() && writes == committed && machine.memory.retired == 0,
                  "Each before/after write refusal preserves its exact physical prefix and "
                  "quarantines CPU");
      static_cast<void>(hw::rejected([&] { static_cast<void>(machine.cpu.checkpoint()); }));
      static_cast<void>(hw::rejected([&] { machine.tick(); }));
      fresh.cpu.restore_checkpoint(checkpoint);
      finish(fresh, input);
      hw::require(fresh.memory.bytes[target(input)] == (input.value >> 8U) &&
                      fresh.memory.bytes[static_cast<std::uint16_t>(target(input) + 1U)] ==
                          (input.value & hw::byte_mask) &&
                      fresh.memory.write_widths.size() == 2 && fresh.memory.native_starts == 1 &&
                      fresh.memory.retired == 1,
                  "Joint healthy recovery completes exactly the two native bytes");
    }
  }
}

void strict(Machine &machine) {
  for (const auto form : forms) {
    const Input input{.form = form};
    machine.prepare(input, Hcs08PrefetchContract::reference_scanner_byte_cycles);
    const auto before = machine.memory.events.size();
    static_cast<void>(hw::rejected([&] { machine.tick(); }));
    hw::require(machine.memory.events.size() == before && machine.memory.native_starts == 0,
                "Existing byte-only contracts retain strict word-store refusal");
    for (unsigned bit = 0; bit < 16; ++bit) {
      machine.prepare({.form = form, .hx_known = hw::word_mask ^ (1U << bit)});
      for (unsigned phase = 1; phase < first_write(input); ++phase) { machine.tick(); }
      static_cast<void>(hw::rejected([&] { machine.tick(); }));
      hw::require(machine.memory.write_widths.empty(),
                  "Incomplete native H:X knowledge refuses before the first word-store byte");
    }
  }
}

void vectors(Machine &machine, Machine &fresh) {
  constexpr std::array<unsigned, 8> values{0, 1, 0xff, 0x100, 0x7fff, 0x8000, 0xff00, 0xffff};
  constexpr std::array<unsigned, 3> masks{hw::byte_mask, hw::flags_fixed, 0x69};
  for (const auto form : forms) {
    for (const auto value : values) {
      for (unsigned flags = 0; flags < hw::byte_values; flags += 3) {
        for (const auto known : masks) {
          check_case(machine, {.form = form, .value = value, .flags = flags, .known = known});
        }
      }
    }
    const Input base{.form = form};
    replay(machine, fresh, base, false);
    replay(machine, fresh, base, true);
    failures(machine, fresh, base);
  }
  check_case(machine, {.form = Form::dir, .argument = hw::byte_mask});
  check_case(machine, {.form = Form::ext, .argument = hw::word_mask});
  check_case(machine, {.form = Form::sp, .argument = 0, .initial_stack = 4});
  replay(
      machine, fresh, {.form = Form::dir, .argument = low_handler + 1, .pc = low_handler}, false);
  replay(machine, fresh, {.form = Form::ext, .argument = handler + 1}, false);
  replay(machine,
         fresh,
         {.form = Form::sp, .argument = stack_offset, .initial_stack = handler + 6 - stack_offset},
         false);
}
}

int main() {
  try {
    Machine machine;
    Machine fresh;
    vectors(machine, fresh);
    strict(machine);
    blofeld::test::run_move_tests();
    std::cout << "Native word-store bus periods, fresh snapshots, captured code and strict "
                 "failures; native MOV read/write continuation passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
