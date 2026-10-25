// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_prefetch_move_test.hpp"

#include "hcs08_fixture.hpp"
#include "hcs08_prefetch_move_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>

namespace {
using namespace blofeld;
using namespace blofeld::test::move;
namespace hw = blofeld::test::hcs08;

Hcs08State expected(const Hcs08State before, const Input input, const unsigned phase) {
  auto result = before;
  if (phase >= 3) {
    constexpr unsigned sign = 0x80;
    result.pc = static_cast<std::uint16_t>(input.pc + 3);
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
  const auto reads = machine.memory.read_widths.size();
  const auto writes = machine.memory.write_widths.size();
  unsigned next_program = input.pc + 3;
  for (unsigned phase = 1; phase <= periods; ++phase) {
    machine.tick();
    Access access{.address = input.source, .value = input.value, .write = false};
    if (phase == 3) {
      access = {.address = input.destination, .value = input.value, .write = true};
    } else if (phase != 1) {
      access = {
          .address = next_program, .value = machine.memory.bytes[next_program], .write = false};
      ++next_program;
    }
    const auto queue = owned(machine.cpu);
    hw::require(machine.memory.events.size() == events + phase &&
                    machine.memory.events.back() == Event{.period = phase, .access = access},
                "Independent MOV DIR/DIR rpwpp address/value/period expectation");
    hw::require(machine.cpu.state() == expected(before, input, phase) &&
                    queue.native_executed == (phase >= 3) && queue.completed_periods == phase &&
                    queue.attempted_periods == phase,
                "Native read retains original PC/flags until its owned w3 commits");
    hw::require(
        machine.memory.native_starts == 1 &&
            machine.memory.retired == static_cast<unsigned>(phase == periods) &&
            machine.memory.write_widths.size() == writes + static_cast<unsigned>(phase >= 3) &&
            machine.memory.read_widths.size() == reads + phase - static_cast<unsigned>(phase >= 3),
        "MOV begins once, reads once, writes once and retires after its final p");
  }
  const auto queue = owned(machine.cpu);
  hw::require(queue.retired_instructions == 1 && queue.queued_bytes == 3 &&
                  queue.ready_bytes == 2 && queue.program_byte_in_flight &&
                  queue.head == input.pc + 3 && !queue.operand.value,
              "MOV retains its actual native temporary and complete successor queue");
}

void same(const Machine &left, const Machine &right) {
  hw::require(left.cpu.state() == right.cpu.state() &&
                  left.cpu.prefetch_state() == right.cpu.prefetch_state() &&
                  left.memory.bytes == right.memory.bytes &&
                  left.memory.known == right.memory.known &&
                  left.memory.partial_known == right.memory.partial_known &&
                  left.memory.events == right.memory.events &&
                  left.memory.accesses == right.memory.accesses &&
                  left.memory.read_widths == right.memory.read_widths &&
                  left.memory.write_widths == right.memory.write_widths &&
                  left.memory.native_starts == right.memory.native_starts &&
                  left.memory.retired == right.memory.retired,
              "Fresh MOV restoration repeats neither source reads nor earlier native hooks");
}

void replay(Machine &parent, Machine &fresh, const Input input) {
  for (unsigned cut = 0; cut <= periods; ++cut) {
    parent.prepare(input);
    const auto before = parent.cpu.state();
    for (unsigned n = 0; n < cut; ++n) { parent.tick(); }
    if (cut == 1 || cut == 2) {
      constexpr unsigned changed_source = 0x7f;
      parent.memory.install(input.source, std::array<std::uint8_t, 1>{changed_source});
      parent.memory.install(input.pc, std::array<std::uint8_t, 3>{nop, nop, nop});
    }
    const auto checkpoint = parent.cpu.checkpoint();
    fresh.memory = parent.memory;
    fresh.cpu.restore_checkpoint(checkpoint);
    same(parent, fresh);
    while (owned(parent.cpu).completed_periods < periods) {
      parent.tick();
      fresh.tick();
      same(parent, fresh);
    }
    hw::require(parent.cpu.state() == expected(before, input, periods) &&
                    parent.memory.bytes[input.destination] == input.value &&
                    parent.memory.native_starts == 1 && parent.memory.retired == 1,
                "Changed source/code and aliasing cannot replace the genuine r1 latch");
  }
}

void failures(Machine &parent, Machine &fresh) {
  const Input input{};
  for (const bool write : {false, true}) {
    for (const bool after : {false, true}) {
      parent.prepare(input);
      const unsigned phase = write ? 3U : 1U;
      for (unsigned n = 1; n < phase; ++n) { parent.tick(); }
      const auto checkpoint = parent.cpu.checkpoint();
      fresh.memory = parent.memory;
      const auto events = parent.memory.events.size();
      if (write) {
        parent.memory.refused_write = input.destination;
      } else {
        parent.memory.refused_read = input.source;
      }
      parent.memory.after = after;
      const auto message = hw::rejected([&] { parent.tick(); });
      hw::require(message.contains("pc=") && message.contains("address=") && parent.cpu.faulted() &&
                      parent.memory.retired == 0 &&
                      parent.memory.events.size() == events + static_cast<unsigned>(after),
                  "Before/after read/write failure preserves the exact prefix and quarantines CPU");
      static_cast<void>(hw::rejected([&] { static_cast<void>(parent.cpu.checkpoint()); }));
      static_cast<void>(hw::rejected([&] { parent.tick(); }));
      fresh.cpu.restore_checkpoint(checkpoint);
      finish(fresh);
      hw::require(fresh.memory.bytes[input.destination] == input.value &&
                      fresh.memory.native_starts == 1 && fresh.memory.retired == 1,
                  "Joint healthy recovery preserves the genuine read-to-write continuation");
    }
  }
}

void strict(Machine &machine) {
  constexpr std::array old{Hcs08PrefetchContract::reference_scanner_byte_cycles,
                           Hcs08PrefetchContract::reference_scanner_word_cycles};
  for (const auto selected : old) {
    machine.prepare({}, selected);
    const auto events = machine.memory.events.size();
    static_cast<void>(hw::rejected([&] { machine.tick(); }));
    hw::require(machine.memory.events.size() == events && machine.memory.native_starts == 0,
                "Existing contracts retain strict MOV timing refusal");
  }
  for (const bool source : {false, true}) {
    const Input input{};
    machine.prepare(input);
    if (source) {
      machine.memory.known.reset(input.source);
      machine.memory.partial_known[input.source] = hw::byte_mask >> 1U;
    } else {
      machine.memory.writable.reset(input.destination);
      machine.tick();
      machine.tick();
    }
    const auto message = hw::rejected([&] { machine.tick(); });
    hw::require(message.contains("address=") && message.contains("pc=") && machine.cpu.faulted() &&
                    machine.memory.retired == 0,
                "Unknown source and read-only destination remain visible failures");
  }
}
}

void blofeld::test::run_move_tests() {
  Machine machine;
  Machine fresh;
  constexpr std::array<unsigned, 3> masks{hw::byte_mask, hw::flags_fixed, 0x69};
  constexpr unsigned flag_stride = 17;
  for (unsigned value = 0; value < hw::byte_values; ++value) {
    for (unsigned flags = 0; flags < hw::byte_values; flags += flag_stride) {
      for (const auto known : masks) {
        check_case(machine, {.value = value, .flags = flags, .known = known});
      }
    }
  }
  check_case(machine, {.source = hw::byte_mask, .destination = 0});
  check_case(machine, {.source = 0, .destination = hw::byte_mask});
  check_case(machine, {.source = 0x20, .destination = 0x20});
  check_case(machine, {.a_known = 0, .hx_known = 0});
  for (const unsigned destination : {0x21U, 0x20U, 0x81U, 0x80U}) {
    replay(machine, fresh, {.destination = destination, .pc = 0x80});
  }
  failures(machine, fresh);
  strict(machine);
}
