// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_fixture.hpp"
#include "indexed_extra.hpp"
#include "indexed_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::indexed;
namespace hw = blofeld::test::hcs08;
constexpr unsigned near_end = 0xff80, high_half = 0x8000, signed_boundary = 0x7fff;
constexpr unsigned positive = 0x7f, negative = 0x80;
constexpr unsigned data_cell = 0x300;
unsigned cases = 0, continuations = 0, rejections = 0;

void check(Machine &machine, const Input input) {
  machine.prepare(input);
  const auto before = machine.cpu.state();
  auto bytes = machine.memory.bytes;
  unsigned next_read = (input.pc + 3) & hw::word_mask;
  for (unsigned phase = 1; phase <= periods(input.form); ++phase) {
    const bool write = phase == write_phase(input.form);
    const unsigned address = write ? target(input) : next_read;
    const unsigned value = write ? input.value : bytes[address];
    if (write) {
      bytes[address] = static_cast<std::uint8_t>(value);
    } else {
      next_read = (next_read + 1) & hw::word_mask;
    }
    machine.tick();
    const auto queue = owned(machine.cpu);
    const Event event{.period = phase,
                      .access = {.address = address, .value = value, .write = write}};
    hw::require(machine.memory.events.size() == phase && machine.memory.events.back() == event &&
                    machine.memory.bytes == bytes,
                "ISA wp/wpp/pwpp performs one actual native byte write and ordered p reads");
    const auto committed = static_cast<unsigned>(phase >= write_phase(input.form));
    hw::require(machine.cpu.state() == expected(before, input, phase) &&
                    queue.native_executed == (committed != 0) &&
                    machine.memory.native_starts == committed &&
                    machine.memory.write_widths.size() == committed &&
                    machine.memory.retired == static_cast<unsigned>(phase == periods(input.form)),
                "Native PC, V/N/Z, raw preserved flags and knownness follow the real w effect");
    hw::require(queue.completed_periods == phase && queue.attempted_periods == phase,
                "Each owned period advances once");
  }
  const auto queue = owned(machine.cpu);
  hw::require(queue.head == ((input.pc + size(input.form)) & hw::word_mask) &&
                  queue.queued_bytes == 3 && queue.ready_bytes == 2 &&
                  queue.program_byte_in_flight && queue.retired_instructions == 1,
              "Actual queued successors survive native indexed addressing and address wrap");
  ++cases;
}

void vectors(Machine &machine) {
  constexpr std::array bases{0U, initial_hx, near_end, hw::word_mask};
  constexpr std::array offsets{
      0U, 1U, positive, hw::byte_mask, signed_boundary, high_half, hw::word_mask};
  for (const auto form : forms) {
    for (const auto offset : offsets) {
      if ((form == Form::ix && offset != 0) || (form == Form::ix1 && offset > hw::byte_mask)) {
        continue;
      }
      for (const auto base : bases) {
        for (unsigned value = 0; value <= hw::byte_mask; ++value) {
          for (const auto flags : {0U, hw::byte_mask}) {
            check(machine,
                  {.form = form, .base = base, .offset = offset, .value = value, .flags = flags});
          }
        }
      }
    }
  }
}

void partial_flags(Machine &machine) {
  for (const auto form : forms) {
    for (unsigned mask = 0; mask <= hw::byte_mask; ++mask) {
      for (const auto value : {0U, negative, hw::byte_mask}) {
        for (const auto flags : {0U, hw::byte_mask}) {
          const Input input{.form = form, .value = value, .flags = flags, .known = mask};
          check(machine, input);
          machine.prepare(input, false);
          const auto before = machine.cpu.state();
          machine.cpu.step();
          hw::require(machine.cpu.state() == expected(before, input, periods(form)) &&
                          machine.memory.write_widths.size() == 1 &&
                          machine.memory.bytes[target(input)] == value &&
                          machine.memory.retired == 1,
                      "Ordinary native STA preserves even unknown I/H/C; defines only V/N/Z");
          ++cases;
        }
      }
    }
  }
}

void same(const Machine &parent, const Machine &fresh) {
  hw::require(parent.cpu.state() == fresh.cpu.state() &&
                  parent.cpu.prefetch_state() == fresh.cpu.prefetch_state() &&
                  parent.memory.bytes == fresh.memory.bytes &&
                  parent.memory.known == fresh.memory.known &&
                  parent.memory.partial_known == fresh.memory.partial_known &&
                  parent.memory.events == fresh.memory.events &&
                  parent.memory.write_widths == fresh.memory.write_widths &&
                  parent.memory.native_starts == fresh.memory.native_starts &&
                  parent.memory.retired == fresh.memory.retired,
              "Fresh owners retain captured code, committed native effects and real bus prefix");
}

void snapshots(Machine &parent, Machine &fresh) {
  constexpr unsigned changed_code = 0x82;
  for (const auto form : forms) {
    for (const auto base : {data_cell, handler, handler + 1, handler + 3}) {
      const Input input{.form = form, .base = base, .value = negative};
      for (unsigned cut = 0; cut <= periods(form); ++cut) {
        parent.prepare(input);
        for (unsigned phase = 0; phase < cut; ++phase) { parent.tick(); }
        const auto checkpoint = parent.cpu.checkpoint();
        fresh.memory = parent.memory;
        fresh.cpu.restore_checkpoint(checkpoint);
        same(parent, fresh);
        if (cut < write_phase(form)) {
          parent.memory.install(handler, std::array{static_cast<std::uint8_t>(changed_code)});
          fresh.memory.install(handler, std::array{static_cast<std::uint8_t>(changed_code)});
        }
        while (owned(parent.cpu).completed_periods < periods(form)) {
          parent.tick();
          fresh.tick();
          same(parent, fresh);
        }
        hw::require(parent.memory.bytes[target(input)] == input.value &&
                        parent.memory.write_widths.size() == 1 && parent.memory.retired == 1,
                    "Changed backing opcode cannot replace already captured STA; w never repeats");
        ++continuations;
      }
    }
  }
  for (const auto form : forms) {
    check(parent,
          {.form = form, .base = data_cell, .value = negative, .pc = hw::space_bytes - size(form)});
  }
}

void unknown_registers(Machine &machine) {
  constexpr unsigned register_bits = 16;
  constexpr unsigned accumulator_bits = 8;
  for (const auto form : forms) {
    for (unsigned bit = 0; bit < register_bits + accumulator_bits; ++bit) {
      Input input{.form = form, .base = data_cell, .value = negative};
      if (bit < register_bits) {
        input.hx_known &= ~(1U << bit);
      } else {
        input.a_known &= ~(1U << (bit - register_bits));
      }
      machine.prepare(input);
      const auto before = machine.memory.bytes;
      for (unsigned phase = 1; phase < write_phase(form); ++phase) { machine.tick(); }
      static_cast<void>(hw::rejected([&] { machine.tick(); }));
      hw::require(machine.cpu.faulted() && machine.memory.bytes == before &&
                      machine.memory.write_widths.empty() && machine.memory.retired == 0 &&
                      owned(machine.cpu).completed_periods == write_phase(form) - 1,
                  "Unknown operands refuse before the genuine write, retaining actual p prefixes");
      ++rejections;
    }
  }
}

unsigned failed_address(const Input input, const unsigned phase) {
  if (phase == write_phase(input.form)) { return target(input); }
  unsigned prior_reads = 0;
  for (unsigned n = 1; n < phase; ++n) {
    if (n != write_phase(input.form)) { ++prior_reads; }
  }
  return (input.pc + 3 + prior_reads) & hw::word_mask;
}

void failure(
    Machine &parent, Machine &fresh, const Input input, const unsigned phase, const bool after) {
  parent.prepare(input);
  for (unsigned n = 1; n < phase; ++n) { parent.tick(); }
  const auto saved = parent.cpu.checkpoint();
  const auto memory = parent.memory;
  const auto before = parent.cpu.state();
  const bool write = phase == write_phase(input.form);
  const unsigned address = failed_address(input, phase);
  parent.memory.after = after;
  (write ? parent.memory.refused_write : parent.memory.refused_read) = address;
  static_cast<void>(hw::rejected([&] { parent.tick(); }));
  auto bytes = memory.bytes;
  if (write && after) { bytes[address] = static_cast<std::uint8_t>(input.value); }
  hw::require(parent.cpu.faulted() && parent.memory.bytes == bytes && parent.memory.retired == 0 &&
                  parent.memory.events.size() == phase - 1 + static_cast<unsigned>(after) &&
                  owned(parent.cpu).attempted_periods == phase &&
                  owned(parent.cpu).completed_periods == phase - 1,
              "Before/after p/w failures retain their actual committed prefix");
  if (!write) {
    hw::require(parent.cpu.state() == before, "Failed p cycle cannot undo native w effects");
  }
  const auto accesses = parent.memory.accesses.size();
  static_cast<void>(hw::rejected([&] { parent.tick(); }));
  hw::require(parent.memory.accesses.size() == accesses, "Quarantined CPU cannot retry IO");
  fresh.memory = memory;
  fresh.cpu.restore_checkpoint(saved);
  finish(fresh, input);
  hw::require(fresh.memory.bytes[target(input)] == input.value &&
                  fresh.memory.write_widths == std::vector<unsigned>{1} &&
                  fresh.memory.native_starts == 1 && fresh.memory.retired == 1,
              "Trusted joint recovery repeats no prefix before its own snapshot");
  ++continuations;
}

void failures(Machine &parent, Machine &fresh) {
  constexpr unsigned payload = 0xa5;
  for (const auto form : forms) {
    const Input input{.form = form, .base = data_cell, .value = payload};
    for (unsigned phase = 1; phase <= periods(form); ++phase) {
      for (const bool after : {false, true}) { failure(parent, fresh, input, phase, after); }
    }
    parent.prepare(input);
    parent.memory.known.reset(target(input));
    finish(parent, input);
    hw::require(parent.memory.known[target(input)] && parent.memory.bytes[target(input)] == payload,
                "Native STA establishes unknown writable RAM without a speculative data read");
    parent.prepare(input);
    parent.memory.writable.reset(target(input));
    for (unsigned n = 1; n < write_phase(form); ++n) { parent.tick(); }
    static_cast<void>(hw::rejected([&] { parent.tick(); }));
    hw::require(parent.cpu.faulted() && parent.memory.bytes[target(input)] == nop &&
                    parent.memory.retired == 0,
                "Read-only destinations remain strict before data mutation");
    ++rejections;
  }
}

void wrapping_instruction(Machine &machine) {
  machine.prepare({.form = Form::ix2, .pc = hw::word_mask - 1});
  const auto before = machine.cpu.state();
  const auto bytes = machine.memory.bytes;
  hw::require(hw::rejected([&] { machine.tick(); }).contains("wrap") && machine.cpu.faulted() &&
                  machine.cpu.state() == before && machine.memory.bytes == bytes &&
                  machine.memory.events.empty(),
              "Unqualified instruction crossing the address limit retains its strict frontier");
  ++rejections;
}

void old_contracts(Machine &machine) {
  constexpr std::array prior{Hcs08PrefetchContract::reference_linear_program_cycles,
                             Hcs08PrefetchContract::reference_data_branch_cycles,
                             Hcs08PrefetchContract::reference_short_branch_refill,
                             Hcs08PrefetchContract::reference_peripheral_program_cycles,
                             Hcs08PrefetchContract::reference_jump_program_cycles,
                             Hcs08PrefetchContract::reference_scanner_byte_cycles,
                             Hcs08PrefetchContract::reference_scanner_word_cycles,
                             Hcs08PrefetchContract::reference_scanner_move_cycles};
  for (const auto form : forms) {
    for (const auto selected : prior) {
      machine.prepare({.form = form}, true, selected);
      static_cast<void>(hw::rejected([&] { machine.tick(); }));
      hw::require(machine.cpu.faulted() && machine.memory.events.empty() &&
                      machine.memory.write_widths.empty() && machine.memory.retired == 0,
                  "Unsupported old indexed stores have no program/data effect");
      ++rejections;
    }
  }
}
}

int main() {
  try {
    Machine parent;
    Machine fresh;
    vectors(parent);
    partial_flags(parent);
    snapshots(parent, fresh);
    unknown_registers(parent);
    old_contracts(parent);
    failures(parent, fresh);
    wrapping_instruction(parent);
    blofeld::test::indexed::inherited_stores();
    std::cout << "Indexed native STA: " << cases << " independent cases, " << continuations
              << " fresh suffixes, " << rejections << " strict controls\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
