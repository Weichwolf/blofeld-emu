// SPDX-License-Identifier: GPL-3.0-only
#include "clear_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::clear;
unsigned cases = 0, copies = 0, refusals = 0;

void check(Machine &m, const Input input) {
  m.prepare(input);
  const auto before = m.cpu.state();
  auto expected = before;
  std::vector<Event> events;
  unsigned next_read = handler + 3;
  for (unsigned period = 1; period <= input.spec.periods; ++period) {
    if (period == input.spec.read) {
      events.push_back({.period = period,
                        .access = {.address = target(input),
                                   .value = input.value & input.memory_known,
                                   .write = false,
                                   .known = static_cast<std::uint8_t>(input.memory_known)}});
    } else if (period == input.spec.write) {
      events.push_back(
          {.period = period, .access = {.address = target(input), .value = 0, .write = true}});
      expected.pc = static_cast<std::uint16_t>(handler + input.spec.bytes);
      expected.ccr = static_cast<std::uint8_t>((before.ccr & ~vnz) | flag_z);
      expected.ccr_known |= vnz;
    } else if (period != input.spec.read + 1) {
      events.push_back(
          {.period = period, .access = {.address = next_read++, .value = nop, .write = false}});
    }
    m.tick();
    const auto queue = owned(m.cpu);
    require(m.memory.events == events && m.cpu.state() == expected &&
                m.memory.bytes[target(input)] == (period < input.spec.write ? input.value : 0) &&
                m.memory.native_starts == static_cast<unsigned>(period >= input.spec.write) &&
                m.memory.retired == static_cast<unsigned>(period == input.spec.periods) &&
                queue.native_executed == (period >= input.spec.write) &&
                queue.completed_periods == period && queue.attempted_periods == period,
            "Literal CLR r/f/w/p sequence, native zero/flags and unchanged unrelated registers");
    require(m.memory.known[target(input)] ==
                (period >= input.spec.write || input.memory_known == byte_mask),
            "Unknown old RAM bits become known only at the native zero write");
    if (period >= input.spec.read) {
      require(queue.clear_read.address == target(input) &&
                  queue.clear_read.value == (input.value & input.memory_known) &&
                  queue.clear_read.known == input.memory_known &&
                  queue.clear_read.in_flight == (period == input.spec.read),
              "Actual old-byte read is retained even though CLR does not consume its value");
    }
  }
  const auto queue = owned(m.cpu);
  require(queue.queued_bytes == 3 && queue.ready_bytes == 2 && queue.program_byte_in_flight &&
              queue.retired_instructions == 1 && m.memory.widths == std::vector<unsigned>{1},
          "One native byte write retires only after the literal final program read");
  ++cases;
}

void matrix(Machine &m) {
  for (const auto spec : specs) {
    for (unsigned value = 0; value < byte_values; ++value) {
      check(m, {.spec = spec, .value = value});
    }
    for (unsigned flags = 0; flags < byte_values; ++flags) {
      check(m, {.spec = spec, .flags = flags});
    }
    for (unsigned known = 0; known < byte_values; ++known) {
      check(m, {.spec = spec, .known = known});
    }
    for (unsigned mask = 0; mask < byte_values; ++mask) {
      check(m, {.spec = spec, .memory_known = mask});
    }
    for (const auto a_known : {0U, flag_n, byte_mask}) {
      for (const auto hx_known : {0U, byte_mask, word_mask}) {
        if ((spec.form == Form::ix || spec.form == Form::ix1) && hx_known != word_mask) {
          continue;
        }
        check(m, {.spec = spec, .a_known = a_known, .hx_known = hx_known});
      }
    }
  }
}

void boundaries(Machine &m) {
  for (const unsigned base : {0U, word_mask}) {
    check(m, {.spec = specs[1], .hx = base});
    for (const unsigned offset : {0U, 1U, byte_mask}) {
      check(m, {.spec = specs[2], .offset = offset, .hx = base});
      if (base == 0) { check(m, {.spec = specs[3], .offset = offset, .sp = word_mask - 5}); }
    }
  }
}

void checkpoints(Machine &parent, Machine &fresh) {
  constexpr unsigned poison = 0x55;
  for (const auto spec : specs) {
    for (const unsigned mask : {byte_mask, 0U, flag_n}) {
      const Input input{.spec = spec, .memory_known = mask};
      for (unsigned cut = 0; cut <= spec.periods; ++cut) {
        for (unsigned repeat = 0; repeat < 2; ++repeat) {
          parent.prepare(input);
          while (owned(parent.cpu).completed_periods < cut) { parent.tick(); }
          const auto saved = parent.cpu.checkpoint();
          fresh.memory = parent.memory;
          fresh.cpu.restore_checkpoint(saved);
          if (cut >= spec.read && cut < spec.write) {
            parent.memory.bytes[target(input)] = poison;
            fresh.memory.bytes[target(input)] = poison;
          }
          finish(parent, input);
          finish(fresh, input);
          require(parent.cpu.state() == fresh.cpu.state() &&
                      parent.cpu.prefetch_state() == fresh.cpu.prefetch_state() &&
                      parent.memory.events == fresh.memory.events &&
                      parent.memory.bytes == fresh.memory.bytes &&
                      parent.memory.known == fresh.memory.known &&
                      parent.memory.partial_known == fresh.memory.partial_known &&
                      fresh.memory.bytes[target(input)] == 0,
                  "Fresh CLR owner restores r/f/w/p phase without repeating side-effect reads");
          ++copies;
        }
      }
    }
  }
}

void strict(Machine &m) {
  for (const auto spec : specs) {
    m.prepare({.spec = spec}, Hcs08PrefetchContract::reference_indexed_store_cycles);
    const auto before = m.cpu.state();
    const auto error = rejected([&] { m.tick(); });
    require(error.contains("unqualified") && m.cpu.faulted() && m.memory.events.empty() &&
                m.cpu.state() == before,
            "Prior indexed-store contract still rejects memory CLR before a bus side effect");
    ++refusals;
  }
  for (const auto spec : {specs[1], specs[2]}) {
    m.prepare({.spec = spec, .hx_known = 0});
    const auto error = rejected([&] { m.tick(); });
    require(error.contains("indexed address is unknown") && error.contains("pc=") &&
                error.contains("address=") && m.memory.events.empty() && m.cpu.faulted(),
            "Unknown HX cannot manufacture a CLR r address");
    ++refusals;
  }
  for (const auto spec : specs) {
    const Input input{.spec = spec};
    m.prepare(input);
    m.memory.known.reset(target(input));
    m.memory.writable.reset(target(input));
    while (owned(m.cpu).completed_periods < spec.read - 1) { m.tick(); }
    const auto events = m.memory.events;
    const auto error = rejected([&] { m.tick(); });
    require(error.contains("read") && error.contains("address=") && error.contains("pc=") &&
                m.memory.events == events && m.cpu.faulted(),
            "Masked CLR capability cannot turn an unsupported non-RAM read into a known zero");
    ++refusals;
  }
}

void failures(Machine &m, Machine &fresh) {
  for (const auto spec : specs) {
    const Input input{.spec = spec};
    for (const bool write : {false, true}) {
      for (const bool after : {false, true}) {
        m.prepare(input);
        while (owned(m.cpu).completed_periods < (write ? spec.write : spec.read) - 1) { m.tick(); }
        const auto saved = m.cpu.checkpoint();
        const auto bytes = m.memory;
        m.memory.after = after;
        (write ? m.memory.refused_write : m.memory.refused_read) = target(input);
        const auto error = rejected([&] { m.tick(); });
        require(error.contains("CLR") && error.contains("address=") && error.contains("pc=") &&
                    m.cpu.faulted(),
                "Before/after actual CLR bus failure quarantines the CPU");
        static_cast<void>(rejected([&] { m.tick(); }));
        static_cast<void>(rejected([&] { static_cast<void>(m.cpu.checkpoint()); }));
        fresh.memory = bytes;
        fresh.cpu.restore_checkpoint(saved);
        finish(fresh, input);
        require(fresh.memory.bytes[target(input)] == 0 && !fresh.cpu.faulted(),
                "Coordinated pre-fault restore to a fresh bus completes the real zero write");
        ++refusals;
      }
    }
  }
}
}

int main() {
  try {
    Machine parent;
    Machine fresh;
    matrix(parent);
    boundaries(parent);
    checkpoints(parent, fresh);
    inherited_stores();
    strict(parent);
    failures(parent, fresh);
    std::cout << "Native memory CLR: " << cases << " independent vectors, " << copies
              << " fresh suffixes, " << refusals << " strict/failure controls.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
