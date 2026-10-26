// SPDX-License-Identifier: GPL-3.0-only
#include "clear_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "modify_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::modify;
unsigned cases = 0, copies = 0, refusals = 0;
constexpr unsigned positive_limit = 0x7f, negative_limit = 0x80, partial_flags = 0xef;

void check(Machine &m, const Input input) {
  m.prepare(input);
  const auto before = m.cpu.state();
  auto expected = before;
  std::vector<Event> events;
  unsigned next_read = handler + 3;
  for (unsigned period = 1; period <= input.spec.periods; ++period) {
    if (period == input.spec.read) {
      events.push_back(
          {.period = period,
           .access = {.address = target(input), .value = input.value, .write = false}});
    } else if (period == input.spec.write) {
      events.push_back(
          {.period = period,
           .access = {.address = target(input), .value = result(input), .write = true}});
      expected.pc = static_cast<std::uint16_t>(handler + input.spec.bytes);
      expected.ccr = static_cast<std::uint8_t>((before.ccr & ~vnz) | flags(input));
      expected.ccr_known |= vnz;
    } else if (period != input.spec.read + 1) {
      events.push_back(
          {.period = period, .access = {.address = next_read++, .value = nop, .write = false}});
    }
    m.tick();
    const auto queue = owned(m.cpu);
    require(m.memory.events == events && m.cpu.state() == expected &&
                m.memory.bytes[target(input)] ==
                    (period < input.spec.write ? input.value : result(input)) &&
                m.memory.native_starts == static_cast<unsigned>(period >= input.spec.read) &&
                m.memory.retired == static_cast<unsigned>(period == input.spec.periods) &&
                queue.native_executed == (period >= input.spec.write) &&
                queue.completed_periods == period && queue.attempted_periods == period,
            "Literal ISA r/f/w/p periods, real native RMW arithmetic/VNZ and preserved A/HX/H/I/C");
  }
  require(m.memory.widths == std::vector<unsigned>{1}, "One native result-byte write");
  ++cases;
}

void profile(Machine &m, const Spec spec, const Operation operation) {
  for (unsigned value = 0; value < byte_values; ++value) {
    check(m, {.spec = spec, .operation = operation, .value = value});
  }
  const unsigned overflow_input =
      operation == Operation::increment ? positive_limit : negative_limit;
  for (unsigned value = 0; value < byte_values; ++value) {
    check(m, {.spec = spec, .operation = operation, .value = overflow_input, .flags = value});
    check(m, {.spec = spec, .operation = operation, .value = overflow_input, .known = value});
  }
  for (const auto a_known : {0U, flag_n, byte_mask}) {
    for (const auto hx_known : {0U, byte_mask, word_mask}) {
      if ((spec.form == Form::ix || spec.form == Form::ix1) && hx_known != word_mask) { continue; }
      check(m, {.spec = spec, .operation = operation, .a_known = a_known, .hx_known = hx_known});
    }
  }
}

void boundaries(Machine &m, const Operation operation) {
  for (const unsigned base : {0U, word_mask}) {
    check(m, {.spec = specs[1], .operation = operation, .hx = base});
    for (const unsigned offset : {0U, 1U, byte_mask}) {
      check(m, {.spec = specs[2], .operation = operation, .offset = offset, .hx = base});
      if (base == 0) {
        check(m, {.spec = specs[3], .operation = operation, .offset = offset, .sp = word_mask - 5});
      }
    }
  }
}

void snapshot_value(Machine &parent, Machine &fresh, const Input input) {
  constexpr unsigned poison = 0x55;
  for (unsigned cut = 0; cut <= input.spec.periods; ++cut) {
    for (unsigned repeat = 0; repeat < 2; ++repeat) {
      parent.prepare(input);
      while (owned(parent.cpu).completed_periods < cut) { parent.tick(); }
      const auto saved = parent.cpu.checkpoint();
      fresh.memory = parent.memory;
      fresh.cpu.restore_checkpoint(saved);
      if (cut >= input.spec.read && cut < input.spec.write) {
        parent.memory.bytes[target(input)] = poison;
        fresh.memory.bytes[target(input)] = poison;
      }
      finish(parent, input);
      finish(fresh, input);
      require(
          parent.cpu.state() == fresh.cpu.state() &&
              parent.cpu.prefetch_state() == fresh.cpu.prefetch_state() &&
              parent.memory.events == fresh.memory.events &&
              parent.memory.bytes == fresh.memory.bytes &&
              parent.memory.known == fresh.memory.known &&
              parent.memory.partial_known == fresh.memory.partial_known &&
              fresh.memory.bytes[target(input)] == result(input),
          "Fresh native RMW resumes its latched operand/temporary/result without rereading data");
      ++copies;
    }
  }
}

void strict(Machine &m, const Spec spec, const Operation operation) {
  const Input input{.spec = spec, .operation = operation};
  m.prepare(input, Hcs08PrefetchContract::reference_clear_program_cycles);
  const auto before = m.cpu.state();
  const auto error = rejected([&] { m.tick(); });
  require(error.contains("unqualified") && m.cpu.faulted() && m.memory.events.empty() &&
              m.cpu.state() == before,
          "Prior CLR contract rejects memory INC/DEC before bus effects");
  ++refusals;
  if (spec.form != Form::ix && spec.form != Form::ix1) { return; }
  for (const unsigned mask : {0U, byte_mask}) {
    m.prepare({.spec = spec, .operation = operation, .hx_known = mask});
    const auto bad = rejected([&] { m.tick(); });
    require(bad.contains("unknown operand") && bad.contains("pc=") && bad.contains("address=") &&
                m.memory.events.empty() && m.cpu.faulted(),
            "Unknown HX cannot supply a native RMW address");
    ++refusals;
  }
}

void unknown_data(Machine &m, const Spec spec, const Operation operation) {
  for (unsigned mask = 0; mask < byte_mask; ++mask) {
    const Input input{.spec = spec, .operation = operation, .memory_known = mask};
    m.prepare(input);
    while (owned(m.cpu).completed_periods < spec.read - 1) { m.tick(); }
    const auto events = m.memory.events;
    const auto error = rejected([&] { m.tick(); });
    require(error.contains("read") && error.contains("address=") && error.contains("pc=") &&
                m.memory.events == events && m.memory.bytes[target(input)] == input.value &&
                m.cpu.faulted(),
            "Unknown old operand cannot manufacture a known increment/decrement");
    ++refusals;
  }
}

void failures(Machine &m, Machine &fresh, const Spec spec, const Operation operation) {
  const Input input{.spec = spec, .operation = operation};
  for (const bool write : {false, true}) {
    for (const bool after : {false, true}) {
      m.prepare(input);
      while (owned(m.cpu).completed_periods < (write ? spec.write : spec.read) - 1) { m.tick(); }
      const auto saved = m.cpu.checkpoint();
      const auto memory = m.memory;
      m.memory.after = after;
      (write ? m.memory.refused_write : m.memory.refused_read) = target(input);
      const auto error = rejected([&] { m.tick(); });
      require(error.contains("INC/DEC") && error.contains("address=") && error.contains("pc=") &&
                  m.cpu.faulted(),
              "RMW before/after bus failure quarantines pending native execution");
      static_cast<void>(rejected([&] { m.tick(); }));
      static_cast<void>(rejected([&] { static_cast<void>(m.cpu.checkpoint()); }));
      fresh.memory = memory;
      fresh.cpu.restore_checkpoint(saved);
      finish(fresh, input);
      require(fresh.memory.bytes[target(input)] == result(input) && !fresh.cpu.faulted(),
              "Coordinated fresh restore completes exactly the original native result write");
      ++refusals;
    }
  }
}

void inherited_clear() {
  namespace cl = blofeld::test::clear;
  cl::Machine parent;
  cl::Machine fresh;
  for (const auto spec : cl::specs) {
    for (const unsigned mask : {0U, byte_mask, negative_limit}) {
      const cl::Input input{.spec = spec, .memory_known = mask};
      for (unsigned cut = 0; cut <= spec.periods; ++cut) {
        parent.prepare(input, contract);
        while (cl::owned(parent.cpu).completed_periods < cut) { parent.tick(); }
        const auto saved = parent.cpu.checkpoint();
        fresh.memory = parent.memory;
        fresh.cpu.restore_checkpoint(saved);
        cl::finish(parent, input);
        cl::finish(fresh, input);
        require(parent.cpu.state() == fresh.cpu.state() &&
                    parent.cpu.prefetch_state() == fresh.cpu.prefetch_state() &&
                    parent.memory.events == fresh.memory.events &&
                    fresh.memory.bytes[cl::target(input)] == 0,
                "New RMW contract retains genuine masked unused CLR reads and native zero writes");
      }
    }
  }
}
}

int main() {
  try {
    Machine parent;
    Machine fresh;
    for (const auto operation : operations) {
      for (const auto spec : specs) {
        profile(parent, spec, operation);
        strict(parent, spec, operation);
        unknown_data(parent, spec, operation);
        failures(parent, fresh, spec, operation);
        for (const unsigned value : {0U, positive_limit, negative_limit, byte_mask}) {
          snapshot_value(
              parent,
              fresh,
              {.spec = spec, .operation = operation, .value = value, .known = partial_flags});
        }
      }
      boundaries(parent, operation);
    }
    inherited_clear();
    inherited_stores();
    std::cout << "Native memory INC/DEC: " << cases << " independent vectors, " << copies
              << " fresh suffixes, " << refusals << " strict/failure controls.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
