// SPDX-License-Identifier: GPL-3.0-only
// Original installer/preparation with a test observation landing pad.
// The landing pad starts a new independent case, never a completion callback or flash success.
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <ios>
#include <iostream>
#include <istream>
#include <ostream>
#include <span>
#include <string>
#include <vector>

namespace {
using blofeld::DspSpace;
using blofeld::test::require;
constexpr unsigned helper_base = 0x2e00;
constexpr unsigned helper_alias = 0x11e00;
constexpr unsigned helper_words = 133;
constexpr unsigned scanner_entry = 0x2e33;
constexpr unsigned copy_entry = 0x2e5f;
constexpr unsigned observation_entry = 0x400;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned descriptor_base = 0x100;
constexpr unsigned hook_address = 0x300;
constexpr unsigned status_slot = 0x46;
constexpr unsigned entry_slot = 0x80;
constexpr unsigned poll_entry = 0x2e28;
constexpr unsigned copy_callback = 0x2e76;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned word_bits = 24;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned memory_mode = 0x600389;
constexpr unsigned absolute_jsr = 0x0bf080; // Documented own absolute JSR, with target extension.
constexpr unsigned state_base = 0xc000;
constexpr unsigned output_base = 0x800;
constexpr unsigned page_bytes = 528;
constexpr unsigned page_words = page_bytes / 3;
constexpr unsigned pages_per_group = 32;
constexpr unsigned data_words = page_words * pages_per_group;
constexpr unsigned group_words = 5640;
constexpr unsigned expected_groups = 1488;
constexpr unsigned marker = 0xddef0e;
constexpr unsigned stride = 0x1610;
constexpr unsigned marker_offset = 0x1601;
constexpr unsigned first_slot = 0x10000;
constexpr unsigned last_slot = 0x14230;
constexpr unsigned output_cookie = 0x5a;
constexpr unsigned guard_cookie = 0x13579b;
constexpr unsigned row_mask = 0xfffe0;
constexpr unsigned scan_selector = 0x1c00;
constexpr unsigned copy_selector = 0x1a00;
constexpr unsigned step_limit = 4096;

unsigned read_be(std::istream &stream, const unsigned count) {
  unsigned value = 0;
  for (unsigned index = 0; index < count; ++index) {
    const int byte = stream.get();
    require(byte != std::char_traits<char>::eof(), "Truncated pinned native input");
    value = (value << 8) | static_cast<unsigned>(byte);
  }
  return value;
}

void write(blofeld::DspBus &bus,
           const DspSpace space,
           const unsigned address,
           const unsigned value) {
  bus.write({.space = space, .address = address, .value = value});
}

unsigned read(blofeld::DspBus &bus, const DspSpace space, const unsigned address) {
  return bus.read({.space = space, .address = address});
}

struct Harness {
  unsigned begin;
};

Harness initialize(blofeld::DspMachine &machine) {
  auto &bus = machine.bus();
  // Conditional test descriptor only; it is not an asserted Desktop/Keyboard runtime table.
  write(bus, DspSpace::x, descriptor_base, 4);
  write(bus, DspSpace::x, descriptor_base + 1, hook_address);
  write(bus, DspSpace::x, descriptor_base + 5, status_slot);
  write(bus, DspSpace::x, descriptor_base + 6, observation_entry);
  write(bus, DspSpace::x, descriptor_base + 7, output_base);
  blofeld::test::DspProgram code;
  code.immediate("sr", blofeld::test::masked_status);
  code.immediate("sp", 0);
  code.immediate("sc", 0);
  for (unsigned index = 0; index < 8; ++index) {
    code.immediate("m" + std::to_string(index), word_mask);
  }
  code.immediate("omr", memory_mode);
  for (unsigned index = 0; index < 3; ++index) { code.emit("nop"); }
  code.immediate("r0", descriptor_base);
  code.word(absolute_jsr);
  code.word(helper_base);
  const auto begin = code.pc();
  for (const auto *reg : {"a", "b", "sp", "sc", "la", "lc"}) { code.immediate(reg, 0); }
  for (unsigned index = 0; index < 8; ++index) {
    code.immediate("m" + std::to_string(index), word_mask);
  }
  code.immediate("sr", blofeld::test::masked_status);
  code.emit("move x:>" + blofeld::test::number(entry_slot) + ",r3");
  code.emit("jsr (r3)");
  code.emit("nop");
  code.install(bus);
  write(bus, DspSpace::p, observation_entry, absolute_jump);
  write(bus, DspSpace::p, observation_entry + 1, begin);
  unsigned steps = 0;
  while (machine.cpu().state().pc != begin && steps < step_limit) {
    machine.cpu().step();
    bus.advance_instruction_cycles(1);
    ++steps;
  }
  require(machine.cpu().state().pc == begin, "Original installer must genuinely return");
  require(read(bus, DspSpace::x, state_base) == 0 &&
              read(bus, DspSpace::x, state_base + 3) == output_base &&
              read(bus, DspSpace::x, state_base + 5) == first_slot &&
              read(bus, DspSpace::x, state_base + 6) == 0,
          "Original installer sets its own state from the declared test descriptor");
  return {.begin = begin};
}

void caller(blofeld::DspMachine &machine, const unsigned entry, const Harness harness) {
  write(machine.bus(), DspSpace::x, entry_slot, entry);
  if (machine.cpu().state().pc == observation_entry) {
    // Real own JMP begins another independent unit case; no resident callback or outcome.
    machine.cpu().step();
    machine.bus().advance_instruction_cycles(1);
  }
  require(machine.cpu().state().pc == harness.begin, "Own case transition must execute natively");
}

std::uint64_t execute(blofeld::DspMachine &machine) {
  for (unsigned count = 0; count < step_limit; ++count) {
    if (machine.cpu().state().pc == observation_entry) { return count; }
    machine.cpu().step();
    machine.bus().advance_instruction_cycles(1);
  }
  throw blofeld::Fault("Original preparation exceeded its independent instruction bound");
}

void verify_code(blofeld::DspBus &bus, const std::span<const unsigned> helper) {
  constexpr unsigned service_operand = 0x2e27;
  constexpr unsigned status_read_operand = 0x2e29;
  constexpr unsigned status_write_operand = 0x2e30;
  constexpr unsigned loop_operand = 0x2e42;
  for (unsigned index = 0; index < helper.size(); ++index) {
    const auto address = helper_base + index;
    auto expected = helper[index];
    if (address == service_operand) { expected = observation_entry; }
    if (address == status_read_operand || address == status_write_operand) {
      expected = status_slot;
    }
    if (address == loop_operand) { expected = hook_address - 2; }
    require(bus.peek_program(address) == expected,
            "Only original installer relocation writes may change its code");
  }
  require(bus.peek_program(hook_address) == absolute_jump &&
              bus.peek_program(hook_address + 1) == poll_entry,
          "Original installer copies its own hook words into declared test RAM");
}

struct Location {
  unsigned slot;
  unsigned row;
};

void scan(blofeld::DspMachine &machine, const Location location, const Harness harness) {
  const auto [slot, row] = location;
  caller(machine, scanner_entry, harness);
  write(machine.bus(), DspSpace::x, state_base, 0);
  write(machine.bus(), DspSpace::x, state_base + 3, output_base);
  write(machine.bus(), DspSpace::x, state_base + 5, slot);
  execute(machine);
  const auto state = machine.cpu().state();
  auto &bus = machine.bus();
  require(state.pc == observation_entry && state.sp == 2 &&
              state.y ==
                  ((static_cast<std::uint64_t>(row & row_mask) << word_bits) | scan_selector) &&
              state.r[0] == copy_entry && (state.x & word_mask) == page_bytes &&
              (state.x >> word_bits) == 0,
          "Original scanner prepares its request from actual frame metadata");
  const auto next_slot = slot == last_slot ? first_slot : slot + stride;
  require(read(bus, DspSpace::x, state_base) == 1 &&
              read(bus, DspSpace::x, state_base + 1) == slot &&
              read(bus, DspSpace::x, state_base + 2) == row &&
              read(bus, DspSpace::x, state_base + 5) == next_slot &&
              read(bus, DspSpace::y, slot + marker_offset) == marker_offset,
          "Original marker consumption, source/row fields and fractional ring reset");
}

void copy_page(blofeld::DspMachine &machine,
               const Location location,
               const std::span<const unsigned> words,
               const Harness harness) {
  const auto [slot, row] = location;
  caller(machine, copy_entry, harness);
  auto &bus = machine.bus();
  write(bus, DspSpace::x, state_base + 1, slot);
  write(bus, DspSpace::x, state_base + 2, row);
  write(bus, DspSpace::x, state_base + 3, output_base);
  for (unsigned index = 0; index < page_bytes; ++index) {
    write(bus, DspSpace::x, output_base + index, output_cookie);
  }
  for (const auto address : {output_base - 1, output_base + page_bytes}) {
    write(bus, DspSpace::x, address, guard_cookie);
  }
  execute(machine);
  for (unsigned index = 0; index < page_words; ++index) {
    for (unsigned byte = 0; byte < 3; ++byte) {
      const auto expected = (words[index] >> ((2 - byte) * 8)) & byte_mask;
      if (read(bus, DspSpace::x, output_base + (3 * index) + byte) != expected) {
        throw blofeld::Fault("Independent original byte transfer differs at source=" +
                             blofeld::hex(slot, 6) + " row-field=" + std::to_string(row) +
                             " word=" + std::to_string(index) + " byte=" + std::to_string(byte));
      }
    }
  }
  const auto state = machine.cpu().state();
  require(state.sp == 1 && state.r[0] == copy_callback && state.r[1] == output_base + page_bytes &&
              state.r[4] == slot + page_words &&
              state.y == ((static_cast<std::uint64_t>(row) << word_bits) | copy_selector) &&
              (state.x & word_mask) == page_bytes && (state.x >> word_bits) == 0 &&
              read(bus, DspSpace::x, state_base + 1) == slot + page_words &&
              read(bus, DspSpace::x, state_base + 4) == copy_callback,
          "Original byte-copy request/cursor/continuation boundary differs");
  for (const auto address : {output_base - 1, output_base + page_bytes}) {
    require(read(bus, DspSpace::x, address) == guard_cookie, "Changed byte-copy output guard");
  }
}

void missing_status_fails(blofeld::DspMachine &machine,
                          const Harness harness,
                          std::ostream &trace) {
  caller(machine, poll_entry, harness);
  try {
    execute(machine);
  } catch (const blofeld::Fault &error) {
    const std::string text = error.what();
    require(text.contains("Unknown DSP memory read at X:0x000046") &&
                text.contains("pc=0x002e28") && text.contains("omr=0x600389"),
            "Missing completion status retains its actual address and original PC");
    trace << "missing-status " << text << '\n';
    return;
  }
  throw blofeld::Fault("Unobserved completion status unexpectedly succeeded");
}

}

int main(const int count, char **arguments) {
  try {
    require(count == 4, "Expected pinned helper, original bulk frames and output trace");
    std::ifstream program(arguments[1], std::ios::binary);
    std::ifstream input(arguments[2], std::ios::binary);
    std::ofstream trace(arguments[3]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    require(program.is_open() && input.is_open(), "Cannot read pinned native inputs");
    blofeld::DspMachine machine;
    std::array<unsigned, helper_words> helper{};
    for (unsigned index = 0; index < helper.size(); ++index) {
      helper[index] = read_be(program, 3);
      write(machine.bus(), DspSpace::x, helper_alias + index, helper[index]);
    }
    require(program.peek() == std::char_traits<char>::eof(), "Unexpected helper suffix");
    const auto harness = initialize(machine);
    verify_code(machine.bus(), helper);
    unsigned pages = 0;
    for (unsigned group = 0; group < expected_groups; ++group) {
      const auto slot = read_be(input, 4);
      require(slot == first_slot + ((group % 4) * stride), "Changed source staging sequence");
      std::vector<unsigned> words(group_words);
      for (unsigned index = 0; index < words.size(); ++index) {
        words[index] = read_be(input, 3);
        write(machine.bus(), DspSpace::y, slot + index, words[index]);
      }
      const auto row = words.at(data_words);
      require(words.at(data_words + 1) == marker && row % pages_per_group == 0,
              "Original sample frame metadata differs");
      scan(machine, {.slot = slot, .row = row}, harness);
      verify_code(machine.bus(), helper);
      for (unsigned page = 0; page < pages_per_group; ++page) {
        copy_page(machine,
                  {.slot = slot + (page * page_words), .row = row + page},
                  std::span<const unsigned>(words).subspan(
                      static_cast<std::size_t>(page) * page_words, page_words),
                  harness);
        ++pages;
      }
      verify_code(machine.bus(), helper);
      require(read(machine.bus(), DspSpace::x, state_base + 6) == 0,
              "No completion callback was supplied by the observation landing pad");
    }
    require(input.peek() == std::char_traits<char>::eof(), "Unexpected sample frame suffix");
    missing_status_fails(machine, harness, trace);
    trace << "coverage " << expected_groups << ' ' << pages << ' '
          << static_cast<std::uint64_t>(pages) * page_bytes << '\n';
    trace << "scope isolated-preparation no-NAND-completion\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
