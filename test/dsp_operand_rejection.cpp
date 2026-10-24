// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_operand_fixture.hpp"
#include "native_access_fixture.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::operands {
namespace {
using native_access::fault;
constexpr unsigned immediate_limit = 31;
constexpr unsigned width_shift = 12;
constexpr unsigned field_mask = 0x3f;
constexpr std::uint64_t first_value = 0x55123456abcdef;
constexpr std::uint64_t second_value = 0xaa654321234567;
constexpr unsigned source_value = 0x789abc;
constexpr unsigned first_extract_control = 0x1038;
constexpr unsigned captured_extract_control = 0x803c;
constexpr unsigned excessive_insert_control = 0x19000;

struct RejectedCase {
  std::string operation;
  std::string diagnostic;
  unsigned control = 0;
  bool bitfield = false;
  bool immediate = false;
  bool b = false;
};

struct BadField {
  unsigned width = 0;
  unsigned offset = 0;
};

std::string instruction(const RejectedCase &item) {
  const auto control = item.immediate ? '#' + number(item.control) : "x0";
  const char *source = "";
  if (item.bitfield) { source = item.operation == "insert" ? ",x1" : ",a"; }
  return item.operation + ' ' + control + source + (item.b ? ",b" : ",a");
}

std::vector<RejectedCase> logical_cases() {
  std::vector<RejectedCase> result;
  constexpr std::array counts{25U, 31U, 32U, 40U, 55U, 56U, 63U};
  for (const auto *operation : {"lsl", "lsr"}) {
    const std::string name(operation);
    for (const auto count : counts) {
      for (unsigned mode = 0; mode < 4; ++mode) {
        const bool immediate = (mode & 1U) != 0;
        if (immediate && count > immediate_limit) { continue; }
        result.push_back({.operation = operation,
                          .diagnostic = name == "lsl" ? "LSL" : "LSR",
                          .control = count,
                          .immediate = immediate,
                          .b = (mode & 2U) != 0});
      }
    }
  }
  return result;
}

void append_fields(std::vector<RejectedCase> &result,
                   const RejectedCase &prototype,
                   const std::span<const BadField> fields) {
  for (const auto field : fields) {
    for (unsigned mode = 0; mode < 4; ++mode) {
      auto item = prototype;
      item.control = (field.width << width_shift) | field.offset;
      item.immediate = (mode & 1U) != 0;
      item.b = (mode & 2U) != 0;
      result.push_back(item);
    }
  }
}

std::vector<RejectedCase> bitfield_cases() {
  std::vector<RejectedCase> result;
  constexpr std::array extract{BadField{.width = 1, .offset = 56},
                               BadField{.width = 56, .offset = 1},
                               BadField{.width = 63, .offset = 0},
                               BadField{.width = 0, .offset = 57},
                               BadField{.width = 8, .offset = 60}};
  constexpr std::array insert{BadField{.width = 25, .offset = 0},
                              BadField{.width = 24, .offset = 33},
                              BadField{.width = 1, .offset = 56},
                              BadField{.width = 0, .offset = 57},
                              BadField{.width = 63, .offset = 63}};
  append_fields(
      result, {.operation = "extract", .diagnostic = "EXTRACT", .bitfield = true}, extract);
  append_fields(
      result, {.operation = "extractu", .diagnostic = "EXTRACTU", .bitfield = true}, extract);
  append_fields(result, {.operation = "insert", .diagnostic = "INSERT", .bitfield = true}, insert);
  return result;
}

unsigned prepare(DspMachine &machine, const RejectedCase &item) {
  DspProgram code;
  seed_operands(code,
                {.a = first_value,
                 .b = second_value,
                 .control = item.control,
                 .source = source_value,
                 .status = all_ccr});
  const auto body = code.pc();
  code.emit(instruction(item));
  code.emit("nop");
  code.install(machine.bus());
  for (unsigned index = 0; index < operand_preamble_steps; ++index) { machine.cpu().step(); }
  return body;
}

std::string rejected_control(const RejectedCase &item) {
  if (!item.bitfield) { return "count " + std::to_string(item.control); }
  return "width=" + std::to_string((item.control >> width_shift) & field_mask) +
         " offset=" + std::to_string(item.control & field_mask);
}

void expect_rejection(DspMachine &machine, const RejectedCase &item, const DspDebugState &before) {
  const auto error = fault([&] { machine.cpu().step(); });
  const auto after = machine.cpu().debug_state();
  auto expected = before.cpu;
  // Only the actual opcode/control fetch advances PC before the rejected body.
  expected.pc += item.bitfield && item.immediate ? 2U : 1U;
  require(error.contains("Unqualified DSP reference " + item.diagnostic) &&
              error.contains(rejected_control(item)) &&
              error.contains("pc=" + hex(before.cpu.pc, 6)) && error.contains("omr=0x000305") &&
              after.cpu == expected && after.native_instructions == before.native_instructions &&
              after.stack_high == before.stack_high && after.stack_low == before.stack_low &&
              after.ep == before.ep && after.sz == before.sz && after.sc == before.sc,
          "Rejected control preserves data/flags/stack and reports the actual instruction: " +
              instruction(item));
  static_cast<void>(fault([&] { machine.cpu().step(); }));
  static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
}

void recovery(const RejectedCase &item) {
  DspMachine machine;
  const auto body = prepare(machine, item);
  const auto before = machine.cpu().debug_state();
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    expect_rejection(machine, item, before);
    machine.restore(saved);
    const std::array nop{0U};
    machine.bus().install_rom({.start = body, .words = nop});
    machine.cpu().step();
    auto expected = before.cpu;
    ++expected.pc;
    require(machine.cpu().state() == expected &&
                machine.cpu().debug_state().native_instructions == before.native_instructions + 1,
            "Trusted prefetch restore recovers after replacing only the unconsumed own opcode");
  }
}
}

void rejection_checks() {
  const auto logical = logical_cases();
  const auto fields = bitfield_cases();
  DspMachine machine;
  const auto check = [&machine](const std::span<const RejectedCase> cases) {
    for (const auto &item : cases) {
      machine.cpu().reset_spi_boot();
      static_cast<void>(prepare(machine, item));
      expect_rejection(machine, item, machine.cpu().debug_state());
    }
  };
  check(logical);
  check(fields);
  for (const auto &item :
       std::array{RejectedCase{.operation = "lsl", .diagnostic = "LSL", .control = immediate_limit},
                  RejectedCase{.operation = "lsr", .diagnostic = "LSR", .control = 32},
                  RejectedCase{.operation = "extract",
                               .diagnostic = "EXTRACT",
                               .control = first_extract_control,
                               .bitfield = true},
                  RejectedCase{.operation = "extractu",
                               .diagnostic = "EXTRACTU",
                               .control = captured_extract_control,
                               .bitfield = true,
                               .immediate = true},
                  RejectedCase{.operation = "insert",
                               .diagnostic = "INSERT",
                               .control = excessive_insert_control,
                               .bitfield = true}}) {
    recovery(item);
  }
  std::cout << logical.size() + fields.size()
            << " unqualified operand cases and five native trusted-recovery cases passed.\n";
}
}
