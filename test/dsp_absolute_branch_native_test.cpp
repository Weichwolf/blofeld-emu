// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>
#include <variant>

namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000;
constexpr unsigned ram = 0x100;
constexpr unsigned ordinary_target = 0x180;
constexpr unsigned maximum_short_target = 0xfff;
constexpr unsigned status = 0xc00300;
constexpr unsigned status_immediate = 0x05f439;
constexpr unsigned jump_extension = 0x0af080;
constexpr unsigned short_absolute = 0x0e0000; // FM13-80:00001110 CCCC aaaaaaaaaaaa.
constexpr unsigned condition_shift = 12;
constexpr unsigned conditions = 16;
constexpr unsigned flag_states = 256;
constexpr unsigned native_cost = 4; // SDK diagnostic units; not physical clocks.
constexpr unsigned carry = 1, overflow = 2, zero = 4, negative = 8;
constexpr unsigned unnormalized = 0x10, extension = 0x20, limit = 0x40;
unsigned cases = 0;
unsigned accesses = 0;

struct Input {
  unsigned condition;
  unsigned ccr;
  unsigned target;
  bool high_source = false;
  bool explicit_retirement = false;
};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

// FM12-17/18: each row is independent of the interpreter's condition decoder.
bool taken(const Input input) {
  const auto ccr = input.ccr;
  const bool c = (ccr & carry) != 0;
  const bool v = (ccr & overflow) != 0;
  const bool z = (ccr & zero) != 0;
  const bool n = (ccr & negative) != 0;
  const bool u = (ccr & unnormalized) != 0;
  const bool e = (ccr & extension) != 0;
  const bool l = (ccr & limit) != 0;
  const std::array rows{!c,
                        n == v,
                        !z,
                        !n,
                        !z && (u || e),
                        !e,
                        !l,
                        !z && n == v,
                        c,
                        n != v,
                        z,
                        n,
                        z || (!u && !e),
                        e,
                        l,
                        z || n != v};
  return rows.at(input.condition);
}

DspDebugState prepare(DspMachine &machine, const Input input) {
  const auto opcode = short_absolute | (input.condition << condition_shift) | input.target;
  if (input.high_source) {
    const std::array code{status_immediate, status | input.ccr, opcode};
    machine.bus().install_rom({.start = boot, .words = code});
  } else {
    const std::array code{status_immediate, status | input.ccr, jump_extension, ram};
    machine.bus().install_rom({.start = boot, .words = code});
    machine.bus().write({.address = ram, .value = opcode});
  }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.cpu().step();
  if (!input.high_source) { machine.cpu().step(); }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == (input.high_source ? boot + 2 : ram) &&
              before.cpu.sr == (status | input.ccr),
          "Actual own setup establishes literal source PC and complete CCR");
  return before;
}

void reference(DspMachine &machine, const Input input, const DspDebugState &before) {
  auto progress =
      machine.begin_reference_instruction(DspReferenceExecution::access_and_retirement_boundaries);
  if (!progress.pending || progress.retired) {
    throw Fault("Jcc starts at its actual native opcode admission");
  }
  const auto *opcode = std::get_if<DspNativeAccess>(&progress.pending->effect);
  require(opcode != nullptr && opcode->kind == DspNativeAccessKind::opcode &&
              opcode->target.space == DspSpace::p && opcode->target.address == before.cpu.pc &&
              opcode->ordinal == 0 && machine.cpu().debug_state() == before,
          "Exactly one owned opcode read precedes the real Jcc result");
  progress = machine.advance_reference_access(progress.pending->ticket);
  ++accesses;
  if (!progress.pending || progress.retired) {
    throw Fault("The actual one-word Jcc result remains held before publication");
  }
  const auto *retire = std::get_if<DspInstructionRetire>(&progress.pending->effect);
  const bool redirect = taken(input);
  require(retire != nullptr && retire->pc == before.cpu.pc &&
              retire->next_pc == (redirect ? input.target : before.cpu.pc + 1) &&
              retire->redirected == redirect && retire->omr == before.cpu.omr &&
              retire->next_omr == before.cpu.omr && machine.cpu().debug_state() == before,
          "Native redirect identity follows the condition even for a sequential destination");
  progress = machine.advance_reference_access(progress.pending->ticket);
  require(progress.retired && !progress.pending,
          "A distinct real retirement publishes the exact native branch result");
}

void one(const Input input) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  if (input.explicit_retirement) {
    reference(machine, input, before);
  } else {
    machine.cpu().step();
  }
  auto expected = before;
  expected.cpu.pc = taken(input) ? input.target : before.cpu.pc + 1;
  ++expected.native_instructions;
  expected.native_cycles += native_cost;
  require(machine.cpu().debug_state() == expected,
          "Independent Jcc PC/whole-register/CCR/all-stack/cost expectation failed:cc=" +
              std::to_string(input.condition) + " flags=" + hex(input.ccr, 2) +
              " target=" + hex(input.target, 6));
  ++cases;
}

unsigned seed(const unsigned condition, const bool result) {
  for (unsigned flags = 0; flags < flag_states; ++flags) {
    if (taken({.condition = condition, .ccr = flags, .target = ordinary_target}) == result) {
      return flags;
    }
  }
  throw Fault("Independent condition table must have both true and false inputs");
}
}

int main() {
  try {
    for (unsigned cc = 0; cc < conditions; ++cc) {
      for (unsigned flags = 0; flags < flag_states; ++flags) {
        for (const bool explicit_result : {false, true}) {
          one({.condition = cc,
               .ccr = flags,
               .target = ordinary_target,
               .explicit_retirement = explicit_result});
        }
      }
      for (const bool result : {false, true}) {
        for (const unsigned target : {0U, ram, ram + 1, maximum_short_target}) {
          for (const bool high : {false, true}) {
            for (const bool explicit_result : {false, true}) {
              one({.condition = cc,
                   .ccr = seed(cc, result),
                   .target = target,
                   .high_source = high,
                   .explicit_retirement = explicit_result});
            }
          }
        }
      }
    }
    std::cout
        << "PASS own short absolute Jcc:cases=" << cases << " opcode-admissions=" << accesses
        << " conditions=16 CCRs=256 absolute12,self,sequential,high-ROM-source;full-state,cost4\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
