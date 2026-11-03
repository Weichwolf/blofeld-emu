// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_norm_expectations.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned profile = 0x600389;
namespace native = dma_test;
using Program = native::NativeProgram;
constexpr unsigned budget = 128;
constexpr unsigned initial_flags = sticky | limit | carry | overflow | zero;
constexpr unsigned fraction_top = 47;
constexpr unsigned fraction_next = 46;
constexpr std::uint64_t mask56 = 0xffffffffffffff;
constexpr std::uint64_t mask9 = 0x1ff;
constexpr std::uint64_t positive_limit = 0x7fffffffffffff;
constexpr std::uint64_t extension_ones = 0xff000000000000;
constexpr std::array sources{0U, 1U, 0x3fU, 0xc0b0U, 0x400000U, 0x7fffffU, 0x800000U, 0xffffffU};
constexpr std::array destinations{std::uint64_t{0},
                                  std::uint64_t{1},
                                  std::uint64_t{0x1000000},
                                  std::uint64_t{0x400000000000},
                                  std::uint64_t{0xff800000000000},
                                  positive_limit,
                                  std::uint64_t{0x80000000000000},
                                  mask56};
unsigned assertions = 0;
unsigned restored = 0;
unsigned suffixes = 0;

void check(const bool condition, const std::string &message) {
  ++assertions;
  if (!condition) { throw Fault(message); }
}

struct Input {
  std::uint64_t destination = 0;
  unsigned immediate = 0;
  bool accumulator_b = false;
  unsigned ccr = 0;
};

// FM13-45/46 and Table5-1: signed 24-bit source, zero-filled low word;
// subtract in 56 bits, publish CCR only and retain both accumulators.
unsigned flags(const Input input) {
  const std::uint64_t source = (static_cast<std::uint64_t>(input.immediate) << word_bits) |
                               ((input.immediate & 0x800000U) != 0 ? extension_ones : 0);
  // Adding 2^56 before subtracting is bounded below 2^57 and cannot underflow.
  const auto difference = (input.destination + mask56 + 1 - source) & mask56;
  unsigned result = input.ccr & (sticky | limit);
  if (source > input.destination) { result |= carry; }
  if (((input.destination ^ source) & (input.destination ^ difference) & sign_bit) != 0) {
    result |= overflow | limit;
  }
  if (difference == 0) { result |= zero; }
  if ((difference & sign_bit) != 0) { result |= negative; }
  if (((difference >> fraction_top) & 1U) == ((difference >> fraction_next) & 1U)) {
    result |= unnormalized;
  }
  const auto integer = difference >> fraction_top;
  if (integer != 0 && integer != mask9) { result |= extension; }
  return result;
}

// FM13-118/131 and CMP13-46, authored literals rather than runtime assembly.
constexpr unsigned move_x1 = 0x45f400;
constexpr unsigned x1_to_omr = 0x04c5ba;
constexpr unsigned move_sr = 0x05f439;
constexpr unsigned compare_immediate = 0x0140c5;
constexpr std::array a_seeds{0x50f400U, 0x54f400U, 0x52f400U};
constexpr std::array b_seeds{0x51f400U, 0x55f400U, 0x53f400U};

void immediate(Program &program, const unsigned opcode, const unsigned value) {
  program.word(opcode);
  program.word(value);
}

void seed(Program &program, const Input input) {
  immediate(program, move_x1, profile);
  program.word(x1_to_omr);
  for (unsigned i = 0; i < 8; ++i) { program.word(0); }
  for (const bool accumulator_b : {false, true}) {
    const auto value = accumulator_b == input.accumulator_b ? input.destination : other_accumulator;
    const auto &seeds = accumulator_b ? b_seeds : a_seeds;
    immediate(program, seeds[0], static_cast<unsigned>(value & word_mask));
    immediate(program, seeds[1], static_cast<unsigned>((value >> word_bits) & word_mask));
    immediate(program, seeds[2], static_cast<unsigned>(value >> data_bits));
  }
  immediate(program, move_sr, masked_status | input.ccr);
}

struct Observation {
  dsp56300::Registers expected;
  dsp56300::Progress progress;
  std::uint64_t ticks = 0;
};

dsp56300::PendingInstruction pending(const OwnDspMachine &machine) {
  const auto value = machine.cpu().pending_instruction();
  if (!value) { throw Fault("Native CMP lost its actual pending publication"); }
  return *value;
}

void restore(OwnDspMachine &machine,
             std::unique_ptr<OwnDspFetch> &fetch,
             const OwnDspFetchCheckpoint &saved) {
  const auto stale = pending(machine).ticket;
  fetch = std::make_unique<OwnDspFetch>(machine, saved);
  const auto registers = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  bool refused = false;
  try {
    static_cast<void>(machine.advance(stale));
  } catch (const std::logic_error &) { refused = true; }
  check(refused && machine.cpu().registers() == registers && machine.cpu().progress() == progress,
        "A restored CMP continuation rejects its previous effect capability without mutation");
  ++restored;
}

void suffix(OwnDspMachine &machine, OwnDspFetch &fetch, const Observation &observation) {
  for (unsigned count = 0; count < budget; ++count) {
    const auto result = machine.advance(pending(machine).ticket);
    check(result.fault.kind == dsp56300::FaultKind::none,
          "Native CMP continuation must retain all admitted effects");
    if (!result.retired) { continue; }
    fetch.synchronize();
    check(
        machine.cpu().registers() == observation.expected &&
            machine.cpu().progress().instructions == observation.progress.instructions + 1 &&
            machine.clock().elapsed_ticks() == observation.ticks,
        "CMP publishes independent signed56 flags, exact PC and complete unchanged operands/stack");
    ++suffixes;
    return;
  }
  throw Fault("Native CMP publication exceeded its bounded actual effects");
}

void replay(OwnDspMachine &machine,
            std::unique_ptr<OwnDspFetch> &fetch,
            const OwnDspFetchCheckpoint &saved,
            const Observation &observation) {
  suffix(machine, *fetch, observation);
  // The retired core has no pending ticket. The restore helper intentionally
  // takes a ticket from the exact suspended state, then restores again to expire it.
  fetch = std::make_unique<OwnDspFetch>(machine, saved);
  restore(machine, fetch, saved);
  suffix(machine, *fetch, observation);
}

void publications(OwnDspMachine &machine, std::unique_ptr<OwnDspFetch> &fetch, const Input input) {
  const auto before = machine.cpu().registers();
  check(before.a == (input.accumulator_b ? other_accumulator : input.destination) &&
            before.b == (input.accumulator_b ? input.destination : other_accumulator) &&
            before.sr == (masked_status | input.ccr) && before.omr == profile,
        "Actual native literal setup supplies both full accumulators, modes and incoming flags");
  Observation observation{.expected = before,
                          .progress = machine.cpu().progress(),
                          .ticks = machine.clock().elapsed_ticks()};
  observation.expected.pc += 2;
  observation.expected.sr = masked_status | flags(input);
  const auto start = fetch->begin();
  check(start.fault.kind == dsp56300::FaultKind::none && !start.retired &&
            pending(machine).effect.kind == dsp56300::EffectKind::alu_update &&
            machine.cpu().registers() == before && machine.cpu().progress() == observation.progress,
        "The captured CMP has a real unpublished ALU effect and unchanged registers/CCR/PC");
  const auto saved = fetch->checkpoint();
  replay(machine, fetch, saved, observation);
  fetch = std::make_unique<OwnDspFetch>(machine, saved);
  const auto published = machine.advance(pending(machine).ticket);
  auto expected = before;
  expected.sr = observation.expected.sr;
  check(
      published.fault.kind == dsp56300::FaultKind::none && !published.retired &&
          machine.cpu().registers() == expected &&
          pending(machine).effect.kind == dsp56300::EffectKind::pcu_update &&
          machine.cpu().progress() == observation.progress,
      "CMP ALU publication changes only qualified status bits before its PCU publication: "
      "pending=" +
          std::to_string(static_cast<unsigned>(pending(machine).effect.kind)) + " PC=" +
          number(machine.cpu().registers().pc) + " SR=" + number(machine.cpu().registers().sr) +
          " expectedSR=" + number(expected.sr) + " exactRegisters=" +
          std::to_string(static_cast<unsigned>(machine.cpu().registers() == expected)) +
          " exactProgress=" +
          std::to_string(static_cast<unsigned>(machine.cpu().progress() == observation.progress)));
  const auto after_alu = fetch->checkpoint();
  replay(machine, fetch, after_alu, observation);
  fetch = std::make_unique<OwnDspFetch>(machine, after_alu);
  const auto pcu = machine.advance(pending(machine).ticket);
  check(
      pcu.fault.kind == dsp56300::FaultKind::none && !pcu.retired &&
          machine.cpu().registers() == observation.expected &&
          pending(machine).effect.kind == dsp56300::EffectKind::retire &&
          machine.cpu().progress() == observation.progress,
      "CMP PCU publication changes the exact PC while final instruction retirement stays pending");
  const auto before_retirement = fetch->checkpoint();
  replay(machine, fetch, before_retirement, observation);
}

void one(const Input input) {
  OwnDspMachine machine(native::native_clocks);
  Program program;
  seed(program, input);
  const auto compare = program.pc();
  program.word(compare_immediate | (input.accumulator_b ? 8U : 0U));
  program.word(input.immediate);
  check(program.pc() == compare + 2,
        "Native long-immediate CMP owns its literal adjacent extension");
  native::reset_native(machine, program);
  for (unsigned count = 0; machine.cpu().registers().pc != compare; ++count) {
    check(count < budget, "Native CMP setup stays within its declared instruction bound");
    native::step_native(machine);
  }
  auto fetch = std::make_unique<OwnDspFetch>(machine);
  for (unsigned i = 0; i < 2; ++i) {
    fetch->issue();
    fetch->capture();
  }
  const auto captured = fetch->view();
  check(captured.used == 2 &&
            captured.words[1] == DspProgramWord{.address = compare + 1, .value = input.immediate},
        "CMP consumes its genuine F2 immediate sample at its actual adjacent program address");
  const std::array changed{input.immediate ^ word_mask};
  machine.bus().install_rom({.start = compare + 1, .words = changed});
  check(machine.bus().read({.address = compare + 1}) == changed[0],
        "Later program backing changes independently of the already captured extension");
  publications(machine, fetch, input);
}

}

int main() {
  try {
    for (const auto destination : destinations) {
      for (const auto immediate : sources) {
        for (const bool accumulator_b : {false, true}) {
          for (const unsigned ccr : {0U, initial_flags}) {
            one({.destination = destination,
                 .immediate = immediate,
                 .accumulator_b = accumulator_b,
                 .ccr = ccr});
          }
        }
      }
    }
    std::cout << "signed56-cases=256 publication-suffixes=" << suffixes
              << " restored-capabilities=" << restored << " assertions=" << assertions << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
