// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned profile = 0x600389;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
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

void seed(DspProgram &program, const Input input) {
  program.immediate("x1", profile);
  program.emit("move x1,omr");
  for (unsigned i = 0; i < 8; ++i) { program.emit("nop"); }
  for (const auto accumulator : {'a', 'b'}) {
    const auto value =
        (accumulator == 'b') == input.accumulator_b ? input.destination : other_accumulator;
    const std::string name(1, accumulator);
    program.immediate(name + '0', value & word_mask);
    program.immediate(name + '1', (value >> word_bits) & word_mask);
    program.immediate(name + '2', value >> data_bits);
  }
  program.immediate("sr", masked_status | input.ccr);
}

void one(const Input input) {
  DspMachine machine;
  DspProgram program;
  seed(program, input);
  const auto compare = program.pc();
  program.emit("cmp #>" + number(input.immediate) + (input.accumulator_b ? ",b" : ",a"));
  check(program.pc() == compare + 2, "Own long-immediate compare requires its actual extension");
  for (unsigned i = 0; i < budget; ++i) { program.emit("nop"); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  // Functional native setup only: SR writes retain their separate automatic
  // pipeline guard. No host register/PC assignment or seed timing claim.
  checkpoint::until(machine, compare);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun clock({.extal_hz = extal,
                     .timebase_hz = timebase,
                     .timing = DspFetchTiming::reference_cycle_order,
                     .execution = DspReferenceExecution::staged_register_boundaries,
                     .pipeline = DspFetchPipeline::automatic_memory},
                    machine);
  for (unsigned cycle = 0; cycle < budget; ++cycle) {
    const auto point =
        clock.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
    if (!point.pipeline) { throw Fault("Own compare lost its automatic pipeline"); }
    if (!point.pipeline->execute1 || point.pipeline->execute1->address != compare) { continue; }
    check(point.pipeline->execution_captured && point.pipeline->e1_extension &&
              point.pipeline->e1_extension->address == compare + 1 &&
              point.pipeline->e1_extension->value == input.immediate,
          "CMP E1 must consume its own genuine adjacent F2 extension");
    const auto before = machine.cpu().state();
    check(before.pc == compare && before.sr == (masked_status | input.ccr),
          "The native CMP candidate cannot publish CCR or PC before E2");
    auto expected = before;
    expected.pc = compare + 2;
    expected.sr = masked_status | flags(input);
    const auto saved = clock.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      if (replay != 0) {
        clock.restore(saved, machine);
        ++restored;
      }
      const auto retirement =
          clock.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
      check(retirement.pipeline && retirement.pipeline->retirement_published &&
                retirement.pipeline->execute2 && retirement.pipeline->execute2->address == compare,
            "The real CMP candidate must retire at its owned E2");
      check(machine.cpu().state() == expected,
            "CMP must publish independent signed56 subtraction flags and preserve all other state");
    }
    return;
  }
  throw Fault("Own long-immediate CMP did not reach bounded actual E1");
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
    std::cout << "signed56-cases=256 restored-E2-suffixes=" << restored
              << " assertions=" << assertions << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
