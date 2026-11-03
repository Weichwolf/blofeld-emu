// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned program_word = 0x42, changed_word = 0x81;
constexpr unsigned unknown_address = 1, unmapped_address = 0x3000;
constexpr unsigned boot_rom = 0xff0000, data_rom = 0x4000;
constexpr unsigned switched_program = 0x1000, switched_data = 0x10000;
constexpr unsigned memory_switch = 0x80, memory_switch_width = 21;
constexpr unsigned transition_cycles = 3;

void require(const bool condition, const std::string &reason) {
  if (!condition) { throw Fault(reason); }
}

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Unsupported real execution access was accepted");
}

void known_and_unknown() {
  DspBus bus;
  const auto initial = bus.program_revision();
  require(!bus.inspect_known_program(unknown_address) &&
              !bus.inspect_known_program(unmapped_address),
          "Compiler lookahead preserves unknown and unmapped code");
  require(rejected([&] {
            static_cast<void>(bus.read({.address = unknown_address}));
          }).contains("Unknown DSP memory") &&
              rejected([&] {
                static_cast<void>(bus.read({.address = unmapped_address}));
              }).contains("Unmapped DSP memory"),
          "Lookahead absence cannot suppress genuine contextual execution refusals");
  bus.write({.value = program_word});
  const auto written = bus.program_revision();
  require(written > initial && bus.inspect_known_program(0) == program_word,
          "A genuinely initialized program word changes the visible code generation");
  bus.write({.value = program_word});
  require(bus.program_revision() == written, "An identical known code store need not invalidate");
  bus.write({.space = DspSpace::x, .address = 0, .value = changed_word});
  require(bus.program_revision() == written && bus.inspect_known_program(0) == program_word,
          "Independent data writes do not invalidate every native block");
  static_cast<void>(bus.take_code_changes());
  require(bus.inspect_known_program(0) == program_word && bus.program_revision() == written &&
              !bus.take_code_changes().all,
          "Compiler inspection neither consumes nor creates diagnostic code changes");
  constexpr std::array words{changed_word};
  bus.install_rom({.space = DspSpace::x, .start = data_rom, .words = words});
  require(bus.program_revision() == written, "Data ROM installation changes no program word");
  bus.install_rom({.start = boot_rom, .words = words});
  const auto installed = bus.program_revision();
  require(installed > written && bus.inspect_known_program(boot_rom) == changed_word,
          "Installed original or authored P ROM participates in code invalidation");
  bus.reset();
  require(bus.program_revision() > installed && bus.inspect_known_program(0) == program_word,
          "Bus reset invalidates the code view while preserving genuine RAM contents");
}

void mapping_changes(const unsigned width) {
  DspBus bus;
  bus.write({.space = DspSpace::x, .address = switched_data, .value = program_word});
  const auto initial = bus.program_revision();
  const auto omr = memory_switch | (width << memory_switch_width);
  bus.observe_omr(omr);
  const auto pending = bus.program_revision();
  require(pending > initial && !bus.inspect_known_program(switched_program),
          "A requested remap invalidates old compiled code before the transition completes");
  bus.advance_instruction_cycles(transition_cycles - 1);
  require(bus.program_revision() == pending && !bus.inspect_known_program(switched_program),
          "Lookahead cannot bypass the still-pending physical map guard");
  bus.advance_instruction_cycles(1);
  const auto mapped = bus.program_revision();
  require(mapped > pending && bus.inspect_known_program(switched_program) == program_word,
          "A completed map exposes the independently expected X-to-P physical word");
  bus.write({.space = DspSpace::x, .address = 0, .value = changed_word});
  require(bus.program_revision() == mapped, "Non-code data remains independent after remap");
  bus.observe_omr(0);
  require(bus.program_revision() > mapped && !bus.inspect_known_program(switched_program),
          "The reverse transition also invalidates cached mapped instructions immediately");
  bus.advance_instruction_cycles(transition_cycles);
  const auto returned = bus.program_revision();
  require(!bus.inspect_known_program(switched_program), "The restored default P gap is absent");
  bus.write({.space = DspSpace::x, .address = switched_data, .value = changed_word});
  require(bus.program_revision() == returned, "Inactive shared storage is currently data only");
  bus.observe_omr(omr);
  bus.advance_instruction_cycles(transition_cycles);
  require(bus.program_revision() > returned &&
              bus.inspect_known_program(switched_program) == changed_word,
          "A later remap invalidates old code and exposes the changed physical data");
}
}

int main() {
  try {
    known_and_unknown();
    for (unsigned width = 0; width < 4; ++width) { mapping_changes(width); }
    std::cout << "Pure known-code view and P/ROM/reset/request/commit revisions pass\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
