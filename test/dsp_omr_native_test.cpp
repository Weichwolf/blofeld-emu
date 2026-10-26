// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_omr_move.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <iostream>
#include <variant>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned switched_program = 0x1000;
constexpr unsigned switched_x = 0x10000;
constexpr unsigned canary = 0x654321;
constexpr unsigned register_form = 0x04c0ba;
constexpr unsigned short_form = 0x0500ba;
constexpr unsigned long_form = 0x05f43a;
constexpr unsigned reverse_form = 0x0447ba;
constexpr unsigned modifier_switch = 0x80;
constexpr unsigned mode_count = 3;
constexpr std::array values{0x305U, 0x389U, 0x200389U, 0x400389U, 0x600389U};

struct Source {
  unsigned field;
  unsigned seed;
};

// Literal fields and seeds independently emitted by the official assembler.
constexpr std::array sources{Source{.field = 4, .seed = 0x44f400},
                             Source{.field = 5, .seed = 0x45f400},
                             Source{.field = 6, .seed = 0x46f400},
                             Source{.field = 7, .seed = 0x47f400},
                             Source{.field = 16, .seed = 0x60f400},
                             Source{.field = 17, .seed = 0x61f400},
                             Source{.field = 18, .seed = 0x62f400},
                             Source{.field = 19, .seed = 0x63f400},
                             Source{.field = 20, .seed = 0x64f400},
                             Source{.field = 21, .seed = 0x65f400},
                             Source{.field = 22, .seed = 0x66f400},
                             Source{.field = 23, .seed = 0x67f400}};

void fields() {
  constexpr unsigned field_count = 64;
  constexpr unsigned immediate_count = 256;
  for (unsigned field = 0; field < field_count; ++field) {
    const bool allowed = std::ranges::any_of(
        sources, [field](const Source source) { return source.field == field; });
    const auto form = decode_omr_move(register_form | (field << 8U));
    require(form.has_value() == allowed && (!form || form->words == 1),
            "All manual source fields retain their independent acceptance and width");
  }
  for (unsigned value = 0; value < immediate_count; ++value) {
    const auto form = decode_omr_move(short_form | (value << 8U));
    require(form && form->words == 1, "Control immediates own one real opcode word");
  }
  const auto form = decode_omr_move(long_form);
  require(form && form->words == 2, "Long control immediate owns its genuine extension");
  require(!decode_omr_move(0) && !decode_omr_move(reverse_form) && !decode_omr_move(0x1000000),
          "NOP, reverse control read and over-wide inputs are not fabricated OMR writes");
}

void before_switch(DspMachine &machine) {
  require(machine.bus().read({.space = DspSpace::x, .address = switched_x}) == canary,
          "Real original X RAM retains the independent physical canary");
  static_cast<void>(native_access::fault(
      [&] { static_cast<void>(machine.bus().read({.address = switched_program})); }));
}

void switch_delays(DspMachine &machine, const bool switching) {
  if (!switching) {
    before_switch(machine);
    return;
  }
  for (unsigned remaining = mode_count; remaining != 0; --remaining) {
    static_cast<void>(native_access::fault(
        [&] { static_cast<void>(machine.bus().read({.address = switched_program})); }));
    machine.bus().advance_instruction_cycles(1);
  }
  require(machine.bus().read({.address = switched_program}) == canary,
          "Only the third actual delay interval exposes preserved X RAM at its P alias");
  static_cast<void>(native_access::fault([&] {
    static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = switched_x}));
  }));
}

struct Input {
  Source source;
  unsigned value;
  unsigned opcode;
  bool extension = false;
};

void one_case(const Input input) {
  DspMachine machine;
  DspProgram code;
  code.word(input.source.seed);
  code.word(input.value);
  const auto pc = code.pc();
  code.word(input.opcode);
  if (input.extension) { code.word(input.value); }
  code.word(0);
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = switched_x, .value = canary});
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, 1);
  const auto before = machine.cpu().debug_state();
  before_switch(machine);
  auto progress =
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries);
  for (unsigned index = 0; index < (input.extension ? 2U : 1U); ++index) {
    const auto &access = reference::request(progress);
    require(access.kind ==
                    (index == 0 ? DspNativeAccessKind::opcode : DspNativeAccessKind::extension) &&
                access.target.address == pc + index && access.omr == before.cpu.omr,
            "Only the genuine captured opcode/extension precedes the native OMR result");
    require(machine.cpu().debug_state() == before,
            "The held control transfer cannot publish speculative CPU/register/frame state");
    progress = machine.advance_reference_access(reference::ticket(progress));
  }
  if (!progress.pending) { throw Fault("OMR write lost its actual held native result"); }
  const auto *retirement = std::get_if<DspInstructionRetire>(&progress.pending->effect);
  const unsigned words = input.extension ? 2U : 1U;
  require(retirement != nullptr && retirement->pc == pc && retirement->next_pc == pc + words &&
              retirement->omr == before.cpu.omr && retirement->next_omr == input.value &&
              !retirement->redirected && machine.cpu().debug_state() == before,
          "The full real native control result remains distinct from ordinary move writeback");
  before_switch(machine);
  const auto saved = machine.checkpoint();
  const auto discarded = reference::ticket(progress);
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    static_cast<void>(native_access::fault(
        [&] { static_cast<void>(machine.advance_reference_access(discarded)); }));
    require(machine.cpu().debug_state() == before, "Restore preserves the unretired native result");
    before_switch(machine);
    progress = machine.advance_reference_access(reference::ticket(machine));
    auto expected = before;
    expected.cpu.pc = pc + words;
    expected.cpu.omr = input.value;
    ++expected.native_instructions;
    expected.native_cycles += words;
    require(progress.retired && !progress.pending && progress.completed &&
                std::holds_alternative<DspInstructionRetire>(*progress.completed) &&
                machine.cpu().debug_state() == expected,
            "Actual native retirement publishes only literal PC/OMR/count effects exactly once");
    switch_delays(machine, (input.value & modifier_switch) != 0);
  }
}
}

int main() {
  try {
    fields();
    for (const auto source : sources) {
      for (const auto value : values) {
        one_case(
            {.source = source, .value = value, .opcode = register_form | (source.field << 8U)});
      }
    }
    for (const auto value : values) {
      one_case({.source = sources[0], .value = value, .opcode = long_form, .extension = true});
    }
    constexpr unsigned short_value = 0x89;
    one_case(
        {.source = sources[0], .value = short_value, .opcode = short_form | (short_value << 8U)});
    std::cout << "66 native deferred OMR cases,132 trusted retirements and independent three-delay "
                 "aliases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
