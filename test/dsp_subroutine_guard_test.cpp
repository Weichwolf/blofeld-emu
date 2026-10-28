// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned omr = 0x300;
constexpr unsigned base = boot + 4;
constexpr unsigned short_call = 0x050800;
constexpr unsigned long_call = 0x0d1080;
constexpr unsigned register_call = 0x0d1880;
constexpr unsigned rts = 0x00000c;
constexpr unsigned rti = 0x000004;
constexpr unsigned ori_ccr = 0x0055f9;
constexpr unsigned immediate_omr = 0x05f43a;
constexpr unsigned immediate_sr = 0x05f439;
constexpr unsigned do_two = 0x060280; // FM DO immediate count2.
constexpr unsigned last = base + 12;
constexpr unsigned loop_flag = 0x8000;
constexpr unsigned do_cost = 5;
constexpr unsigned internal_top = 15;
constexpr unsigned padding = 32;
constexpr unsigned bound = 64;
constexpr unsigned displacement_count = 512;
constexpr unsigned short_sign = 256;
constexpr unsigned low_bits = 31;
constexpr unsigned fetch_base = 0x100;

void fields() {
  for (unsigned raw = 0; raw < displacement_count; ++raw) {
    const auto opcode = short_call | ((raw / (low_bits + 1)) * 64) | (raw % (low_bits + 1));
    const auto expected = raw < short_sign ? raw : word_mask - (displacement_count - raw) + 1;
    const auto form = decode_branch_form(opcode);
    require(form && form->subroutine_call && !form->subroutine_return && form->words() == 1 &&
                form->displacement == expected && !form->relative_extension,
            "All512 literal FM split-field/sign expectations retain the BSR discriminator");
  }
  const auto call = decode_branch_form(long_call);
  const auto ret = decode_branch_form(rts);
  require(call && call->subroutine_call && call->relative_extension && call->words() == 2 &&
              !call->absolute_extension && ret && ret->subroutine_return && ret->words() == 1,
          "Long BSR is relative and RTS needs no fabricated extension");
}

void refused_decode(const unsigned opcode) {
  try {
    static_cast<void>(decode_pipeline_word(
        {.address = base, .value = opcode}, omr, DspFetchPipeline::automatic_memory));
  } catch (const Fault &error) {
    const std::string text = error.what();
    require(text.contains("Unqualified automatic DSP decode P:" + hex(base, 6)) &&
                text.contains("opcode=" + hex(opcode, 6)) && text.contains("pc=" + hex(base, 6)) &&
                text.contains("omr=" + hex(omr, 6)),
            "Unadmitted call/return/CCR forms retain exact Decode context");
    return;
  }
  throw Fault("Unsupported subroutine/control opcode was admitted");
}

void width_owner(const unsigned opcode, const bool two, const unsigned variant) {
  DspBus bus;
  bus.write({.address = fetch_base, .value = opcode});
  bus.write({.address = fetch_base + 1, .value = 16});
  bus.write({.address = fetch_base + 2, .value = 0});
  DspProgramFetch fetch;
  fetch.start(fetch_base, omr);
  for (unsigned index = 0; index < 3; ++index) {
    fetch.issue();
    fetch.capture(bus);
  }
  const auto actual = fetch.queued(0);
  const auto extension = fetch.queued(1);
  if (!actual || !extension) {
    throw Fault("Guard lost genuine opcode or successor F2 occurrence");
  }
  const DspBranchDecision valid{.word = actual->word,
                                .extension = two ? std::optional(extension->word) : std::nullopt,
                                .omr = omr,
                                .next_pc = fetch_base + 16,
                                .taken = true};
  auto invalid = valid;
  auto identity = actual->occurrence;
  if (variant == 0) {
    invalid.extension = two ? std::nullopt : std::optional(extension->word);
  } else if (variant == 1) {
    invalid.word.value ^= 1U;
  } else {
    ++identity;
  }
  const auto before = fetch.view();
  bool refused = false;
  try {
    fetch.predict_branch(invalid, identity);
  } catch (const Fault &error) {
    const std::string text = error.what();
    require(fetch.view() == before && text.contains("P:") && text.contains("pc=") &&
                text.contains("omr="),
            "Bad width/value/identity cannot mutate the owned queue");
    refused = true;
  }
  require(refused, "Invalid BSR/RTS fetch ownership unexpectedly succeeded");
  fetch.predict_branch(valid, actual->occurrence);
  require(fetch.view().used == (two ? 2U : 1U), "Valid actual width remains usable after refusal");
}

DspDebugState
prepare(DspMachine &machine, const std::span<const unsigned> words, const unsigned calls = 0) {
  DspProgram code;
  for (const auto word : {immediate_omr, omr, immediate_sr, masked_status}) { code.word(word); }
  for (const auto word : words) { code.word(word); }
  for (unsigned index = 0; index < padding; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, 2 + calls);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == base && before.cpu.omr == omr && before.cpu.sp == calls &&
              before.sc == calls,
          "Actual own ROM seeds and bounded genuine calls establish context");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

void fault_suffix(DspFetchRun &run,
                  DspMachine &machine,
                  const DspDebugState &expected,
                  const std::string &marker) {
  for (unsigned index = 0; index < bound; ++index) {
    try {
      static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
    } catch (const Fault &error) {
      const std::string text = error.what();
      require(text.contains(marker) && text.contains("pc=") && text.contains("omr=") &&
                  run.faulted() && machine.cpu().debug_state() == expected,
              "Refusal preserves exact literal whole CPU/counters/all frames: " + text);
      bool quarantined = false;
      try {
        static_cast<void>(run.checkpoint(machine));
      } catch (const Fault &) { quarantined = true; }
      require(quarantined, "Faulted execution cannot supply a trusted joint checkpoint");
      return;
    }
  }
  throw Fault("Expected strict subroutine context refusal did not occur");
}

void restored_fault(DspMachine &machine,
                    const DspDebugState &expected,
                    const std::string &marker,
                    const unsigned cut) {
  auto input = pipeline::contract();
  input.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(input, machine);
  pipeline::clocks(run, machine, cut);
  const auto saved = run.checkpoint(machine);
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    run.restore(saved, machine);
    fault_suffix(run, machine, expected, marker);
  }
}

void stack_underflow(const unsigned guards, const unsigned cut) {
  std::vector<unsigned> code(guards, 0);
  code.push_back(rts);
  DspMachine machine;
  auto expected = prepare(machine, code);
  // Guard AG1 is cycle guards+4; older NOP E2 begins at7,including the fault period.
  const auto retired = guards > 2 ? guards - 2 : 0;
  expected.cpu.pc += retired;
  expected.native_instructions += retired;
  expected.native_cycles += retired;
  restored_fault(machine,
                 expected,
                 "Unqualified native subroutine stack read P:" + hex(base + guards, 6),
                 cut);
}

void stack_overflow_refusal(const unsigned cut) {
  DspMachine machine;
  const std::array code{short_call}; // Actual self-BSR fills15 frames before timed admission.
  const auto expected = prepare(machine, code, internal_top);
  for (unsigned frame = 1; frame <= internal_top; ++frame) {
    require(expected.stack_high.at(frame) == base + 1 &&
                expected.stack_low.at(frame) == masked_status,
            "Every preloader frame contains literal following PC and SR");
  }
  restored_fault(
      machine, expected, "Unqualified native subroutine stack read P:" + hex(base, 6), cut);
}

void active_loop_refusal(const DspProgramWord instruction, const unsigned cut) {
  DspMachine machine;
  const std::array code{do_two, last, instruction.value, 0U, 0U};
  auto expected = prepare(machine, code);
  const auto before = expected.cpu;
  expected.cpu.pc = base + 2;
  expected.cpu.la = last;
  expected.cpu.lc = 2;
  expected.cpu.sp = 2;
  expected.sc = 2;
  expected.cpu.sr |= loop_flag;
  expected.stack_high.at(1) = before.la;
  expected.stack_low.at(1) = before.lc;
  expected.stack_high.at(2) = base + 2;
  expected.stack_low.at(2) = before.sr;
  expected.cpu.stack_pc = base + 2;
  expected.cpu.stack_sr = before.sr;
  ++expected.native_instructions;
  expected.native_cycles += do_cost;
  restored_fault(
      machine, expected, "Unqualified subroutine/active DO read P:" + hex(base + 2, 6), cut);
}
}

int main() {
  try {
    fields();
    for (unsigned reg = 0; reg < 8; ++reg) { refused_decode(register_call | (reg << 8U)); }
    for (const auto opcode : {rti, ori_ccr}) { refused_decode(opcode); }
    for (const auto opcode : {short_call, long_call, rts}) {
      for (unsigned variant = 0; variant < 3; ++variant) {
        width_owner(opcode, opcode == long_call, variant);
      }
    }
    for (const auto guards : {0U, 1U, 6U}) {
      for (const auto cut : {0U, guards + 3}) { stack_underflow(guards, cut); }
    }
    for (const auto cut : {0U, 3U}) { stack_overflow_refusal(cut); }
    for (const auto opcode : {short_call | 4U, rts}) {
      for (const auto cut : {0U, 7U}) { active_loop_refusal({.value = opcode}, cut); }
    }
    std::cout << "512 BSR fields,10 Decode refusals,9 ownership refusals,24 restored stack/DO "
                 "faults passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
