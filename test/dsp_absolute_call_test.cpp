// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned start = 0x3fa;
constexpr unsigned program_base = 0;
constexpr unsigned reference_omr = 0x300;
constexpr unsigned switched_omr = 0x380, reset_program_end = 0x1000;
constexpr unsigned immediate_omr = 0x05f43a;
constexpr unsigned seed_steps = 23;
constexpr unsigned register_seed_steps = 15, register_seed_words = 2 * register_seed_steps;
constexpr unsigned caller = 0x400;
constexpr unsigned default_target = 0x440;
constexpr unsigned inner = 0x3c0;
constexpr unsigned return_opcode = 0x00000c; // FM13-168 RTS.
constexpr unsigned andi_ccr = 0x0055b9;      // FM ANDI #$55,CCR, EE=1.
constexpr unsigned first_e2 = 7;
constexpr unsigned tail_words = 7;
constexpr unsigned short_call_base = 0x0d0000; // FM13-89: zero-extended absolute12 JSR.
constexpr unsigned long_call_opcode = 0x0bf080;
constexpr unsigned register_call_opcode = 0x0be080;
constexpr unsigned immediate_r0 = 0x60f400, register_shift = 16;
constexpr unsigned register_field_shift = 8, register_count = 8;
constexpr unsigned illegal_word = 7;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned maximum_short_target = 0xfff;
constexpr unsigned wide_target = 0x2400, rom_target = boot + 0x100;
constexpr unsigned condition_mask = 0xff;
constexpr unsigned condition_operand = 0x55;

// Independently encoded MOVEC SR/LA/LC and MOVE long X0/X1/Y0/Y1 seeds.
constexpr std::array seed_words{0x05f439U,
                                0xc00300U,
                                0x05f43eU,
                                0x55aa33U,
                                0x05f43fU,
                                0x33U,
                                0x44f400U,
                                0x112233U,
                                0x45f400U,
                                0x445566U,
                                0x46f400U,
                                0x778899U,
                                0x47f400U,
                                0xaabbccU};

enum class Kind : std::uint8_t { nested, sequential, maximum, minimum, andi, wide, rom };
enum class Form : std::uint8_t { short_absolute, extension, address_register };
enum class Effect : std::uint8_t { nop, call, ret, flags };

struct Instruction {
  unsigned pc;
  unsigned opcode;
  unsigned words = 1;
  unsigned cost = 1;
  Effect effect = Effect::nop;
  unsigned target = 0;
};

struct Program {
  std::vector<std::uint32_t> words = std::vector<std::uint32_t>(0x430, 0);
  std::vector<Instruction> path;
  std::vector<unsigned> preamble;
  unsigned omr = reference_omr;
  std::vector<unsigned> rom = std::vector<unsigned>(tail_words + 2, 0);
};

struct CallTarget {
  unsigned address, reg;
};

unsigned encoded(const Form form, const CallTarget target) {
  if (form == Form::extension) { return long_call_opcode; }
  if (form == Form::address_register) {
    return register_call_opcode | (target.reg << register_field_shift);
  }
  return short_call_base | target.address;
}

void append(Program &code, const Instruction instruction) {
  if (instruction.pc >= rom_target) {
    code.rom.at(instruction.pc - rom_target) = instruction.opcode;
  } else {
    code.words.at(instruction.pc - program_base) = instruction.opcode;
  }
  code.path.push_back(instruction);
}

Program program(const Kind kind, const unsigned flags, const Form form, const unsigned reg) {
  Program code;
  // The max12 target needs real mapped speculative successors beyond P:fff.
  // Wide RAM likewise requires the documented MS=1/MSW=00 partition.
  if (kind == Kind::maximum || kind == Kind::wide) { code.omr = switched_omr; }
  std::vector<unsigned> seeds(seed_words.begin(), seed_words.end());
  seeds.at(1) |= flags;
  unsigned target = default_target;
  if (kind == Kind::maximum) { target = maximum_short_target; }
  if (kind == Kind::minimum) { target = 0; }
  if (kind == Kind::wide) { target = wide_target; }
  if (kind == Kind::rom) { target = rom_target; }
  if (target < rom_target && target + tail_words > code.words.size()) {
    code.words.resize(target + tail_words, 0);
  }
  for (unsigned index = 0; index < register_count; ++index) {
    seeds.push_back(immediate_r0 | (index << register_shift));
    seeds.push_back(index == reg ? target : inner);
  }
  code.preamble = seeds;
  code.preamble.insert(code.preamble.end(), 6, 0);
  code.preamble.push_back(absolute_jump);
  code.preamble.push_back(start);
  for (unsigned pc = start; pc < caller; ++pc) { append(code, {.pc = pc, .opcode = 0}); }
  const bool extended = form == Form::extension;
  const unsigned width = extended ? 2U : 1U;
  const unsigned cost = extended ? 4U : 3U; // FM A-1: T3 + long-absolute1.
  const unsigned return_pc = caller + width;
  append(code,
         {.pc = caller,
          .opcode = encoded(form, {.address = target, .reg = reg}),
          .words = width,
          .cost = cost,
          .effect = Effect::call,
          .target = target});
  if (extended) { code.words.at(caller + 1 - program_base) = target; }
  const unsigned nested_register = (reg + 1) % register_count;
  if (kind == Kind::nested) {
    append(code,
           {.pc = target,
            .opcode = encoded(form, {.address = inner, .reg = nested_register}),
            .words = width,
            .cost = cost,
            .effect = Effect::call,
            .target = inner});
    if (extended) { code.words.at(target + 1 - program_base) = inner; }
    append(code, {.pc = inner, .opcode = return_opcode, .cost = 3, .effect = Effect::ret});
    append(code, {.pc = target + width, .opcode = return_opcode, .cost = 3, .effect = Effect::ret});
  } else {
    if (kind == Kind::andi) {
      append(code, {.pc = target, .opcode = andi_ccr, .cost = 3, .effect = Effect::flags});
      ++target;
    }
    append(code, {.pc = target, .opcode = return_opcode, .cost = 3, .effect = Effect::ret});
  }
  unsigned tail = return_pc;
  if (kind == Kind::sequential) {
    append(code,
           {.pc = tail,
            .opcode = encoded(form, {.address = inner, .reg = nested_register}),
            .words = width,
            .cost = cost,
            .effect = Effect::call,
            .target = inner});
    append(code, {.pc = inner, .opcode = return_opcode, .cost = 3, .effect = Effect::ret});
    if (extended) { code.words.at(tail + 1 - program_base) = inner; }
    tail += width;
  }
  for (unsigned pc = tail; pc < tail + tail_words; ++pc) { append(code, {.pc = pc, .opcode = 0}); }
  return code;
}

void install(DspMachine &machine, const Program &code) {
  DspProgram image;
  // LA/LC and ordinary registers are actual native component preconditions.
  // The subsequent OMR write, waits and branch use one continuous clocked owner.
  for (unsigned index = 0; index < register_seed_words; ++index) {
    image.word(code.preamble.at(index));
  }
  image.word(immediate_omr);
  image.word(code.omr);
  for (std::size_t index = register_seed_words; index < code.preamble.size(); ++index) {
    image.word(code.preamble.at(index));
  }
  image.install(machine.bus());
  machine.bus().install_rom({.start = rom_target, .words = code.rom});
  for (std::size_t index = 0; index < code.words.size() && index < reset_program_end; ++index) {
    machine.bus().write(
        {.address = program_base + static_cast<unsigned>(index), .value = code.words.at(index)});
  }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, register_seed_steps);
  require(machine.cpu().state().pc == boot + register_seed_words &&
              machine.cpu().debug_state().native_instructions == register_seed_steps,
          "Actual native seed instructions stop before OMR without invented register/PC state");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
}

DspDebugState prepare(DspFetchRun &run, DspMachine &machine, const Program &code) {
  const auto jump_address = boot + static_cast<unsigned>(code.preamble.size());
  constexpr unsigned seed_budget = 256;
  bool retired = false;
  for (unsigned clock = 0; clock < seed_budget; ++clock) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = pipeline::stages(cycle);
    if (phase.retirement_published && phase.execute2 && phase.execute2->address == jump_address) {
      retired = true;
      break;
    }
  }
  require(retired && machine.bus().stable_program_context(wide_target, code.omr),
          "Actual native OMR retirement and following pipeline periods complete memory switching");
  for (std::size_t index = reset_program_end; index < code.words.size(); ++index) {
    machine.bus().write(
        {.address = program_base + static_cast<unsigned>(index), .value = code.words.at(index)});
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == start && before.cpu.omr == code.omr && before.cpu.sp == 0 &&
              before.sc == 0 && before.native_instructions == seed_steps,
          "The same live clocked owner reaches the actual own JMP retirement without skipped seed "
          "instructions");
  return before;
}

void retire(DspDebugState &expected, const Instruction instruction) {
  require(expected.cpu.pc == instruction.pc, "Literal own control path owns the next native E2");
  auto &cpu = expected.cpu;
  const auto next = instruction.pc + instruction.words;
  if (instruction.effect == Effect::call) {
    ++cpu.sp;
    ++expected.sc;
    expected.stack_high.at(cpu.sp) = next;
    expected.stack_low.at(cpu.sp) = cpu.sr;
    cpu.pc = instruction.target;
  } else if (instruction.effect == Effect::ret) {
    cpu.pc = expected.stack_high.at(cpu.sp);
    --cpu.sp;
    --expected.sc;
  } else {
    cpu.pc = next;
    if (instruction.effect == Effect::flags) {
      cpu.sr = (cpu.sr & ~condition_mask) | (cpu.sr & condition_operand);
    }
  }
  cpu.stack_pc = expected.stack_high.at(cpu.sp);
  cpu.stack_sr = expected.stack_low.at(cpu.sp);
  ++expected.native_instructions;
  expected.native_cycles += instruction.cost;
}

struct Oracle {
  DspDebugState expected;
  std::size_t retired = 0;
  unsigned next_e2 = first_e2;
};

void step(DspFetchRun &run, DspMachine &machine, const Program &code, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const bool due = cycle.completed_cycles == oracle.next_e2 && oracle.retired < code.path.size();
  require(phase.retirement_published == due,
          "Fixed selected E2 periods match independently qualified FM absolute JSR3/4 and RTS3 "
          "windows");
  if (due) {
    const auto instruction = code.path.at(oracle.retired);
    require(phase.execute2 && phase.execute2->address == instruction.pc &&
                phase.execute2->value == instruction.opcode,
            "Native retirement consumes the actual instruction on the literal call/return path");
    retire(oracle.expected, instruction);
    oracle.next_e2 += instruction.cost;
    ++oracle.retired;
  }
  if (phase.decode_accepted && phase.decode) {
    for (const auto instruction : code.path) {
      if (instruction.effect != Effect::call || instruction.pc != phase.decode->address) {
        continue;
      }
      // Only an accepted occurrence owns its actual F2 pair. Discarded
      // speculative words must remain intact for a later return/refetch.
      for (unsigned offset = 0; offset < instruction.words; ++offset) {
        machine.bus().write({.address = instruction.pc + offset, .value = illegal_word});
      }
    }
  }
  require(
      machine.cpu().debug_state() == oracle.expected,
      "Every period preserves literal whole CPU,counters and all16 retained stack slots at pc=" +
          hex(machine.cpu().state().pc, 6) + " period=" + hex(cycle.completed_cycles, 4));
}

unsigned streams = 0, restores = 0, clones = 0;

void finish(DspFetchRun &run, DspMachine &machine, const Program &code, Oracle &oracle) {
  while (oracle.retired < code.path.size()) { step(run, machine, code, oracle); }
  require(machine.cpu().state().sp == 0 && machine.cpu().debug_state().sc == 0,
          "Actual nested/sequential returns balance the original native stack");
}

void one_case(const Kind kind,
              const unsigned flags,
              const Form form,
              const unsigned reg,
              const bool snapshots) {
  const auto code = program(kind, flags, form, reg);
  DspMachine machine;
  install(machine, code);
  auto contract = pipeline::contract();
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  const auto before = prepare(run, machine, code);
  // FM A-1: the initial absolute JMP occupies3+lab1 periods before target E2.
  constexpr unsigned absolute_jump_cost = 4;
  Oracle oracle{.expected = before,
                .next_e2 = static_cast<unsigned>(run.completed_cycles()) + absolute_jump_cost};

  struct Cut {
    DspFetchCheckpoint checkpoint;
    Oracle oracle;
  };

  std::vector<Cut> cuts;
  while (oracle.retired < code.path.size()) {
    if (snapshots) { cuts.push_back({.checkpoint = run.checkpoint(machine), .oracle = oracle}); }
    step(run, machine, code, oracle);
  }
  for (const auto &cut : cuts) {
    for (unsigned suffix = 0; suffix < 2; ++suffix) {
      run.restore(cut.checkpoint, machine);
      oracle = cut.oracle;
      finish(run, machine, code, oracle);
      ++restores;
    }
    DspMachine target;
    DspFetchRun clone(cut.checkpoint, target);
    auto expected = cut.oracle;
    finish(clone, target, code, expected);
    require(target.cpu().debug_state() == machine.cpu().debug_state() &&
                clone.elapsed_ticks() == run.elapsed_ticks(),
            "Fresh ownership preserves every literal stack slot and the full clock suffix");
    ++clones;
  }
  ++streams;
}
}

int main() {
  try {
    for (const auto form : {Form::short_absolute, Form::extension, Form::address_register}) {
      for (unsigned reg = 0; reg < (form == Form::address_register ? register_count : 1U); ++reg) {
        for (const auto kind : {Kind::nested,
                                Kind::sequential,
                                Kind::maximum,
                                Kind::minimum,
                                Kind::andi,
                                Kind::wide,
                                Kind::rom}) {
          if (form == Form::short_absolute && (kind == Kind::wide || kind == Kind::rom)) {
            continue;
          }
          for (const unsigned flags : {0U, 0xaaU}) {
            one_case(kind, flags, form, reg, kind == Kind::nested && flags != 0);
          }
        }
      }
    }
    std::cout << "PASS ABSOLUTE_CALL_CLOCKED streams=" << streams << " restores=" << restores
              << " clones=" << clones << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
