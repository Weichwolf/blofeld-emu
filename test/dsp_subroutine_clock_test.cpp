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
#include <optional>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned start = 0x3fa;
constexpr unsigned program_base = 0x100;
constexpr unsigned reference_omr = 0x300;
constexpr unsigned immediate_omr = 0x05f43a;
constexpr unsigned preamble_words = 22;
constexpr unsigned caller = 0x400;
constexpr unsigned default_target = 0x440;
constexpr unsigned inner = 0x3c0;
constexpr unsigned return_opcode = 0x00000c; // FM13-168 RTS.
constexpr unsigned andi_ccr = 0x0055b9;      // FM ANDI #$55,CCR, EE=1.
constexpr unsigned first_e2 = 7;
constexpr unsigned tail_words = 7;
constexpr unsigned short_call_base = 0x050800; // FM13-37 split signed9 BSR.
constexpr unsigned long_call_opcode = 0x0d1080;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned short_low_mask = 0x1f;
constexpr unsigned short_positive_limit = 255;
constexpr unsigned long_negative_displacement = 0xffff00; // Signed24 -256.
constexpr unsigned condition_mask = 0xff;
constexpr unsigned condition_operand = 0x55;

enum class Kind : std::uint8_t { nested, sequential, maximum, minimum, long_backward, andi };
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
};

unsigned short_call(const unsigned pc, const unsigned target) {
  // FM13-37: signed9 bits split aaaa/aaaaa; independently vendor-assembled.
  const auto displacement = (target + 0x200U - pc) & 0x1ffU;
  return short_call_base | ((displacement >> 5U) << 6U) | (displacement & short_low_mask);
}

void append(Program &code, const Instruction instruction) {
  code.words.at(instruction.pc - program_base) = instruction.opcode;
  code.path.push_back(instruction);
}

Program program(const Kind kind, const unsigned flags) {
  Program code;
  const std::array seeds{0x05f439U,
                         0xc00300U | flags,
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
  for (std::size_t index = 0; index < seeds.size(); ++index) {
    code.words.at(index) = seeds[index];
  }
  code.words.at(seeds.size() + 6) = absolute_jump; // Actual JMP after six seed guards.
  code.words.at(seeds.size() + 7) = start;
  for (unsigned pc = start; pc < caller; ++pc) { append(code, {.pc = pc, .opcode = 0}); }
  const bool long_call = kind == Kind::long_backward;
  unsigned target = default_target;
  if (kind == Kind::maximum) { target = caller + short_positive_limit; }
  if (kind == Kind::minimum || long_call) { target = caller - 256; }
  const unsigned return_pc = caller + (long_call ? 2U : 1U);
  append(code,
         {.pc = caller,
          .opcode = long_call ? long_call_opcode : short_call(caller, target),
          .words = long_call ? 2U : 1U,
          .cost = long_call ? 5U : 4U,
          .effect = Effect::call,
          .target = target});
  if (long_call) { code.words.at(caller + 1 - program_base) = long_negative_displacement; }
  if (kind == Kind::nested) {
    append(code,
           {.pc = target,
            .opcode = short_call(target, inner),
            .cost = 4,
            .effect = Effect::call,
            .target = inner});
    append(code, {.pc = inner, .opcode = return_opcode, .cost = 3, .effect = Effect::ret});
    append(code, {.pc = target + 1, .opcode = return_opcode, .cost = 3, .effect = Effect::ret});
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
            .opcode = short_call(tail, inner),
            .cost = 4,
            .effect = Effect::call,
            .target = inner});
    append(code, {.pc = inner, .opcode = return_opcode, .cost = 3, .effect = Effect::ret});
    ++tail;
  }
  for (unsigned pc = tail; pc < tail + tail_words; ++pc) { append(code, {.pc = pc, .opcode = 0}); }
  return code;
}

DspDebugState prepare(DspMachine &machine, const Program &code) {
  DspProgram image;
  // Explicit own reset preamble reaches the reference RAM program by actual JMP.
  image.word(immediate_omr);
  image.word(reference_omr);
  for (unsigned index = 0; index < preamble_words; ++index) { image.word(code.words.at(index)); }
  image.install(machine.bus());
  for (std::size_t index = 0; index < code.words.size(); ++index) {
    machine.bus().write(
        {.address = program_base + static_cast<unsigned>(index), .value = code.words.at(index)});
  }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, 15); // Actual OMR/seven seeds,six NOPs,the real JMP.
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == start && before.cpu.omr == reference_omr && before.cpu.sp == 0 &&
              before.sc == 0,
          "Own native preamble reaches the empty-stack caller guards");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
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
          "Fixed selected E2 periods match independently qualified joint 3/4/5-period windows");
  if (due) {
    const auto instruction = code.path.at(oracle.retired);
    require(phase.execute2 && phase.execute2->address == instruction.pc &&
                phase.execute2->value == instruction.opcode,
            "Native retirement consumes the actual instruction on the literal call/return path");
    retire(oracle.expected, instruction);
    oracle.next_e2 += instruction.cost;
    ++oracle.retired;
  }
  require(
      machine.cpu().debug_state() == oracle.expected,
      "Every period preserves literal whole CPU,counters and all16 retained stack slots at pc=" +
          hex(machine.cpu().state().pc, 6) + " period=" + hex(cycle.completed_cycles, 4));
}

void one_case(const Kind kind, const unsigned flags, const std::optional<unsigned> cut) {
  const auto code = program(kind, flags);
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, code)};
  auto contract = pipeline::contract();
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  const auto finish = [&] {
    while (oracle.retired < code.path.size()) { step(run, machine, code, oracle); }
    require(machine.cpu().state().sp == 0 && machine.cpu().debug_state().sc == 0,
            "Actual nested/sequential returns balance the original native stack");
  };
  if (!cut) {
    finish();
    return;
  }
  while (run.completed_cycles() < *cut) { step(run, machine, code, oracle); }
  const auto saved = run.checkpoint(machine);
  const auto prefix = oracle;
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    run.restore(saved, machine);
    oracle = prefix;
    finish();
  }
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Select streams or snapshots");
    const std::string_view mode(arguments[1]);
    require(mode == "streams" || mode == "snapshots", "Unknown private probe scope");
    unsigned cases = 0;
    for (const auto kind : {Kind::nested,
                            Kind::sequential,
                            Kind::maximum,
                            Kind::minimum,
                            Kind::long_backward,
                            Kind::andi}) {
      for (const auto flags : {0U, 0xaaU}) {
        if (mode == "streams") {
          one_case(kind, flags, std::nullopt);
          ++cases;
        } else {
          const auto code = program(kind, flags);
          unsigned end = first_e2;
          for (std::size_t index = 0; index + 1 < code.path.size(); ++index) {
            end += code.path.at(index).cost;
          }
          for (unsigned cut = 0; cut <= end; ++cut) {
            one_case(kind, flags, cut);
            ++cases;
          }
        }
      }
    }
    std::cout << mode << " passed " << cases << " own call/return cases\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
