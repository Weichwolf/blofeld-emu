// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
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
constexpr unsigned program_base = 0x100;
constexpr unsigned start = 0x1fa;
constexpr unsigned setup = 0x200;
constexpr unsigned body = setup + 2;
constexpr unsigned callee = 0x240;
constexpr unsigned inner_setup = 0x246;
constexpr unsigned helper = 0x2c0;
constexpr unsigned deepest = 0x300;
constexpr unsigned image_size = 0x230;
constexpr unsigned guards = 6;
constexpr unsigned tail = 7;
constexpr unsigned first_e2 = 7;
constexpr unsigned maximum_periods = 2048;
constexpr unsigned omr = 0x300;
constexpr unsigned sr_seed = 0xc00300;
constexpr unsigned la_seed = 0x55aa33;
constexpr unsigned lc_seed = 0x33;
constexpr unsigned x0_seed = 0x112233;
constexpr unsigned x1_seed = 0x445566;
constexpr unsigned y0_seed = 0x778899;
constexpr unsigned flags_seed = 0xaa;
constexpr unsigned ccr_mask = 0xff;
constexpr unsigned loop_flags = 0x18000;
constexpr unsigned loop_flag = 0x8000;
constexpr unsigned short_call_base = 0x050800;
constexpr unsigned short_modulus = 512;
constexpr unsigned short_low_mask = 31;
constexpr unsigned short_field_shift = 5;
constexpr unsigned short_opcode_shift = 6;
constexpr unsigned dor_immediate = 0x060090;
constexpr unsigned dor_y1 = 0x06c710;
constexpr unsigned do_y1 = 0x06c700;
constexpr unsigned rts = 0x00000c;
constexpr unsigned andi_ccr = 0x0055b9;
constexpr unsigned andi_mask = 0x55;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned immediate_omr = 0x05f43a;
constexpr unsigned preamble_words = 22;
constexpr unsigned preamble_instructions = 15;
constexpr unsigned instruction_cost = 1;
constexpr unsigned call_cost = 4;
constexpr unsigned return_cost = 3;
constexpr unsigned ordinary_loop_cost = 5;
constexpr unsigned degenerate_loop_cost = 6;
constexpr unsigned zero_loop_cost = 8;
constexpr unsigned call_body_words = 13;
constexpr unsigned simple_callee_guards = 4;
constexpr unsigned refill_count = 176;
constexpr unsigned repeated_count = 17;
constexpr unsigned high_count_one = 0x10001;
constexpr unsigned high_count_two = 0x123456;
constexpr unsigned prefix_body_steps = 8;

enum class Kind : std::uint8_t {
  immediate,
  y1,
  absolute,
  call,
  nested,
  called_loop,
  called_nested
};
enum class Effect : std::uint8_t { nop, loop, call, ret, flags };

struct Case {
  Kind kind;
  unsigned count;
  unsigned size;
  unsigned flags = flags_seed;
  bool prefix = false;
};

struct Instruction {
  unsigned opcode = 0;
  unsigned cost = instruction_cost;
  Effect effect = Effect::nop;
  unsigned target = 0;
  unsigned last = 0;
  unsigned count = 0;
};

struct Program {
  std::vector<unsigned> words = std::vector<unsigned>(image_size, 0);
  std::vector<Instruction> instructions = std::vector<Instruction>(image_size);
  unsigned stop = 0;
};

unsigned put(Program &code, const unsigned pc, const Instruction instruction) {
  code.words.at(pc - program_base) = instruction.opcode;
  code.instructions.at(pc - program_base) = instruction;
  return pc + 1;
}

unsigned call(Program &code, const unsigned pc, const unsigned target) {
  const auto delta = (target + short_modulus - pc) & (short_modulus - 1);
  return put(code,
             pc,
             {.opcode = short_call_base | ((delta >> short_field_shift) << short_opcode_shift) |
                        (delta & short_low_mask),
              .cost = call_cost,
              .effect = Effect::call,
              .target = target});
}

void loop(Program &code, const unsigned pc, const Case sample) {
  const auto count = sample.count;
  const auto size = sample.size;
  const auto kind = sample.kind;
  const auto last = pc + 2 + size - 1;
  auto opcode = dor_y1;
  if (kind == Kind::immediate) {
    opcode = dor_immediate | ((count & ccr_mask) << 8U) | (count >> 8U);
  } else if (kind == Kind::absolute) {
    opcode = do_y1;
  }
  auto cost = ordinary_loop_cost;
  if (count == 0) {
    cost = zero_loop_cost;
  } else if (size == 1) {
    cost = degenerate_loop_cost;
  }
  static_cast<void>(put(code,
                        pc,
                        {.opcode = opcode,
                         .cost = cost,
                         .effect = Effect::loop,
                         .target = pc + 2,
                         .last = last,
                         .count = count}));
  code.words.at(pc + 1 - program_base) = kind == Kind::absolute ? last : last - pc;
}

void callees(Program &code, const Case sample) {
  const bool called_loop = sample.kind == Kind::called_loop || sample.kind == Kind::called_nested;
  const bool nested = sample.kind == Kind::nested || sample.kind == Kind::called_nested;
  unsigned pc = callee;
  const unsigned guard_count = sample.kind == Kind::call ? simple_callee_guards : guards;
  pc += guard_count;
  if (called_loop) {
    const unsigned size = nested ? call_body_words : 3U;
    loop(code, inner_setup, {.kind = Kind::immediate, .count = 2, .size = size});
    if (nested) { static_cast<void>(call(code, inner_setup + 2 + guards, helper)); }
    pc = inner_setup + 2 + size + guards;
  } else if (nested) {
    pc = call(code, pc, helper) + guards;
  }
  if (!called_loop && !nested) {
    pc = put(code, pc, {.opcode = andi_ccr, .cost = return_cost, .effect = Effect::flags});
  }
  static_cast<void>(put(code, pc, {.opcode = rts, .cost = return_cost, .effect = Effect::ret}));
  if (!nested) { return; }
  pc = helper + (called_loop ? guards : simple_callee_guards);
  if (called_loop) {
    pc = call(code, pc, deepest) + guards;
    static_cast<void>(put(code, pc, {.opcode = rts, .cost = return_cost, .effect = Effect::ret}));
    pc = deepest + simple_callee_guards;
  }
  pc = put(code, pc, {.opcode = andi_ccr, .cost = return_cost, .effect = Effect::flags});
  static_cast<void>(put(code, pc, {.opcode = rts, .cost = return_cost, .effect = Effect::ret}));
}

Program program(const Case sample) {
  Program code;
  const std::array seeds{0x05f439U,
                         sr_seed | sample.flags,
                         0x05f43eU,
                         la_seed,
                         0x05f43fU,
                         lc_seed,
                         0x44f400U,
                         x0_seed,
                         0x45f400U,
                         x1_seed,
                         0x46f400U,
                         y0_seed,
                         0x47f400U,
                         sample.count};
  for (std::size_t index = 0; index < seeds.size(); ++index) {
    code.words.at(index) = seeds[index];
  }
  code.words.at(seeds.size() + guards) = absolute_jump;
  code.words.at(seeds.size() + guards + 1) = start;
  loop(code, setup, sample);
  code.stop = body + sample.size + tail;
  if (sample.kind == Kind::call || sample.kind == Kind::nested ||
      sample.kind == Kind::called_loop || sample.kind == Kind::called_nested) {
    static_cast<void>(call(code, body + guards, callee));
    callees(code, sample);
  }
  return code;
}

DspDebugState prepare(DspMachine &machine, const Program &code) {
  DspProgram image;
  image.word(immediate_omr);
  image.word(omr);
  for (unsigned index = 0; index < preamble_words; ++index) { image.word(code.words.at(index)); }
  image.install(machine.bus());
  for (std::size_t index = 0; index < code.words.size(); ++index) {
    machine.bus().write(
        {.address = program_base + static_cast<unsigned>(index), .value = code.words.at(index)});
  }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, preamble_instructions);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == start && before.cpu.omr == omr && before.cpu.sp == 0 && before.sc == 0,
          "Actual own reset preamble establishes reference RAM entry and empty stack");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

struct LoopFrame {
  DspState restore;
  unsigned sc;
  unsigned body;
  unsigned last;
  unsigned remaining;
};

struct Oracle {
  DspDebugState expected;
  std::vector<LoopFrame> loops;
  unsigned next_e2 = first_e2;
  unsigned body_steps = 0;
  unsigned retired = 0;
};

struct StackWords {
  unsigned high;
  unsigned low;
};

void push(DspDebugState &expected, const StackWords frame) {
  ++expected.cpu.sp;
  ++expected.sc;
  expected.stack_high.at(expected.cpu.sp) = frame.high;
  expected.stack_low.at(expected.cpu.sp) = frame.low;
}

void retire(Oracle &oracle, const Program &code, const Case sample) {
  auto &expected = oracle.expected;
  auto &cpu = expected.cpu;
  const auto pc = cpu.pc;
  const auto instruction = code.instructions.at(pc - program_base);
  ++cpu.pc;
  if (instruction.effect == Effect::loop) {
    if (instruction.count == 0) {
      // DO/DOR annulment retains the two native stack writes (FM13-56/61).
      expected.stack_high.at(cpu.sp + 1) = cpu.la;
      expected.stack_low.at(cpu.sp + 1) = cpu.lc;
      expected.stack_high.at(cpu.sp + 2) = instruction.target;
      expected.stack_low.at(cpu.sp + 2) = cpu.sr;
      cpu.pc = instruction.last + 1;
    } else {
      oracle.loops.push_back({.restore = cpu,
                              .sc = expected.sc,
                              .body = instruction.target,
                              .last = instruction.last,
                              .remaining = instruction.count});
      push(expected, {.high = cpu.la, .low = cpu.lc});
      push(expected, {.high = instruction.target, .low = cpu.sr});
      cpu.pc = instruction.target;
      cpu.la = instruction.last;
      cpu.lc = instruction.count;
      cpu.sr = (cpu.sr & ~loop_flags) | loop_flag;
    }
  } else if (instruction.effect == Effect::call) {
    push(expected, {.high = cpu.pc, .low = cpu.sr});
    cpu.pc = instruction.target;
  } else if (instruction.effect == Effect::ret) {
    cpu.pc = expected.stack_high.at(cpu.sp);
    --cpu.sp;
    --expected.sc;
  } else if (instruction.effect == Effect::flags) {
    cpu.sr = (cpu.sr & ~ccr_mask) | (cpu.sr & andi_mask);
  }
  if (pc >= body && pc < body + sample.size) { ++oracle.body_steps; }
  if (!oracle.loops.empty() && pc == oracle.loops.back().last) {
    auto &frame = oracle.loops.back();
    if (--frame.remaining != 0) {
      cpu.pc = frame.body;
      cpu.lc = frame.remaining;
    } else {
      cpu.la = frame.restore.la;
      cpu.lc = frame.restore.lc;
      cpu.sp = frame.restore.sp;
      expected.sc = frame.sc;
      cpu.sr = (cpu.sr & ~loop_flags) | (frame.restore.sr & loop_flags);
      oracle.loops.pop_back();
    }
  }
  cpu.stack_pc = expected.stack_high.at(cpu.sp);
  cpu.stack_sr = expected.stack_low.at(cpu.sp);
  ++expected.native_instructions;
  // Native table counters exclude the separately checked setup Decode holds.
  expected.native_cycles +=
      instruction.effect == Effect::loop ? ordinary_loop_cost : instruction.cost;
  oracle.next_e2 += instruction.cost;
  ++oracle.retired;
}

bool done(const Oracle &oracle, const Program &code, const Case sample) {
  return sample.prefix ? oracle.body_steps == prefix_body_steps
                       : oracle.expected.cpu.pc == code.stop;
}

void step(
    DspFetchRun &run, DspMachine &machine, Oracle &oracle, const Program &code, const Case sample) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const bool due = cycle.completed_cycles == oracle.next_e2;
  require(phase.retirement_published == due,
          "E2 schedule matches complete independent NXP loop/call timing windows");
  if (due) {
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc,
            "Actual E2 owns the independently predicted loop/call instruction");
    retire(oracle, code, sample);
  }
  require(machine.cpu().debug_state() == oracle.expected,
          "Every period matches full native CPU,counters and all16 stack slots at pc=" +
              hex(machine.cpu().state().pc, 6) + " period=" + hex(cycle.completed_cycles, 4));
  require(cycle.completed_cycles < maximum_periods, "Own bounded window must terminate");
}

unsigned one_case(const Case sample, const std::optional<unsigned> cut) {
  const auto code = program(sample);
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, code), .loops = {}};
  auto contract = pipeline::contract();
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  const auto finish = [&] {
    while (!done(oracle, code, sample)) { step(run, machine, oracle, code, sample); }
    require(sample.prefix || (oracle.loops.empty() && machine.cpu().state().sp == 0),
            "Completed native nested loops/calls restore the prior stack");
  };
  if (!cut) {
    finish();
    return static_cast<unsigned>(run.completed_cycles());
  }
  while (run.completed_cycles() < *cut) { step(run, machine, oracle, code, sample); }
  const auto saved = run.checkpoint(machine);
  const auto prefix = oracle;
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    run.restore(saved, machine);
    oracle = prefix;
    require(machine.cpu().debug_state() == oracle.expected,
            "Restored joint checkpoint retains the complete literal native state");
    finish();
  }
  return static_cast<unsigned>(run.completed_cycles());
}

std::vector<Case> cases() {
  std::vector<Case> result;
  for (const unsigned count : {0U, 1U, 3U, repeated_count}) {
    result.push_back({Kind::y1, count, 3});
  }
  for (const unsigned count : {0U, 3U}) {
    result.push_back({Kind::y1, count, 1});
    for (const unsigned size : {1U, 3U}) { result.push_back({Kind::immediate, count, size}); }
    result.push_back({Kind::absolute, count, 3});
  }
  result.push_back({Kind::immediate, refill_count, 3});
  for (const unsigned count : {high_count_one, high_count_two}) {
    result.push_back({Kind::y1, count, 3, flags_seed, true});
  }
  for (const auto kind : {Kind::call, Kind::nested}) {
    result.push_back({kind, 3, call_body_words});
  }
  result.push_back({Kind::y1, 3, 3, 0});
  for (const auto kind : {Kind::called_loop, Kind::called_nested}) {
    result.push_back({kind, 2, call_body_words});
  }
  return result;
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Select streams or snapshots");
    const std::string_view mode(arguments[1]);
    require(mode == "streams" || mode == "snapshots", "Unknown DOR validation scope");
    unsigned checked = 0;
    for (const auto sample : cases()) {
      if (mode == "streams") {
        static_cast<void>(one_case(sample, std::nullopt));
        ++checked;
      } else if (sample.kind == Kind::called_loop || sample.kind == Kind::called_nested) {
        // New composition only: every owned fetch,call,loop and E1/E2 checkpoint.
        const auto periods = one_case(sample, std::nullopt);
        for (unsigned cut = 0; cut <= periods; ++cut) {
          static_cast<void>(one_case(sample, cut));
          ++checked;
        }
      }
    }
    std::cout << mode << " passed " << checked << " bounded native DOR cases\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
