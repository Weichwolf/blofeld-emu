// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_accumulator_transfer.hpp"
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
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned ram = 0x100, start = 0x1fa, entry = 0x200, callee = 0x240;
constexpr unsigned image_words = 0x200, guard_words = 6, tail_words = 7;
constexpr unsigned first_e2 = 7, maximum_periods = 512;
constexpr unsigned seed_words = 34, seed_instructions = 21;
constexpr unsigned omr = 0x300, sr_seed = 0xc003aa, la_seed = 0x55aa33, lc_seed = 0x33;
constexpr unsigned high_word = 0x812345, old_word = 0x33, y1_seed = 0xaabbcc;
constexpr unsigned x0_seed = 0x112233, x1_seed = 0x445566, y0_seed = 0x778899;
constexpr unsigned a0_seed = 0xd4e5f6, a1_seed = 0x112244, a2_seed = 0xe9;
constexpr unsigned b0_seed = 0x6c7d8e, b1_seed = 0x557799, b2_seed = 0x12;
constexpr unsigned a0_code = 8, a2_code = 10, a1_code = 12, b1_code = 13, r0_code = 16;
constexpr unsigned n0_code = 24, offset_streams = 704;
constexpr unsigned mover = 0x200000, mover_source_shift = 13, mover_target_shift = 8;
constexpr unsigned immediate_source_shift = 16, word_shift = 24, byte_mask = 0xff;
constexpr unsigned loop_flags = 0x18000, loop_flag = 0x8000, scale_shift = 10;
constexpr unsigned short_bsr = 0x050800, short_modulus = 512, low_field_mask = 31;
constexpr unsigned low_field_shift = 5, upper_field_shift = 6;
constexpr unsigned jump = 0x0af080, dor_y1 = 0x06c710, dor_two = 0x060290, rts = 0x00000c;
constexpr unsigned move_omr = 0x05f43a, move_a1 = 0x54f400, move_b1 = 0x55f400;
constexpr unsigned helper_setup = 0x246, helper_body = 0x248, helper_last = 0x24a;
constexpr unsigned helper_write = 0x251, helper_value = 0x55, helper_return = 0x259;
constexpr unsigned first_count = 17, expected_streams = 118;
constexpr unsigned long_writer = 32, short_writer = 33;
constexpr std::array destinations{4U, 5U, 6U, 7U, 16U, 17U, 18U, 19U, 20U, 21U, 22U, 23U};

enum class Kind : std::uint8_t { raw, count, producer, call };
enum class Effect : std::uint8_t { nop, write, loop, call, ret };

struct Case {
  Kind kind = Kind::raw;
  unsigned source = a1_code;
  unsigned destination = 7;
  unsigned value = 3;
  unsigned scale = 0;
  unsigned writer = 0;
  unsigned gap = 0;

  [[nodiscard]] bool interlock() const {
    return writer != 0 && writer != long_writer && gap == 0 &&
           (writer == short_writer || (writer & 1U) == (source & 1U));
  }
};

struct Instruction {
  unsigned opcode = 0;
  unsigned words = 1;
  unsigned cost = 1;
  unsigned hold_before = 0;
  Effect effect = Effect::nop;
  unsigned target = 0;
  unsigned value = 0;
  unsigned last = 0;
};

struct Program {
  std::vector<unsigned> words = std::vector<unsigned>(image_words, 0);
  std::vector<Instruction> instructions = std::vector<Instruction>(image_words);
  unsigned stop = 0;
};

unsigned put(Program &code, const unsigned pc, const Instruction instruction) {
  code.words.at(pc - ram) = instruction.opcode;
  code.instructions.at(pc - ram) = instruction;
  return pc + instruction.words;
}

struct LoopSpec {
  unsigned count;
  bool immediate;
};

unsigned opening(Program &code, const unsigned pc, const LoopSpec loop) {
  const unsigned last = pc + 4;
  static_cast<void>(put(code,
                        pc,
                        {.opcode = loop.immediate ? dor_two : dor_y1,
                         .words = 2,
                         .cost = loop.count == 0 ? 8U : 5U,
                         .effect = Effect::loop,
                         .target = pc + 2,
                         .value = loop.count,
                         .last = last}));
  code.words.at(pc + 1 - ram) = 4;
  return pc + 2;
}

void seeds(Program &code, const Case sample) {
  const bool replaces_source = sample.writer == sample.source || sample.writer >= long_writer;
  const auto selected = replaces_source ? old_word : sample.value;
  const std::array initial{0x05f439U, sr_seed | (sample.scale << scale_shift),
                           0x05f43eU, la_seed,
                           0x05f43fU, lc_seed,
                           0x44f400U, sample.writer != 0 ? sample.value : x0_seed,
                           0x45f400U, x1_seed,
                           0x46f400U, y0_seed,
                           0x47f400U, y1_seed,
                           0x50f400U, a0_seed,
                           move_a1,   sample.source == a1_code ? selected : a1_seed,
                           0x52f400U, a2_seed,
                           0x51f400U, b0_seed,
                           move_b1,   sample.source == b1_code ? selected : b1_seed,
                           0x53f400U, b2_seed};
  for (std::size_t index = 0; index < initial.size(); ++index) {
    code.words.at(index) = initial[index];
  }
  code.words.at(initial.size() + guard_words) = jump;
  code.words.at(initial.size() + guard_words + 1) = start;
}

unsigned producer(Program &code, const unsigned pc, const Case sample) {
  Instruction instruction{.effect = Effect::write, .target = sample.writer, .value = sample.value};
  if (sample.writer == long_writer) {
    instruction.opcode = sample.source == a1_code ? move_a1 : move_b1;
    instruction.words = 2;
    instruction.cost = 2;
    instruction.target = sample.source;
    code.words.at(pc + 1 - ram) = sample.value;
  } else if (sample.writer == short_writer) {
    instruction.opcode = mover | (sample.source << immediate_source_shift) | (sample.value << 8U);
    instruction.target = sample.source;
  } else {
    instruction.opcode = mover | (4U << mover_source_shift) | (sample.writer << mover_target_shift);
  }
  return put(code, pc, instruction);
}

Program program(const Case sample) {
  Program code;
  seeds(code, sample);
  unsigned pc = entry;
  if (sample.writer != 0) { pc = producer(code, pc, sample) + sample.gap; }
  pc = put(code,
           pc,
           {.opcode = mover | (sample.source << mover_source_shift) |
                      (sample.destination << mover_target_shift),
            .hold_before = sample.interlock() ? 1U : 0U,
            .effect = Effect::write,
            .target = sample.destination,
            .value = sample.value});
  if (sample.kind == Kind::count) { pc += sample.gap; }
  if (sample.kind == Kind::call) {
    const auto displacement = (callee + short_modulus - pc) & (short_modulus - 1);
    pc = put(code,
             pc,
             {.opcode = short_bsr | ((displacement >> low_field_shift) << upper_field_shift) |
                        (displacement & low_field_mask),
              .cost = 4,
              .effect = Effect::call,
              .target = callee}) +
         guard_words;
    require(opening(code, helper_setup, {.count = 2, .immediate = true}) == helper_body &&
                code.instructions.at(helper_setup - ram).last == helper_last,
            "Callee loop extent retains the literal independently assembled addresses");
    static_cast<void>(put(code,
                          helper_write,
                          {.opcode = move_a1,
                           .words = 2,
                           .cost = 2,
                           .effect = Effect::write,
                           .target = a1_code,
                           .value = helper_value}));
    code.words.at(helper_write + 1 - ram) = helper_value;
    static_cast<void>(put(code, helper_return, {.opcode = rts, .cost = 3, .effect = Effect::ret}));
  }
  if (sample.kind != Kind::raw && sample.destination < n0_code) {
    pc = opening(code, pc, {.count = sample.value, .immediate = false}) + 3;
  }
  code.stop = pc + tail_words;
  return code;
}

DspDebugState prepare(DspMachine &machine, const Program &code, const Case sample) {
  DspProgram image;
  image.word(move_omr);
  image.word(omr);
  for (unsigned index = 0; index < seed_words; ++index) { image.word(code.words.at(index)); }
  image.install(machine.bus());
  for (std::size_t index = 0; index < code.words.size(); ++index) {
    machine.bus().write(
        {.address = ram + static_cast<unsigned>(index), .value = code.words.at(index)});
  }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, seed_instructions);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == start && before.cpu.omr == omr && before.cpu.sp == 0 && before.sc == 0,
          "Own native reset preamble establishes the literal reference entry");
  const auto selected =
      sample.writer == sample.source || sample.writer >= long_writer ? old_word : sample.value;
  const auto initial_a1 = sample.source == a1_code ? selected : a1_seed;
  const auto initial_b1 = sample.source == b1_code ? selected : b1_seed;
  const auto initial_x0 = sample.writer != 0 ? sample.value : x0_seed;
  const auto initial_a =
      (std::uint64_t{a2_seed} << data_bits) | (std::uint64_t{initial_a1} << word_shift) | a0_seed;
  const auto initial_b =
      (std::uint64_t{b2_seed} << data_bits) | (std::uint64_t{initial_b1} << word_shift) | b0_seed;
  require(
      before.cpu.a == initial_a && before.cpu.b == initial_b &&
          before.cpu.x == ((std::uint64_t{x1_seed} << word_shift) | initial_x0) &&
          before.cpu.y == ((std::uint64_t{y1_seed} << word_shift) | y0_seed) &&
          before.cpu.sr == (sr_seed | (sample.scale << scale_shift)) && before.cpu.la == la_seed &&
          before.cpu.lc == lc_seed,
      "Every literal seed,overflowing full accumulator and scaling bit is established natively");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

void write_word(DspState &cpu, const Instruction instruction) {
  const auto reg = instruction.target;
  const auto value = static_cast<std::uint64_t>(instruction.value);
  if (reg < a0_code) {
    auto &pair = reg < 6 ? cpu.x : cpu.y;
    const auto shift = (reg & 1U) * word_shift;
    pair = (pair & ~(static_cast<std::uint64_t>(word_mask) << shift)) | (value << shift);
  } else if (reg < r0_code) {
    auto &accumulator = (reg & 1U) == 0 ? cpu.a : cpu.b;
    unsigned part = 1;
    if (reg < a2_code) {
      part = 0;
    } else if (reg < a1_code) {
      part = 2;
    }
    const auto shift = part * word_shift;
    const std::uint64_t mask = part == 2 ? byte_mask : word_mask;
    accumulator = (accumulator & ~(mask << shift)) | (value << shift);
  } else if (reg < n0_code) {
    cpu.r.at(reg - r0_code) = instruction.value;
  } else {
    cpu.n.at(reg - n0_code) = instruction.value;
  }
}

DspMoveFields destination_fields(const unsigned reg) {
  DspMoveFields fields;
  if (reg < a0_code) {
    auto &pair = reg < 6 ? fields.x : fields.y;
    pair = static_cast<std::uint8_t>(1U << (reg & 1U));
  } else if (reg < r0_code) {
    auto &accumulator = (reg & 1U) == 0 ? fields.a : fields.b;
    unsigned part = 1;
    if (reg < a2_code) {
      part = 0;
    } else if (reg < a1_code) {
      part = 2;
    }
    accumulator = static_cast<std::uint8_t>(1U << part);
  } else if (reg < n0_code) {
    fields.r = static_cast<std::uint8_t>(1U << (reg - r0_code));
  } else {
    fields.n = static_cast<std::uint8_t>(1U << (reg - n0_code));
  }
  return fields;
}

struct Frame {
  DspState restore;
  unsigned sc;
  unsigned body;
  unsigned last;
  unsigned remaining;
};

struct Oracle {
  DspDebugState expected;
  std::vector<Frame> loops;
  unsigned next_e2 = first_e2;
};

struct StackWords {
  unsigned high;
  unsigned low;
};

void push(DspDebugState &expected, const StackWords words) {
  ++expected.cpu.sp;
  ++expected.sc;
  expected.stack_high.at(expected.cpu.sp) = words.high;
  expected.stack_low.at(expected.cpu.sp) = words.low;
}

void open(Oracle &oracle, const Instruction instruction) {
  auto &expected = oracle.expected;
  auto &cpu = expected.cpu;
  if (instruction.value == 0) {
    expected.stack_high.at(cpu.sp + 1) = cpu.la;
    expected.stack_low.at(cpu.sp + 1) = cpu.lc;
    expected.stack_high.at(cpu.sp + 2) = instruction.target;
    expected.stack_low.at(cpu.sp + 2) = cpu.sr;
    cpu.pc = instruction.last + 1;
    return;
  }
  oracle.loops.push_back({.restore = cpu,
                          .sc = expected.sc,
                          .body = instruction.target,
                          .last = instruction.last,
                          .remaining = instruction.value});
  push(expected, {.high = cpu.la, .low = cpu.lc});
  push(expected, {.high = instruction.target, .low = cpu.sr});
  cpu.pc = instruction.target;
  cpu.la = instruction.last;
  cpu.lc = instruction.value;
  cpu.sr = (cpu.sr & ~loop_flags) | loop_flag;
}

void retire(Oracle &oracle, const Program &code) {
  auto &expected = oracle.expected;
  auto &cpu = expected.cpu;
  const auto pc = cpu.pc;
  const auto instruction = code.instructions.at(pc - ram);
  cpu.pc += instruction.words;
  switch (instruction.effect) {
    case Effect::nop:
    case Effect::write: break; // Destination already published by genuine native E1.
    case Effect::loop: open(oracle, instruction); break;
    case Effect::call:
      push(expected, {.high = cpu.pc, .low = cpu.sr});
      cpu.pc = instruction.target;
      break;
    case Effect::ret:
      cpu.pc = expected.stack_high.at(cpu.sp);
      --cpu.sp;
      --expected.sc;
      break;
    default: throw Fault("Unknown own raw-middle effect");
  }
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
  expected.native_cycles += instruction.effect == Effect::loop ? 5U : instruction.cost;
  oracle.next_e2 += instruction.cost + code.instructions.at(cpu.pc - ram).hold_before;
}

void state_difference(const DspDebugState &actual, const DspDebugState &expected) {
  if (actual == expected) { return; }
  std::cerr << "state-difference cpu=" << (actual.cpu == expected.cpu)
            << " cycles=" << actual.native_cycles << '/' << expected.native_cycles
            << " instructions=" << actual.native_instructions << '/' << expected.native_instructions
            << " ep=" << actual.ep << '/' << expected.ep << " sz=" << actual.sz << '/'
            << expected.sz << " sc=" << actual.sc << '/' << expected.sc
            << " stack=" << (actual.stack_high == expected.stack_high) << '/'
            << (actual.stack_low == expected.stack_low) << '\n';
  for (unsigned index = 0; index < actual.cpu.n.size(); ++index) {
    if (actual.cpu.n.at(index) != expected.cpu.n.at(index)) {
      std::cerr << "N" << index << '=' << hex(actual.cpu.n.at(index), 6)
                << " expected=" << hex(expected.cpu.n.at(index), 6) << '\n';
    }
  }
}

void step(DspFetchRun &run, DspMachine &machine, Oracle &oracle, const Program &code) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const bool due = cycle.completed_cycles == oracle.next_e2;
  require(phase.retirement_published == due,
          "E2 placement matches qualified nominal costs and same-accumulator interlock");
  if (due) {
    const auto &instruction = code.instructions.at(oracle.expected.cpu.pc - ram);
    require(phase.execute2 && phase.execute2->address == oracle.expected.cpu.pc &&
                phase.execute2->value == instruction.opcode,
            "Actual native E2 owns the independent instruction and PC");
    retire(oracle, code);
  }
  const auto &next = code.instructions.at(oracle.expected.cpu.pc - ram);
  const bool write_due =
      cycle.completed_cycles + 1 == oracle.next_e2 && next.effect == Effect::write;
  require(phase.transfer.has_value() == write_due,
          "Native register fields publish only on the independently selected E1 period");
  if (write_due) {
    require(phase.transfer && phase.execute1 && phase.execute1->address == oracle.expected.cpu.pc &&
                phase.transfer->pc == oracle.expected.cpu.pc &&
                phase.transfer->opcode == next.opcode &&
                phase.transfer->fields == destination_fields(next.target),
            "Exact raw register lane and actual instruction own E1 publication");
    write_word(oracle.expected.cpu, next);
  }
  state_difference(machine.cpu().debug_state(), oracle.expected);
  require(machine.cpu().debug_state() == oracle.expected,
          "Every period preserves full CPU,table counters and all16 stack cells at pc=" +
              hex(machine.cpu().state().pc, 6) + " period=" + hex(cycle.completed_cycles, 4));
  require(cycle.completed_cycles < maximum_periods, "Own raw-middle window is bounded");
}

unsigned one_case(const Case sample, const std::optional<unsigned> cut) {
  const auto code = program(sample);
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, code, sample), .loops = {}};
  auto input = pipeline::contract();
  input.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(input, machine);
  const auto finish = [&] {
    while (oracle.expected.cpu.pc != code.stop) { step(run, machine, oracle, code); }
    require(oracle.loops.empty() && machine.cpu().state().sp == 0,
            "Native call/loop completion restores the prior owned stack");
  };
  if (!cut) {
    finish();
    return static_cast<unsigned>(run.completed_cycles());
  }
  while (run.completed_cycles() < *cut) { step(run, machine, oracle, code); }
  const auto saved = run.checkpoint(machine);
  const auto prefix = oracle;
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    run.restore(saved, machine);
    oracle = prefix;
    require(machine.cpu().debug_state() == oracle.expected,
            "Joint restore preserves complete native state");
    finish();
  }
  return static_cast<unsigned>(run.completed_cycles());
}

std::vector<Case> cases() {
  std::vector<Case> result;
  for (const auto source : {a1_code, b1_code}) {
    for (const auto destination : destinations) {
      for (const auto value : {3U, high_word}) {
        result.push_back({.source = source, .destination = destination, .value = value});
      }
    }
    for (const unsigned scale : {1U, 2U, 3U}) {
      result.push_back({.source = source, .value = high_word, .scale = scale});
    }
    for (const unsigned value : {0U, 1U, 3U, first_count}) {
      for (const unsigned gap : {0U, 1U, 2U}) {
        result.push_back({.kind = Kind::count, .source = source, .value = value, .gap = gap});
      }
    }
    const auto low_bit = source & 1U;
    const std::array writers{a0_code + low_bit,
                             a1_code + low_bit,
                             a2_code + low_bit,
                             a1_code + (1U - low_bit),
                             long_writer,
                             short_writer};
    for (const auto writer : writers) {
      for (const unsigned gap : {0U, 1U, 2U}) {
        result.push_back({.kind = Kind::producer, .source = source, .writer = writer, .gap = gap});
      }
    }
  }
  for (const auto value : {0U, 1U, 3U, first_count}) {
    result.push_back({.kind = Kind::call, .value = value});
  }
  require(result.size() == expected_streams,
          "All118 independently qualified own programs are retained");
  return result;
}

std::vector<Case> offset_cases() {
  constexpr std::array values{0U, 1U, 0x7fffffU, high_word, word_mask};
  std::vector<Case> result;
  for (const auto source : {a1_code, b1_code}) {
    const auto low_bit = source & 1U;
    const std::array writers{a0_code + low_bit,
                             a1_code + low_bit,
                             a2_code + low_bit,
                             a1_code + (1U - low_bit),
                             long_writer,
                             short_writer};
    for (unsigned index = 0; index < 8; ++index) {
      const auto destination = n0_code + index;
      for (const auto value : values) {
        for (unsigned scale = 0; scale < 4; ++scale) {
          result.push_back(
              {.source = source, .destination = destination, .value = value, .scale = scale});
        }
      }
      for (const auto writer : writers) {
        for (unsigned gap = 0; gap < 4; ++gap) {
          result.push_back({.kind = Kind::producer,
                            .source = source,
                            .destination = destination,
                            .writer = writer,
                            .gap = gap});
        }
      }
    }
  }
  require(result.size() == offset_streams, "All704 own raw offset programs are present");
  return result;
}

bool offset_snapshot(const Case sample) {
  if (sample.kind == Kind::raw) {
    return sample.scale == 3 && (sample.value == 0 || sample.value == high_word);
  }
  return sample.destination == n0_code + 4 && sample.gap == 0 && sample.writer == sample.source;
}

bool snapshot_case(const Case sample) {
  if (sample.kind == Kind::raw) { return sample.scale == 3; }
  if (sample.kind == Kind::count || sample.kind == Kind::call) {
    return sample.gap == 0 && (sample.value == 0 || sample.value == 3);
  }
  return sample.gap == 0 &&
         (sample.writer >= long_writer || (sample.writer & 1U) == (sample.source & 1U));
}

void guards() {
  const auto refused = [](const unsigned opcode) {
    require(!middle_accumulator_transfer_source(opcode),
            "Unqualified raw source/destination remains excluded");
    try {
      static_cast<void>(decode_pipeline_word(
          {.address = entry, .value = opcode}, omr, DspFetchPipeline::automatic_memory));
    } catch (const Fault &error) {
      const std::string text = error.what();
      require(text.contains("P:" + hex(entry, 6)) && text.contains("pc=" + hex(entry, 6)) &&
                  text.contains("omr=" + hex(omr, 6)),
              "Excluded raw transfer reports actual Decode context");
      return;
    }
    throw Fault("Unqualified raw transfer was admitted");
  };
  for (const unsigned source : {8U, 9U, 10U, 11U}) {
    for (const auto destination : destinations) {
      refused(mover | (source << mover_source_shift) | (destination << 8U));
    }
  }
  for (const auto source : {a1_code, b1_code}) {
    for (unsigned destination = a0_code; destination < r0_code; ++destination) {
      refused(mover | (source << mover_source_shift) | (destination << 8U));
    }
  }
  for (const unsigned source : {8U, 9U, 10U, 11U, 14U, 15U}) {
    for (unsigned index = 0; index < 8; ++index) {
      refused(mover | (source << mover_source_shift) | ((n0_code + index) << 8U));
    }
  }
  for (const auto source : {a1_code, b1_code}) {
    for (unsigned index = 0; index < 8; ++index) {
      for (const unsigned alu : {0x01U, 0x10U, 0x51U}) {
        refused(mover | (source << mover_source_shift) | ((n0_code + index) << 8U) | alu);
      }
    }
  }
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Select streams,snapshots,offsets,offset-snapshots or guards");
    const std::string_view mode(arguments[1]);
    if (mode == "guards") {
      guards();
      std::cout << "Raw-middle Decode guards passed\n";
      return 0;
    }
    const bool offsets = mode == "offsets" || mode == "offset-snapshots";
    const bool streams = mode == "streams" || mode == "offsets";
    require(streams || mode == "snapshots" || mode == "offset-snapshots",
            "Unknown raw-middle scope");
    unsigned checked = 0;
    for (const auto sample : offsets ? offset_cases() : cases()) {
      if (streams) {
        static_cast<void>(one_case(sample, std::nullopt));
        ++checked;
      } else if (offsets ? offset_snapshot(sample) : snapshot_case(sample)) {
        const auto periods = one_case(sample, std::nullopt);
        for (unsigned cut = 0; cut <= periods; ++cut) {
          static_cast<void>(one_case(sample, cut));
          ++checked;
        }
      }
    }
    std::cout << mode << " passed " << checked << " raw-middle cases\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
