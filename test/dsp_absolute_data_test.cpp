// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
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
#include "dsp_scalar_store_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::require;
constexpr unsigned entry = 0x100, low_data = 0x407, high_data = 0xc234;
constexpr unsigned ordinary_omr = 0x300, canary = 0x654321, flag_pattern = 0xaa;
constexpr unsigned register_bits = 24, byte_mask = 0xff, sign_byte = 0x80;
constexpr unsigned source_mask = 7, high_source_mask = 0x18;
constexpr unsigned high_source_shift = 17, source_shift = 16;
constexpr unsigned absolute_mode = 0x7000, read_bit = 0x8000, y_bit = 0x80000;
constexpr unsigned scalar_base = 0x400000, control_base = 0x057020, control_y = 0x40;
constexpr unsigned full_part = 3, extension_part = 2;
constexpr unsigned illegal_word = 7, padding = 16;
constexpr std::uint64_t word_mask = 0xffffff, accumulator_mask = 0xffffffffffffff;
constexpr std::array values{0U, 0x800001U, 0xffffffU};
constexpr std::array<std::uint64_t, 3> accumulators{0, 0x7f7fffff123456, 0x80800000654321};
// FM MOVE register fields (Table12-14), then the eight ordinary MOVEC modifiers.
enum class Kind : std::uint8_t { x, y, a, b, r, n, m };
enum class Prefix : std::uint8_t { none, r, n, m };

struct Register {
  const char *name;
  unsigned field;
  Kind kind;
  unsigned part;
};

constexpr std::array registers{Register{.name = "x0", .field = 4, .kind = Kind::x, .part = 0},
                               Register{.name = "x1", .field = 5, .kind = Kind::x, .part = 1},
                               Register{.name = "y0", .field = 6, .kind = Kind::y, .part = 0},
                               Register{.name = "y1", .field = 7, .kind = Kind::y, .part = 1},
                               Register{.name = "a0", .field = 8, .kind = Kind::a, .part = 0},
                               Register{.name = "b0", .field = 9, .kind = Kind::b, .part = 0},
                               Register{.name = "a2", .field = 10, .kind = Kind::a, .part = 2},
                               Register{.name = "b2", .field = 11, .kind = Kind::b, .part = 2},
                               Register{.name = "a1", .field = 12, .kind = Kind::a, .part = 1},
                               Register{.name = "b1", .field = 13, .kind = Kind::b, .part = 1},
                               Register{.name = "a", .field = 14, .kind = Kind::a, .part = 3},
                               Register{.name = "b", .field = 15, .kind = Kind::b, .part = 3},
                               Register{.name = "r0", .field = 16, .kind = Kind::r, .part = 0},
                               Register{.name = "r1", .field = 17, .kind = Kind::r, .part = 1},
                               Register{.name = "r2", .field = 18, .kind = Kind::r, .part = 2},
                               Register{.name = "r3", .field = 19, .kind = Kind::r, .part = 3},
                               Register{.name = "r4", .field = 20, .kind = Kind::r, .part = 4},
                               Register{.name = "r5", .field = 21, .kind = Kind::r, .part = 5},
                               Register{.name = "r6", .field = 22, .kind = Kind::r, .part = 6},
                               Register{.name = "r7", .field = 23, .kind = Kind::r, .part = 7},
                               Register{.name = "n0", .field = 24, .kind = Kind::n, .part = 0},
                               Register{.name = "n1", .field = 25, .kind = Kind::n, .part = 1},
                               Register{.name = "n2", .field = 26, .kind = Kind::n, .part = 2},
                               Register{.name = "n3", .field = 27, .kind = Kind::n, .part = 3},
                               Register{.name = "n4", .field = 28, .kind = Kind::n, .part = 4},
                               Register{.name = "n5", .field = 29, .kind = Kind::n, .part = 5},
                               Register{.name = "n6", .field = 30, .kind = Kind::n, .part = 6},
                               Register{.name = "n7", .field = 31, .kind = Kind::n, .part = 7},
                               Register{.name = "m0", .field = 0, .kind = Kind::m, .part = 0},
                               Register{.name = "m1", .field = 1, .kind = Kind::m, .part = 1},
                               Register{.name = "m2", .field = 2, .kind = Kind::m, .part = 2},
                               Register{.name = "m3", .field = 3, .kind = Kind::m, .part = 3},
                               Register{.name = "m4", .field = 4, .kind = Kind::m, .part = 4},
                               Register{.name = "m5", .field = 5, .kind = Kind::m, .part = 5},
                               Register{.name = "m6", .field = 6, .kind = Kind::m, .part = 6},
                               Register{.name = "m7", .field = 7, .kind = Kind::m, .part = 7}};
// Independent NXP-assembled one-word X1->R0/N0/M0 producers, no arithmetic.
constexpr std::array prefix_words{0U, 0x20b000U, 0x20b800U, 0x04c5a0U};

struct Input {
  unsigned reg = 0, variant = 0, scale = 0, flags = 0, address = low_data;
  bool load = false, y = false;
  Prefix prefix = Prefix::none;
};

unsigned opcode(const Input sample) {
  const auto reg = registers.at(sample.reg);
  if (reg.kind == Kind::m) {
    return control_base | reg.field | (sample.load ? read_bit : 0U) | (sample.y ? control_y : 0U);
  }
  return scalar_base | ((reg.field & high_source_mask) << high_source_shift) |
         ((reg.field & source_mask) << source_shift) | absolute_mode |
         (sample.load ? read_bit : 0U) | (sample.y ? y_bit : 0U);
}

unsigned instruction(const Input sample) {
  return entry + 1 + (sample.prefix != Prefix::none ? 1U : 0U);
}

unsigned e1(const Input sample) {
  return 7 + (sample.prefix != Prefix::none ? 1U : 0U);
}

void prefix(DspState &state, const Input sample) {
  const auto value = static_cast<unsigned>((state.x >> register_bits) & word_mask);
  switch (sample.prefix) {
    case Prefix::r: state.r[0] = value; break;
    case Prefix::n: state.n[0] = value; break;
    case Prefix::m: state.m[0] = value; break;
    case Prefix::none: break;
    default: throw Fault("Invalid own absolute-memory prefix");
  }
}

std::uint64_t &accumulator(DspState &state, const Kind kind) {
  return kind == Kind::a ? state.a : state.b;
}

void assign(DspState &state, const Register reg, const unsigned value) {
  if (reg.kind == Kind::r) {
    state.r.at(reg.part) = value;
    return;
  }
  if (reg.kind == Kind::n) {
    state.n.at(reg.part) = value;
    return;
  }
  if (reg.kind == Kind::m) {
    state.m.at(reg.part) = value;
    return;
  }
  if (reg.kind == Kind::x || reg.kind == Kind::y) {
    auto &pair = reg.kind == Kind::x ? state.x : state.y;
    const unsigned shift = reg.part * register_bits;
    pair = (pair & ~(word_mask << shift)) | (std::uint64_t{value} << shift);
    return;
  }
  auto &pair = accumulator(state, reg.kind);
  if (reg.part == full_part) {
    constexpr std::uint64_t negative_extension = 0xff000000000000;
    constexpr unsigned sign = 0x800000;
    pair =
        (std::uint64_t{value} << register_bits) | ((value & sign) != 0 ? negative_extension : 0U);
    return;
  }
  const unsigned shift = reg.part * register_bits;
  const auto mask = reg.part == extension_part ? std::uint64_t{byte_mask} : word_mask;
  pair = (pair & ~(mask << shift)) | ((std::uint64_t{value} & mask) << shift);
}

test::scalar_store::Result stored(DspState state, const Input sample) {
  prefix(state, sample);
  const auto reg = registers.at(sample.reg);
  if (reg.kind == Kind::r) { return {.value = state.r.at(reg.part), .flags = sample.flags}; }
  if (reg.kind == Kind::n) { return {.value = state.n.at(reg.part), .flags = sample.flags}; }
  if (reg.kind == Kind::m) { return {.value = state.m.at(reg.part), .flags = sample.flags}; }
  const bool data = reg.kind == Kind::x || reg.kind == Kind::y;
  auto pair = accumulator(state, reg.kind);
  if (data) { pair = reg.kind == Kind::x ? state.x : state.y; }
  if (!data && reg.part == full_part) {
    return test::scalar_store::expected(
        {.value = pair, .scale = sample.scale, .flags = sample.flags});
  }
  auto result = static_cast<unsigned>((pair >> (reg.part * register_bits)) & word_mask);
  if (!data && reg.part == extension_part && (result & sign_byte) != 0) {
    constexpr unsigned extension_fill = 0xffff00;
    result = (result & byte_mask) | extension_fill;
  }
  return {.value = result, .flags = sample.flags};
}

DspDebugState prepare(DspMachine &machine, const Input sample) {
  test::DspProgram code;
  code.immediate("omr", ordinary_omr);
  for (unsigned index = 0; index < 8; ++index) {
    code.immediate("r" + std::to_string(index), low_data + index);
    code.immediate("n" + std::to_string(index), index + 1);
    code.immediate("m" + std::to_string(index), word_mask);
  }
  for (const auto *const name : {"x0", "x1", "y0", "y1"}) { code.immediate(name, canary); }
  for (const bool b : {false, true}) {
    const auto value =
        b ? (accumulators.at(sample.variant) ^ accumulator_mask) : accumulators.at(sample.variant);
    for (unsigned part = 0; part < 3; ++part) {
      code.immediate(std::string(b ? "b" : "a") + std::to_string(part),
                     (value >> (part * register_bits)) &
                         (part == extension_part ? byte_mask : word_mask));
    }
  }
  const auto reg = registers.at(sample.reg);
  if (reg.kind != Kind::a && reg.kind != Kind::b) {
    code.immediate(reg.name, values.at(sample.variant));
  }
  code.immediate("sr", test::masked_status | sample.scale | sample.flags);
  code.word(test::scalar_store::absolute_jump);
  code.word(entry);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  test::checkpoint::until(machine, entry);
  for (unsigned pc = entry; pc < entry + padding; ++pc) {
    machine.bus().write({.address = pc, .value = 0});
  }
  if (sample.prefix != Prefix::none) {
    machine.bus().write(
        {.address = entry + 1, .value = prefix_words.at(static_cast<unsigned>(sample.prefix))});
  }
  machine.bus().write({.address = instruction(sample), .value = opcode(sample)});
  machine.bus().write({.address = instruction(sample) + 1, .value = sample.address});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {sample.address - 1, sample.address, sample.address + 1}) {
      machine.bus().write({.space = space, .address = address, .value = canary});
    }
  }
  if (sample.load) {
    machine.bus().write({.space = sample.y ? DspSpace::y : DspSpace::x,
                         .address = sample.address,
                         .value = values.at(sample.variant)});
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.omr == ordinary_omr &&
              before.cpu.sr == (test::masked_status | sample.scale | sample.flags),
          "Actual native component seeds establish PC/mode/flags without register or PC injection");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

DspDebugState expected(const DspDebugState &before, const Input sample, const unsigned clock) {
  auto result = before;
  if (sample.prefix != Prefix::none && clock >= 7) { prefix(result.cpu, sample); }
  if (sample.load && clock >= e1(sample)) {
    assign(result.cpu, registers.at(sample.reg), values.at(sample.variant));
  }
  if (clock >= 7) {
    ++result.cpu.pc;
    ++result.native_instructions;
    ++result.native_cycles;
  }
  if (sample.prefix != Prefix::none && clock >= 8) {
    ++result.cpu.pc;
    ++result.native_instructions;
    ++result.native_cycles;
  }
  if (clock >= e1(sample) + 1) {
    result.cpu.pc += 2;
    ++result.native_instructions;
    result.native_cycles += 2;
    if (!sample.load) {
      result.cpu.sr = test::masked_status | sample.scale | stored(before.cpu, sample).flags;
    }
  }
  return result;
}

void cells(DspMachine &machine,
           const DspDebugState &before,
           const Input sample,
           const unsigned clock) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {sample.address - 1, sample.address, sample.address + 1}) {
      unsigned value = canary;
      if (address == sample.address && (space == DspSpace::y) == sample.y) {
        if (sample.load) {
          value = values.at(sample.variant);
        } else if (clock >= e1(sample)) {
          value = stored(before.cpu, sample).value;
        }
      }
      require(
          machine.bus().read({.space = space, .address = address}) == value,
          "Independent source words preserve neighbors and the opposite memory space every period");
    }
  }
}

void stages(const DspPipelineCycle &phase, const Input sample, const unsigned clock) {
  const DspProgramWord word{.address = instruction(sample), .value = opcode(sample)};
  const DspProgramWord extension{.address = instruction(sample) + 1, .value = sample.address};
  const auto decode = e1(sample) - 3;
  if (clock == decode) {
    require(phase.decode_accepted && phase.decode == word && phase.decode_words == 2,
            "Literal two-word opcode enters Decode from its actual F2 occurrence");
  }
  require(phase.extension_decode.has_value() == (clock == decode + 1) &&
              phase.extension_execute2.has_value() == (clock == decode + 5),
          "The extension traverses its own Decode/E2 periods without native retirement");
  if (clock == decode + 1) {
    require(phase.ag1 == word && phase.ag1_extension == extension &&
                phase.extension_decode == extension,
            "AG1 retains both actual F2 words after the backing program cells change");
  }
  if (clock == e1(sample)) {
    require(phase.execute1 == word && phase.e1_extension == extension,
            "Native E1 consumes the owned opcode and complete original extension");
  }
  if (clock == decode + 5) {
    require(phase.extension_execute2 == extension && !phase.retirement_published,
            "Extension E2 drains its actual word without a second native instruction");
  }
}

void step(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Input sample) {
  const auto cycle = run.advance_cycle(machine, test::fetch_clock::input());
  if (!cycle.pipeline) { throw Fault("Absolute-memory observation lost its actual pipeline"); }
  const auto &phase = *cycle.pipeline;
  const auto clock = static_cast<unsigned>(cycle.completed_cycles);
  stages(phase, sample, clock);
  const bool retired =
      clock == 7 || (sample.prefix != Prefix::none && clock == 8) || clock == e1(sample) + 1;
  require(phase.retirement_published == retired && phase.agu_dependencies == 0 &&
              !phase.memory_source_stall,
          "Literal absolute addresses use no R0/N0/M0 address dependency or invented source stall");
  require(phase.memory.has_value() == (clock == e1(sample)),
          "Exactly one native E1 data access owns the two-word move");
  if (phase.memory) {
    const auto &data = phase.memory->data;
    require(data.target.space == (sample.y ? DspSpace::y : DspSpace::x) &&
                data.target.address == sample.address &&
                phase.memory->address.kind == DspAddressStageKind::absolute_long &&
                !phase.memory->address.pointer_update,
            "Actual F2 extension supplies the complete address and space,without an AGU pointer "
            "update");
  }
  const auto wanted = expected(before, sample, clock);
  if (machine.cpu().debug_state() != wanted) {
    std::cerr << "clock=" << clock << " reg=" << registers.at(sample.reg).name
              << " load=" << sample.load << " y=" << sample.y
              << " prefix=" << static_cast<unsigned>(sample.prefix) << '\n';
  }
  require(machine.cpu().debug_state() == wanted,
          "Complete native state/counters/all stack slots follow independent E1/E2 expectations");
  cells(machine, before, sample, clock);
  require(run.elapsed_ticks() == clock * test::pipeline::period,
          "Absolute data transfers preserve the fixed reference clock ledger");
  if (phase.decode_accepted && phase.decode && phase.decode->address == instruction(sample)) {
    machine.bus().write({.address = instruction(sample), .value = illegal_word});
    machine.bus().write({.address = instruction(sample) + 1, .value = illegal_word});
  }
}

unsigned streams = 0, restores = 0, clones = 0;

void finish(DspFetchRun &run,
            DspMachine &machine,
            const DspDebugState &before,
            const Input sample) {
  while (run.completed_cycles() < e1(sample) + 2) { step(run, machine, before, sample); }
}

void one(const Input sample, const bool snapshots = false) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  DspFetchRun run(test::scalar_store::contract(), machine);
  std::vector<DspFetchCheckpoint> cuts;
  while (run.completed_cycles() < e1(sample) + 2) {
    if (snapshots) { cuts.push_back(run.checkpoint(machine)); }
    step(run, machine, before, sample);
  }
  for (const auto &cut : cuts) {
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(cut, machine);
      finish(run, machine, before, sample);
      ++restores;
    }
    DspMachine target;
    DspFetchRun clone(cut, target);
    finish(clone, target, before, sample);
    require(target.cpu().debug_state() == machine.cpu().debug_state() &&
                clone.elapsed_ticks() == run.elapsed_ticks(),
            "Fresh ownership retains literal fields,actual F2 words and the complete suffix clock");
    ++clones;
  }
  ++streams;
}

void variants(const Input sample) {
  for (const unsigned address : {low_data, high_data}) {
    for (unsigned variant = 0; variant < values.size(); ++variant) {
      for (const unsigned flags : {0U, flag_pattern}) {
        auto next = sample;
        next.address = address;
        next.variant = variant;
        next.flags = flags;
        one(next, address == low_data && variant == 1 && flags == flag_pattern);
      }
    }
  }
}

void control_windows(Input sample) {
  sample.variant = 1;
  sample.flags = flag_pattern;
  for (const unsigned scale : {test::scale_up, test::scale_down}) {
    sample.scale = scale;
    one(sample);
  }
  sample.scale = 0;
  const auto reg = registers.at(sample.reg);
  const bool selected =
      reg.name == std::string("r6") ||
      (reg.part == 0 && (reg.kind == Kind::r || reg.kind == Kind::n || reg.kind == Kind::m));
  for (const auto prefix : {Prefix::r, Prefix::n, Prefix::m}) {
    sample.prefix = prefix;
    one(sample, selected);
  }
}

void register_cases(const unsigned reg) {
  for (const bool load : {false, true}) {
    for (const bool y : {false, true}) {
      const Input sample{.reg = reg, .load = load, .y = y};
      variants(sample);
      control_windows(sample);
    }
  }
}
}

int main() {
  try {
    for (unsigned reg = 0; reg < registers.size(); ++reg) { register_cases(reg); }
    std::cout << "PASS ABSOLUTE_DATA streams=" << streams << " restores=" << restores
              << " clones=" << clones << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
