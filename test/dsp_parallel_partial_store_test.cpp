// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_repeat_fetch.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_parallel_middle_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::require;
constexpr unsigned entry = 0x100;
constexpr unsigned body = entry + 1;
constexpr unsigned data = 0x400;
constexpr unsigned last_data = 0x4ff;
constexpr unsigned canary = 0x654321;
constexpr unsigned ordinary_omr = 0x300;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned modulo_256 = 0x80ff;
constexpr unsigned ring_size = 256;
constexpr unsigned word_bits = 24;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned byte_sign = 0x80;
constexpr unsigned extension_fill = 0xffff00;
constexpr unsigned sign_word = 0x800000;
constexpr unsigned padding = 16;
constexpr unsigned last_clock = 8;
constexpr unsigned illegal_word = 7;
constexpr std::uint64_t sign_extension = 0xff000000000000;
constexpr std::uint64_t period = 45;
constexpr std::array source_fields{0U, 4U, 5U, 6U, 7U};
constexpr std::array source_names{"", "x0", "y0", "x1", "y1"};
constexpr std::array part_names{"a0", "b0", "a2", "b2", "a1", "b1"};
constexpr std::array fields{8U, 9U, 10U, 11U, 12U, 13U};
constexpr std::array scales{0U, 0x400U, 0x800U};
constexpr std::array flag_patterns{0x55U, 0xaaU};
constexpr std::array data_seeds{0x234567U, 0x800001U, 0x765432U, 0xffffffU};
constexpr std::array<std::uint64_t, 3> a_values{0, 0x7f765432123456, 0xff800001ffffff};
constexpr std::array<std::uint64_t, 3> b_values{
    0xffffffffffffff, 0x80876543654321, 0x008000007fffff};

enum class Producer : std::uint8_t { none, part, transfer, add };

struct Input {
  unsigned source = 0;
  unsigned part = 0;
  unsigned mode = 3;
  unsigned reg = 0;
  unsigned address = data;
  unsigned variant = 0;
  unsigned scale = 0;
  unsigned flags = 0;
  bool b = false;
  bool y = false;
  bool modulo = false;
  Producer producer = Producer::none;
  unsigned producer_part = 0;
  unsigned gap = 0;
  bool producer_b = false;
};

unsigned instruction(const Input sample) {
  return body + sample.gap;
}

bool source_hold(const Input sample) {
  const bool bank =
      sample.producer == Producer::part ? (sample.producer_part & 1U) != 0 : sample.producer_b;
  return sample.producer != Producer::none && sample.gap == 0 && bank == ((sample.part & 1U) != 0);
}

unsigned write_clock(const Input sample) {
  return 7 + sample.gap + (source_hold(sample) ? 1U : 0U);
}

unsigned retire_clock(const Input sample) {
  return last_clock + sample.gap + (source_hold(sample) ? 1U : 0U);
}

unsigned producer_opcode(const Input sample) {
  constexpr unsigned register_base = 0x200000;
  constexpr unsigned source_x0 = 4U << 13U;
  constexpr unsigned tfr_x0 = 0x41;
  constexpr unsigned add_opposite = 0x10;
  switch (sample.producer) {
    case Producer::none: return 0;
    case Producer::part: return register_base | source_x0 | (fields.at(sample.producer_part) << 8U);
    case Producer::transfer: return register_base | tfr_x0 | (sample.producer_b ? 8U : 0U);
    case Producer::add: return register_base | add_opposite | (sample.producer_b ? 8U : 0U);
    default: throw Fault("Undefined own producer");
  }
}

std::string producer_text(const Input sample) {
  switch (sample.producer) {
    case Producer::none: return "nop";
    case Producer::part: return "move x0," + std::string(part_names.at(sample.producer_part));
    case Producer::transfer: return std::string("tfr x0,") + (sample.producer_b ? 'b' : 'a');
    case Producer::add: return std::string("add ") + (sample.producer_b ? "a,b" : "b,a");
    default: throw Fault("Undefined own producer text");
  }
}

unsigned opcode(const Input sample) {
  constexpr unsigned scalar_base = 0x400000;
  constexpr unsigned effective = 0x4000;
  constexpr unsigned high_fields = 0x18;
  constexpr unsigned high_shift = 17;
  constexpr unsigned field_shift = 16;
  constexpr unsigned mode_shift = 11;
  constexpr unsigned register_shift = 8;
  constexpr unsigned y_bit = 0x80000;
  const auto field = fields.at(sample.part);
  return scalar_base | ((field & high_fields) << high_shift) | ((field & 7U) << field_shift) |
         effective | (sample.mode << mode_shift) | (sample.reg << register_shift) |
         (sample.y ? y_bit : 0U) | (source_fields.at(sample.source) << 4U) | (sample.b ? 8U : 0U) |
         1U;
}

std::string address_text(const Input sample) {
  const auto r = "r" + std::to_string(sample.reg);
  const auto n = "n" + std::to_string(sample.reg);
  auto base = '(' + r + ')';
  switch (sample.mode) {
    case 0: return base + '-' + n;
    case 1: return base + '+' + n;
    case 2: return base + '-';
    case 3: return base + '+';
    case 4: return base;
    default: throw Fault("Undefined own post-update store mode");
  }
}

void encoding(const Input sample) {
  const auto source =
      sample.source == 0 ? std::string(sample.b ? "a" : "b") : source_names.at(sample.source);
  const auto text = "tfr " + source + ',' + (sample.b ? "b" : "a") + ' ' +
                    part_names.at(sample.part) + (sample.y ? ",y:" : ",x:") + address_text(sample);
  require(test::checkpoint::assemble(text.c_str()) == std::vector<std::uint32_t>{opcode(sample)},
          "Own assembly matches independent TFR,raw-part,space,Rn and mode bit fields");
  require(test::checkpoint::assemble(producer_text(sample).c_str()) ==
              std::vector<std::uint32_t>{producer_opcode(sample)},
          "Producer encodings independently follow FM MOVER/TFR/ADD fields");
}

std::uint64_t extended(const unsigned value) {
  return (std::uint64_t{value} << word_bits) | ((value & sign_word) != 0 ? sign_extension : 0U);
}

std::uint64_t transferred(const DspState &before, const Input sample) {
  switch (sample.source) {
    case 0: return sample.b ? before.a : before.b;
    case 1: return extended(static_cast<unsigned>(before.x & word_mask));
    case 2: return extended(static_cast<unsigned>(before.y & word_mask));
    case 3: return extended(static_cast<unsigned>((before.x >> word_bits) & word_mask));
    case 4: return extended(static_cast<unsigned>((before.y >> word_bits) & word_mask));
    default: throw Fault("Undefined own internal TFR source");
  }
}

unsigned stored(const DspState &before, const Input sample) {
  const auto accumulator = (sample.part & 1U) == 0 ? before.a : before.b;
  const auto part = sample.part / 2;
  constexpr std::array shifts{0U, word_bits * 2, word_bits};
  const auto shift = shifts.at(part);
  auto value = static_cast<unsigned>((accumulator >> shift) & (part == 1 ? byte_mask : word_mask));
  if (part == 1 && (value & byte_sign) != 0) { value |= extension_fill; }
  return value;
}

DspState produced(const DspState &before, const Input sample) {
  auto result = before;
  if (sample.producer == Producer::part) {
    constexpr std::array shifts{0U, word_bits * 2, word_bits};
    const auto part = sample.producer_part / 2;
    const auto shift = shifts.at(part);
    const std::uint64_t mask = part == 1 ? byte_mask : word_mask;
    auto &accumulator = (sample.producer_part & 1U) == 0 ? result.a : result.b;
    accumulator = (accumulator & ~(mask << shift)) | ((data_seeds.at(0) & mask) << shift);
  } else if (sample.producer == Producer::transfer) {
    (sample.producer_b ? result.b : result.a) = extended(data_seeds.at(0));
  } else if (sample.producer == Producer::add) {
    const auto arithmetic = test::parallel_middle::arithmetic(
        before,
        sample.producer_b ? test::parallel_middle::Alu::add_b : test::parallel_middle::Alu::add_a);
    (sample.producer_b ? result.b : result.a) = arithmetic.value;
    result.sr = arithmetic.status;
  }
  return result;
}

unsigned pointer(const Input sample) {
  constexpr std::array deltas{-3, 3, -1, 1, 0};
  const int delta = deltas.at(sample.mode);
  if (sample.modulo) {
    const auto low = static_cast<unsigned>(
        (static_cast<int>(sample.address & byte_mask) + delta + static_cast<int>(ring_size)) %
        static_cast<int>(ring_size));
    return (sample.address & ~byte_mask) | low;
  }
  return static_cast<unsigned>(static_cast<int>(sample.address) + delta);
}

DspFetchRunInputs contract() {
  auto result = test::fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

void seed(test::DspProgram &code, const Input sample) {
  code.immediate("omr", ordinary_omr);
  code.immediate("sz", 0);
  for (unsigned i = 0; i < 8; ++i) {
    code.immediate("r" + std::to_string(i), canary + i);
    code.immediate("n" + std::to_string(i), 3);
    code.immediate("m" + std::to_string(i), word_mask);
  }
  code.immediate("r" + std::to_string(sample.reg), sample.address);
  code.immediate("m" + std::to_string(sample.reg), sample.modulo ? modulo_256 : word_mask);
  constexpr std::array names{"x0", "x1", "y0", "y1"};
  for (unsigned i = 0; i < names.size(); ++i) { code.immediate(names.at(i), data_seeds.at(i)); }
  for (const bool b : {false, true}) {
    const auto value = b ? b_values.at(sample.variant) : a_values.at(sample.variant);
    for (unsigned i = 0; i < 3; ++i) {
      code.immediate(std::string(b ? "b" : "a") + std::to_string(i),
                     (value >> (i * word_bits)) & (i == 2 ? byte_mask : word_mask));
    }
  }
}

DspDebugState prepare(DspMachine &machine, const Input sample) {
  test::DspProgram code;
  seed(code, sample);
  code.immediate("sr", test::masked_status | sample.scale | sample.flags);
  for (unsigned i = 0; i < 5; ++i) { code.word(0); }
  code.word(absolute_jump);
  code.word(entry);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  test::checkpoint::until(machine, entry);
  for (unsigned pc = entry; pc < entry + padding; ++pc) { machine.bus().write({.address = pc}); }
  machine.bus().write({.address = entry, .value = producer_opcode(sample)});
  machine.bus().write({.address = instruction(sample), .value = opcode(sample)});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {sample.address - 1, sample.address, sample.address + 1}) {
      machine.bus().write({.space = space, .address = address, .value = canary});
    }
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.omr == ordinary_omr &&
              before.cpu.a == a_values.at(sample.variant) &&
              before.cpu.b == b_values.at(sample.variant) &&
              before.cpu.x == ((std::uint64_t{data_seeds.at(1)} << word_bits) | data_seeds.at(0)) &&
              before.cpu.y == ((std::uint64_t{data_seeds.at(3)} << word_bits) | data_seeds.at(2)) &&
              before.cpu.sr == (test::masked_status | sample.scale | sample.flags),
          "Real seed instructions establish literal TFR/source/flag inputs");
  for (unsigned i = 0; i < before.cpu.r.size(); ++i) {
    require(before.cpu.r.at(i) == (i == sample.reg ? sample.address : canary + i) &&
                before.cpu.n.at(i) == 3 &&
                before.cpu.m.at(i) == (i == sample.reg && sample.modulo ? modulo_256 : word_mask),
            "Actual R/N/M seed values and nonlinear modifier cache preconditions");
  }
  static_cast<void>(machine.begin_reference_instruction(contract().execution));
  return before;
}

DspDebugState expected(const DspDebugState &before, const Input sample, const unsigned clock) {
  auto result = before;
  const auto after = produced(before.cpu, sample);
  const bool raw_due = sample.producer == Producer::part && clock >= 6;
  const bool alu_due =
      (sample.producer == Producer::transfer || sample.producer == Producer::add) && clock >= 7;
  if (raw_due || alu_due) {
    result.cpu.a = after.a;
    result.cpu.b = after.b;
    result.cpu.sr = after.sr;
  }
  if (clock >= 6 + sample.gap) { result.cpu.r.at(sample.reg) = pointer(sample); }
  const unsigned leading = clock >= 7 ? std::min(clock - 6, sample.gap + 1) : 0U;
  result.cpu.pc += leading;
  result.native_instructions += leading;
  result.native_cycles += leading;
  if (clock >= retire_clock(sample)) {
    (sample.b ? result.cpu.b : result.cpu.a) = transferred(after, sample);
    ++result.cpu.pc;
    ++result.native_instructions;
    ++result.native_cycles;
  }
  return result;
}

void step(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Input sample) {
  const auto cycle = run.advance_cycle(machine, test::fetch_clock::input());
  if (!cycle.pipeline) { throw Fault("Own partial/TFR store lost its pipeline"); }
  const auto &phase = *cycle.pipeline;
  const auto clock = static_cast<unsigned>(cycle.completed_cycles);
  const DspProgramWord word{.address = instruction(sample), .value = opcode(sample)};
  if (clock == 4 + sample.gap) {
    require(phase.decode_accepted && phase.decode == word && phase.decode_words == 1,
            "The actual one-word F2 occurrence owns Decode");
    machine.bus().write({.address = instruction(sample), .value = illegal_word});
  }
  if (clock == write_clock(sample)) {
    require(phase.execute1 == word && phase.memory &&
                phase.memory->address.kind == DspAddressStageKind::register_agu &&
                phase.memory->address.pointer_update == (sample.mode != 4) &&
                phase.memory->address.update.index == sample.reg &&
                phase.memory->address.update.before == sample.address &&
                phase.memory->address.update.value == pointer(sample) &&
                phase.memory->data.kind == DspNativeAccessKind::write &&
                phase.memory->data.target.space == (sample.y ? DspSpace::y : DspSpace::x) &&
                phase.memory->data.target.address == sample.address &&
                phase.memory->data.value == stored(produced(before.cpu, sample), sample),
            "E1 performs exactly the old raw-part write after actual native AG1/AG2");
  }
  const bool source_wait = source_hold(sample) && clock == 7 + sample.gap;
  const unsigned fetches = clock == 1 || source_wait ? 0U : 1U;
  require(!source_wait || !cycle.issued_address, "A source wait holds genuine F1 as well as F2");
  require(cycle.bus.core_access_count == fetches + (clock == write_clock(sample) ? 1U : 0U) &&
              phase.memory.has_value() == (clock == write_clock(sample)) &&
              phase.memory_source_stall == source_wait && phase.agu_dependencies == 0 &&
              phase.retirement_published ==
                  ((clock >= 7 && clock <= 7 + sample.gap) || clock == retire_clock(sample)),
          "One genuine write and exact F2 footprint without invented stalls or retirement");
  const auto wanted = expected(before, sample, clock);
  const auto actual = machine.cpu().debug_state();
  if (actual != wanted) {
    std::cerr << "clock=" << clock << " source=" << sample.source << " part=" << sample.part
              << " producer=" << static_cast<unsigned>(sample.producer) << " gap=" << sample.gap
              << " mode=" << sample.mode << " reg=" << sample.reg << " b=" << sample.b
              << " y=" << sample.y << " modulo=" << sample.modulo << " pc=" << hex(actual.cpu.pc, 6)
              << '/' << hex(wanted.cpu.pc, 6) << " sr=" << hex(actual.cpu.sr, 6) << '/'
              << hex(wanted.cpu.sr, 6) << " a=" << hex(actual.cpu.a, 14) << '/'
              << hex(wanted.cpu.a, 14) << " b=" << hex(actual.cpu.b, 14) << '/'
              << hex(wanted.cpu.b, 14) << '\n';
  }
  require(actual == wanted, "Whole native state follows independent AG2/E1/E2 expectations");
  require(run.elapsed_ticks() == clock * period, "Exact period ledger includes every clock");
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : {sample.address - 1, sample.address, sample.address + 1}) {
      const unsigned value = clock >= write_clock(sample) && address == sample.address &&
                                     (space == DspSpace::y) == sample.y
                                 ? stored(produced(before.cpu, sample), sample)
                                 : canary;
      require(machine.bus().read({.space = space, .address = address}) == value,
              "Only the actual E1 cell changes; neighbors and the opposite space remain intact");
    }
  }
}

unsigned windows = 0;
unsigned restores = 0;
unsigned clones = 0;

void finish(DspFetchRun &run,
            DspMachine &machine,
            const DspDebugState &before,
            const Input sample) {
  while (run.completed_cycles() < retire_clock(sample)) { step(run, machine, before, sample); }
}

void one(const Input sample) {
  encoding(sample);
  DspMachine machine;
  const auto before = prepare(machine, sample);
  DspFetchRun run(contract(), machine);
  const bool base_cut = sample.producer == Producer::none && sample.source == 4 &&
                        sample.part == 4 && !sample.b && sample.mode == 3 && sample.variant == 1 &&
                        sample.scale == 0 &&
                        ((!sample.modulo && sample.flags == flag_patterns.at(0)) ||
                         (sample.modulo && !sample.y && sample.reg == 2 &&
                          sample.address == last_data && sample.flags == flag_patterns.at(1)));
  const bool producer_cut = sample.gap == 0 && !sample.y && sample.scale == 0 &&
                            sample.flags == flag_patterns.at(0) &&
                            ((sample.producer == Producer::add && !sample.producer_b &&
                              ((sample.part == 4 && !sample.b && sample.source == 4) ||
                               (sample.part == 5 && sample.b && sample.source == 0))) ||
                             (sample.producer == Producer::part && sample.producer_part == 0 &&
                              sample.part == 4 && !sample.b && sample.source == 4));
  const bool snapshots = base_cut || producer_cut;
  std::vector<DspFetchCheckpoint> cuts;
  while (run.completed_cycles() < retire_clock(sample)) {
    if (snapshots) { cuts.push_back(run.checkpoint(machine)); }
    step(run, machine, before, sample);
  }
  for (const auto &cut : cuts) {
    run.restore(cut, machine);
    finish(run, machine, before, sample);
    ++restores;
    DspMachine target;
    DspFetchRun clone(cut, target);
    finish(clone, target, before, sample);
    require(target.cpu().debug_state() == machine.cpu().debug_state(),
            "Fresh ownership retains captured words and genuine old-source write/TFR state");
    ++clones;
  }
  ++windows;
}

void variants(Input sample) {
  for (unsigned variant = 0; variant < a_values.size(); ++variant) {
    for (const auto scale : scales) {
      for (const auto flags : flag_patterns) {
        for (unsigned mode = 0; mode <= 4; ++mode) {
          sample.variant = variant;
          sample.scale = scale;
          sample.flags = flags;
          sample.mode = mode;
          sample.reg =
              (sample.source + sample.part + mode + (sample.b ? 1U : 0U) + (sample.y ? 2U : 0U)) &
              7U;
          one(sample);
        }
      }
    }
  }
}

void matrix() {
  for (unsigned source = 0; source < source_fields.size(); ++source) {
    for (const bool b : {false, true}) {
      for (const bool y : {false, true}) {
        for (unsigned part = 0; part < part_names.size(); ++part) {
          variants({.source = source, .part = part, .b = b, .y = y});
        }
      }
    }
  }
}

void wrap_variants(Input sample) {
  for (unsigned reg = 0; reg < 8; ++reg) {
    for (unsigned mode = 0; mode <= 4; ++mode) {
      for (const unsigned address : {data, last_data}) {
        sample.reg = reg;
        sample.mode = mode;
        sample.address = address;
        one(sample);
      }
    }
  }
}

void wraps() {
  for (const bool b : {false, true}) {
    for (const bool y : {false, true}) {
      for (const unsigned part : {4U, 5U}) {
        wrap_variants({.source = 4,
                       .part = part,
                       .variant = 1,
                       .flags = flag_patterns.at(1),
                       .b = b,
                       .y = y,
                       .modulo = true});
      }
    }
  }
}

void producer_variants(Input sample) {
  for (const unsigned source : {0U, 1U, 4U}) {
    for (unsigned gap = 0; gap < 3; ++gap) {
      for (const unsigned scale : {0U, 0x800U}) {
        for (const auto flags : flag_patterns) {
          sample.source = source;
          sample.gap = gap;
          sample.scale = scale;
          sample.flags = flags;
          sample.variant = 1;
          sample.reg = (sample.source + sample.part + sample.producer_part + gap +
                        (sample.producer_b ? 1U : 0U)) &
                       7U;
          one(sample);
        }
      }
    }
  }
}

void producers() {
  for (unsigned part = 0; part < fields.size(); ++part) {
    for (const bool b : {false, true}) {
      for (const bool y : {false, true}) {
        Input sample{.part = part, .b = b, .y = y, .producer = Producer::part};
        for (unsigned writer = 0; writer < fields.size(); ++writer) {
          sample.producer_part = writer;
          producer_variants(sample);
        }
        for (const auto producer : {Producer::transfer, Producer::add}) {
          sample.producer = producer;
          for (const bool bank : {false, true}) {
            sample.producer_b = bank;
            producer_variants(sample);
          }
        }
      }
    }
  }
}

void guards() {
  constexpr unsigned alu_mask = 0xff;
  constexpr unsigned unsupported_add = 0x40;
  for (unsigned part = 0; part < fields.size(); ++part) {
    Input sample{.source = 4, .part = part};
    const auto word = opcode(sample);
    require(parallel_partial_tfr_store(word) && !partial_accumulator_scalar_store(word) &&
                !fetch_repeat_body(word),
            "Parallel TFR stores do not widen standalone or REP body qualification");
    require(!decode_memory_stage((word & ~alu_mask) | unsupported_add),
            "Other parallel ALU/partial-store families retain their separate qualification");
    for (const unsigned mode : {5U, 6U, 7U}) {
      sample.mode = mode;
      require(!decode_memory_stage(opcode(sample)),
              "Pre-update and extended parallel partial stores remain outside this scope");
    }
  }
  std::cout << "PARALLEL_PARTIAL guards=30 passed\n";
}
}

int main() {
  try {
    std::cout << "PARALLEL_PARTIAL begin\n" << std::flush;
    matrix();
    wraps();
    std::cout << "PARALLEL_PARTIAL base-windows=" << windows << " passed\n" << std::flush;
    producers();
    guards();
    std::cout << "PARALLEL_PARTIAL windows=" << windows << " restores=" << restores
              << " clones=" << clones << " passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "PARALLEL_PARTIAL windows=" << windows << ": " << error.what() << '\n';
    return 1;
  }
}
