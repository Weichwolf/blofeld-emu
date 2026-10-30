// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_accumulator_transfer.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_accumulator_clock_fixture.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_mpy_recurrence_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "native_access_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test;
namespace transfer = blofeld::test::accumulator_clock;
constexpr unsigned entry = 0x100, tail = 32, bound = 32, normal_omr = 0x300;
constexpr unsigned mover = 0x200000, full_a = 14, full_b = 15;
constexpr unsigned multiply = 0x80, negative_product = 4, target_b = 8;
constexpr unsigned initial_flags = 0x3f, sr_move = 0x05f439, jump = 0x0af080;
constexpr unsigned seed_count = 13, scale_shift = 10;
constexpr unsigned omr_move = 0x05f43a, data_move = 0x44f400, extension_mask = 0xff;
constexpr std::uint64_t inactive_accumulator = 0xffb12345765432;
constexpr std::int64_t signed_span = 0x1000000;
constexpr unsigned sign24 = 0x800000;
constexpr std::array pairs{std::array{0U, 0U},
                           std::array{2U, 2U},
                           std::array{1U, 0U},
                           std::array{3U, 2U},
                           std::array{0U, 3U},
                           std::array{2U, 0U},
                           std::array{1U, 2U},
                           std::array{3U, 1U}};
constexpr std::array data_vectors{std::array{0x400000U, 0xe00000U, 0x600000U, 0xc00000U},
                                  std::array{0U, 1U, 0xffffffU, 0x800000U},
                                  std::array{0x800000U, 0x7fffffU, 0x800000U, 0x7fffffU}};

struct Sample {
  bool source_b = false, alu_b = false, negate = false;
  unsigned destination = 6, pair = 5, scale = 0, value = 1, data = 0;
  unsigned flags = initial_flags, producer = 0, gap = 0;

  [[nodiscard]] unsigned writer() const { return entry + 1 + (producer != 0 ? 1 + gap : 0); }

  [[nodiscard]] unsigned stall() const {
    return producer != 0 && gap == 0 && (producer == 2) == source_b ? 1U : 0U;
  }
};

unsigned opcode(const Sample sample) {
  return mover | ((sample.source_b ? full_b : full_a) << 13U) | (sample.destination << 8U) |
         multiply | (sample.pair << 4U) | (sample.alu_b ? target_b : 0U) |
         (sample.negate ? negative_product : 0U);
}

std::int64_t signed_word(const unsigned value) {
  return static_cast<std::int64_t>(value) - ((value & sign24) != 0 ? signed_span : 0);
}

std::uint64_t product(const DspState &cpu, const unsigned pair, const bool negate) {
  const std::array data{static_cast<unsigned>(cpu.x & word_mask),
                        static_cast<unsigned>((cpu.x >> word_bits) & word_mask),
                        static_cast<unsigned>(cpu.y & word_mask),
                        static_cast<unsigned>((cpu.y >> word_bits) & word_mask)};
  const auto operands = pairs.at(pair);
  const auto multiplied = 2 * signed_word(data.at(operands[0])) * signed_word(data.at(operands[1]));
  return static_cast<std::uint64_t>(negate ? -multiplied : multiplied) & accumulator_mask;
}

DspDebugState prepare(DspMachine &machine, const Sample sample) {
  DspProgram program;
  program.word(omr_move);
  program.word(normal_omr);
  const auto data = data_vectors.at(sample.data);
  for (unsigned index = 0; index < data.size(); ++index) {
    program.word(data_move | (index << 16U));
    program.word(data.at(index));
  }
  for (unsigned index = 0; index < transfer::seed_opcodes.size(); ++index) {
    const bool selected = (index >= 3) == sample.source_b;
    const auto value = selected ? transfer::values.at(sample.value) : inactive_accumulator;
    const auto part = index % 3;
    program.word(transfer::seed_opcodes.at(index));
    program.word(static_cast<unsigned>((value >> (part * word_bits)) &
                                       (part == 2 ? extension_mask : word_mask)));
  }
  const auto status = masked_status | sample.flags | (sample.scale << scale_shift);
  program.word(sr_move);
  program.word(status);
  program.word(jump);
  program.word(entry);
  program.install(machine.bus());
  for (unsigned pc = entry; pc < entry + tail; ++pc) { machine.bus().write({.address = pc}); }
  if (sample.producer != 0) {
    machine.bus().write(
        {.address = entry + 1,
         .value = mover | multiply | (5U << 4U) | (sample.producer == 2 ? target_b : 0U)});
  }
  machine.bus().write({.address = sample.writer(), .value = opcode(sample)});
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, seed_count);
  const auto before = machine.cpu().debug_state();
  const auto a = sample.source_b ? inactive_accumulator : transfer::values.at(sample.value);
  const auto b = sample.source_b ? transfer::values.at(sample.value) : inactive_accumulator;
  require(before.cpu.pc == entry && before.cpu.omr == normal_omr && before.cpu.sr == status &&
              before.cpu.a == a && before.cpu.b == b &&
              before.cpu.x == (std::uint64_t{data[1]} << word_bits | data[0]) &&
              before.cpu.y == (std::uint64_t{data[3]} << word_bits | data[2]),
          "Real native setup establishes independent full operands and status");
  return before;
}

struct Result {
  std::uint64_t value;
  unsigned status;
};

Result arithmetic(const DspState &cpu, const unsigned pair, const bool negate) {
  const auto value = product(cpu, pair, negate);
  constexpr unsigned changed = overflow | extension | unnormalized | negative | zero;
  return {.value = value,
          .status = (cpu.sr & ~changed) | value_flags({.value = value, .status = cpu.sr})};
}

void alu(DspState &cpu, const Result result, const bool b) {
  (b ? cpu.b : cpu.a) = result.value;
  cpu.sr = result.status;
}

void check(const DspMachine &machine, const DspDebugState &expected, const Sample sample) {
  const auto actual = machine.cpu().debug_state();
  if (actual != expected) {
    std::cerr << "opcode=" << hex(opcode(sample), 6) << " pc=" << hex(actual.cpu.pc, 6) << '/'
              << hex(expected.cpu.pc, 6) << " sr=" << hex(actual.cpu.sr, 6) << '/'
              << hex(expected.cpu.sr, 6) << " a=" << hex(actual.cpu.a, 14) << '/'
              << hex(expected.cpu.a, 14) << " b=" << hex(actual.cpu.b, 14) << '/'
              << hex(expected.cpu.b, 14) << " x=" << hex(actual.cpu.x, 12) << '/'
              << hex(expected.cpu.x, 12) << " y=" << hex(actual.cpu.y, 12) << '/'
              << hex(expected.cpu.y, 12) << '\n';
  }
  require(actual == expected,
          "Independent complete CPU/stack/counters;old transfer,new multiply and flags");
}

Result parallel(DspState &cpu, const Sample sample) {
  auto result = arithmetic(cpu, sample.pair, sample.negate);
  const auto moved =
      transfer::reference({.source = sample.source_b ? transfer::Source::b : transfer::Source::a,
                           .destination = sample.destination,
                           .scale = sample.scale,
                           .value = sample.source_b ? cpu.b : cpu.a});
  result.status |= moved.flags;
  transfer::destination(cpu, {.destination = sample.destination}, moved.value);
  return result;
}

void native(const Sample sample) {
  DspMachine machine;
  auto expected = prepare(machine, sample);
  // The directed native matrix has no preceding producer; automatic windows test it separately.
  require(sample.producer == 0, "Native operand matrix has a single parallel instruction");
  auto result = parallel(expected.cpu, sample);
  alu(expected.cpu, result, sample.alu_b);
  expected.cpu.pc += 2;
  expected.native_instructions += 2;
  expected.native_cycles += 2;
  machine.cpu().step();
  machine.cpu().step();
  check(machine, expected, sample);
}

struct Oracle {
  DspDebugState expected;
  std::optional<Result> producer = std::nullopt, parallel = std::nullopt;
  bool moved = false, done = false;
};

void clock(DspMachine &machine, DspFetchRun &run, const Sample sample, Oracle &oracle) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(cycle);
  const auto writer_e1 = sample.writer() - entry + 6 + sample.stall();
  if (phase.retirement_published) {
    if (!phase.execute2 || phase.execute2->address != oracle.expected.cpu.pc) {
      throw Fault("Actual ordered native E2 identity");
    }
    const auto pc = phase.execute2->address;
    if (pc == entry + 1 && sample.producer != 0) {
      if (!oracle.producer) { throw Fault("Actual preceding multiply has its own result"); }
      alu(oracle.expected.cpu, *oracle.producer, sample.producer == 2);
    } else if (pc == sample.writer()) {
      if (!oracle.parallel || !oracle.moved || cycle.completed_cycles != writer_e1 + 1) {
        throw Fault("Parallel ALU publishes exactly after its real transfer E1");
      }
      alu(oracle.expected.cpu, *oracle.parallel, sample.alu_b);
    }
    ++oracle.expected.cpu.pc;
    ++oracle.expected.native_instructions;
    ++oracle.expected.native_cycles;
    oracle.done = pc == sample.writer() + 3;
  }
  if (phase.execute1 && phase.execution_captured) {
    const auto pc = phase.execute1->address;
    if (pc == entry + 1 && sample.producer != 0) {
      oracle.producer = arithmetic(oracle.expected.cpu, 5, false);
    } else if (pc == sample.writer()) {
      require(phase.transfer && cycle.completed_cycles == writer_e1,
              "Full source transfer has its independently scheduled E1");
      oracle.parallel = parallel(oracle.expected.cpu, sample);
      oracle.moved = true;
    }
  }
  check(machine, oracle.expected, sample);
  require(run.elapsed_ticks() == cycle.completed_cycles * pipeline::period,
          "Clock advances independently of operand stalls");
}

void finish(DspMachine &machine, DspFetchRun &run, const Sample sample, Oracle &oracle) {
  for (unsigned count = 0; count < bound && !oracle.done; ++count) {
    clock(machine, run, sample, oracle);
  }
  require(oracle.done, "Bounded own multiply/transfer stream completes");
}

void stream(const Sample sample, const unsigned cut = 0) {
  DspMachine machine;
  Oracle oracle{.expected = prepare(machine, sample)};
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(transfer::contract(), machine);
  if (cut == 0) {
    finish(machine, run, sample, oracle);
    return;
  }
  while (run.completed_cycles() < cut) { clock(machine, run, sample, oracle); }
  const auto saved = run.checkpoint(machine);
  const auto prefix = oracle;
  finish(machine, run, sample, oracle);
  const auto end = oracle.expected;
  const auto ticks = run.elapsed_ticks();
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    auto suffix = prefix;
    finish(machine, run, sample, suffix);
    require(suffix.expected == end && run.elapsed_ticks() == ticks,
            "Trusted restored suffix retains literal endpoint and clock");
  }
  DspMachine target;
  DspFetchRun clone(saved, target);
  auto suffix = prefix;
  finish(target, clone, sample, suffix);
  require(suffix.expected == end && clone.elapsed_ticks() == ticks,
          "Fresh owner retains actual old-source transfer and pending ALU");
}

void guards() {
  constexpr unsigned prefix = mover | (full_a << 13U) | (6U << 8U);
  constexpr unsigned mpy_y0_x0 = 0xd0;
  // Other ALUs, raw accumulator parts and accumulator destinations remain separate contracts.
  constexpr std::array excluded{prefix | 0xd1U,
                                prefix | 0xd2U,
                                prefix | 0xd3U,
                                prefix | 0x10U,
                                prefix | 0x51U,
                                mover | (12U << 13U) | (6U << 8U) | mpy_y0_x0,
                                mover | (8U << 13U) | (6U << 8U) | mpy_y0_x0,
                                mover | (full_a << 13U) | (full_a << 8U) | mpy_y0_x0};
  for (const auto word : excluded) {
    require(!accumulator_transfer_source(word),
            "Unqualified neighbors have no full-source admission");
    const auto error = native_access::fault([&] {
      static_cast<void>(decode_pipeline_word(
          {.address = entry, .value = word}, normal_omr, DspFetchPipeline::automatic_memory));
    });
    require(error.contains("Unqualified automatic DSP decode") && error.contains("pc=") &&
                error.contains("omr="),
            "Unsupported parallel ALU/part/destination retains exact refusal context");
  }
  DspMachine machine;
  const Sample sample{.scale = 3};
  const auto before = prepare(machine, sample);
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(transfer::contract(), machine);
  pipeline::clocks(run, machine, 6);
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = native_access::fault(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("accumulator MOVE scaling") && error.contains("pc=") &&
                error.contains("sr=") && run.faulted() && run.completed_cycles() == 6 &&
                machine.cpu().debug_state() == before,
            "Reserved scaling refuses before real MPY/transfer effects in both trusted suffixes");
  }
}

template <typename Action>
void products(Action action, const bool source, const bool b, const bool negate) {
  for (unsigned pair = 0; pair < pairs.size(); ++pair) {
    for (const auto destination : transfer::destinations) {
      action(Sample{.source_b = source,
                    .alu_b = b,
                    .negate = negate,
                    .destination = destination,
                    .pair = pair});
    }
  }
  for (unsigned scale = 0; scale < 3; ++scale) {
    for (unsigned value = 0; value < transfer::values.size(); ++value) {
      for (unsigned data = 0; data < data_vectors.size(); ++data) {
        for (const unsigned flags : {initial_flags, extension_mask}) {
          action(Sample{.source_b = source,
                        .alu_b = b,
                        .negate = negate,
                        .scale = scale,
                        .value = value,
                        .data = data,
                        .flags = flags});
        }
      }
    }
  }
}

template <typename Action> void matrix(Action action) {
  for (const bool source : {false, true}) {
    for (const bool b : {false, true}) {
      for (const bool negate : {false, true}) { products(action, source, b, negate); }
    }
  }
}

void producers() {
  for (const bool source : {false, true}) {
    for (const bool b : {false, true}) {
      for (const unsigned producer : {1U, 2U}) {
        for (const unsigned gap : {0U, 1U}) {
          stream({.source_b = source, .alu_b = b, .producer = producer, .gap = gap});
        }
      }
    }
  }
}

void snapshots() {
  for (const bool source : {false, true}) {
    for (const bool b : {false, true}) {
      for (unsigned scale = 0; scale < 3; ++scale) {
        for (unsigned cut = 4; cut <= 10; ++cut) {
          stream({.source_b = source, .alu_b = b, .scale = scale, .value = 2, .producer = 1}, cut);
        }
      }
    }
  }
}
}

int main(const int argc, const char *const argv[]) {
  try {
    require(argc == 2, "Select native, clocks, producers, snapshots, guards or repro");
    const std::string_view selected(argv[1]);
    if (selected == "native") {
      matrix(native);
      std::cout << "MPY full transfers:1920 independent native cases\n";
    } else if (selected == "clocks") {
      matrix([](const Sample s) { stream(s); });
      std::cout << "MPY full transfers:1920 independent automatic streams\n";
    } else if (selected == "recurrence") {
      mpy_recurrence();
      std::cout
          << "MPY full transfers:7 dependent MPY/MACR stores;multiple wrap;2 suffixes/1 clone\n";
    } else if (selected == "repro") {
      stream({});
    } else if (selected == "guards") {
      guards();
      std::cout << "MPY full transfers:8 decode guards/2 restored scaling refusals\n";
    } else if (selected == "producers") {
      producers();
      std::cout << "MPY full transfers:16 producer windows\n";
    } else if (selected == "snapshots") {
      snapshots();
      std::cout << "MPY full transfers:84 cuts/168 suffixes/84 fresh owners\n";
    } else {
      throw Fault("Unknown multiply-transfer test selector");
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
