// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned guards = 4;
constexpr unsigned anchor = pipeline::body + guards + 2 + guards;
constexpr unsigned producer = anchor + 1;
constexpr unsigned maximum_clocks = 160;
constexpr unsigned register_do = 0x06c000;
constexpr unsigned asr_a = 0x200022;
constexpr unsigned move_x0_a = 0x208e00;
constexpr unsigned immediate_x0 = 0x44f400;
constexpr unsigned ccr_arithmetic = 0x3f;
constexpr unsigned unnormalized = 0x10;
constexpr unsigned zero = 4;
constexpr std::uint64_t low_word = 0xffffff;

enum class Source : std::uint8_t { x0 = 4, r0 = 16, a = 14 };

struct Case {
  Source source = Source::x0;
  unsigned count = 2;
  unsigned size = 3;
  unsigned gap = 0;
  bool arithmetic = false;
  bool register_move = false;

  [[nodiscard]] const char *name() const {
    switch (source) {
      case Source::x0: return "x0";
      case Source::r0: return "r0";
      case Source::a: return "a";
      default: throw Fault("Unknown independent DO source fixture");
    }
  }

  [[nodiscard]] unsigned producer_words() const { return arithmetic || register_move ? 1U : 2U; }

  [[nodiscard]] unsigned setup() const { return producer + producer_words() + gap; }

  [[nodiscard]] unsigned first() const { return setup() + 2; }

  [[nodiscard]] unsigned after() const { return first() + size; }

  [[nodiscard]] unsigned decode_clock() const { return 14 + producer_words() + gap; }

  [[nodiscard]] unsigned interlock() const {
    return (arithmetic || register_move) && gap == 0 ? 1U : 0U;
  }

  [[nodiscard]] unsigned setup_delay() const {
    if (count == 0) { return 8; }
    return size == 1 ? 6U : 5U;
  }
};

struct Flow {
  std::vector<unsigned> pcs;
  std::vector<unsigned> next;

  void sequential(const unsigned pc, const unsigned words = 1) {
    pcs.push_back(pc);
    next.push_back(pc + words);
  }
};

Flow expected_flow(const Case sample) {
  Flow flow;
  const auto prefix_words = sample.register_move ? 2U : 0U;
  if (sample.register_move) { flow.sequential(pipeline::body, 2); }
  for (unsigned pc = pipeline::body + prefix_words; pc < pipeline::body + guards; ++pc) {
    flow.sequential(pc);
  }
  flow.sequential(pipeline::body + guards, 2);
  for (unsigned pc = pipeline::body + guards + 2; pc <= anchor; ++pc) { flow.sequential(pc); }
  flow.sequential(producer, sample.producer_words());
  for (unsigned pc = producer + sample.producer_words(); pc < sample.setup(); ++pc) {
    flow.sequential(pc);
  }
  flow.sequential(sample.setup(), 2);
  flow.next.back() = sample.count == 0 ? sample.after() : sample.first();
  for (unsigned iteration = 0; iteration < sample.count; ++iteration) {
    for (unsigned pc = sample.first(); pc < sample.after(); ++pc) { flow.sequential(pc); }
    flow.next.back() = iteration + 1 == sample.count ? sample.after() : sample.first();
  }
  return flow;
}

void prepare(DspMachine &machine, const Case sample) {
  const auto seed_count = sample.arithmetic ? 2 * sample.count : sample.count + 3;
  const auto seed = "move #>" + number(seed_count) + "," + sample.name();
  const auto move = "move #>" + number(sample.count) + "," + sample.name();
  const auto loop = "do " + std::string(sample.name()) + "," + number(sample.after());
  const auto x0_seed = "move #>" + number(sample.count) + ",x0";
  std::vector<const char *> code;
  if (sample.register_move) { code.push_back(x0_seed.c_str()); }
  code.insert(code.end(), guards - (sample.register_move ? 2U : 0U), "nop");
  code.push_back(seed.c_str());
  code.insert(code.end(), guards + 1, "nop");
  const auto *operation = sample.arithmetic ? "asr a" : move.c_str();
  if (sample.register_move) { operation = "move x0,a"; }
  code.push_back(operation);
  code.insert(code.end(), sample.gap, "nop");
  code.push_back(loop.c_str());
  code.insert(code.end(), sample.size, "nop");
  pipeline::prepare(machine, code);
  const auto opcode = register_do | (static_cast<unsigned>(sample.source) << 8);
  require(machine.bus().inspect_program(sample.setup()) == opcode &&
              machine.bus().inspect_program(sample.setup() + 1) == sample.after() - 1,
          "Actual installed DO opcode/source/last-address fields match independent encodings");
  if (sample.register_move) {
    require(machine.bus().inspect_program(producer) == move_x0_a &&
                machine.bus().inspect_program(pipeline::body) == immediate_x0 &&
                machine.bus().inspect_program(pipeline::body + 1) == sample.count,
            "Actual single-word MOVE and native X0 seed match independent bit fields");
  }
  if (sample.arithmetic) {
    require(machine.bus().inspect_program(producer) == asr_a,
            "Actual ASR A agrees with the independently encoded FM13-17 instruction");
  }
}

void final_state(const DspMachine &machine,
                 const DspDebugState &before,
                 const Case sample,
                 const Flow &flow) {
  auto expected = before;
  expected.cpu.pc = sample.after();
  switch (sample.source) {
    case Source::x0: expected.cpu.x = (before.cpu.x & ~low_word) | sample.count; break;
    case Source::r0: expected.cpu.r.front() = sample.count; break;
    case Source::a: expected.cpu.a = static_cast<std::uint64_t>(sample.count) << word_bits; break;
    default: throw Fault("Unknown independent DO result source");
  }
  if (sample.register_move) { expected.cpu.x = (before.cpu.x & ~low_word) | sample.count; }
  if (sample.arithmetic) {
    expected.cpu.sr = (before.cpu.sr & ~ccr_arithmetic) | unnormalized;
    if (sample.count == 0) { expected.cpu.sr |= zero; }
  }
  expected.stack_high.at(1) = before.cpu.la;
  expected.stack_low.at(1) = before.cpu.lc;
  expected.stack_high.at(2) = sample.first();
  expected.stack_low.at(2) = expected.cpu.sr;
  const auto actual = machine.cpu().debug_state();
  require(actual.cpu == expected.cpu && actual.stack_high == expected.stack_high &&
              actual.stack_low == expected.stack_low && actual.sc == expected.sc &&
              actual.ep == expected.ep && actual.sz == expected.sz &&
              actual.native_instructions == before.native_instructions + flow.pcs.size(),
          "Independent source/CCR/full CPU/physical stack/frame/count expectations hold");
}

struct Observation {
  unsigned retired = 0;
  unsigned target_decode = 0;
  unsigned stalls = 0;
};

void check_phases(const DspFetchCycleResult &cycle,
                  const unsigned clock,
                  const Case sample,
                  Observation &observed) {
  const auto &phase = pipeline::stages(cycle);
  const auto target = sample.count == 0 ? sample.after() : sample.first();
  if (phase.decode_accepted && phase.decode && phase.decode->address == sample.setup()) {
    require(clock == sample.decode_clock(), "DO Decode follows independent prefix word bandwidth");
  }
  if (phase.decode_accepted && phase.decode && phase.decode->address == target &&
      observed.target_decode == 0) {
    observed.target_decode = clock;
  }
  if (phase.memory_source_stall) {
    ++observed.stalls;
    require(sample.interlock() == 1 && clock == sample.decode_clock() + 3 && phase.execute1 &&
                phase.execute1->address == sample.setup() && !phase.execution_captured &&
                phase.retirement_published && !cycle.issued_address,
            "The actual one-period source interlock retires the producer and holds DO E1/F1");
  }
  if (clock == sample.decode_clock() + 3 + sample.interlock()) {
    require(phase.execute1 && phase.execute1->address == sample.setup() &&
                phase.execution_captured && cycle.issued_address == target,
            "Actual native DO E1 selects its first F1 after the independently expected interlock");
  }
  if (clock == sample.decode_clock() + 4 + sample.interlock()) {
    require(phase.execute2 && phase.execute2->address == sample.setup() &&
                phase.retirement_published && cycle.bus.fetch && cycle.bus.fetch->used != 0 &&
                cycle.bus.fetch->words.at(cycle.bus.fetch->used - 1).address == target,
            "Actual native DO E2 publishes while the selected first F2 is sampled");
  }
}

void run_case(const Case sample) {
  DspMachine machine;
  prepare(machine, sample);
  const auto before = machine.cpu().debug_state();
  const auto flow = expected_flow(sample);
  auto inputs = pipeline::contract();
  inputs.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(inputs, machine);
  Observation observed;
  for (unsigned clock = 1; clock < maximum_clocks; ++clock) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    check_phases(cycle, clock, sample, observed);
    const auto &phase = pipeline::stages(cycle);
    if (phase.retirement_published) {
      require(observed.retired < flow.pcs.size() && phase.execute2 &&
                  phase.execute2->address == flow.pcs.at(observed.retired) &&
                  machine.cpu().state().pc == flow.next.at(observed.retired),
              "Every native E2 matches independent real producer/DO/iteration PC flow");
      ++observed.retired;
    }
    require(cycle.bus.core_access_count <= 1,
            "Only one real P read is admitted per period in this memory-free stream");
    if (observed.retired == flow.pcs.size() && observed.target_decode != 0) { break; }
  }
  require(observed.retired == flow.pcs.size() && observed.stalls == sample.interlock() &&
              observed.target_decode ==
                  sample.decode_clock() + sample.setup_delay() + sample.interlock(),
          "DO register counts use independent native source/interlock/setup/zero timing");
  final_state(machine, before, sample, flow);
}

void matrix() {
  for (const unsigned count : std::array{0U, 1U, 2U, 4U}) {
    for (const unsigned size : std::array{1U, 3U}) {
      for (const unsigned gap : std::array{0U, 1U}) {
        run_case(
            {.source = Source::a, .count = count, .size = size, .gap = gap, .register_move = true});
      }
    }
  }
  for (const unsigned gap : std::array{2U, 3U, 4U}) {
    run_case({.source = Source::a, .gap = gap, .register_move = true});
  }
}
}

int main(const int argc, const char *const *const argv) {
  try {
    if (argc == 2 && std::string_view(argv[1]) == "interlock") {
      run_case({.source = Source::a, .arithmetic = true});
    } else if (argc == 2 && std::string_view(argv[1]) == "transfer-interlock") {
      run_case({.source = Source::a, .register_move = true});
    } else {
      require(argc == 1, "Unknown DO source fixture argument");
      matrix();
    }
    std::cout
        << "Automatic native DO register source and independent producer/interlock cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
