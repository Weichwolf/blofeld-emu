// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_dma_pipeline_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::dma_pipeline {
namespace {
constexpr unsigned entry = 0x100;
constexpr unsigned dma_target = 0x300;
constexpr unsigned program_word_copy = 0x880248;
constexpr unsigned short_branch_plus_three = 0x050c03;
constexpr unsigned long_movem_load_x1 = 0x07f085;
constexpr unsigned long_movem_store_a1 = 0x07708c;
constexpr unsigned program_store_target = 0x301;
constexpr unsigned data_mask = 0xffffff;
constexpr unsigned data_shift = 24;
constexpr unsigned max_cycles = 96;
constexpr unsigned filler_words = 32;
unsigned comparisons = 0;
unsigned held_periods = 0;
unsigned restored_suffixes = 0;

template <typename T> const T &known(const std::optional<T> &value) {
  if (!value) { throw Fault("Own timing fixture lost required native metadata"); }
  return *value;
}

struct Code {
  std::vector<std::uint32_t> words;
  unsigned control = 0;
  unsigned end = 0;
  unsigned result_a1 = 0;
  unsigned control_cost = 0;
  bool load = false;
  bool store = false;
  bool repeat = false;

  unsigned emit(const char *text) {
    const auto pc = entry + static_cast<unsigned>(words.size());
    const auto instruction = test::checkpoint::assemble(text);
    words.insert(words.end(), instruction.begin(), instruction.end());
    return pc;
  }
};

Code make_code(const unsigned index) {
  Code code;
  if (index == 0) {
    code.emit("clr a");
    code.control = entry + static_cast<unsigned>(code.words.size());
    code.words.push_back(short_branch_plus_three);
    code.emit("inc a");
    code.emit("inc a");
    code.emit("add x0,a");
    code.result_a1 = second;
    code.control_cost = 4;
  } else if (index == 1) {
    code.emit("clr a");
    code.emit("move #>1,x1");
    code.control = code.emit("rep #3");
    code.emit("nop");
    code.result_a1 = 0;
    code.repeat = true;
    code.control_cost = 5;
  } else if (index == 2) {
    code.emit("nop");
    code.control = entry + static_cast<unsigned>(code.words.size());
    code.words.push_back(long_movem_load_x1);
    code.words.push_back(dma_target);
    code.control_cost = 7;
    code.load = true;
  } else {
    code.emit("clr a");
    code.emit("add x0,a");
    code.control = entry + static_cast<unsigned>(code.words.size());
    code.words.push_back(long_movem_store_a1);
    code.words.push_back(program_store_target);
    code.control_cost = 7;
    code.result_a1 = second;
    code.store = true;
  }
  code.end = code.emit("nop");
  for (unsigned pad = 0; pad < filler_words; ++pad) { code.emit("nop"); }
  return code;
}

void install(Rig &rig, const Code &code) {
  test::checkpoint::ram(rig.machine.bus(), entry, code.words);
  rig.machine.bus().write({.address = dma_target, .value = untouched});
  rig.machine.bus().write({.address = program_store_target, .value = untouched});
}

void unchanged_code(Rig &rig, const Code &code) {
  for (std::size_t index = 0; index < code.words.size(); ++index) {
    require(rig.machine.bus().read({.address = entry + static_cast<unsigned>(index)}) ==
                code.words[index],
            "DMA data in another P word cannot alter the actual executable RAM program");
  }
}

struct Point {
  DspDebugState cpu;
  DspPipelineView pipeline;
  DspProgramFetchView fetch;
};

struct Reference {
  std::vector<Point> points;
  unsigned conflict = 0;
};

Reference reference(const Code &code) {
  Rig rig({}, program_word_copy, 0, dma_target, true);
  install(rig, code);
  Reference result;
  unsigned repeated_bodies = 0;
  result.points.push_back(
      {rig.machine.cpu().debug_state(), rig.pipeline(), rig.machine.reference_fetch()});
  for (unsigned step = 1; step < max_cycles; ++step) {
    const auto before = rig.machine.reference_fetch();
    const auto cpu = rig.machine.cpu().debug_state();
    const auto point = rig.tick();
    const auto &stages = known(point.pipeline);
    require(!point.bus.dma.phase && !point.bus.dma.word,
            "The quiet enabled channel cannot invent an external request");
    if (stages.execute1 && stages.execute1->address == code.control && before.pending_address &&
        point.bus.fetch && result.conflict == 0) {
      result.conflict = step;
    }
    if (stages.retirement_published && stages.execute2 &&
        stages.execute2->address == code.control) {
      require(rig.machine.cpu().debug_state().native_cycles ==
                  cpu.native_cycles + code.control_cost,
              "Literal independent branch,REP and long MOVEM instruction costs remain intact");
    }
    if (code.repeat && stages.retirement_published && stages.execute2 &&
        stages.execute2->address == code.control + 1) {
      ++repeated_bodies;
    }
    result.points.push_back(
        {rig.machine.cpu().debug_state(), rig.pipeline(), rig.machine.reference_fetch()});
    unchanged_code(rig, code);
    if (stages.retirement_published && stages.execute2 && stages.execute2->address == code.end) {
      require(result.conflict > 1, "Each control case must expose an actual pending F2 at E1");
      if (code.repeat) {
        require(repeated_bodies == 3, "The real REP body retires exactly three literal NOPs");
      }
      if (!code.load) {
        require(rig.machine.cpu().state().a ==
                    (static_cast<std::uint64_t>(code.result_a1) << data_shift),
                "The branch skips both INCs,REP leaves cleared A intact and the ALU store keeps "
                "its source");
      }
      require(rig.memory(DspSpace::p, program_store_target) == (code.store ? second : untouched),
              "The adjacent ALU writer supplies the literal native A1 program-memory store");
      return result;
    }
  }
  throw Fault("Own timing reference exceeded its fixed cycle bound");
}

void matches(Rig &rig, const Code &code, const Point &expected) {
  auto cpu = expected.cpu;
  if (code.load && ((cpu.cpu.x >> data_shift) & data_mask) == untouched) {
    cpu.cpu.x = (cpu.cpu.x & data_mask) | (static_cast<std::uint64_t>(first) << data_shift);
  }
  require(rig.machine.cpu().debug_state() == cpu,
          "Every actual CPU,stack and native-counter field matches the literal-data-adjusted "
          "reference");
  const auto pipeline = rig.pipeline();
  require(pipeline.stages == expected.pipeline.stages &&
              pipeline.awaiting_retirement == expected.pipeline.awaiting_retirement &&
              pipeline.decode_address == expected.pipeline.decode_address,
          "Every real pipeline owner resumes its same reference position after DMA hold");
  const auto fetch = rig.machine.reference_fetch();
  require(fetch.pending_address == expected.fetch.pending_address &&
              fetch.used == expected.fetch.used && fetch.words == expected.fetch.words,
          "Actual issued/captured program words survive the whole DMA conflict");
  unchanged_code(rig, code);
  ++comparisons;
}

void suffix(Rig &rig, const Code &code, const Reference &quiet, const unsigned waits) {
  const auto cpu = rig.machine.cpu().debug_state();
  const auto pipeline = rig.pipeline();
  const auto fetch = rig.machine.reference_fetch();
  for (unsigned held = 0; held <= waits; ++held) {
    const auto point = rig.tick(held == waits ? DspDmaGrant::available : DspDmaGrant::wait, 1);
    require(point.bus.dma.core_stall && point.bus.dma.phase &&
                point.bus.dma.phase->kind == DspDmaPhaseKind::destination_write &&
                point.bus.dma.phase->value == first &&
                static_cast<bool>(point.bus.dma.word) == (held == waits),
            "The genuine captured P-destination word owns each held cycle including completion");
    require(
        rig.machine.cpu().debug_state() == cpu && rig.pipeline().stages == pipeline.stages &&
            rig.pipeline().awaiting_retirement == pipeline.awaiting_retirement &&
            rig.machine.reference_fetch().pending_address == fetch.pending_address,
        "The actual F2/E2/AG2 and nominal-control owners remain frozen throughout the conflict");
    unchanged_code(rig, code);
    ++held_periods;
  }
  for (unsigned index = quiet.conflict; index < quiet.points.size(); ++index) {
    const auto point = rig.tick(DspDmaGrant::available, 1);
    require(!point.bus.dma.core_stall && !point.bus.dma.phase,
            "The one-shot DMA transfer cannot replay after the native pipeline resumes");
    matches(rig, code, quiet.points.at(index));
  }
  require(rig.run().completed_cycles() == quiet.points.size() - 1 + waits + 1 &&
              rig.memory(DspSpace::p, dma_target) == first,
          "Completion retains every nominal control period plus exactly the real DMA hold cycles");
  require(rig.memory(DspSpace::p, program_store_target) == (code.store ? second : untouched),
          "DMA holds preserve the actual ALU-to-MOVEM source readiness and literal data");
  ++restored_suffixes;
}

void one_case(const Code &code) {
  const auto quiet = reference(code);
  Rig rig({}, program_word_copy, 0, dma_target, true);
  install(rig, code);
  for (unsigned step = 1; step < quiet.conflict; ++step) {
    const bool trigger = step + 1 == quiet.conflict;
    const auto point = rig.tick(DspDmaGrant::available, trigger ? 1U : 0U);
    if (trigger) {
      require(point.bus.dma.phase && point.bus.dma.phase->kind == DspDmaPhaseKind::source_read &&
                  !point.bus.dma.word && !point.bus.dma.core_stall,
              "One real source capture precedes the selected F2/control-latency conflict");
    }
    matches(rig, code, quiet.points.at(step));
  }
  const auto checkpoint = rig.run().checkpoint(rig.machine);
  for (const unsigned waits : {1U, 3U, 5U}) {
    for (unsigned replay = 0; replay < 2; ++replay) {
      rig.run().restore(checkpoint, rig.machine);
      suffix(rig, code, quiet, waits);
    }
  }
}

void read_only_status() {
  for (const bool active : {false, true}) {
    Rig rig({"movep #>0,x:<<$fffff4"}, word_copy & (active ? data_mask : ~enable));
    for (unsigned index = 0; index < max_cycles && !rig.approaching(rig.body_pc.front()); ++index) {
      static_cast<void>(rig.tick());
    }
    require(rig.approaching(rig.body_pc.front()),
            "The real status write reaches its owned E1 boundary");
    const auto before = rig.machine.cpu().debug_state();
    const auto time = rig.run().elapsed_ticks();
    const auto dma_cycle = rig.machine.bus().last_dma_cycle();
    const auto error = reject([&] { static_cast<void>(rig.tick()); },
                              "Read-only DMA status writes must be refused during preflight");
    require(error.find("0xfffff4") != std::string::npos && error.find("pc=") != std::string::npos &&
                rig.machine.cpu().debug_state() == before && rig.run().elapsed_ticks() == time &&
                rig.machine.bus().last_dma_cycle() == dma_cycle,
            "Invalid status writes cannot sample DMA,retire older E2 or complete elapsed time");
  }
}
}
}

int main() {
  try {
    for (unsigned index = 0; index < 4; ++index) {
      std::cout << "case=" << index << '\n';
      blofeld::test::dma_pipeline::one_case(blofeld::test::dma_pipeline::make_code(index));
    }
    blofeld::test::dma_pipeline::read_only_status();
    std::cout << "cases=4 full-state-comparisons=" << blofeld::test::dma_pipeline::comparisons
              << " held-periods=" << blofeld::test::dma_pipeline::held_periods
              << " restored-suffixes=" << blofeld::test::dma_pipeline::restored_suffixes
              << " assertions=" << blofeld::test::dma_pipeline::assertions
              << " expected-refusals=" << blofeld::test::dma_pipeline::refusals << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
