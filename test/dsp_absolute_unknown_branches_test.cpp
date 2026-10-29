// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000;
constexpr unsigned base = 0x100;
constexpr unsigned distant = 0x180;
constexpr unsigned sequential = base + 1;
constexpr unsigned status = 0xc00300;
constexpr unsigned ordinary_omr = 0x300;
constexpr unsigned jump = 0x0af080;
constexpr unsigned sr_immediate = 0x05f439;
constexpr unsigned omr_immediate = 0x05f43a;
constexpr unsigned jne_short = 0x0e2000; // FM13-80: NE, Z clear.
constexpr unsigned jeq_short = 0x0ea000; // EQ, Z set; false in the own seed.
constexpr unsigned return_opcode = 0x00000c;
constexpr unsigned all_ones = 0xffffff;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 540000000;
constexpr unsigned cycle_bound = 32;
constexpr unsigned branch_native_cost = 4;
constexpr unsigned tail_padding = 8;

struct Input {
  unsigned known_tail = 0;
  unsigned unused_value = 0;
  bool taken = true;
  bool same_address = false;
  bool write_after_sample = false;

  [[nodiscard]] unsigned target() const { return same_address ? sequential : distant; }

  [[nodiscard]] unsigned selected() const { return taken ? target() : sequential; }

  [[nodiscard]] bool fails() const { return !taken && known_tail == 0 && !write_after_sample; }

  [[nodiscard]] unsigned periods() const { return taken ? 11U : 12U; }
};

struct Sample {
  unsigned period;
  unsigned address;
  std::optional<unsigned> data;
  bool operator==(const Sample &) const = default;
};

struct Observation {
  std::vector<Sample> reads;
  bool branch_retired = false;
  bool target_retired = false;
  bool written = false;
  bool operator==(const Observation &) const = default;
};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

DspDebugState prepare(DspMachine &machine, const Input input) {
  const std::array setup{omr_immediate, ordinary_omr, sr_immediate, status, jump, base};
  machine.bus().install_rom({.start = boot, .words = setup});
  machine.bus().write(
      {.address = base, .value = (input.taken ? jne_short : jeq_short) | input.target()});
  for (unsigned i = 0; i < input.known_tail; ++i) {
    machine.bus().write({.address = sequential + i, .value = input.unused_value});
  }
  // All subsequent executable words are own NOPs. Only the first fallthrough
  // cell is deliberately absent in the selected unknown/rewrite vectors.
  for (unsigned i = 1; i < tail_padding; ++i) { machine.bus().write({.address = sequential + i}); }
  for (unsigned i = 0; i < tail_padding; ++i) { machine.bus().write({.address = distant + i}); }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  for (unsigned i = 0; i < 3; ++i) { machine.cpu().step(); }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == base && before.cpu.omr == ordinary_omr && before.cpu.sr == status,
          "Genuine own native setup establishes the literal conditional source and empty stack");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

DspFetchRunInputs contract() {
  return {.extal_hz = extal_hz,
          .timebase_hz = timebase_hz,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory};
}

DspDebugState expected(DspDebugState before, const Input input, const bool target) {
  before.cpu.pc = input.selected() + (target ? 1U : 0U);
  before.native_instructions += target ? 2U : 1U;
  before.native_cycles += branch_native_cost + (target ? 1U : 0U);
  return before;
}

void observe(DspMachine &machine,
             const DspFetchCycleResult &point,
             const Input input,
             const DspDebugState &before,
             Observation &observed) {
  const auto period = static_cast<unsigned>(point.completed_cycles);
  if (point.bus.fetch && point.bus.fetch->used != 0) {
    const auto read = point.bus.fetch->words.at(point.bus.fetch->used - 1);
    observed.reads.push_back({.period = period, .address = read.address, .data = read.data});
    if (read.address == sequential && !read.data && input.write_after_sample && !observed.written) {
      require(period == 3, "The original fallthrough F2 precedes the declared owned refill");
      machine.bus().write({.address = sequential});
      observed.written = true;
    }
  }
  if (!point.pipeline || !point.pipeline->retirement_published) { return; }
  const auto word = point.pipeline->execute2;
  if (!word) { throw Fault("Actual retirement lost its executed own instruction"); }
  if (word->address == base) {
    require(period == 7 && !observed.branch_retired &&
                machine.cpu().debug_state() == expected(before, input, false),
            "Actual branch E2 matches independent whole CPU,stack and native branch4 cost");
    observed.branch_retired = true;
  } else {
    require(word->address == input.selected() && observed.branch_retired &&
                !observed.target_retired &&
                machine.cpu().debug_state() == expected(before, input, true),
            "Only the selected actual NOP may follow the native branch");
    observed.target_retired = true;
  }
}

std::string suffix(DspMachine &machine,
                   DspFetchRun &run,
                   const Input input,
                   const DspDebugState &before,
                   Observation &observed) {
  try {
    while (!observed.target_retired) {
      require(run.completed_cycles() < cycle_bound, "Own conditional window exceeded its bound");
      const auto point =
          run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
      observe(machine, point, input, before, observed);
    }
  } catch (const Fault &error) { return error.what(); }
  return {};
}

void validate(const DspMachine &machine,
              const DspFetchRun &run,
              const Input input,
              const DspDebugState &before,
              const Observation &observed,
              const std::string &error) {
  if (input.fails()) {
    require(error.contains("Unknown DSP program sample read P:" + hex(sequential, 6)) &&
                run.faulted() && run.completed_cycles() == 7 && observed.branch_retired &&
                !observed.target_retired &&
                machine.cpu().debug_state() == expected(before, input, false),
            "The actually selected unknown fallthrough refuses before its CPU effects: " + error);
  } else {
    require(error.empty() && !run.faulted() && observed.target_retired &&
                run.completed_cycles() == input.periods() &&
                machine.cpu().debug_state() == expected(before, input, true),
            "Known actual refill preserves full state and primary false Decode delay: " + error);
  }
  const auto initial =
      Sample{.period = 3,
             .address = sequential,
             .data = input.known_tail != 0 ? std::optional(input.unused_value) : std::nullopt};
  require(observed.reads.size() >= 4 && observed.reads.at(1) == initial,
          "Every source-knowledge variant has the same actual pre-branch F2 address and period");
  const auto refill = Sample{.period = 6,
                             .address = input.selected(),
                             .data = input.fails() ? std::nullopt : std::optional(0U)};
  require(std::ranges::find(observed.reads, refill) != observed.reads.end(),
          "The existing reference owner performs a fresh F2 even for false/sequential targets");
}

void one(const Input input, const unsigned cut) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  DspFetchRun run(contract(), machine);
  Observation observed;
  while (run.completed_cycles() < cut) {
    const auto point =
        run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
    observe(machine, point, input, before, observed);
  }
  const auto saved = run.checkpoint(machine);
  const auto prefix = observed;
  const auto first_error = suffix(machine, run, input, before, observed);
  validate(machine, run, input, before, observed, first_error);
  const auto first = observed;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    observed = prefix;
    const auto error = suffix(machine, run, input, before, observed);
    validate(machine, run, input, before, observed, error);
    require(error == first_error && observed == first,
            "Coordinated branch checkpoints preserve actual F2 knowledge,CPU results and fault "
            "suffixes");
  }
}

void vector(const Input input) {
  for (unsigned cut = 3; cut <= 7; ++cut) { one(input, cut); }
  std::cout << "OBSERVED taken=" << input.taken << " sequential=" << input.same_address
            << " known-tail=" << input.known_tail << " rewrite=" << input.write_after_sample
            << " refused=" << input.fails() << " first-F2=3 refill-F2=6 cuts=5 suffixes=10\n";
}
}

int main() {
  try {
    for (const unsigned known : {0U, 1U, 2U}) {
      for (const unsigned value : {0U, return_opcode, all_ones}) {
        vector({.known_tail = known, .unused_value = value});
      }
    }
    vector({.known_tail = tail_padding, .taken = false});
    for (const bool taken : {false, true}) {
      vector({.known_tail = tail_padding, .taken = taken, .same_address = true});
      vector({.taken = taken, .same_address = true, .write_after_sample = true});
    }
    vector({.taken = false});
    vector({.taken = false, .write_after_sample = true});
    std::cout << "PASS16 short-absolute reference-schedule "
                 "observations,80cuts/160suffixes;false-discard hardware "
                 "contract remains open\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
