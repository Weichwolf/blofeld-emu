// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000;
constexpr unsigned base = 0x100;
constexpr unsigned ordinary_omr = 0x300;
constexpr unsigned status = 0xc00300;
constexpr unsigned jump = 0x0af080;
constexpr unsigned sr_immediate = 0x05f439;
constexpr unsigned omr_immediate = 0x05f43a;
constexpr unsigned unknown_load = 0x44a000; // FM AA6: MOVE X:$20,X0.
constexpr unsigned all_ones = 0xffffff;
constexpr unsigned cycle_limit = 16;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 540000000;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

DspDebugState prepare(DspMachine &machine, const std::optional<unsigned> opcode) {
  const std::array words{omr_immediate, ordinary_omr, sr_immediate, status, jump, base};
  machine.bus().install_rom({.start = boot, .words = words});
  if (opcode) { machine.bus().write({.address = base, .value = *opcode}); }
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  for (unsigned i = 0; i < 3; ++i) { machine.cpu().step(); }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == base && before.cpu.sr == status && before.cpu.omr == ordinary_omr,
          "Genuine own ROM setup enters the independently specified RAM instruction");
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

void advance(DspFetchRun &run, DspMachine &machine) {
  static_cast<void>(
      run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available}));
}

std::string refused(DspFetchRun &run, DspMachine &machine) {
  try {
    while (run.completed_cycles() < cycle_limit) { advance(run, machine); }
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Unknown actual instruction or operand was silently admitted");
}

void opcode_and_snapshot() {
  DspMachine machine;
  const auto before = prepare(machine, std::nullopt);
  DspFetchRun run(contract(), machine);
  advance(run, machine);
  advance(run, machine);
  const auto fetch = machine.reference_fetch();
  require(run.completed_cycles() == 2 && fetch.used == 1 && fetch.words.front().address == base &&
              !fetch.words.front().data,
          "The actual second-period F2 records unknown data without executing a NOP");
  const auto saved = run.checkpoint(machine);
  const auto message = "Unknown DSP program sample read P:" + hex(base, 6) + " pc=" + hex(base, 6) +
                       " omr=" + hex(ordinary_omr, 6);
  for (const unsigned replacement : {0U, all_ones}) {
    if (replacement != 0) {
      run.restore(saved, machine);
      require(machine.reference_fetch() == fetch,
              "Joint restore retains actual absent-value samples and their occurrence metadata");
    }
    machine.bus().write({.address = base, .value = replacement});
    const auto error = refused(run, machine);
    require(error.contains(message) && run.faulted() && run.completed_cycles() == 2 &&
                machine.cpu().debug_state() == before,
            "Decode refuses the retained unknown sample despite a later RAM write: " + error);
  }
}

void operand_refusal(const bool extension) {
  DspMachine machine;
  const auto before = prepare(machine, extension ? omr_immediate : unknown_load);
  if (!extension) {
    for (unsigned i = 1; i < cycle_limit; ++i) { machine.bus().write({.address = base + i}); }
  }
  DspFetchRun run(contract(), machine);
  const auto error = refused(run, machine);
  const auto expected = extension ? "Unknown DSP program sample read P:" + hex(base + 1, 6)
                                  : std::string("Unknown DSP memory read at X:0x000020");
  require(error.contains(expected) && error.contains("pc=" + hex(base, 6)) &&
              error.contains("omr=" + hex(ordinary_omr, 6)) && run.faulted() &&
              machine.cpu().debug_state() == before,
          "Unknown actual extension/data refuses with exact context before native effects: " +
              error);
}
}

int main() {
  try {
    opcode_and_snapshot();
    operand_refusal(true);
    operand_refusal(false);
    std::cout << "PASS unknown actual opcode,used extension,data and joint snapshot/write guards\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
