// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using test::require;
namespace banks = test::banks;
namespace dma = test::dma_test;
constexpr unsigned boot = 0xff0000, padding = 16, program_words = 36;
constexpr unsigned extal = 12000000, timebase = 1080000000;
constexpr unsigned first = 0xffff82, tplr = 0xffff83, last = 0xffff8f, io = 0xffff80;
constexpr unsigned store_opcode = 0x07f400, store_period = 22, retire_period = 23,
                   last_nop_period = 22;
constexpr unsigned payload = 0x123456, canary = 0xa55a33, initial = 0x154321;
constexpr unsigned load = 17, compare = 31, source_select = 0x200000, reserved_control = 0x400000;
constexpr std::uint64_t reset_ticks = 180;
unsigned writes = 0, faults = 0, suffixes = 0;

struct Input {
  unsigned address, value, request;
  bool valid = true;
};

DspFetchRunInputs settings() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory,
          .resets = DspFetchResetTiming::reference_cycle_boundary};
}

void prepare(DspMachine &machine, DspFetchRun &run) {
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks, "Real reset hold");
  run.release_hardware_reset(machine);
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Real reset recovery");
  for (unsigned timer = 0; timer < 3; ++timer) {
    const auto control = last - (4 * timer);
    machine.bus().timers().write({.address = control - 1, .value = load});
    machine.bus().timers().write({.address = control - 2, .value = compare});
  }
  machine.bus().timers().write({.address = tplr, .value = initial});
  machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = payload});
  machine.bus().write({.space = DspSpace::y, .address = dma::destination, .value = canary});
  banks::configure(machine.bus());
}

void check_port(DspMachine &machine, const Input input, const unsigned period) {
  const auto &timers = machine.bus().timers();
  const bool changed = period >= store_period;
  if (input.address == tplr) {
    require(timers.read(tplr) == (changed ? input.value : initial),
            "Native E1 writes the actual TPLR latch");
    return;
  }
  const auto kind = input.address & 3U;
  if (kind == 1) {
    require(timers.read(input.address) == (changed ? input.value : compare),
            "Native E1 writes the actual TCPR latch");
  } else if (kind == 3) {
    require(timers.read(input.address, 1) == (changed ? input.value : 0U),
            "Native E1 retains actual TE control");
    require(timers.read(input.address - 3) ==
                (period >= store_period && input.value == 1 ? load : 0U),
            "Only a real subsequent CLK/2 edge preloads an enabled counter");
  }
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, DspDebugState &expected) {
  const auto next = static_cast<unsigned>(run.completed_cycles() + 1);
  const auto cycle = run.advance_cycle(machine,
                                       {.known_requests = 15,
                                        .requests = next == input.request ? 1U : 0U,
                                        .grant = DspDmaGrant::available});
  const auto &phase = test::pipeline::stages(cycle);
  const bool retired = (next >= 7 && next <= last_nop_period) || next == retire_period;
  require(phase.retirement_published == retired,
          "Independent two-word MOVEP/NOP pipeline costs fix retirements");
  if (retired) {
    expected.cpu.pc += next == retire_period ? 2U : 1U;
    ++expected.native_instructions;
    expected.native_cycles += next == retire_period ? 2U : 1U;
  }
  require(machine.cpu().debug_state() == expected,
          "The actual full CPU/CCR/stack matches the literal own program");
  require(phase.memory.has_value() == (next == store_period),
          "Exactly one native E1 peripheral transaction: period=" + std::to_string(next) +
              " memory=" + std::to_string(static_cast<unsigned>(phase.memory.has_value())) +
              " bus-memory=" + std::to_string(static_cast<unsigned>(cycle.bus.memory.has_value())));
  if (phase.memory) {
    const auto &access = phase.memory->data;
    require(access.kind == DspNativeAccessKind::write && access.target.address == input.address &&
                access.value == input.value && access.pc == boot + padding,
            "Native MOVEP produces the expected address, payload and instruction context");
  }
  const auto &dma_result = cycle.bus.dma;
  require(!dma_result.core_stall && dma_result.word.has_value() == (next == input.request + 1),
          "Independent RAM DMA completes once without stealing a timer-store cycle");
  if (dma_result.word) {
    require(dma_result.word->value == payload, "Real DMA source payload is retained");
  }
  require(machine.bus().read({.space = DspSpace::y, .address = dma::destination}) ==
              (next > input.request ? payload : canary),
          "DMA changes its destination only after the requested two-cycle transfer");
  check_port(machine, input, next);
}

void window(const Input input) {
  DspMachine machine;
  std::array<std::uint32_t, program_words> program{};
  // FM13-135: MOVEP immediate-to-qq, effective-address field $34.
  // FM table2-1: opcode E1 overlaps the following extension E2;
  // the extension does not insert a cycle before opcode E1.
  program.at(padding) = store_opcode | (input.address - io);
  program.at(padding + 1) = input.value;
  machine.bus().install_rom({.start = boot, .words = program});
  DspFetchRun run(settings(),
                  machine,
                  DspHardwareResetInputs{
                      .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  prepare(machine, run);
  auto expected = machine.cpu().debug_state();
  while (run.completed_cycles() < store_period - 1) { step(run, machine, input, expected); }
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    auto state = expected;
    if (input.valid) {
      while (run.completed_cycles() < retire_period) { step(run, machine, input, state); }
      if ((input.address & 3U) == 2) {
        const auto control = input.address + 1;
        machine.bus().timers().write({.address = control, .value = 1});
        machine.bus().timers().advance_clk2(1);
        require(machine.bus().timers().read(control - 3) == input.value,
                "A future real preload independently proves the write-only TLR payload");
      }
      ++suffixes;
    } else {
      bool failed = false;
      try {
        step(run, machine, input, state);
      } catch (const Fault &error) {
        const std::string message(error.what());
        require(!message.contains("with active DMA") && message.contains("pc=") &&
                    (message.contains("Read-only") || message.contains("Unsupported DSP")),
                "Native timer validation retains its precise read-only/source/reserved failure");
        failed = true;
      }
      auto older = expected;
      ++older.cpu.pc;
      ++older.native_instructions;
      ++older.native_cycles;
      require(failed && run.faulted() && machine.cpu().debug_state() == older,
              "Invalid timer writes cannot fabricate successful retirement or CPU flags");
    }
  }
  input.valid ? ++writes : ++faults;
}
}

int main() {
  try {
    for (unsigned address = tplr; address <= last; ++address) {
      if ((address & 3U) == 0) { continue; }
      for (const unsigned request : {21U, 22U, 23U}) {
        for (const unsigned value : {0U, 1U}) {
          window({.address = address, .value = value, .request = request});
        }
      }
    }
    for (const unsigned address : {first, first + 2, first + 6, first + 10}) {
      window({.address = address, .value = 0, .request = last_nop_period, .valid = false});
    }
    window({.address = tplr, .value = source_select, .request = last_nop_period, .valid = false});
    for (unsigned timer = 0; timer < 3; ++timer) {
      window({.address = last - (4 * timer),
              .value = reserved_control,
              .request = last_nop_period,
              .valid = false});
    }
    std::cout << "Timer MOVEP writes=" << writes << " faults=" << faults << " suffixes=" << suffixes
              << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
