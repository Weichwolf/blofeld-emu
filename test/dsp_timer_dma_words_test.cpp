// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_bit_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "esai_fixture.hpp"

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
constexpr unsigned boot = 0xff0000, padding = 16, program_words = 40;
constexpr unsigned first = 0xffff82, tplr = 0xffff83, last = 0xffff8f, io = 0xffff80;
constexpr unsigned data_mask = 0xffffff, prescale_mask = 0x1fffff, prescale_bits = 21,
                   word_bits = 24;
constexpr unsigned source_period = 21, read_period = 22, write_period = 23, release_period = 24,
                   end_period = 25;
constexpr unsigned payload = 0x123456, replacement = 0x654321, canary = 0xa55a33;
constexpr unsigned extal = 12000000, timebase = 1080000000;
constexpr std::uint64_t reset_ticks = 180;
unsigned valid = 0, read_faults = 0, write_faults = 0, suffixes = 0, clones = 0;

struct Input {
  unsigned address, bit, operation;
  bool old;
};

bool prescaler(const Input input) {
  return input.address == first || input.address == tplr;
}

bool load_register(const Input input) {
  return input.address > tplr && (input.address & 3U) == 2;
}

bool writable(const Input input) {
  return input.address == tplr || (input.address & 3U) == 1;
}

unsigned seed(const Input input) {
  const auto mask = prescaler(input) ? prescale_mask : data_mask;
  return input.old ? mask : mask & ~(1U << input.bit);
}

unsigned changed(const Input input) {
  const auto initial = seed(input);
  const auto bit = 1U << input.bit;
  switch (input.operation) {
    case 0: return initial & ~bit;
    case 1: return initial | bit;
    case 2: return initial ^ bit;
    case 3: return initial;
    default: throw Fault("Invalid own timer bit operation");
  }
}

DspFetchRunInputs settings() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory,
          .resets = DspFetchResetTiming::reference_cycle_boundary};
}

void prepare(DspMachine &machine, DspFetchRun &run, const Input input) {
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Actual reset hold");
  run.release_hardware_reset(machine);
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Actual reset recovery");
  auto &timers = machine.bus().timers();
  const auto initial = seed(input);
  if (input.address == first) {
    constexpr unsigned timer2 = last - 8;
    constexpr unsigned pce_enable = 0x8001;
    timers.write({.address = tplr, .value = initial});
    timers.write({.address = timer2, .value = pce_enable});
    timers.write({.address = timer2, .value = 0});
  } else if (input.address > tplr && (input.address & 3U) == 0) {
    const auto csr = input.address + 3;
    timers.write({.address = csr - 1, .value = initial});
    timers.write({.address = csr - 2, .value = 0});
    timers.write({.address = csr, .value = 1});
    timers.advance_clk2(1);
    timers.write({.address = csr, .value = 0});
  } else {
    timers.write({.address = input.address, .value = initial});
  }
  if (!load_register(input)) {
    require(timers.read(input.address) == initial,
            "Independent setup establishes every readable timer word exactly");
  }
  machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = payload});
  machine.bus().write({.space = DspSpace::y, .address = dma::destination, .value = canary});
  banks::configure(machine.bus());
}

void check_dma(const DspFetchCycleResult &cycle, DspMachine &machine) {
  const auto period = cycle.completed_cycles;
  const auto &result = cycle.bus.dma;
  const bool active = period >= source_period && period <= release_period;
  require(
      !result.core_stall && result.phase.has_value() == active &&
          result.word.has_value() == (period == release_period),
      "One real IRQA request retains a bounded held transfer through timer E1/E2 and drains once");
  if (result.phase) {
    const bool source = period == source_period;
    require(result.phase->channel == 0 &&
                result.phase->kind ==
                    (source ? DspDmaPhaseKind::source_read : DspDmaPhaseKind::destination_write),
            "Actual DMA source and held destination stay independent of the timer register");
    if (!source) {
      require(result.phase->value == payload, "The held phase retains its actual captured word");
    }
  }
  if (result.word) {
    require(result.word->value == payload,
            "Real DMA commits its old payload after backing RAM mutation");
  }
  if (period == source_period) {
    machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = replacement});
  }
  require(machine.bus().read({.space = DspSpace::y, .address = dma::destination}) ==
              (period >= release_period ? payload : canary),
          "Destination changes once at the real drain period");
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, DspDebugState &expected) {
  const auto next = static_cast<unsigned>(run.completed_cycles() + 1);
  const bool held = next > source_period && next < release_period;
  const auto cycle =
      run.advance_cycle(machine,
                        {.known_requests = 15,
                         .requests = next == source_period ? 1U : 0U,
                         .grant = held ? DspDmaGrant::wait : DspDmaGrant::available});
  const auto &phase = test::pipeline::stages(cycle);
  const bool retired = next >= 7 && next != release_period;
  require(phase.retirement_published == retired,
          "Literal NOP/bit costs fix every native retirement and one post-bit hold");
  if (retired) {
    ++expected.cpu.pc;
    ++expected.native_instructions;
    expected.native_cycles += next == write_period ? 2U : 1U;
    if (next == write_period) {
      expected.cpu.sr = (expected.cpu.sr & ~1U) | static_cast<unsigned>(input.old);
    }
  }
  require(machine.cpu().debug_state() == expected,
          "Every period matches independent full CPU/stack/CCR truth");
  require(phase.memory.has_value() == (next == read_period),
          "The native selected bit is read once in actual E1");
  require(cycle.bus.bit_write.has_value() == (next == write_period && input.operation != 3),
          "Only actual mutating E2 writes once;BTST never writes");
  if (!load_register(input)) {
    require(machine.bus().timers().read(input.address) ==
                (next >= write_period ? changed(input) : seed(input)),
            "Native word writeback uses captured full data and the independent bit truth table");
  }
  check_dma(cycle, machine);
  require(run.timer_clk2_edges() == next / 2,
          "Real CLK/2 advances without clocking any disabled timer");
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, DspDebugState expected) {
  while (run.completed_cycles() < end_period) { step(run, machine, input, expected); }
}

void failure(DspFetchRun &run,
             DspMachine &machine,
             const Input input,
             DspDebugState expected,
             const bool read) {
  const auto error = test::esai::rejects([&] { step(run, machine, input, expected); });
  require(error.contains("pc=") && error.contains("omr=") && !error.contains("callback/route") &&
              !error.contains("with active DMA") &&
              (error.contains("Write-only") || error.contains("Read-only") ||
               error.contains("Unsupported DSP prescaler")),
          "Native timer read/write checks retain precise errors after the real route is admitted");
  if (read) {
    ++expected.cpu.pc;
    ++expected.native_instructions;
    ++expected.native_cycles;
  }
  require(
      run.faulted() && machine.cpu().debug_state() == expected &&
          run.completed_cycles() == (read ? source_period : read_period),
      "A failed timer read retains the older NOP;a failed E2 invents no bit retirement or Carry");
}

void window(const Input input) {
  DspMachine machine;
  std::array<std::uint32_t, program_words> program{};
  program.at(padding) = test::peripheral_bit::operations.at(input.operation).low |
                        ((input.address - io) << 8U) | input.bit;
  machine.bus().install_rom({.start = boot, .words = program});
  DspFetchRun run(settings(),
                  machine,
                  DspHardwareResetInputs{
                      .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  prepare(machine, run, input);
  auto state = machine.cpu().debug_state();
  while (run.completed_cycles() < source_period) { step(run, machine, input, state); }
  const bool read_error = load_register(input);
  const bool write_error =
      input.operation != 3 &&
      (!writable(input) || (input.address == tplr && changed(input) > prescale_mask));
  if (!read_error) { step(run, machine, input, state); }
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    if (read_error || write_error) {
      failure(run, machine, input, state, read_error);
    } else {
      finish(run, machine, input, state);
    }
    ++suffixes;
  }
  DspMachine target;
  DspFetchRun cloned(saved, target);
  if (read_error || write_error) {
    failure(cloned, target, input, state, read_error);
  } else {
    finish(cloned, target, input, state);
  }
  ++clones;
  if (read_error) {
    ++read_faults;
  } else if (write_error) {
    ++write_faults;
  } else {
    ++valid;
  }
}
}

int main() {
  try {
    for (unsigned address = first; address <= last; ++address) {
      if (address > tplr && (address & 3U) == 3) { continue; }
      for (unsigned bit = 0; bit < word_bits; ++bit) {
        for (const bool old : {false, true}) {
          if (old && prescaler({.address = address, .bit = bit, .operation = 0, .old = old}) &&
              bit >= prescale_bits) {
            continue;
          }
          if (old && address > tplr && (address & 3U) == 2) { continue; }
          for (unsigned operation = 0; operation < 4; ++operation) {
            window({.address = address, .bit = bit, .operation = operation, .old = old});
          }
        }
      }
    }
    std::cout << "Timer word bits valid=" << valid << " read-faults=" << read_faults
              << " write-faults=" << write_faults << " suffixes=" << suffixes
              << " clones=" << clones << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
