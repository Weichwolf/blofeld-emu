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
// FM13-136/Table12-13 and independent asm56300 low-X/X0 template $044480.
constexpr unsigned read_opcode = 0x044480, di = 0x1000, prescale_enable = 0x8001;
constexpr unsigned capture_period = 20, e1_period = 22, end_period = 26, release_period = 25;
constexpr unsigned prescale = 3, new_prescale = 7, compare = 31, new_compare = 42;
constexpr unsigned payload = 0x123456, replacement = 0x654321, canary = 0xa55a33;
constexpr unsigned extal = 12000000, timebase = 1080000000;
constexpr std::uint64_t reset_ticks = 180;
unsigned windows = 0, faults = 0, suffixes = 0, clones = 0;

enum class Failure : std::uint8_t { none, write_only, unknown_di, unknown_compare };

struct Input {
  unsigned address, request;
  bool wait = false, level = false;
  Failure failure = Failure::none;
};

DspFetchRunInputs settings() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory,
          .resets = DspFetchResetTiming::reference_cycle_boundary};
}

unsigned sample(const Input input) {
  constexpr unsigned sampled_edges = (capture_period / 2) - 1;
  if (input.address == first) { return prescale - (sampled_edges % (prescale + 1)); }
  if (input.address == tplr) { return prescale; }
  const auto index = (last - input.address) / 4;
  switch (input.address & 3U) {
    case 0: return index == 2 ? (sampled_edges / (prescale + 1)) - 1 : sampled_edges - 1;
    case 1: return compare;
    case 3: return index == 2 ? prescale_enable : 1U | (input.level ? di : 0U);
    default: throw Fault("Own read oracle cannot sample a write-only timer load");
  }
}

void prepare(DspMachine &machine, DspFetchRun &run, const Input input) {
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Actual reset hold");
  run.release_hardware_reset(machine);
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Actual reset recovery");
  auto &timers = machine.bus().timers();
  timers.write({.address = tplr, .value = prescale});
  for (unsigned index = 0; index < 3; ++index) {
    const auto csr = last - (4 * index);
    const bool missing_compare =
        input.failure == Failure::unknown_compare && input.address == csr - 2;
    if (index < 2 && (input.failure != Failure::unknown_di || input.address != csr)) {
      timers.set_tio_drive(index, input.level);
    }
    timers.write({.address = csr - 1, .value = 0});
    if (!missing_compare) {
      timers.write({.address = csr - 2, .value = compare});
      timers.write({.address = csr, .value = index == 2 ? prescale_enable : 1U});
    }
  }
  machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = payload});
  machine.bus().write({.space = DspSpace::y, .address = dma::destination, .value = canary});
  banks::configure(machine.bus());
}

void check_dma(const DspFetchCycleResult &cycle, DspMachine &machine, const Input input) {
  const auto period = cycle.completed_cycles;
  const auto completed = input.wait ? release_period : input.request + 1;
  const auto &result = cycle.bus.dma;
  const bool source = period == input.request;
  const bool destination = period > input.request && period <= completed;
  require(!result.core_stall && result.phase.has_value() == (source || destination),
          "One actual IRQA edge creates only its independent source/destination phases");
  if (result.phase) {
    require(result.phase->channel == 0 &&
                result.phase->kind ==
                    (source ? DspDmaPhaseKind::source_read : DspDmaPhaseKind::destination_write),
            "The actual DMA phase order survives AG1 sampling and native E1/E2");
    if (destination) {
      require(result.phase->value == payload,
              "Held destination retains the real captured source word");
    }
  }
  require(result.word.has_value() == (period == completed),
          "Bounded holds release exactly one completed DMA transfer");
  if (result.word) {
    require(result.word->value == payload, "DMA commits the old captured RAM payload");
  }
  if (source) {
    machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = replacement});
  }
  require(machine.bus().read({.space = DspSpace::y, .address = dma::destination}) ==
              (period >= completed ? payload : canary),
          "Fresh clones and restores change the destination exactly once after the real held phase "
          "drains");
}

void check_clocks(DspMachine &machine, const DspFetchRun &run, const Input input) {
  const auto period = run.completed_cycles();
  const auto edges = static_cast<unsigned>(period / 2);
  const auto pulses = edges / (prescale + 1);
  auto expected_prescaler = prescale - (edges % (prescale + 1));
  constexpr unsigned third_pulse_edges = 12;
  if (input.address == tplr && period > capture_period && edges >= third_pulse_edges) {
    expected_prescaler = new_prescale - ((edges - third_pulse_edges) % (new_prescale + 1));
  }
  auto &timers = machine.bus().timers();
  require(run.timer_clk2_edges() == edges && timers.read(first) == expected_prescaler,
          "Actual CLK/2 edges and delayed prescaler reload follow literal divide-by-four/eight "
          "arithmetic");
  for (unsigned index = 0; index < 3; ++index) {
    const auto prescaled_count = pulses == 0 ? 0 : pulses - 1;
    const auto normal_count = edges == 0 ? 0 : edges - 1;
    const auto count = index == 2 ? prescaled_count : normal_count;
    require(timers.read(last - (4 * index) - 3) == count,
            "Independent counters advance without a fabricated load or compare event");
  }
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, DspDebugState &expected) {
  const auto next = static_cast<unsigned>(run.completed_cycles() + 1);
  const bool held = input.wait && next > input.request && next < release_period;
  const auto cycle =
      run.advance_cycle(machine,
                        {.known_requests = 15,
                         .requests = next == input.request ? 1U : 0U,
                         .grant = held ? DspDmaGrant::wait : DspDmaGrant::available});
  const auto &phase = test::pipeline::stages(cycle);
  require(phase.retirement_published == (next >= 7),
          "Literal one-word MOVEP/NOP costs fix every retirement");
  if (next >= 7) {
    ++expected.cpu.pc;
    ++expected.native_instructions;
    ++expected.native_cycles;
  }
  if (next == e1_period) {
    expected.cpu.x = (expected.cpu.x & ~std::uint64_t{test::word_mask}) | sample(input);
  }
  require(machine.cpu().debug_state() == expected,
          "Every period retains the independent complete CPU/CCR/stack and actual AG1-captured X0 "
          "value");
  require(cycle.bus.peripheral_read.has_value() == (next == capture_period),
          "The actual queued reader samples exactly once in AG1");
  check_dma(cycle, machine, input);
  if (input.failure == Failure::none) { check_clocks(machine, run, input); }
}

void mutate(DspMachine &machine, const Input input) {
  if (input.address == tplr) {
    machine.bus().timers().write({.address = tplr, .value = new_prescale});
  } else if ((input.address & 3U) == 1) {
    machine.bus().timers().write({.address = input.address, .value = new_compare});
  } else if (input.address != first && (input.address & 3U) == 3 && input.address != last - 8) {
    machine.bus().timers().set_tio_drive((last - input.address) / 4, !input.level);
  }
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, DspDebugState expected) {
  while (run.completed_cycles() < end_period) { step(run, machine, input, expected); }
}

void rejected(DspFetchRun &run,
              DspMachine &machine,
              const Input input,
              const DspDebugState &expected) {
  const auto saved = run.checkpoint(machine);
  std::string previous;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    auto state = expected;
    const auto error = test::esai::rejects([&] { step(run, machine, input, state); });
    require(!error.contains("with active DMA") && error.contains("pc=") && error.contains("omr=") &&
                (error.contains("Write-only") || error.contains("Unknown DSP timer")),
            "A real invalid timer read retains its specific address/operation/PC context");
    require(run.faulted() && run.completed_cycles() == capture_period - 1 &&
                machine.cpu().debug_state() == expected,
            "AG1 read failure precedes the older NOP retirement and cannot invent X0 or flags");
    if (replay != 0) {
      require(error == previous, "Joint restore reproduces the exact native strict read failure");
    }
    previous = error;
  }
  ++faults;
}

void window(const Input input) {
  DspMachine machine;
  std::array<std::uint32_t, program_words> program{};
  program.at(padding) = read_opcode | (input.address - io);
  machine.bus().install_rom({.start = boot, .words = program});
  DspFetchRun run(settings(),
                  machine,
                  DspHardwareResetInputs{
                      .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  prepare(machine, run, input);
  auto expected = machine.cpu().debug_state();
  while (run.completed_cycles() < capture_period - 1) { step(run, machine, input, expected); }
  if (input.failure != Failure::none) {
    rejected(run, machine, input, expected);
    return;
  }
  step(run, machine, input, expected);
  mutate(machine, input);
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    finish(run, machine, input, expected);
    ++suffixes;
  }
  DspMachine target;
  DspFetchRun cloned(saved, target);
  finish(cloned, target, input, expected);
  ++clones;
  ++windows;
}
}

int main() {
  try {
    for (unsigned address = first; address <= last; ++address) {
      const bool write_only = address > tplr && (address & 3U) == 2;
      if (write_only) {
        window({.address = address,
                .request = capture_period - 1,
                .wait = true,
                .failure = Failure::write_only});
        continue;
      }
      for (const unsigned request : {19U, 20U, 21U}) {
        for (const bool wait : {false, true}) {
          for (const bool level : {false, true}) {
            window({.address = address, .request = request, .wait = wait, .level = level});
          }
        }
      }
    }
    for (unsigned index = 0; index < 3; ++index) {
      const auto csr = last - (4 * index);
      window({.address = csr - 2,
              .request = capture_period - 1,
              .wait = true,
              .failure = Failure::unknown_compare});
      if (index < 2) {
        window({.address = csr,
                .request = capture_period - 1,
                .wait = true,
                .failure = Failure::unknown_di});
      }
    }
    std::cout << "Timer reads=" << windows << " faults=" << faults << " suffixes=" << suffixes
              << " clones=" << clones << ";AG1=20,E1=22,held-DMA-drain=25\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
