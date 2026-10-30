// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
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
constexpr unsigned boot = 0xff0000, extal = 12000000, timebase = 1080000000;
constexpr unsigned padding = 16, before_read = 21, read_period = 22, write_period = 23;
constexpr unsigned first_timer_word = 0xffff82, first_control = 0xffff8f, flags = 0x300000;
constexpr unsigned program_words = 33, peripheral_base = 0xffff80;
constexpr unsigned compare_flag = 0x200000, wrap_load = 0xfffffe, word_mask = 0xffffff;
constexpr unsigned payload = 0x123456, replacement = 0x654321, canary = 0xa55a33;
constexpr std::uint64_t reset_ticks = 180;
// FM chapter13 short-I/O bit encodings, bit0. UM chapter11: TE bit0, W1C20/21.
constexpr std::array operations{0x010000U, 0x010020U, 0x014000U, 0x014020U};
unsigned footprints = 0, windows = 0, suffixes = 0, clones = 0;

struct Input {
  unsigned timer, operation, request;
  bool asynchronous, wait;
};

unsigned control(const Input input) {
  return first_control - (4 * input.timer);
}

DspFetchRunInputs settings() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory,
          .resets = DspFetchResetTiming::reference_cycle_boundary};
}

DspFetchCycleInputs inputs(const Input input, const unsigned next) {
  return {.known_requests = 15,
          .requests = next == input.request ? 1U : 0U,
          .grant = input.wait && next > input.request ? DspDmaGrant::wait : DspDmaGrant::available};
}

void prepare(DspMachine &machine, DspFetchRun &run, const Input input) {
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Actual reset is held for two EXTAL periods");
  run.release_hardware_reset(machine);
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Actual reset recovers for two EXTAL periods");
  auto &timers = machine.bus().timers();
  timers.write({.address = control(input) - 1, .value = input.asynchronous ? 0U : wrap_load});
  timers.write({.address = control(input) - 2, .value = input.asynchronous ? 10U : word_mask});
  timers.write({.address = control(input), .value = 1});
  if (!input.asynchronous) { timers.advance_clk2(3); }
  machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = payload});
  machine.bus().write({.space = DspSpace::y, .address = dma::destination, .value = canary});
  banks::configure(machine.bus());
}

void check_dma(const DspFetchCycleResult &cycle, DspMachine &machine, const Input input) {
  const auto period = cycle.completed_cycles;
  const auto &result = cycle.bus.dma;
  require(!result.core_stall, "A timer register cannot stall a distinct DMA RAM path");
  const bool source = period == input.request;
  const bool destination = period > input.request && (input.wait || period == input.request + 1);
  require(result.phase.has_value() == (source || destination),
          "One genuine IRQA edge creates only its actual two DMA phases");
  if (result.phase) {
    require(result.phase->channel == 0 &&
                result.phase->kind ==
                    (source ? DspDmaPhaseKind::source_read : DspDmaPhaseKind::destination_write),
            "The literal source/destination order survives native timer E1 and E2");
  }
  require(result.word.has_value() == (destination && !input.wait),
          "A held destination cannot create an extra DMA completion");
  if (result.word) {
    require(result.word->value == payload, "DMA commits the captured old RAM word");
  }
  if (source) {
    machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = replacement});
  }
  require(machine.bus().read({.space = DspSpace::y, .address = dma::destination}) ==
              (!input.wait && period > input.request ? payload : canary),
          "Independent destination canary changes only on the actual completed DMA write");
}

void check_timer(DspMachine &machine, const DspFetchRun &run, const Input input) {
  const auto period = run.completed_cycles();
  const auto edges = period / 2;
  const bool committed = period == write_period && input.operation != 3;
  const bool enabled = !committed || input.operation == 1;
  const auto increments = edges == 0 ? 0 : edges - 1;
  const auto expected_count = static_cast<unsigned>(input.asynchronous ? increments : edges);
  const unsigned asynchronous_flags = period >= read_period ? compare_flag : 0U;
  unsigned expected_flags = input.asynchronous ? asynchronous_flags : flags;
  if (committed && (!input.asynchronous || !enabled)) { expected_flags = 0; }
  const auto &timers = machine.bus().timers();
  require(run.timer_clk2_edges() == edges && timers.read(control(input) - 3) == expected_count &&
              timers.read(control(input), flags | 1) ==
                  (expected_flags | static_cast<unsigned>(enabled)),
          "Independent CLK/2 counts and W1C semantics retain new post-read flags exactly");
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, DspDebugState &expected) {
  const auto cycle =
      run.advance_cycle(machine, inputs(input, static_cast<unsigned>(run.completed_cycles() + 1)));
  const auto period = cycle.completed_cycles;
  check_dma(cycle, machine, input);
  const auto &phase = test::pipeline::stages(cycle);
  const bool retires = period >= 7;
  require(phase.retirement_published == retires, "Literal NOP/bit program fixes every retirement");
  if (retires) {
    ++expected.cpu.pc;
    ++expected.native_instructions;
    expected.native_cycles += period == write_period ? 2U : 1U;
    if (period == write_period) { expected.cpu.sr |= 1; }
  }
  require(machine.cpu().debug_state() == expected,
          "Every period retains the independent complete CPU/CCR/stack state");
  require(cycle.bus.bit_write.has_value() == (period == write_period && input.operation != 3),
          "Native E2 commits mutating operations once; BTST never writes");
  check_timer(machine, run, input);
}

void window(const Input input) {
  DspMachine machine;
  std::array<std::uint32_t, program_words> program{};
  program.at(padding) = operations.at(input.operation) | ((control(input) - peripheral_base) << 8U);
  machine.bus().install_rom({.start = boot, .words = program});
  DspFetchRun run(settings(),
                  machine,
                  DspHardwareResetInputs{
                      .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  prepare(machine, run, input);
  auto expected = machine.cpu().debug_state();
  while (run.completed_cycles() < before_read) { step(run, machine, input, expected); }
  const auto before = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(before, machine);
    auto state = expected;
    step(run, machine, input, state);
    const auto captured = run.checkpoint(machine);
    for (unsigned tail = 0; tail < 2; ++tail) {
      run.restore(captured, machine);
      auto final = state;
      step(run, machine, input, final);
      ++suffixes;
    }
    DspMachine target;
    DspFetchRun cloned(captured, target);
    step(cloned, target, input, state);
    ++clones;
  }
  ++windows;
}

void footprint(const DspMemoryAccess access, const bool admitted) {
  DspBus bus;
  const std::array fetch{DspMemoryAccess{.target = {.address = banks::p_rom_begin}}};
  static_cast<void>(bus.advance_dma(banks::inputs({.index = 0}), banks::use(fetch)));
  bus.write({.space = DspSpace::x, .address = dma::ring_start, .value = payload});
  banks::configure(bus, {.control = dma::software_copy});
  const auto source = bus.advance_dma(banks::inputs({.index = 1}), banks::use(fetch));
  require(source.phase && source.phase->kind == DspDmaPhaseKind::source_read,
          "Real DMA source capture precedes each footprint");
  bus.write({.space = DspSpace::x, .address = dma::ring_start, .value = replacement});
  const std::array core{fetch[0], access};
  if (admitted) {
    const auto result = bus.advance_dma(banks::inputs({.index = 2}), banks::use(core));
    require(result.word && result.word->value == payload && !result.core_stall,
            "All selected timer words leave the distinct real DMA destination resource free");
  } else {
    const auto error = test::esai::rejects(
        [&] { static_cast<void>(bus.advance_dma(banks::inputs({.index = 2}), banks::use(core))); });
    require(error.contains("Unqualified") && bus.last_dma_cycle() == 1,
            "Unsupported neighbors, bank aliases and footprints reject before DMA state changes");
  }
  ++footprints;
}

void resources() {
  for (unsigned address = first_timer_word; address <= first_control; ++address) {
    for (const auto phase : {DspMemoryPhase::peripheral_bit,
                             DspMemoryPhase::peripheral_sample,
                             DspMemoryPhase::execute}) {
      for (const auto operation : {DspMemoryOperation::read, DspMemoryOperation::write}) {
        const bool admitted =
            phase == DspMemoryPhase::peripheral_bit ||
            (phase == DspMemoryPhase::peripheral_sample && operation == DspMemoryOperation::read) ||
            (phase == DspMemoryPhase::execute && operation == DspMemoryOperation::write);
        footprint({.target = {.space = DspSpace::x, .address = address},
                   .operation = operation,
                   .phase = phase},
                  admitted);
        footprint({.target = {.space = DspSpace::y, .address = address},
                   .operation = operation,
                   .phase = phase},
                  false);
      }
    }
  }
  for (const auto address :
       {0xffff80U, 0xffff81U, 0xffff90U, 0xffff91U, 0xffffa0U, 0xffffbfU, 0xffffd2U}) {
    footprint({.target = {.space = DspSpace::x, .address = address},
               .operation = DspMemoryOperation::write},
              false);
  }
}
}

int main() {
  try {
    resources();
    for (unsigned timer = 0; timer < 3; ++timer) {
      for (unsigned operation = 0; operation < operations.size(); ++operation) {
        for (const bool asynchronous : {false, true}) {
          for (const unsigned request : {21U, 22U, 23U}) {
            for (const bool wait : {false, true}) {
              window({.timer = timer,
                      .operation = operation,
                      .request = request,
                      .asynchronous = asynchronous,
                      .wait = wait});
            }
          }
        }
      }
    }
    std::cout << "Timer/DMA footprints=" << footprints << " native-windows=" << windows
              << " suffixes=" << suffixes << " clones=" << clones << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
