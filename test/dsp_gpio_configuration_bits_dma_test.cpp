// SPDX-License-Identifier: GPL-3.0-only
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_bit_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using test::require;
namespace bit = test::peripheral_bit;
namespace dma = test::dma_test;
namespace pipeline = test::pipeline;
constexpr unsigned first = 0x123456, second = 0x654321, canary = 0xa55a33;
constexpr unsigned seed_count = 10, seed_words = seed_count * 2;
unsigned configuration_offset = 1;
constexpr unsigned body = test::boot + seed_words, end = 15;

struct Schedule {
  unsigned request, wait_begin, wait_end, completed;
};

// Independently selected IRQA edges put source/destination/held phases across E1/E2.
constexpr std::array schedules{
    Schedule{.request = 6, .wait_begin = 0, .wait_end = 0, .completed = 7},
    Schedule{.request = 7, .wait_begin = 0, .wait_end = 0, .completed = 8},
    Schedule{.request = 8, .wait_begin = 0, .wait_end = 0, .completed = 9},
    Schedule{.request = 6, .wait_begin = 7, .wait_end = 9, .completed = 10}};

struct Oracle {
  DspDebugState cpu;
  unsigned retired = 0, reads = 0, writes = 0, transfers = 0;
};

unsigned configuration_opcode(const bit::Input input) {
  constexpr unsigned address_field = 0x3f00;
  const auto port = bit::ports.at(input.port);
  const auto address = port.address + configuration_offset;
  const bool high = address >= bit::high_begin;
  const auto base = high ? bit::high_begin : bit::peripheral_begin;
  return (bit::opcode(input) & ~address_field) | ((address - base) << 8U);
}

unsigned branch_opcode(const bit::Input input) {
  // DSP56300FM13-26/27: literal pp/qq BRCLR, 24-bit relative extension.
  constexpr unsigned pp = 0x0cc000;
  constexpr unsigned qq = 0x048000;
  const auto port = bit::ports.at(input.port);
  const auto address = port.address + configuration_offset;
  const bool high = address >= bit::high_begin;
  return (high ? pp : qq) | ((address - (high ? bit::high_begin : bit::peripheral_begin)) << 8U) |
         (port.space == DspSpace::y ? 0x40U : 0U) | input.bit;
}

constexpr unsigned branch_displacement = 8;

Oracle
prepare_configuration(DspMachine &machine, const bit::Input input, const bool branch = false) {
  const auto port = bit::ports.at(input.port);
  test::DspProgram code;
  const auto put = [&](const DspStore store) {
    code.emit("movep #>" + test::number(store.value) + ',' +
              std::string(store.space == DspSpace::y ? "y:<<" : "x:<<") +
              test::number(store.address));
  };
  put({.address = bit::bcr, .value = bit::pin_bcr});
  put({.space = port.space, .address = port.address + 2});
  put({.space = port.space, .address = port.address + 1});
  put({.space = port.space, .address = port.address, .value = 0});
  put({.space = port.space,
       .address = port.address + configuration_offset,
       .value = bit::seed(input)});
  put({.address = dma::source0, .value = dma::ring_start});
  put({.address = dma::destination0, .value = dma::destination});
  put({.address = dma::counter0});
  put({.address = dma::control0, .value = dma::word_copy});
  code.immediate("sr", bit::status | static_cast<unsigned>(input.carry_in));
  code.word(0);
  code.word(branch ? branch_opcode(input) : configuration_opcode(input));
  if (branch) { code.word(branch_displacement); }
  for (unsigned index = 0; index < 32; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = first});
  machine.bus().write({.space = DspSpace::y, .address = dma::destination, .value = canary});
  static_cast<void>(machine.advance_dma({.cycle = 0,
                                         .known_requests = 15,
                                         .timing = DspDmaTiming::reference_sampled_cycles,
                                         .grant = DspDmaGrant::available}));
  test::checkpoint::run(machine, seed_count);
  const auto state = machine.cpu().debug_state();
  require(state.cpu.pc == body && state.native_instructions == seed_count &&
              state.native_cycles == seed_words &&
              machine.bus().read({.space = DspSpace::x, .address = dma::control0}) ==
                  dma::word_copy,
          "Ten genuine native seeds establish exact configuration, DMA and CPU state");
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  return {.cpu = state};
}

void transfer(const DspFetchCycleResult &cycle,
              DspMachine &machine,
              const Schedule schedule,
              Oracle &oracle) {
  const auto period = cycle.completed_cycles;
  const auto &result = cycle.bus.dma;
  require(!result.core_stall, "A distinct GPIO bit cannot stall the independent DMA RAM transfer");
  if (period >= schedule.request && period <= schedule.completed) {
    require(result.phase && result.phase->channel == 0 &&
                result.phase->kind == (period == schedule.request
                                           ? DspDmaPhaseKind::source_read
                                           : DspDmaPhaseKind::destination_write),
            "Each source and captured destination phase follows the literal two-clock DMA rule");
  } else {
    require(!result.phase, "One physical IRQA rising edge cannot create extra DMA phases");
  }
  if (period == schedule.request) {
    machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = second});
  }
  require(result.word.has_value() == (period == schedule.completed),
          "Global waits retain the destination phase and defer exactly one real write");
  if (result.word) {
    require(result.word->channel == 0 && result.word->value == first &&
                result.word->source == DspDmaAddress{.address = dma::ring_start} &&
                result.word->destination ==
                    DspDmaAddress{.space = DspDmaSpace::y, .address = dma::destination},
            "The real DMA commits its captured source once despite later backing RAM mutation");
    ++oracle.transfers;
  }
  require(machine.bus().read({.space = DspSpace::y, .address = dma::destination}) ==
              (period >= schedule.completed ? first : canary),
          "Independent destination bytes change only at the actual completed DMA write");
}

void step(DspFetchRun &run,
          DspMachine &machine,
          const bit::Input input,
          const Schedule schedule,
          Oracle &oracle) {
  const auto next = run.completed_cycles() + 1;
  const bool waiting = next >= schedule.wait_begin && next <= schedule.wait_end;
  const auto cycle =
      run.advance_cycle(machine,
                        {.known_requests = 15,
                         .requests = next == schedule.request ? 1U : 0U,
                         .grant = waiting ? DspDmaGrant::wait : DspDmaGrant::available});
  transfer(cycle, machine, schedule, oracle);
  const auto &phase = pipeline::stages(cycle);
  const auto period = cycle.completed_cycles;
  const bool mutating = input.operation != 3;
  if (phase.memory) {
    const auto &access = phase.memory->data;
    const auto port = bit::ports.at(input.port);
    require(period == 7 && access.kind == DspNativeAccessKind::bit && access.pc == body + 1 &&
                access.target.space == port.space &&
                access.target.address == port.address + configuration_offset && access.bit &&
                access.bit->index == input.bit &&
                access.bit->operation == bit::operations.at(input.operation).kind &&
                phase.memory->bit_read == mutating &&
                (mutating ? !access.value : access.value == static_cast<unsigned>(input.old)),
            "Real native E1 captures only the selected GPIO old bit, never a fabricated read");
    ++oracle.reads;
  }
  if (cycle.bus.bit_write) {
    const auto &access = *cycle.bus.bit_write;
    require(mutating && period == 8 && access.pc == body + 1 && access.bit &&
                access.bit->index == input.bit &&
                access.value == static_cast<unsigned>(input.old) &&
                access.stage == DspNativeAccessStage::after,
            "E2 performs the real bit write once and returns captured E1 old data for Carry");
    ++oracle.writes;
  }
  const bool retires = period == 7 || period == 8 || (period >= 10 && period <= end);
  require(phase.retirement_published == retires,
          "Independent native instruction costs fix retirements");
  if (retires) {
    require(phase.execute2 && phase.execute2->address == oracle.cpu.cpu.pc,
            "Only actual own program words retire");
    const bool selected = oracle.retired == 1;
    if (selected) { oracle.cpu.cpu.sr = bit::status | static_cast<unsigned>(input.old); }
    ++oracle.cpu.cpu.pc;
    ++oracle.cpu.native_instructions;
    oracle.cpu.native_cycles += selected ? 2U : 1U;
    ++oracle.retired;
  }
  const auto port = bit::ports.at(input.port);
  require(machine.cpu().debug_state() == oracle.cpu &&
              machine.bus().read(
                  {.space = port.space, .address = port.address + configuration_offset}) ==
                  (period >= 8 ? bit::value(input) : bit::seed(input)),
          "Every cycle preserves the independent full CPU/CCR/stack and GPIO truth table");
}

void finish(DspFetchRun &run,
            DspMachine &machine,
            const bit::Input input,
            const Schedule schedule,
            Oracle &oracle) {
  while (run.completed_cycles() < end) { step(run, machine, input, schedule, oracle); }
  require(oracle.retired == 8 && oracle.reads == 1 &&
              oracle.writes == static_cast<unsigned>(input.operation != 3) &&
              oracle.transfers == 1 && run.elapsed_ticks() == end * pipeline::period &&
              machine.bus().read({.space = DspSpace::x, .address = dma::control0}) ==
                  (dma::word_copy & ~dma::enable),
          "One genuine GPIO operation and one actual DMA transfer finish with exact clocks/counts");
}

unsigned windows = 0, suffixes = 0, clones = 0;

void window(const bit::Input input, const Schedule schedule) {
  DspMachine machine;
  auto oracle = prepare_configuration(machine, input);
  DspFetchRun run(bit::contract(), machine);
  finish(run, machine, input, schedule, oracle);
  ++windows;
}

void snapshots(const bit::Input input, const Schedule schedule) {
  for (unsigned cut = 5; cut <= 10; ++cut) {
    DspMachine machine;
    auto oracle = prepare_configuration(machine, input);
    DspFetchRun run(bit::contract(), machine);
    while (run.completed_cycles() < cut) { step(run, machine, input, schedule, oracle); }
    const auto saved = run.checkpoint(machine);
    const auto prefix = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      oracle = prefix;
      finish(run, machine, input, schedule, oracle);
      ++suffixes;
    }
    DspMachine clone;
    DspFetchRun cloned(saved, clone);
    oracle = prefix;
    finish(cloned, clone, input, schedule, oracle);
    ++clones;
  }
}

void branch_step(DspFetchRun &run,
                 DspMachine &machine,
                 const bit::Input input,
                 const Schedule schedule,
                 Oracle &oracle) {
  const auto next = run.completed_cycles() + 1;
  const bool waiting = next >= schedule.wait_begin && next <= schedule.wait_end;
  const auto cycle =
      run.advance_cycle(machine,
                        {.known_requests = 15,
                         .requests = next == schedule.request ? 1U : 0U,
                         .grant = waiting ? DspDmaGrant::wait : DspDmaGrant::available});
  transfer(cycle, machine, schedule, oracle);
  const auto &phase = pipeline::stages(cycle);
  const auto port = bit::ports.at(input.port);
  require(cycle.bus.peripheral_read.has_value() == (next == 5),
          "BRCLR samples once at the declared AG1 boundary");
  if (cycle.bus.peripheral_read) {
    require(std::ranges::any_of(cycle.bus.core_accesses,
                                [&](const auto access) {
                                  return access.target.space == port.space &&
                                         access.target.address ==
                                             port.address + configuration_offset &&
                                         access.operation == DspMemoryOperation::read &&
                                         access.phase == DspMemoryPhase::peripheral_sample;
                                }),
            "The owned branch sample has the exact configuration address/read footprint");
    ++oracle.reads;
  }
  require(!cycle.bus.bit_write, "BRCLR cannot modify the configuration or invent a bit write");
  if (phase.retirement_published) {
    const bool selected = oracle.retired == 1;
    if (selected) {
      oracle.cpu.cpu.pc = body + 1 + (input.old ? 2U : branch_displacement);
    } else {
      ++oracle.cpu.cpu.pc;
    }
    ++oracle.retired;
    ++oracle.cpu.native_instructions;
  }
  const auto actual = machine.cpu().debug_state();
  require(actual.cpu == oracle.cpu.cpu &&
              actual.native_instructions == oracle.cpu.native_instructions &&
              machine.bus().read(
                  {.space = port.space, .address = port.address + configuration_offset}) ==
                  bit::seed(input),
          "Literal relative target/fallthrough and unchanged full CPU/CCR/configuration hold at "
          "every retirement");
}

void branch_finish(DspFetchRun &run,
                   DspMachine &machine,
                   const bit::Input input,
                   const Schedule schedule,
                   Oracle &oracle) {
  constexpr unsigned branch_end = 20;
  while (run.completed_cycles() < branch_end) {
    branch_step(run, machine, input, schedule, oracle);
  }
  require(oracle.reads == 1 && oracle.retired >= 2 && oracle.transfers == 1 &&
              run.elapsed_ticks() == branch_end * pipeline::period,
          "One real configuration branch and captured DMA complete on declared reference clocks");
}

void branch_cases(const unsigned port, const unsigned index) {
  for (const bool old : {false, true}) {
    for (const auto schedule : schedules) {
      for (const unsigned cut : {5U, 7U, 8U, 10U}) {
        const bit::Input input{
            .port = port, .operation = 3, .bit = index, .old = old, .carry_in = !old};
        DspMachine machine;
        auto oracle = prepare_configuration(machine, input, true);
        DspFetchRun run(bit::contract(), machine);
        while (run.completed_cycles() < cut) { branch_step(run, machine, input, schedule, oracle); }
        const auto saved = run.checkpoint(machine);
        const auto prefix = oracle;
        for (unsigned replay = 0; replay < 2; ++replay) {
          run.restore(saved, machine);
          oracle = prefix;
          branch_finish(run, machine, input, schedule, oracle);
          ++suffixes;
        }
        DspMachine fresh;
        DspFetchRun cloned(saved, fresh);
        oracle = prefix;
        branch_finish(cloned, fresh, input, schedule, oracle);
        ++clones;
      }
    }
  }
}

void truth_cases(const unsigned port, const unsigned operation, const unsigned index) {
  for (const bool old : {false, true}) {
    for (const bool carry : {false, true}) {
      for (const auto schedule : schedules) {
        window({.port = port, .operation = operation, .bit = index, .old = old, .carry_in = carry},
               schedule);
      }
    }
  }
}

void checks() {
  for (unsigned port = 0; port < bit::ports.size(); ++port) {
    branch_cases(port, 0);
    branch_cases(port, bit::ports.at(port).bits - 1);
    for (unsigned operation = 0; operation < bit::operations.size(); ++operation) {
      for (unsigned index = 0; index < bit::ports[port].bits; ++index) {
        truth_cases(port, operation, index);
      }
      for (const bool old : {false, true}) {
        for (const auto schedule : schedules) {
          snapshots({.port = port,
                     .operation = operation,
                     .bit = bit::ports[port].bits - 1,
                     .old = old,
                     .carry_in = !old},
                    schedule);
        }
      }
    }
  }
}
}

int main() {
  try {
    for (const unsigned offset : {1U, 2U}) {
      configuration_offset = offset;
      checks();
    }
    std::cout << "Native GPIO configuration-bit/DMA windows=" << windows << " suffixes=" << suffixes
              << " clones=" << clones << ";74configuration bits,full CPU,real captured DMA\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
