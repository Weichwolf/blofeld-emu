// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000, padding = 16, program_words = 36;
constexpr unsigned extal = 12000000, timebase = 1080000000;
constexpr unsigned bcr = 0xfffffb, port_f_bcr = 0x01ffff;
constexpr unsigned direction = 0xfffff6, control = 0xfffff7, pins = 0x7ff;
constexpr unsigned source0 = 0xffffef, destination0 = 0xffffee;
constexpr unsigned counter0 = 0xffffed, control0 = 0xffffec;
constexpr unsigned source = 0x660, destination = 0x900, word_copy = 0x880244;
constexpr unsigned payload = 0x123456, changed_source = 0x654321, canary = 0xa55a33;
// FM13 MOVEP: immediate EA=$34, Y:pp destination; pp is relative to $ffffc0.
constexpr unsigned movep_y_immediate = 0x09f480, pp_base = 0xffffc0;
constexpr unsigned store_period = 22, retire_period = 23;
constexpr std::uint64_t reset_ticks = 180;
unsigned windows = 0, cuts = 0, suffixes = 0, clones = 0, faults = 0, assertions = 0;

struct Schedule {
  unsigned request, wait_first, wait_last, commit;
};

constexpr std::array schedules{
    Schedule{.request = 20, .wait_first = 0, .wait_last = 0, .commit = 21},
    Schedule{.request = 21, .wait_first = 0, .wait_last = 0, .commit = 22},
    Schedule{.request = 22, .wait_first = 0, .wait_last = 0, .commit = 23},
    Schedule{.request = 19, .wait_first = 20, .wait_last = 22, .commit = 23}};

struct Input {
  unsigned address, value;
  Schedule schedule;
  unsigned peer = 0;
  bool configure_bcr = true;
};

void require(const bool condition, const std::string_view message) {
  ++assertions;
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

void put(DspBus &bus, const unsigned address, const unsigned value) {
  bus.write({.space = DspSpace::x, .address = address, .value = value});
}

DspFetchRunInputs settings() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory,
          .resets = DspFetchResetTiming::reference_cycle_boundary};
}

void install(DspMachine &machine, const Input input) {
  std::array<std::uint32_t, program_words> program{};
  program.at(padding) = movep_y_immediate | (input.address - pp_base);
  program.at(padding + 1) = input.value;
  machine.bus().install_rom({.start = boot, .words = program});
}

void prepare(DspMachine &machine, DspFetchRun &run, const Input input) {
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Real hardware reset hold");
  run.release_hardware_reset(machine);
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Real reset recovery");
  auto &bus = machine.bus();
  if (input.configure_bcr) { put(bus, bcr, port_f_bcr); }
  const auto other =
      input.address == control ? DspGpioRegister::direction : DspGpioRegister::control;
  bus.gpio(DspPort::f).write({.reg = other, .value = input.peer});
  put(bus, source, payload);
  bus.write({.space = DspSpace::y, .address = destination, .value = canary});
  put(bus, source0, source);
  put(bus, destination0, destination);
  put(bus, counter0, 0);
  put(bus, control0, word_copy);
}

void dma(const DspFetchCycleResult &cycle, DspBus &bus, const Schedule schedule) {
  const auto period = cycle.completed_cycles;
  const auto &result = cycle.bus.dma;
  require(!result.core_stall, "An internal-memory DMA phase does not occupy the Port-F BIU");
  const bool active = period >= schedule.request && period <= schedule.commit;
  require(result.phase.has_value() == active, "Only one real IRQA edge creates DMA phases");
  if (result.phase) {
    require(result.phase->channel == 0 &&
                result.phase->kind == (period == schedule.request
                                           ? DspDmaPhaseKind::source_read
                                           : DspDmaPhaseKind::destination_write),
            "Literal source/destination/held phase order");
  }
  require(result.word.has_value() == (period == schedule.commit),
          "A requested global wait holds DMA without delaying the independent core store");
  if (result.word) {
    require(result.word->value == payload && result.word->source.address == source &&
                result.word->destination.address == destination,
            "DMA commits the original captured source exactly once");
  }
  if (period == schedule.request) { put(bus, source, changed_source); }
  require(bus.read({.space = DspSpace::y, .address = destination}) ==
              (period >= schedule.commit ? payload : canary),
          "Destination contents have an independent literal commit boundary");
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, DspDebugState &expected) {
  const auto next = static_cast<unsigned>(run.completed_cycles() + 1);
  const auto schedule = input.schedule;
  const bool waiting = next >= schedule.wait_first && next <= schedule.wait_last;
  const auto cycle =
      run.advance_cycle(machine,
                        {.known_requests = 15,
                         .requests = next == schedule.request ? 1U : 0U,
                         .grant = waiting ? DspDmaGrant::wait : DspDmaGrant::available});
  if (!cycle.pipeline) { throw Fault("Actual automatic pipeline must own execution"); }
  const auto &phase = *cycle.pipeline;
  const bool retired = (next >= 7 && next <= store_period) || next == retire_period;
  require(phase.retirement_published == retired, "Independent NOP/MOVEP retirement periods");
  if (retired) {
    expected.cpu.pc += next == retire_period ? 2U : 1U;
    ++expected.native_instructions;
    expected.native_cycles += next == retire_period ? 2U : 1U;
  }
  require(machine.cpu().debug_state() == expected,
          "Every period preserves complete native CPU/CCR/stack and literal PC/counters");
  require(phase.memory.has_value() == (next == store_period), "Exactly one native E1 store");
  if (phase.memory) {
    const auto &access = phase.memory->data;
    require(access.kind == DspNativeAccessKind::write && access.target.space == DspSpace::y &&
                access.target.address == input.address && access.value == input.value &&
                access.pc == boot + padding,
            "Native MOVEP owns the expected actual address/value/instruction context");
  }
  dma(cycle, machine.bus(), schedule);
  require(machine.bus().read({.space = DspSpace::y, .address = input.address}) ==
              (next >= store_period ? input.value : 0U),
          "The real GPIO configuration changes only at native E1, before E2 retirement");
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, DspDebugState expected) {
  while (run.completed_cycles() < retire_period) { step(run, machine, input, expected); }
  require(expected.cpu.pc == boot + padding + 2 && expected.native_instructions == padding + 1 &&
              expected.native_cycles == padding + 2 &&
              machine.bus().read({.space = DspSpace::x, .address = control0}) ==
                  (word_copy & ~0x800000U),
          "One real MOVEP and exactly one completed DMA word retain literal final counts");
}

void window(const Input input, const bool snapshots) {
  DspMachine machine;
  install(machine, input);
  DspFetchRun run(settings(),
                  machine,
                  DspHardwareResetInputs{
                      .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  prepare(machine, run, input);
  auto expected = machine.cpu().debug_state();
  if (!snapshots) {
    finish(run, machine, input, expected);
    ++windows;
    return;
  }
  for (const unsigned cut : {20U, 21U, 22U}) {
    while (run.completed_cycles() < cut) { step(run, machine, input, expected); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      finish(run, machine, input, expected);
      ++suffixes;
    }
    DspMachine clone;
    DspFetchRun cloned(saved, clone);
    finish(cloned, clone, input, expected);
    ++clones;
    ++cuts;
    run.restore(saved, machine);
  }
}

void rejection(const Input input, const std::string_view reason) {
  DspMachine machine;
  install(machine, input);
  DspFetchRun run(settings(),
                  machine,
                  DspHardwareResetInputs{
                      .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  prepare(machine, run, input);
  auto expected = machine.cpu().debug_state();
  // Before E1, the same independent NOP stream and real DMA source run normally.
  while (run.completed_cycles() < store_period - 1) {
    const auto next = static_cast<unsigned>(run.completed_cycles() + 1);
    static_cast<void>(run.advance_cycle(machine,
                                        {.known_requests = 15,
                                         .requests = next == store_period - 1 ? 1U : 0U,
                                         .grant = DspDmaGrant::available}));
    if (next >= 7) {
      ++expected.cpu.pc;
      ++expected.native_instructions;
      ++expected.native_cycles;
    }
    require(machine.cpu().debug_state() == expected, "Literal full-state prefix before refusal");
  }
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    std::string message;
    try {
      static_cast<void>(
          run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available}));
    } catch (const Fault &error) { message = error.what(); }
    auto older = expected;
    ++older.cpu.pc;
    ++older.native_instructions;
    ++older.native_cycles;
    require(run.faulted() && message.contains(reason) && message.contains("pc=") &&
                !message.contains("with active DMA") && machine.cpu().debug_state() == older,
            "Native BCR/reserved-pin validation fails without a fabricated MOVEP retirement");
    const auto reg =
        input.address == control ? DspGpioRegister::control : DspGpioRegister::direction;
    require(machine.bus().gpio(DspPort::f).read(reg) == 0 &&
                machine.bus().read({.space = DspSpace::y, .address = destination}) == payload,
            "Rejected GPIO write preserves its latch and the already completed real DMA word");
    ++faults;
  }
}
}

int main() {
  try {
    for (const unsigned address : {control, direction}) {
      for (const auto schedule : schedules) {
        for (unsigned bit = 0; bit < 11; ++bit) {
          window({.address = address, .value = 1U << bit, .schedule = schedule}, false);
        }
        for (const unsigned value : {0U, pins}) {
          const Input input{.address = address, .value = value, .schedule = schedule};
          window(input, false);
          window(input, true);
        }
      }
      rejection({.address = address, .value = pins + 1, .schedule = schedules[1]}, "reserved-bit");
      rejection({.address = address, .value = 1, .schedule = schedules[1], .peer = 1},
                "Reserved DSP Port F");
      rejection({.address = address, .value = 1, .schedule = schedules[1], .configure_bcr = false},
                "requires BCR");
    }
    std::cout << "Port-F DMA windows=" << windows << " cuts=" << cuts << " suffixes=" << suffixes
              << " clones=" << clones << " native-faults=" << faults << " assertions=" << assertions
              << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
