// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_checkpoint_fixture.hpp"
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
constexpr unsigned seed_count = 7, body = test::boot + (2 * seed_count), end = 12;
constexpr unsigned source0 = 0xffffef, destination0 = 0xffffee;
constexpr unsigned counter0 = 0xffffed, control0 = 0xffffec;
constexpr unsigned source = 0x660, destination = 0x900, word_copy = 0x880244;
constexpr unsigned payload = 0x123456, changed_source = 0x654321, canary = 0xa55a33;
constexpr unsigned bcr = 0xfffffb, port_bcr = 0x01ffff, b_field = 0x100;
constexpr unsigned fraction = 0x654321, source_shift = 24, extension_shift = 48;
constexpr unsigned extension_mask = 0xff;
constexpr unsigned positive_limit = 0x7fffff, negative_limit = 0x800000;
constexpr unsigned unchanged_flags = 0x3f;
constexpr std::uint64_t other_accumulator = 0x55a1b2c3d4e5f6;
constexpr std::uint64_t positive_overflow = 0x01000000000000;
constexpr std::uint64_t negative_overflow = 0x80000000000000;

struct Port {
  DspSpace space;
  unsigned address, bits, opcode;
};

// UM maps; FM13-136/Table12-13: full A field14, B field15, split qq bit5.
constexpr std::array ports{
    Port{.space = DspSpace::x, .address = 0xffffbd, .bits = 12, .opcode = 0x04cedd},
    Port{.space = DspSpace::y, .address = 0xffff9d, .bits = 12, .opcode = 0x04ce3d},
    Port{.space = DspSpace::y, .address = 0xfffff5, .bits = 11, .opcode = 0x09ce35},
    Port{.space = DspSpace::x, .address = 0xffffd5, .bits = 2, .opcode = 0x08ce15}};

struct Schedule {
  unsigned request, wait_first, wait_last, commit;
};

constexpr std::array schedules{
    Schedule{.request = 5, .wait_first = 0, .wait_last = 0, .commit = 6},
    Schedule{.request = 6, .wait_first = 0, .wait_last = 0, .commit = 7},
    Schedule{.request = 7, .wait_first = 0, .wait_last = 0, .commit = 8},
    Schedule{.request = 6, .wait_first = 7, .wait_last = 9, .commit = 10}};

struct Input {
  unsigned port = 0, result = 0, flags = 0;
  std::uint64_t value = 0;
  Schedule schedule = schedules[0];
  bool b = false;
};

unsigned windows = 0, cuts = 0, suffixes = 0, clones = 0, faults = 0;

unsigned mask(const Input input) {
  return (1U << ports.at(input.port).bits) - 1;
}

DspFetchRunInputs contract() {
  auto result = test::pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

void put(DspBus &bus, const unsigned address, const unsigned value) {
  bus.write({.space = DspSpace::x, .address = address, .value = value});
}

DspDebugState prepare(DspMachine &machine, const Input input) {
  test::DspProgram program;
  const auto a = input.b ? other_accumulator : input.value;
  const auto b = input.b ? input.value : other_accumulator;
  for (const bool which : {false, true}) {
    const std::string name = which ? "b" : "a";
    const auto value = which ? b : a;
    program.immediate(name + '0', value & test::word_mask);
    program.immediate(name + '1', (value >> source_shift) & test::word_mask);
    program.immediate(name + '2', (value >> extension_shift) & extension_mask);
  }
  program.immediate("sr", test::masked_status | input.flags);
  program.word(0);
  program.word(ports.at(input.port).opcode | (input.b ? b_field : 0U));
  for (unsigned index = 0; index < 16; ++index) { program.word(0); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  test::checkpoint::run(machine, seed_count);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.a == a && before.cpu.b == b &&
              before.cpu.sr == (test::masked_status | input.flags) &&
              before.native_instructions == seed_count &&
              before.native_cycles == std::uint64_t{2} * seed_count,
          "Seven native seeds establish both independent full accumulators, flags and counters");
  auto &bus = machine.bus();
  put(bus, bcr, port_bcr);
  const auto port = ports.at(input.port);
  bus.write({.space = port.space, .address = port.address + 2});
  bus.write({.space = port.space, .address = port.address + 1, .value = mask(input)});
  bus.write({.space = port.space, .address = port.address, .value = mask(input) ^ 1U});
  put(bus, source, payload);
  bus.write({.space = DspSpace::y, .address = destination, .value = canary});
  put(bus, source0, source);
  put(bus, destination0, destination);
  put(bus, counter0, 0);
  put(bus, control0, word_copy);
  static_cast<void>(machine.advance_dma({.cycle = 0,
                                         .known_requests = 15,
                                         .timing = DspDmaTiming::reference_sampled_cycles,
                                         .grant = DspDmaGrant::available}));
  static_cast<void>(machine.begin_reference_instruction(test::pipeline::execution));
  return before;
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
  const auto &phase = test::pipeline::stages(cycle);
  require(phase.retirement_published == (next >= 7) && !phase.memory_source_stall,
          "Independent one-word NOP/MOVEP costs; no adjacent accumulator producer");
  if (next >= 7) {
    ++expected.cpu.pc;
    ++expected.native_instructions;
    ++expected.native_cycles;
  }
  require(machine.cpu().debug_state() == expected,
          "Every period retains full CPU/CCR/stack, both sources and literal PC/counters");
  require(phase.memory.has_value() == (next == 7), "Exactly one actual E1 full-source store");
  const auto port = ports.at(input.port);
  if (phase.memory) {
    const auto &access = phase.memory->data;
    require(access.kind == DspNativeAccessKind::write && access.target.space == port.space &&
                access.target.address == port.address && access.pc == body + 1 &&
                access.value == input.result,
            "Full A/B supplies the literal unscaled word, never a raw low lane or other source");
  }
  require(machine.bus().read({.space = port.space, .address = port.address}) ==
              (next >= 7 ? input.result : mask(input) ^ 1U),
          "The genuine GPIO latch changes at E1 and retains the full-source value");
  const auto &dma = cycle.bus.dma;
  require(!dma.core_stall &&
              dma.phase.has_value() == (next >= schedule.request && next <= schedule.commit) &&
              dma.word.has_value() == (next == schedule.commit),
          "The independent real DMA phases and global waits retain literal boundaries");
  if (dma.phase) {
    require(dma.phase->channel == 0 &&
                dma.phase->kind == (next == schedule.request ? DspDmaPhaseKind::source_read
                                                             : DspDmaPhaseKind::destination_write),
            "One genuine IRQA edge selects only its original source/captured destination");
  }
  if (next == schedule.request) { put(machine.bus(), source, changed_source); }
  require(machine.bus().read({.space = DspSpace::y, .address = destination}) ==
              (next >= schedule.commit ? payload : canary),
          "A later source mutation cannot alter the captured DMA word");
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, DspDebugState expected) {
  while (run.completed_cycles() < end) { step(run, machine, input, expected); }
}

void window(const Input input, const bool snapshots) {
  DspMachine machine;
  auto expected = prepare(machine, input);
  DspFetchRun run(contract(), machine);
  if (!snapshots) {
    finish(run, machine, input, expected);
    ++windows;
    return;
  }
  for (const unsigned cut : {6U, 7U, 8U}) {
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

void rejected(const Input input) {
  DspMachine machine;
  auto expected = prepare(machine, input);
  DspFetchRun run(contract(), machine);
  while (run.completed_cycles() < 6) { step(run, machine, input, expected); }
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
    require(run.faulted() && message.contains("reserved-bit") &&
                message.contains("value=" + hex(input.result, 6)) && message.contains("pc=") &&
                machine.cpu().debug_state() == older,
            "A rejected limited word retains the older NOP and cannot publish store S/L flags");
    const auto port = ports.at(input.port);
    require(machine.bus().read({.space = port.space, .address = port.address}) ==
                    (mask(input) ^ 1U) &&
                machine.bus().read({.space = DspSpace::y, .address = destination}) == payload,
            "GPIO rejects the real limited word without rolling back the older DMA write");
    ++faults;
  }
}
}

int main() {
  try {
    for (unsigned port = 0; port < ports.size(); ++port) {
      for (const bool b : {false, true}) {
        for (const auto schedule : schedules) {
          for (const unsigned result : {0U, mask({.port = port})}) {
            const auto value =
                result == 0 ? 0U : (static_cast<std::uint64_t>(result) << source_shift) | fraction;
            for (const unsigned flags : {0U, unchanged_flags}) {
              window({.port = port,
                      .result = result,
                      .flags = flags,
                      .value = value,
                      .schedule = schedule,
                      .b = b},
                     false);
            }
          }
          const auto value =
              (static_cast<std::uint64_t>(mask({.port = port})) << source_shift) | fraction;
          window({.port = port,
                  .result = mask({.port = port}),
                  .value = value,
                  .schedule = schedule,
                  .b = b},
                 true);
        }
        rejected({.port = port,
                  .result = positive_limit,
                  .value = positive_overflow,
                  .schedule = schedules[1],
                  .b = b});
        rejected({.port = port,
                  .result = negative_limit,
                  .value = negative_overflow,
                  .schedule = schedules[1],
                  .b = b});
      }
    }
    std::cout << "Full GPIO word/DMA windows=" << windows << " cuts=" << cuts
              << " suffixes=" << suffixes << " clones=" << clones << " faults=" << faults << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
