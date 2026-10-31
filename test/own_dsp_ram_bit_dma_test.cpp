// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>

namespace {
using namespace blofeld;
using namespace dsp56300;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .switch_timing = DspClockSwitch::reference_cycle_boundary};
constexpr Word boot = 0xff0000, entry = 0x100, normal_omr = 0x300;
constexpr Word status = 0xc003ff, canary = 0x654321, pattern = 0xa55aa5;
constexpr Word movec_omr = 0x05f43a, movec_sr = 0x05f439, immediate_x0 = 0x44f400;
constexpr Word long_jump = 0x0af080, positive = 32, negative = word_mask - 31;
constexpr Word source_register = 0xffffef, destination_register = 0xffffee;
constexpr Word counter_register = 0xffffed, control_register = 0xffffec;
constexpr Word x_copy = 0x880240, y_copy = 0x880244, p_copy = 0x880248;
constexpr Word source_ram = 0x800, destination_ram = 0x900, program_guard = 0x1f0;
constexpr unsigned ram_words = 64, base_cost = 5, bound = 32;
constexpr std::uint64_t hold_ticks = 90, recovery_ticks = 135, period = 45;
constexpr auto model = OwnDspNativeCycleModel::isolated_effect;
constexpr std::array endpoints{std::array{0U, 0U}, std::array{6U, 10U}, std::array{63U, 23U}};
enum class Mode : std::uint8_t { quiet, before, after, program_hold, source };

struct Sample {
  unsigned address = 0, bit = 0, waits = 0;
  Word displacement = positive;
  bool y = false, set = false, level = false;
  Mode mode = Mode::quiet;
};

void require(const bool value, const char *message) {
  if (!value) { throw std::runtime_error(message); }
}

Word opcode(const Sample sample) {
  // FM13-27/30:literal fixed-AA6 BRCLR/BRSET,with independent space/sense/bit fields.
  constexpr Word family = 0x0c8080;
  constexpr Word y_bit = 0x40;
  constexpr Word set_bit = 0x20;
  return family | (sample.address << 8U) | (sample.y ? y_bit : 0U) | (sample.set ? set_bit : 0U) |
         sample.bit;
}

Word value(const Sample sample, const bool level) {
  const Word mask = 1U << sample.bit;
  return ((pattern ^ (sample.address << 8U)) & ~mask) | (level ? mask : 0U);
}

DspAddress operand(const Sample sample) {
  return {.space = sample.y ? DspSpace::y : DspSpace::x, .address = sample.address};
}

DspAddress destination(const Sample sample) {
  if (sample.mode == Mode::program_hold) { return {.address = program_guard}; }
  if (sample.mode == Mode::before || sample.mode == Mode::after) { return operand(sample); }
  return {.space = DspSpace::y, .address = destination_ram};
}

DspAddress source(const Sample sample) {
  if (sample.mode == Mode::source) { return operand(sample); }
  return {.space = DspSpace::x, .address = source_ram};
}

void complete(OwnDspMachine &machine) {
  auto step = machine.begin_fetch();
  for (unsigned effect = 0; effect < bound && step.pending; ++effect) {
    step = machine.advance(step.pending->ticket);
  }
  require(step.retired && step.fault.kind == FaultKind::none, "Literal native setup failed");
}

void prepare(OwnDspMachine &machine, const Sample sample) {
  const std::array<Word, 8> setup{
      movec_omr, normal_omr, movec_sr, status, immediate_x0, canary, long_jump, entry};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot, .words = setup});
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3});
  require(machine.advance_hardware_reset_ticks(hold_ticks) == hold_ticks, "Lost reset hold");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(recovery_ticks) == recovery_ticks,
          "Lost reset recovery");
  for (unsigned instruction = 0; instruction < 4; ++instruction) { complete(machine); }
  const auto regs = machine.cpu().registers();
  require(regs.pc == entry && regs.omr == normal_omr && regs.sr == status && regs.x == canary,
          "Real reset and native setup must establish PC,OMR,CCR and X0");
  for (unsigned pc = entry - ram_words; pc < entry + ram_words; ++pc) {
    machine.bus().write({.address = pc});
  }
  machine.bus().write({.address = entry, .value = opcode(sample)});
  machine.bus().write({.address = entry + 1, .value = sample.displacement});
  machine.bus().write({.address = program_guard});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = 0; address < ram_words; ++address) {
      machine.bus().write({.space = space, .address = address, .value = canary});
    }
  }
  machine.bus().write({.space = operand(sample).space,
                       .address = sample.address,
                       .value = value(sample, sample.level)});
  machine.bus().write(
      {.space = DspSpace::x, .address = source_ram, .value = value(sample, !sample.level)});
  machine.bus().write({.space = DspSpace::y, .address = destination_ram, .value = canary});
}

void configure(DspBus &bus, const Sample sample) {
  const auto output = destination(sample);
  Word control = y_copy;
  if (output.space == DspSpace::x) { control = x_copy; }
  if (output.space == DspSpace::p) { control = p_copy; }
  if (source(sample).space == DspSpace::y) { control |= 1U; }
  // Testbench writes actual DCR/DSR/DDR/DCO registers while the CPU is stopped.
  // FM10:DTM1,IRQA,fixed source/destination spaces;no invented transfer response.
  for (const auto store :
       {DspStore{.space = DspSpace::x, .address = source_register, .value = source(sample).address},
        DspStore{.space = DspSpace::x, .address = destination_register, .value = output.address},
        DspStore{.space = DspSpace::x, .address = counter_register},
        DspStore{.space = DspSpace::x, .address = control_register, .value = control}}) {
    bus.write(store);
  }
}

struct Rig {
  OwnDspMachine machine{clocks};
  std::unique_ptr<OwnDspFetch> fetch;

  explicit Rig(const Sample sample) {
    prepare(machine, sample);
    fetch = std::make_unique<OwnDspFetch>(machine);
    for (unsigned word = 0; word < 2; ++word) {
      fetch->issue();
      require(fetch->view().pending_address == entry + word, "Actual F1 must own the literal pair");
      fetch->capture();
    }
    require(fetch->begin().fault.kind == FaultKind::none, "Actual captured pair was rejected");
    configure(machine.bus(), sample);
  }

  explicit Rig(const OwnDspFetchCheckpoint &saved)
      : fetch(std::make_unique<OwnDspFetch>(machine, saved)) {}

  void restore(const OwnDspFetchCheckpoint &saved) {
    fetch = std::make_unique<OwnDspFetch>(machine, saved);
  }
};

struct Oracle {
  Registers initial;
  Progress progress;
  unsigned cycle = 0, reads = 0, pcu = 0, retirements = 0, writes = 0, holds = 0;
};

unsigned final_cycle(const Sample sample) {
  if (sample.mode == Mode::quiet) { return 3; }
  if (sample.mode == Mode::before) { return 5 + sample.waits; }
  if (sample.mode == Mode::program_hold) { return 4 + sample.waits; }
  return 5;
}

bool admit(const Sample sample, const unsigned cycle) {
  if (cycle == 0) { return false; }
  if (sample.mode == Mode::before) { return cycle >= 2; }
  if (sample.mode == Mode::after || sample.mode == Mode::source) {
    return cycle == 1 || cycle >= 4;
  }
  return true;
}

bool held(const Sample sample, const unsigned cycle) {
  return (sample.mode == Mode::before || sample.mode == Mode::program_hold) && cycle >= 2 &&
         cycle <= 2 + sample.waits;
}

void dma_phase(const OwnDspFetchCycleResult &result, const Sample sample, const unsigned cycle) {
  const unsigned first = sample.mode == Mode::after ? 2U : 1U;
  const unsigned write = sample.mode == Mode::after || sample.mode == Mode::source ? 3U : 2U;
  const unsigned last = write + sample.waits;
  const bool selected = sample.mode != Mode::quiet && cycle >= first && cycle <= last;
  require(result.dma.phase.has_value() == selected, "Only the requested actual DMA phases execute");
  require(result.dma.word.has_value() == (selected && cycle == last),
          "One actual DMA word completes at the independently selected reference clock");
  if (!selected) { return; }
  if (!result.dma.phase) { throw std::runtime_error("Selected DMA phase is absent"); }
  const auto &phase = *result.dma.phase;
  require(phase.kind == (cycle < write ? DspDmaPhaseKind::source_read
                                       : DspDmaPhaseKind::destination_write) &&
              phase.source.address == source(sample).address &&
              phase.destination.address == destination(sample).address,
          "Real source contention and destination holds retain their original endpoints");
  if (cycle >= write) {
    const auto expected = value(sample, sample.mode == Mode::source ? sample.level : !sample.level);
    require(phase.value == expected, "Every held destination retains its actual captured word");
  }
}

void native(const OwnDspFetchCycleResult &result,
            const std::optional<PendingInstruction> &pending,
            const Sample sample,
            Oracle &oracle) {
  const bool expected =
      admit(sample, oracle.cycle) && !held(sample, oracle.cycle) && oracle.retirements == 0;
  require(result.native.has_value() == expected, "Only actual unheld admitted effects can execute");
  if (!result.native) { return; }
  if (!pending) { throw std::runtime_error("Admitted native effect lost its actual owner"); }
  require(pending->effect.pc == entry && pending->effect.omr == normal_omr &&
              result.native->fault.kind == FaultKind::none,
          "The genuine pending native effect must retain its instruction context");
  const auto &effect = pending->effect;
  if (effect.kind == EffectKind::read) {
    require(effect.space == (sample.y ? Space::y : Space::x) && effect.address == sample.address &&
                pending->opcode == opcode(sample),
            "One actual AA6 RAM read owns its source");
    ++oracle.reads;
  } else if (effect.kind == EffectKind::pcu_update) {
    ++oracle.pcu;
  } else {
    require(effect.kind == EffectKind::retire && result.native->retired,
            "Read-only AA6 cannot invent RMW,AGU,ALU or bit-I/O effects");
    ++oracle.retirements;
  }
}

Word cell(const Sample sample, const Oracle &oracle, const DspAddress address) {
  const auto output = destination(sample);
  if (oracle.writes != 0 && address.space == output.space && address.address == output.address) {
    return value(sample, sample.mode == Mode::source ? sample.level : !sample.level);
  }
  if (address.space == operand(sample).space && address.address == sample.address) {
    return value(sample, sample.level);
  }
  return address.space == DspSpace::p ? 0U : canary;
}

void memory(Rig &rig, const Sample sample, const Oracle &oracle) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = 0; address < ram_words; ++address) {
      require(rig.machine.bus().read({.space = space, .address = address}) ==
                  cell(sample, oracle, {.space = space, .address = address}),
              "Every low RAM cell and opposite-bank canary follows actual DMA writes only");
    }
  }
  for (const auto target : {DspAddress{.space = DspSpace::y, .address = destination_ram},
                            DspAddress{.address = program_guard}}) {
    require(rig.machine.bus().read(target) == cell(sample, oracle, target),
            "Independent destination guards expose every actual DMA write");
  }
  require(rig.machine.bus().read({.space = DspSpace::x, .address = source_ram}) ==
              value(sample, !sample.level),
          "The DMA source cannot change during a held transfer");
  for (unsigned pc = entry - ram_words; pc < entry + ram_words; ++pc) {
    Word expected = 0;
    if (pc == entry) { expected = opcode(sample); }
    if (pc == entry + 1) { expected = sample.displacement; }
    require(rig.machine.bus().read({.address = pc}) == expected,
            "Actual executable and prefetched backing words cannot be rewritten by the test");
  }
}

void verify(Rig &rig, const Sample sample, const Oracle &oracle) {
  auto expected = oracle.initial;
  if (oracle.pcu != 0) {
    const bool level = sample.mode == Mode::before ? !sample.level : sample.level;
    expected.pc = level == sample.set ? (entry + sample.displacement) & word_mask : entry + 2;
  }
  auto progress = oracle.progress;
  progress.instructions += oracle.retirements;
  progress.estimated_cycles += static_cast<std::uint64_t>(oracle.retirements) * base_cost;
  require(
      rig.machine.cpu().registers() == expected && rig.machine.cpu().progress() == progress,
      "Literal branch truth,all CPU/CCR/stack fields and native base counters hold every cycle");
  const auto cycles = oracle.cycle + 1;
  require(rig.machine.clock().elapsed_ticks() == hold_ticks + recovery_ticks + cycles * period &&
              rig.machine.clock().native_cycle_units() == cycles &&
              rig.machine.bus().last_dma_cycle() == oracle.cycle,
          "Actual DMA holds preserve CPU ownership while PLL/reference time continues");
  memory(rig, sample, oracle);
}

void step(Rig &rig, const Sample sample, Oracle &oracle) {
  const unsigned first = sample.mode == Mode::after ? 2U : 1U;
  const auto requests = sample.mode != Mode::quiet && oracle.cycle >= first ? 1U : 0U;
  const auto grant = held(sample, oracle.cycle) && oracle.cycle < 2 + sample.waits
                         ? DspDmaGrant::wait
                         : DspDmaGrant::available;
  const auto pending = rig.machine.cpu().pending_instruction();
  const auto before = rig.machine.cpu().registers();
  const auto fetch = rig.fetch->view();
  const auto result =
      rig.fetch->advance_dma_pipeline({.cycle = oracle.cycle,
                                       .known_requests = 15,
                                       .requests = requests,
                                       .timing = DspDmaTiming::reference_sampled_cycles,
                                       .grant = grant},
                                      admit(sample, oracle.cycle) && oracle.retirements == 0,
                                      model);
  dma_phase(result, sample, oracle.cycle);
  require(result.dma.core_stall == held(sample, oracle.cycle),
          "Only real RAM or issued-F2 conflicts may hold the CPU");
  if (result.dma.core_stall) {
    ++oracle.holds;
    require(!result.captured && !result.issued_address && rig.fetch->view() == fetch &&
                rig.machine.cpu().registers() == before,
            "DMA freezes the actual pending F2/native owners without reissuing or resampling");
  }
  if (result.dma.word) { ++oracle.writes; }
  native(result, pending, sample, oracle);
  verify(rig, sample, oracle);
  if (result.native && result.native->retired) { rig.fetch->synchronize(); }
  ++oracle.cycle;
}

void finish(Rig &rig, const Sample sample, Oracle oracle) {
  while (oracle.cycle <= final_cycle(sample)) { step(rig, sample, oracle); }
  const unsigned hold_count =
      sample.mode == Mode::before || sample.mode == Mode::program_hold ? 1 + sample.waits : 0U;
  require(
      oracle.reads == 1 && oracle.pcu == 1 && oracle.retirements == 1 &&
          oracle.writes == (sample.mode == Mode::quiet ? 0U : 1U) && oracle.holds == hold_count,
      "One actual RAM sample,one PCU publication,one retirement,and the literal DMA word/holds");
}

unsigned cases = 0, restores = 0, clones = 0;

void one(const Sample sample) {
  Rig rig(sample);
  Oracle oracle{.initial = rig.machine.cpu().registers(), .progress = rig.machine.cpu().progress()};
  const bool snapshots = sample.address == 6 && sample.bit == 10 &&
                         sample.displacement == positive &&
                         (sample.waits == 3 || sample.mode == Mode::quiet ||
                          sample.mode == Mode::after || sample.mode == Mode::source);
  if (!snapshots) {
    finish(rig, sample, oracle);
  } else {
    while (oracle.cycle <= final_cycle(sample)) {
      const auto saved = rig.fetch->checkpoint();
      finish(rig, sample, oracle);
      rig.restore(saved);
      ++restores;
      Rig fresh(saved);
      finish(fresh, sample, oracle);
      ++clones;
      step(rig, sample, oracle);
    }
  }
  ++cases;
}

void variants(Sample sample) {
  for (const auto mode :
       {Mode::quiet, Mode::before, Mode::after, Mode::program_hold, Mode::source}) {
    sample.mode = mode;
    for (const unsigned waits : {0U, 1U, 3U}) {
      if (waits != 0 && mode != Mode::before && mode != Mode::program_hold) { continue; }
      sample.waits = waits;
      one(sample);
    }
  }
}

void run() {
  for (const auto endpoint : endpoints) {
    for (const bool y : {false, true}) {
      for (const bool set : {false, true}) {
        for (const bool level : {false, true}) {
          for (const auto displacement : {positive, negative}) {
            variants({.address = endpoint[0],
                      .bit = endpoint[1],
                      .displacement = displacement,
                      .y = y,
                      .set = set,
                      .level = level});
          }
        }
      }
    }
  }
  constexpr unsigned expected_cases = 432;
  constexpr unsigned expected_cuts = 264;
  require(cases == expected_cases && restores == expected_cuts && clones == expected_cuts,
          "Every literal DMA case and before/after-sample checkpoint cut must execute");
}
}

int main() {
  try {
    std::cout << "OWN_RAM_DMA begin\n" << std::flush;
    run();
    std::cout << "OWN_RAM_DMA cases=" << cases << " restores=" << restores << " clones=" << clones
              << " passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "OWN_RAM_DMA cases=" << cases << ": " << error.what() << '\n';
    return 1;
  }
}
