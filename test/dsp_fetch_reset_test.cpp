// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned extal = 12000000, timebase = 540000000, period = 45;
constexpr unsigned pipeline_cuts = 18;
constexpr unsigned branch_back = 0x050fdf, ram = 0x100, marker = 0x123456;
constexpr unsigned reset_status = 0xc00300, expected_omr = 0x305, reset_pctl = 0x04401d;
constexpr unsigned held_ticks = 91, partial_recovery = 44, recovery_cycles = 3;
constexpr unsigned whole_recovery = recovery_cycles * period, suffix_cycles = 32;
constexpr std::uint64_t accumulator_unit = std::uint64_t{1} << 24U;

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected native fetch-reset rejection");
}

DspHardwareResetInputs pins() {
  return {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = recovery_cycles};
}

DspFetchRunInputs inputs(const bool qualified = true) {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory,
          .resets = qualified ? DspFetchResetTiming::reference_cycle_boundary
                              : DspFetchResetTiming::unqualified};
}

DspFetchCycleInputs cycle_inputs() {
  return {.known_requests = 15, .grant = DspDmaGrant::available};
}

DspMachine &prepare(DspMachine &machine) {
  DspProgram program;
  program.immediate("x1", 1);
  program.emit("clr a");
  program.emit("add x1,a");
  program.word(branch_back);
  constexpr unsigned padding = 32;
  for (unsigned i = 0; i < padding; ++i) { program.emit("nop"); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().write({.space = DspSpace::x, .address = ram, .value = marker});
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return machine;
}

struct Ledger {
  std::uint64_t cycles = 0, ticks = 0, reset_ticks = 0, clk2 = 0, epoch_start = 0, epoch = 1;
  std::uint64_t adds = 0;
  unsigned pc = boot;
};

struct Native {
  DspDebugState cpu;
  DspProgramFetchView fetch;
  std::array<std::optional<DspProgramWord>, 3> stages;
  std::optional<DspProgramWord> retirement;
  bool operator==(const Native &) const = default;
};

struct Fixture {
  explicit Fixture(const bool qualified = true) : clock(inputs(qualified), prepare(machine)) {}

  [[nodiscard]] Native native() const {
    const auto pipeline = clock.pipeline();
    if (!pipeline) { throw Fault("Native automatic reset fixture requires a pipeline"); }
    return {.cpu = machine.cpu().debug_state(),
            .fetch = machine.reference_fetch(),
            .stages = pipeline->stages,
            .retirement = pipeline->awaiting_retirement};
  }

  void time() const {
    require(clock.completed_cycles() == ledger.cycles && clock.elapsed_ticks() == ledger.ticks &&
                clock.hardware_reset_ticks() == ledger.reset_ticks &&
                clock.timer_clk2_edges() == ledger.clk2 && clock.native_epoch() == ledger.epoch &&
                ledger.ticks == (ledger.cycles * period) + ledger.reset_ticks,
            "Lifetime fetch/reset/CLK2/epoch ledgers match independent integer expectations");
  }

  void advance(const unsigned count) {
    for (unsigned i = 0; i < count; ++i) {
      const auto result = clock.advance_cycle(machine, cycle_inputs());
      require(result.start_ticks == ledger.ticks && result.period_ticks == period,
              "Every genuine native fetch period retains the independent bypass clock");
      ledger.ticks += period;
      ledger.clk2 += (ledger.cycles - ledger.epoch_start) & 1U;
      ++ledger.cycles;
      require(result.completed_cycles == ledger.cycles && result.completion_ticks == ledger.ticks,
              "Native lifetime cycles continue across reset instead of counting held time");
      if (!result.pipeline) { throw Fault("Actual cycle lost its automatic pipeline result"); }
      const auto &phase = *result.pipeline;
      if (phase.retirement_published) {
        require(phase.execute2 && phase.execute2->address == ledger.pc,
                "Post-reset native execution follows independent MOVE/CLR/ADD/BRA addresses");
        if (ledger.pc == boot) {
          ledger.pc = boot + 2;
        } else if (ledger.pc == boot + 2) {
          require(machine.cpu().state().a == 0, "Native CLR defines the literal zero result");
          ledger.pc = boot + 3;
        } else if (ledger.pc == boot + 3) {
          ++ledger.adds;
          require(machine.cpu().state().a == ledger.adds * accumulator_unit,
                  "Each actual native ADD has its independently counted full result");
          ledger.pc = boot + 4;
        } else {
          ledger.pc = boot + 3;
        }
      }
      trace.push_back(native());
      time();
    }
  }

  void begin() {
    clock.begin_hardware_reset(machine, pins());
    ++ledger.epoch;
    ledger.epoch_start = ledger.cycles;
    ledger.adds = 0;
    ledger.pc = boot;
    require(machine.hardware_reset_active(), "Native machine must expose its active held reset");
    const auto cpu = machine.cpu().debug_state();
    require(cpu.cpu.pc == boot && cpu.cpu.omr == expected_omr && cpu.cpu.sr == reset_status &&
                cpu.cpu.sp == 0 && cpu.cpu.vba == 0 && cpu.sc == 0 &&
                cpu.native_instructions == 0 && cpu.native_cycles == 0,
            "Actual native reset defines independent PC/modes/SR/stack/counter values");
    for (const auto modifier : cpu.cpu.m) {
      require(modifier == word_mask, "Every native modulo register returns to linear mode");
    }
    const auto current = native();
    require(!current.fetch.active && current.fetch.used == 0 && !current.fetch.pending_address &&
                !current.retirement && machine.bus().pll().read() == reset_pctl &&
                machine.bus().read({.space = DspSpace::x, .address = ram}) == marker,
            "Reset flushes actual F1/F2/candidate state and preserves known RAM");
    for (const auto &stage : current.stages) {
      require(!stage, "No old decoded native instruction survives hardware reset");
    }
    time();
  }

  void held(const std::uint64_t requested, const std::uint64_t expected) {
    const auto before = native();
    require(clock.advance_hardware_reset_ticks(machine, requested) == expected,
            "Only the independently expected hold/recovery duration is consumed");
    ledger.ticks += expected;
    ledger.reset_ticks += expected;
    require(machine.cpu().debug_state() == before.cpu && !machine.bus().last_dma_cycle(),
            "No native instruction or DMA phase executes during held/recovering EXTAL time");
    time();
  }

  void gated() {
    const auto before = native();
    const DspDmaInputs dma{.cycle = ledger.cycles,
                           .known_requests = 15,
                           .timing = DspDmaTiming::reference_sampled_cycles,
                           .grant = DspDmaGrant::available};
    require(machine.hardware_reset_active() && rejects([&] {
                                                 static_cast<void>(machine.advance_dma(dma));
                                               }).contains("hardware reset") &&
                rejects([&] {
                  static_cast<void>(machine.advance_reference_bus_cycle(
                      {.dma = dma, .order = DspReferenceBusOrder::dma_then_fetch_then_native}));
                }).contains("hardware reset") &&
                rejects([&] { machine.cpu().reset_spi_boot(); }).contains("hardware reset") &&
                rejects([&] {
                  static_cast<void>(machine.begin_reference_instruction(
                      DspReferenceExecution::staged_register_boundaries));
                }).contains("hardware reset") &&
                rejects([&] {
                  static_cast<void>(clock.advance_cycle(machine, cycle_inputs()));
                }).contains("gated") &&
                rejects([&] {
                  static_cast<void>(clock.next_cycle_ticks(machine));
                }).contains("gated") &&
                rejects([&] { machine.cpu().step(); }).contains("hardware reset") &&
                native() == before && !machine.bus().last_dma_cycle() && !clock.faulted(),
            "Machine/core/native/reference/DMA admissions respect the actual held reset gate");
    time();
  }

  DspMachine machine;
  DspFetchRun clock;
  Ledger ledger;
  std::vector<Native> trace;
};

void reset_case(const unsigned prefix) {
  Fixture f;
  f.advance(prefix);
  f.begin();
  f.gated();
  f.held(held_ticks, held_ticks);
  const auto state = f.clock.hardware_reset_state();
  require(state && state->phase == DspHardwareResetPhase::asserted &&
              state->asserted_extal_cycles == 2 && state->phase_ticks == 1,
          "91 reference ticks retain two whole45tick EXTAL periods and one fractional tick");
  const auto checkpoint = f.clock.checkpoint(f.machine);
  const auto ledger = f.ledger;
  std::vector<Native> expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.clock.restore(checkpoint, f.machine);
    f.ledger = ledger;
    f.trace.clear();
    f.clock.release_hardware_reset(f.machine);
    f.gated();
    require(f.clock.hardware_reset_remaining_ticks() == whole_recovery,
            "Release starts exactly three supplied bypass EXTAL periods");
    f.held(whole_recovery - 1, whole_recovery - 1);
    f.gated();
    f.held(period, 1);
    require(!f.clock.hardware_reset_active() && !f.machine.hardware_reset_active() &&
                f.machine.reference_fetch().active && f.machine.reference_fetch().used == 0 &&
                f.ledger.clk2 == ledger.clk2,
            "Recovery clamps at its literal endpoint before any native F2/CLK2 period");
    f.advance(suffix_cycles);
    require(f.ledger.adds > 0, "Recovered native program performs genuine arithmetic");
    if (replay == 0) {
      expected = f.trace;
    } else {
      require(f.trace == expected, "Trusted held snapshot repeats the entire native suffix");
    }
  }
}

void recovery_checkpoint() {
  Fixture f;
  f.advance(7);
  f.begin();
  f.held(held_ticks, held_ticks);
  f.clock.release_hardware_reset(f.machine);
  f.held(partial_recovery, partial_recovery);
  const auto checkpoint = f.clock.checkpoint(f.machine);
  const auto saved = f.ledger;
  const auto native = f.native();
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    f.clock.restore(checkpoint, f.machine);
    f.ledger = saved;
    require(f.native() == native && f.clock.hardware_reset_remaining_ticks() == held_ticks,
            "Recovery snapshot retains the native suspended boot opcode and44tick phase");
    f.held(whole_recovery, held_ticks);
    f.advance(suffix_cycles);
    f.begin();
    f.held(2ULL * period, 2ULL * period);
    f.clock.release_hardware_reset(f.machine);
    f.held(whole_recovery, whole_recovery);
    f.advance(suffix_cycles);
  }
}

void pin_guards(const unsigned kind) {
  Fixture f(kind != 0);
  f.advance(7);
  auto invalid = pins();
  if (kind == 1) { invalid.mode_known = 7; }
  if (kind == 2) { invalid.mode_levels = 4; }
  if (kind == 3) { invalid.pinit.reset(); }
  if (kind == 4) { invalid.pinit = true; }
  if (kind == 5) { invalid.recovery_cycles = 1; }
  const auto before = f.native();
  require(!rejects([&] { f.clock.begin_hardware_reset(f.machine, invalid); }).empty() &&
              f.native() == before && !f.clock.hardware_reset_active() && !f.clock.faulted(),
          "Unknown/unsupported pins,recovery or default contract reject before native reset");
  f.advance(1);
}

void phase_and_overflow() {
  Fixture f;
  f.advance(1);
  f.begin();
  f.held((2 * period) - 1, (2 * period) - 1);
  const auto before = f.native();
  require(!rejects([&] { f.clock.release_hardware_reset(f.machine); }).empty() &&
              !rejects([&] { f.clock.begin_hardware_reset(f.machine, pins()); }).empty() &&
              !rejects([&] { f.clock.advance_hardware_reset_ticks(f.machine, 0); }).empty() &&
              f.native() == before && !f.clock.faulted(),
          "Early release,reassertion and zero duration preserve the held native owner");
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  require(!rejects([&] { f.clock.advance_hardware_reset_ticks(f.machine, maximum); }).empty() &&
              f.native() == before && !f.clock.faulted(),
          "Lifetime-time overflow rejects before consuming a fabricated reset interval");
  f.time();
  f.held(1, 1);
  f.clock.release_hardware_reset(f.machine);
  f.held(whole_recovery, whole_recovery);
  f.advance(suffix_cycles);
}

namespace store {
constexpr unsigned entry = 0x100, data = 0x400, canary = 0x654321;
constexpr unsigned high = 0x234567, low = 0x12345, opcode = 0x485c00;

void prepare(DspMachine &machine) {
  DspProgram program;
  program.immediate("r4", data);
  program.immediate("a0", low);
  program.immediate("a1", high);
  program.immediate("a2", 0);
  for (unsigned i = 0; i < 5; ++i) { program.emit("nop"); }
  constexpr unsigned absolute_jump = 0x0af080;
  program.word(absolute_jump);
  program.word(entry);
  constexpr unsigned prefetch_padding = 32;
  for (unsigned i = 0; i < prefetch_padding; ++i) { program.emit("nop"); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  constexpr unsigned maximum_instructions = 16;
  constexpr unsigned padding = 32;
  for (unsigned i = 0; i < maximum_instructions && machine.cpu().state().pc != entry; ++i) {
    machine.cpu().step();
  }
  require(machine.cpu().state().pc == entry && machine.cpu().state().r[4] == data &&
              machine.cpu().state().a == ((static_cast<std::uint64_t>(high) << word_bits) | low),
          "Own native instructions establish the independent long-store source and pointer");
  for (unsigned i = 0; i < padding; ++i) {
    machine.bus().write({.address = entry + i, .value = i == 0 ? opcode : 0});
  }
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    machine.bus().write({.space = space, .address = data, .value = canary});
  }
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
}
}

void pending_store(const bool before) {
  constexpr unsigned high = 0x234567;
  constexpr unsigned low = 0x12345;
  constexpr unsigned maximum_cycles = 128;

  DspMachine machine;
  store::prepare(machine);
  DspFetchRun clock(inputs(), machine);
  bool found = false;
  for (unsigned count = 0; count < maximum_cycles && !found; ++count) {
    const auto cycle = clock.advance_cycle(machine, cycle_inputs());
    if (!cycle.pipeline) { throw Fault("Long-store reset fixture lost its actual pipeline"); }
    const auto &phase = *cycle.pipeline;
    found = before ? phase.ag1 && phase.ag1->address == store::entry
                   : phase.memory && phase.memory->data.pc == store::entry &&
                         phase.memory->data.kind == DspNativeAccessKind::write;
  }
  require(found, "Actual AG1/store completion reaches the requested reset boundary");
  const auto expected_x = before ? store::canary : high;
  const auto expected_y = before ? store::canary : low;
  const auto check_memory = [&] {
    require(machine.bus().read({.space = DspSpace::x, .address = store::data}) == expected_x &&
                machine.bus().read({.space = DspSpace::y, .address = store::data}) == expected_y,
            "Reset retains completed native X/Y writes and never commits pending AG1 stores");
  };
  check_memory();
  const auto cycles = clock.completed_cycles();
  const auto ticks = clock.elapsed_ticks();
  clock.begin_hardware_reset(machine, pins());
  check_memory();
  require(clock.advance_hardware_reset_ticks(machine, held_ticks) == held_ticks,
          "Pending-store reset retains the independent assertion duration");
  const auto saved = clock.checkpoint(machine);
  machine.bus().write({.space = DspSpace::x, .address = store::data, .value = 7});
  clock.restore(saved, machine);
  check_memory();
  clock.release_hardware_reset(machine);
  require(clock.advance_hardware_reset_ticks(machine, whole_recovery) == whole_recovery &&
              clock.completed_cycles() == cycles &&
              clock.elapsed_ticks() == ticks + held_ticks + whole_recovery,
          "Native memory-stage cancellation contributes only actual reset/recovery time");
  check_memory();
  bool retired = false;
  for (unsigned count = 0; count < maximum_cycles && !retired; ++count) {
    const auto cycle = clock.advance_cycle(machine, cycle_inputs());
    if (!cycle.pipeline) { throw Fault("Recovered long store lost its native retirement"); }
    const auto &phase = *cycle.pipeline;
    retired =
        phase.retirement_published && phase.execute2 && phase.execute2->address == store::entry;
  }
  require(retired && machine.cpu().state().r[4] == store::data + 1 &&
              machine.bus().read({.space = DspSpace::x, .address = store::data}) == high &&
              machine.bus().read({.space = DspSpace::y, .address = store::data}) == low,
          "Cold native setup executes the original owned long store again with literal results");
}
}

int main() {
  try {
    for (unsigned prefix = 0; prefix < pipeline_cuts; ++prefix) { reset_case(prefix); }
    recovery_checkpoint();
    for (unsigned kind = 0; kind < 6; ++kind) { pin_guards(kind); }
    phase_and_overflow();
    pending_store(true);
    pending_store(false);
    std::cout
        << "28 native pipeline reset scenarios passed:18 cycle cuts,1 recovery,7 guards,2 stores\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
