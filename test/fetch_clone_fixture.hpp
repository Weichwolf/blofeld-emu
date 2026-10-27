// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace clone_test {
using namespace blofeld;
using blofeld::test::require;
constexpr unsigned oscillator = 12000000, timebase = 540000000, boot = 0xff0000;
constexpr unsigned entry = 0x8000, mcu_ram = 0x1200, marker = 0x5a;
constexpr unsigned dsp_ram = 0x200, dsp_word = 0x123456, vector_ticks = 9225;
constexpr std::uint64_t period = 45;
constexpr unsigned source_words = 512, reset_edges = 195, settle_edge = 131;
constexpr unsigned nop = 0xa7, unsecured = 0xfe, word_bits = 24;
constexpr std::uint64_t unit = std::uint64_t{1} << 24U;

inline McuFlashImage image() {
  // Own LDAA #5a;STAA $1200;NOP, with literal1+3+1 native periods.
  const std::array<std::uint8_t, 6> code{0x86, marker, 0x7a, 0x12, 0, 0xa7};
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code.at(i); }
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned byte_mask = 0xff;
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

inline McuRunContract contract(const unsigned phase = 0) {
  return {.clock = {.oscillator_hz = oscillator,
                    .initial_oscillator_phase_ticks = phase,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

inline DspFetchRunInputs dsp_inputs() {
  return {.extal_hz = oscillator,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory,
          .resets = DspFetchResetTiming::reference_cycle_boundary};
}

inline McuDspFetchInputs inputs(const bool held) {
  McuDspFetchInputs result{.dsp = dsp_inputs(),
                           .shared_write_order = McuDspTieOrder::dsp_before_mcu,
                           .maximum_dsp_cycles_per_period = 16,
                           .reset_order = McuDspResetOrder::dsp_before_mcu};
  if (held) {
    result.initial_reset = McuDspFetchInitialReset{
        .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2};
  }
  return result;
}

inline DspFetchCycleInputs pins() {
  // Explicit isolated laboratory DMA inputs, not a production board default.
  return {.known_requests = 15, .grant = DspDmaGrant::available};
}

inline DspMachine &prepare(DspMachine &dsp, const bool held = false) {
  blofeld::test::DspProgram program;
  program.immediate("x1", 1);
  program.emit("clr a");
  for (unsigned i = 0; i < source_words; ++i) { program.emit("add x1,a"); }
  program.install(dsp.bus());
  dsp.bus().pll().observe_pinit(false);
  // Own fixture data establishes known/unknown RAM before native execution.
  dsp.bus().write({.space = DspSpace::x, .address = dsp_ram, .value = dsp_word});
  if (!held) {
    dsp.cpu().reset_spi_boot();
    static_cast<void>(
        dsp.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  }
  return dsp;
}

struct Frame {
  DspDebugState cpu;
  DspProgramFetchView fetch;
  std::array<std::optional<DspProgramWord>, 3> stages;
  std::optional<DspProgramWord> retiring;
  std::optional<DspState> operands;
  std::array<std::optional<DspProgramWord>, 5> phases;
  std::optional<DspState> e1, e2, candidate;
  std::uint32_t decode_address;
  std::uint64_t cycles, ticks, reset_ticks;
  bool operator==(const Frame &) const = default;
};

inline Frame frame(const DspMachine &dsp, const DspFetchRun &run) {
  const auto view = run.pipeline();
  if (!view) { throw Fault("Actual automatic pipeline required"); }
  return {.cpu = dsp.cpu().debug_state(),
          .fetch = dsp.reference_fetch(),
          .stages = view->stages,
          .retiring = view->awaiting_retirement,
          .operands = view->awaiting_inputs,
          .phases = {view->cycle.decode,
                     view->cycle.ag1,
                     view->cycle.ag2,
                     view->cycle.execute1,
                     view->cycle.execute2},
          .e1 = view->cycle.e1_operands,
          .e2 = view->cycle.e2_operands,
          .candidate = view->cycle.candidate_inputs,
          .decode_address = view->decode_address,
          .cycles = run.completed_cycles(),
          .ticks = run.elapsed_ticks(),
          .reset_ticks = run.hardware_reset_ticks()};
}

inline void independent(const DspFetchCycleResult &cycle, const DspMachine &dsp) {
  require(cycle.period_ticks == period && cycle.pipeline,
          "Every actual pipeline period retains its literal45tick clock");
  if (!cycle.pipeline) { throw Fault("Missing actual pipeline"); }
  if (!cycle.pipeline->retirement_published) { return; }
  const auto retired = cycle.pipeline->execute2;
  if (!retired) { throw Fault("Actual E2 word required for native retirement"); }
  const auto cpu = dsp.cpu().state();
  if (retired->address == boot) {
    require((cpu.x >> word_bits) == 1, "Own MOVE establishes independent X1=1");
  } else if (retired->address == boot + 2) {
    require(cpu.a == 0, "Own CLR has independent zero accumulator result");
  } else {
    require(retired->address >= boot + 3 &&
                cpu.a == static_cast<std::uint64_t>(retired->address - boot - 2) * unit,
            "Every real unrolled ADD has an independently derived full accumulator result");
  }
}

inline std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected strict checkpoint transfer refusal");
}

struct Cursor {
  std::shared_ptr<const DspFetchEventIdentity> identity = std::make_shared<DspFetchEventIdentity>();
  std::array<std::uint64_t, 7> ticks{0, 45, 6000, 9300, 9405, 10000, 11565};
  unsigned position = 0, queries = 0;
  bool forbidden = false;
  std::vector<std::uint64_t> observed;

  DspFetchExternalEvents events() {
    return {.identity = identity,
            .next_ticks = [this]() -> std::optional<std::uint64_t> {
              ++queries;
              require(!forbidden, "Snapshot staging must not query live external input");
              if (position < ticks.size()) { return ticks.at(position); }
              return std::nullopt;
            },
            .observe =
                [this](const auto time) {
                  require(!forbidden && time == ticks.at(position),
                          "Only actual timeline advancement may consume the next external event");
                  observed.push_back(time);
                  ++position;
                },
            .order = DspExternalOrder::external_before_dsp,
            .maximum_events = 4};
  }
};

struct Owners {
  explicit Owners(const unsigned phase = 0)
      : initial_phase(phase),
        mcu(image(), BLOFELD_DEFAULT_SLA, contract(phase), McuPowerOnInputs{3}) {}

  Owners(const Owners &) = delete;
  Owners &operator=(const Owners &) = delete;

  ~Owners() {
    mcu.set_cycle_observer({});
    mcu.set_reset_period_observer({});
  }

  void start(const bool held, Cursor &cursor) {
    prepare(dsp, held);
    schedule = std::make_unique<McuDspFetchSchedule>(inputs(held), mcu, dsp, cursor.events());
    attach();
  }

  void attach() {
    mcu.set_cycle_observer([this](const Hc12BusCycle cycle) {
      schedule->begin_cycle(cycle, false, pins, [this](const auto &result) { record(result); });
    });
    mcu.set_reset_period_observer([this](const McuResetPeriod period) {
      schedule->begin_reset_period(period, pins, [this](const auto &result) { record(result); });
    });
  }

  void record(const DspFetchCycleResult &result) {
    independent(result, dsp);
    trace.push_back(frame(dsp, schedule->run()));
  }

  void finish_reset() {
    if (!mcu.system_reset_active()) { return; }
    mcu.advance_system_reset(reset_edges, McuResetPad::high);
    schedule->end_system_reset();
  }

  void step() {
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    mcu.cpu().step();
    schedule->end_instruction();
  }

  const unsigned initial_phase;
  McuMachine mcu;
  DspMachine dsp;
  std::unique_ptr<McuDspFetchSchedule> schedule;
  std::vector<Frame> trace;
};

inline auto cpu(const CpuState s) {
  return std::tuple{s.pc, s.d, s.x, s.y, s.sp, s.ccr};
}

struct Endpoint {
  std::optional<decltype(cpu(CpuState{}))> mcu;
  Frame dsp;
  McuFlashImage nvm;
  DspFetchScheduleState timeline;
  std::optional<McuResetState> reset;
  std::optional<DspHardwareResetState> dsp_reset;
  std::optional<std::uint64_t> dsp_remaining;
  std::uint64_t dsp_epoch;
  std::uint64_t mcu_ticks, bus_cycles, edges, epoch, end;
};

inline Endpoint endpoint(const Owners &owner) {
  // Asserted reset has no initialized native CPU checkpoint. Compare qualified
  // reset/clock state there, and every CPU register after the real vector fetch.
  return {.mcu = owner.mcu.system_reset_active() ? std::nullopt
                                                 : std::optional{cpu(owner.mcu.cpu().state())},
          .dsp = frame(owner.dsp, owner.schedule->run()),
          .nvm = owner.mcu.bus().power_off_flash_image(),
          .timeline = owner.schedule->timeline_state(),
          .reset = owner.mcu.system_reset_state(),
          .dsp_reset = owner.schedule->run().hardware_reset_state(),
          .dsp_remaining = owner.schedule->run().hardware_reset_remaining_ticks(),
          .dsp_epoch = owner.schedule->run().native_epoch(),
          .mcu_ticks = owner.mcu.run_clock().elapsed_ticks(),
          .bus_cycles = owner.mcu.run_clock().bus_cycles(),
          .edges = owner.mcu.run_clock().reset_sysclk_edges(),
          .epoch = owner.mcu.run_clock().native_epoch(),
          .end = owner.schedule->end_ticks()};
}

inline bool equal(const Endpoint &a, const Endpoint &b) {
  return a.mcu == b.mcu && a.dsp == b.dsp && a.nvm.bytes == b.nvm.bytes &&
         a.nvm.known == b.nvm.known && a.nvm.fresh == b.nvm.fresh && a.timeline == b.timeline &&
         a.reset == b.reset && a.dsp_reset == b.dsp_reset && a.dsp_remaining == b.dsp_remaining &&
         a.dsp_epoch == b.dsp_epoch && a.mcu_ticks == b.mcu_ticks && a.bus_cycles == b.bus_cycles &&
         a.edges == b.edges && a.epoch == b.epoch && a.end == b.end;
}
}
