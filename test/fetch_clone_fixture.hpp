// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
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
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace clone_test {
using namespace blofeld;
constexpr unsigned oscillator = 12000000, timebase = 540000000, boot = 0xff0000;
constexpr unsigned entry = 0x8000, mcu_ram = 0x1200, marker = 0x5a;
constexpr unsigned dsp_ram = 0x200, dsp_word = 0x123456, vector_ticks = 9225;
constexpr std::uint64_t period = 45;
constexpr unsigned source_words = 512, reset_edges = 195, settle_edge = 131;
constexpr unsigned nop = 0xa7, unsecured = 0xfe, word_bits = 24;
constexpr auto timing =
    OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts;
constexpr DspRunClockInputs dsp_clock{.extal_hz = oscillator,
                                      .timebase_hz = timebase,
                                      .switch_timing = DspClockSwitch::reference_cycle_boundary};
constexpr OwnDspResetInputs reset_pins{
    .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2};
constexpr std::uint64_t unit = std::uint64_t{1} << 24U;
constexpr std::array<std::uint64_t, 7> external_ticks{0, period, 6000, 9300, 9405, 10000, 11565};

inline void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

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

inline McuDspRuntimeInputs inputs() {
  return {.timing = timing,
          .maximum_cycles_per_period = 16,
          .writes = McuWriteOrder::before_mcu,
          .reset_dsp_first = true,
          .initial_reset = reset_pins};
}

inline DspDmaInputs pins() {
  return {.known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

inline void prepare(OwnDspMachine &dsp, const bool held = false) {
  // Independently encoded FM MOVE #>1,X1; CLR A; unrolled ADD X1,A.
  constexpr unsigned immediate_x1 = 0x45f400;
  constexpr unsigned clear_a = 0x200013;
  constexpr unsigned add_x1 = 0x200060;
  std::array<std::uint32_t, source_words + 3> code{};
  code[0] = immediate_x1;
  code[1] = 1;
  code[2] = clear_a;
  for (unsigned i = 3; i < code.size(); ++i) { code.at(i) = add_x1; }
  dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = code});
  dsp.bus().write({.space = DspSpace::x, .address = dsp_ram, .value = dsp_word});
  if (held) { return; }
  dsp.begin_hardware_reset(reset_pins);
  require(dsp.advance_hardware_reset_ticks(period * 2) == period * 2, "Actual DSP reset hold");
  dsp.release_hardware_reset();
  require(dsp.advance_hardware_reset_ticks(period * 2) == period * 2, "Actual DSP reset recovery");
}

struct Frame {
  dsp56300::Registers cpu;
  dsp56300::Progress progress;
  std::optional<dsp56300::InstructionEffect> pending;
  std::optional<DspProgramFetchView> fetch;
  std::uint64_t cycles, ticks, reset_ticks, clk2;
  bool interrupt_pending;
  bool operator==(const Frame &) const = default;
};

inline Frame frame(const OwnDspMachine &dsp, const OwnDspFetch *fetch = nullptr) {
  const auto pending = dsp.cpu().pending_instruction();
  return {.cpu = dsp.cpu().registers(),
          .progress = dsp.cpu().progress(),
          .pending = pending ? std::optional{pending->effect} : std::nullopt,
          .fetch = fetch != nullptr ? std::optional{fetch->view()} : std::nullopt,
          .cycles = dsp.clock().native_cycle_units(),
          .ticks = dsp.clock().elapsed_ticks(),
          .reset_ticks = dsp.clock().hardware_reset_ticks(),
          .clk2 = dsp.clock().timer_clk2_edges(),
          .interrupt_pending = dsp.interrupt_pending()};
}

struct CycleFrame {
  Frame chip;
  std::optional<std::uint32_t> issued, native_pc;
  bool captured, started, retired;
  bool operator==(const CycleFrame &) const = default;
};

inline void independent(const OwnDspFetchCycleResult &cycle, const OwnDspMachine &dsp) {
  require(dsp.clock().elapsed_ticks() == dsp.clock().hardware_reset_ticks() +
                                             (dsp.clock().native_cycle_units() * period) &&
              dsp.clock().timer_clk2_edges() == dsp.clock().native_cycle_units() / 2,
          "Every accepted F1/F2/effect period retains independent45tick and CLK2 costs");
  if (!cycle.native || !cycle.native->retired) { return; }
  const auto retired = cycle.native_pc;
  if (!retired) { throw Fault("Actual native retirement address required"); }
  const auto cpu = dsp.cpu().registers();
  if (*retired == boot) {
    require((cpu.x >> word_bits) == 1, "Own MOVE establishes independent X1=1");
  } else if (*retired == boot + 2) {
    require(cpu.a == 0, "Own CLR has independent zero accumulator result");
  } else {
    require(*retired >= boot + 3 && cpu.a == static_cast<std::uint64_t>(*retired - boot - 2) * unit,
            "Every real unrolled ADD has an independently derived full accumulator result");
  }
}

inline CycleFrame observe(const OwnDspFetchCycleResult &cycle,
                          const OwnDspMachine &dsp,
                          const OwnDspFetch *fetch = nullptr) {
  independent(cycle, dsp);
  return {.chip = frame(dsp, fetch),
          .issued = cycle.issued_address,
          .native_pc = cycle.native_pc,
          .captured = cycle.captured,
          .started = cycle.instruction_started,
          .retired = cycle.native && cycle.native->retired};
}

inline CycleFrame advance(OwnDspFetch &fetch, OwnDspMachine &dsp) {
  auto input = pins();
  input.cycle = dsp.clock().native_cycle_units();
  return observe(fetch.advance_dma_automatic(input, timing), dsp, &fetch);
}

inline std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const std::exception &error) { return error.what(); }
  throw Fault("Expected strict native checkpoint transfer refusal");
}

struct Cursor {
  std::shared_ptr<const bool> identity = std::make_shared<const bool>(false);
  std::array<std::uint64_t, 7> ticks = external_ticks;
  unsigned position = 0, queries = 0;
  bool forbidden = false;
  std::vector<std::uint64_t> observed;

  DspRuntimeEvents events() {
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
                          "Only actual advancement may consume the next external event");
                  observed.push_back(time);
                  ++position;
                },
            .order = DspPeerOrder::peer_first,
            .maximum_events_per_period = 4};
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
    prepare(dsp, true);
    schedule = std::make_unique<McuDspRuntime>(inputs(), mcu, dsp, cursor.events());
    attach();
    if (!held) {
      const unsigned minimum_edges = 2 + (initial_phase == 0 ? 0U : 1U);
      require(mcu.advance_system_reset(minimum_edges, McuResetPad::unknown) == minimum_edges,
              "Actual MCU reset edges provide the DSP's minimum reset hold");
      schedule->release_dsp_reset();
    }
  }

  void attach() {
    mcu.set_cycle_observer([this](const Hc12BusCycle cycle) {
      schedule->begin_cycle(cycle, false, pins, [this](const auto &result) { record(result); });
    });
    mcu.set_reset_period_observer([this](const McuResetPeriod period) {
      schedule->begin_reset_period(period, pins, [this](const auto &result) { record(result); });
    });
  }

  void record(const DspRuntimeCycle &cycle) {
    require(cycle.completion_ticks - cycle.start_ticks == period &&
                cycle.completion_ticks == dsp.clock().elapsed_ticks(),
            "Actual joint DSP periods retain their common-time completion");
    trace.push_back(observe(cycle.result, dsp));
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
  OwnDspMachine dsp{dsp_clock};
  std::unique_ptr<McuDspRuntime> schedule;
  std::vector<CycleFrame> trace;
};

inline auto cpu(const CpuState s) {
  return std::tuple{s.pc, s.d, s.x, s.y, s.sp, s.ccr};
}

struct Endpoint {
  std::optional<decltype(cpu(CpuState{}))> mcu;
  Frame dsp;
  McuFlashImage nvm;
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
          .dsp = frame(owner.dsp),
          .nvm = owner.mcu.bus().power_off_flash_image(),
          .reset = owner.mcu.system_reset_state(),
          .dsp_reset = owner.dsp.clock().hardware_reset_state(),
          .dsp_remaining = owner.dsp.clock().hardware_reset_remaining_ticks(),
          .dsp_epoch = owner.dsp.clock().native_epoch(),
          .mcu_ticks = owner.mcu.run_clock().elapsed_ticks(),
          .bus_cycles = owner.mcu.run_clock().bus_cycles(),
          .edges = owner.mcu.run_clock().reset_sysclk_edges(),
          .epoch = owner.mcu.run_clock().native_epoch(),
          .end = owner.schedule->end_ticks()};
}

inline bool equal(const Endpoint &a, const Endpoint &b) {
  return a.mcu == b.mcu && a.dsp == b.dsp && a.nvm.bytes == b.nvm.bytes &&
         a.nvm.known == b.nvm.known && a.nvm.fresh == b.nvm.fresh && a.reset == b.reset &&
         a.dsp_reset == b.dsp_reset && a.dsp_remaining == b.dsp_remaining &&
         a.dsp_epoch == b.dsp_epoch && a.mcu_ticks == b.mcu_ticks && a.bus_cycles == b.bus_cycles &&
         a.edges == b.edges && a.epoch == b.epoch && a.end == b.end;
}
}
