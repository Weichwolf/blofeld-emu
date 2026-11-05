// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_cop.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <tuple>

namespace {
using namespace blofeld;
constexpr unsigned oscillator_hz = 12000000, common_hz = 540000000;
constexpr unsigned initial_common = 450, recovery_end = 540, dsp_period = 45, common_factor = 9;
constexpr unsigned boot = 0xff0000;
constexpr unsigned entry = 0x8000, cop_handler = 0x8100, ram = 0x1200, marker = 0x5a;
constexpr unsigned byte_mask = 0xff, nop = 0xa7, protection = 0x7f0d, security = 0x7f0f;
constexpr unsigned cop_edges = 16384, cop_ticks = 81920, reset_edges = 195;
constexpr unsigned unsecured = 0xfe, branch_self = 0xfe, vector_ticks = 50, wait_ticks = 140;
constexpr unsigned branch_back = 0x050fdf;
constexpr std::uint64_t accumulator_unit = std::uint64_t{1} << 24U;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected native MCU/DSP COP/reset rejection");
}

McuFlashImage image(const bool waiting) {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  if (waiting) {
    // Own LDS #2000;WAI. Literal2+7entry periods;all nine frame bytes fit SRAM.
    constexpr std::array<std::uint8_t, 4> code{0xcf, 0x20, 0, 0x3e};
    for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code[i]; }
  } else {
    // Own BRA -2,exactly three unstretched periods per completed native branch.
    bytes[0] = 0x20;
    bytes[1] = branch_self;
  }
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  bytes[bytes.size() - 6] = cop_handler >> 8U;
  bytes[bytes.size() - 5] = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_inputs(const unsigned phase) {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .initial_oscillator_phase_ticks = phase,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspRuntimeInputs inputs(const bool reset_first) {
  return {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          .maximum_cycles_per_period = 16,
          .wait_dsp_first = true,
          .reset_dsp_first = reset_first,
          .initial_reset = OwnDspResetInputs{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
}

DspDmaInputs pins() {
  return {.known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

void initialize(OwnDspMachine &dsp, const bool failing) {
  // Independently encoded MOVE #>1,X1;CLR A;ADD X1,A;BRA *-1,or a reserved Y store.
  constexpr unsigned move_x1 = 0x45f400;
  constexpr unsigned clear_a = 0x200013;
  constexpr unsigned add_x1 = 0x200060;
  constexpr unsigned move_r0 = 0x60f400;
  constexpr unsigned move_x0 = 0x44f400;
  constexpr unsigned store_y = 0x4c6000;
  constexpr unsigned reserved = 0x2000;
  constexpr unsigned padding = 64;
  std::array<std::uint32_t, padding> program{};
  if (failing) {
    program[0] = move_r0;
    program[1] = reserved;
    program[2] = move_x0;
    program[3] = 7;
    program[4] = store_y;
  } else {
    program[0] = move_x1;
    program[1] = 1;
    program[2] = clear_a;
    program[3] = add_x1;
    program[4] = branch_back;
  }
  dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = program});
}

struct Fixture {
  explicit Fixture(const bool waiting = false,
                   const unsigned phase = 0,
                   const bool reset_first = true,
                   const bool fault = false)
      : mcu(image(waiting), BLOFELD_DEFAULT_SLA, mcu_inputs(phase)), failing(fault) {
    initialize(dsp, fault);
    runtime = std::make_unique<McuDspRuntime>(inputs(reset_first), mcu, dsp);
    runtime->release_dsp_reset();
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    attach();
    if (waiting) {
      step();
      step();
      require(mcu.cpu().waiting() && mcu.run_clock().bus_cycles() == 14 &&
                  mcu.run_clock().elapsed_ticks() == wait_ticks,
              "Native LDS/WAI creates its actual frame before any idle COP interval");
    }
  }

  void attach() {
    mcu.set_cycle_observer([this](const auto period) {
      runtime->begin_cycle(period, false, pins, [this](const auto &result) { record(result); });
    });
    mcu.set_wait_period_observer([this](const auto period) {
      runtime->begin_wait_period(period, pins, [this](const auto &result) { record(result); });
    });
    mcu.set_reset_period_observer([this](const auto period) {
      runtime->begin_reset_period(period, pins, [this](const auto &result) { record(result); });
    });
  }

  void record(const DspRuntimeCycle &cycle) {
    const auto &result = cycle.result;
    ++cycles;
    require(cycle.cycle + 1 == cycles && cycle.completion_ticks - cycle.start_ticks == dsp_period &&
                cycle.completion_ticks == recovery_end + (cycles * dsp_period),
            "Each actual native DSP period retains its literal hold/recovery and45tick clock");

    if (failing || !result.native || !result.native->retired) { return; }
    require(result.native_pc && *result.native_pc == expected_pc,
            "Native retirements retain the independent MOVE/CLR/ADD/BRA sequence");
    if (expected_pc == boot) {
      expected_pc = boot + 2;
    } else if (expected_pc == boot + 2) {
      require(dsp.cpu().registers().a == 0, "Native CLR defines the zero accumulator");
      expected_pc = boot + 3;
    } else if (expected_pc == boot + 3) {
      ++adds;
      require(dsp.cpu().registers().a == adds * accumulator_unit,
              "Every native ADD changes the full accumulator independently of MCU reset");
      expected_pc = boot + 4;
    } else {
      expected_pc = boot + 3;
    }
  }

  void step() {
    static_cast<void>(mcu.cpu().step());
    runtime->end_instruction();
  }

  void time() const {
    const auto common = mcu.run_clock().elapsed_ticks() * common_factor;
    require(runtime->end_ticks() == common && cycles == (common - recovery_end) / dsp_period &&
                runtime->completed_cycles() == cycles &&
                dsp.clock().elapsed_ticks() == recovery_end + (cycles * dsp_period) &&
                dsp.clock().hardware_reset_ticks() == recovery_end &&
                dsp.clock().timer_clk2_edges() == cycles / 2,
            "Actual MCU,DSP,held/recovery and CLK2 ledgers equal the independent integer horizon");
  }

  void begin(const McuResetSource source, const unsigned sync = 3) {
    runtime->begin_system_reset(source, sync);
    attach(); // Native MCU reset replaces its owner and deliberately drops observers.
  }

  McuMachine mcu;
  OwnDspMachine dsp{{.extal_hz = oscillator_hz,
                     .timebase_hz = common_hz,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary}};
  std::unique_ptr<McuDspRuntime> runtime;
  std::uint64_t cycles = 0, adds = 0;
  unsigned expected_pc = boot;
  bool failing;
};

struct ResetScenario {
  unsigned sync, phase;
};

void external_reset(const ResetScenario scenario) {
  const unsigned sync = scenario.sync;
  const unsigned phase = scenario.phase;
  Fixture f(false, phase);
  f.step();
  f.mcu.bus().write(ram, marker);
  const auto start = f.mcu.run_clock().elapsed_ticks();
  const auto before = f.dsp.cpu().registers();
  const auto adds = f.adds;
  f.begin(McuResetSource::external, sync);
  require(f.dsp.cpu().registers() == before, "An explicit MCU reset does not reset the DSP");
  const unsigned edges = 128 + sync + 64;
  require(f.mcu.advance_system_reset(edges, McuResetPad::high) == edges,
          "All drive/synchronization/settling edges precede the real native reset vector");
  f.runtime->end_system_reset();
  const auto end = start + (edges * 5ULL) - phase + vector_ticks;
  require(f.mcu.run_clock().elapsed_ticks() == end && f.mcu.run_clock().native_epoch() == 2 &&
              f.mcu.run_clock().reset_sysclk_edges() == edges && f.mcu.cpu().state().pc == entry &&
              f.mcu.cpu().state().ccr == hc12::reset_ccr && f.mcu.bus().read(ram) == marker &&
              f.adds > adds && !f.runtime->faulted(),
          "Native external reset preserves SRAM and DSP work,with literal OSC/vector durations");
  f.time();
}

void cop_timeout(const bool waiting, const bool phy, const unsigned phase) {
  Fixture f(waiting, phase);
  if (phy) {
    constexpr unsigned enable_phy = 0x40;
    constexpr unsigned select_phy = 0x80;
    f.mcu.bus().write(hc12::usb_control, enable_phy);
    f.mcu.bus().observe_phy({.edges = 15});
    f.mcu.bus().write(hc12::clock_select, select_phy);
  }
  f.mcu.bus().write(hc12::cop_control, 1);
  const auto start = f.mcu.run_clock().elapsed_ticks();
  const auto bus = f.mcu.run_clock().bus_cycles();
  const auto idle = f.mcu.idle_bus_cycles();
  const auto duration = cop_ticks - phase;
  const auto period = phy ? 2U : 10U;
  const auto completed = (duration - 1) / period;
  const auto reason = rejects([&] {
    constexpr unsigned bound = cop_edges * 3;
    for (unsigned i = 0; i < bound; ++i) {
      if (waiting) {
        static_cast<void>(f.mcu.advance_wait(1));
        f.runtime->end_wait();
      } else {
        f.step();
      }
    }
  });
  require(reason.contains("COP system reset (timeout)") && f.runtime->faulted() &&
              !f.dsp.clock().faulted() && f.mcu.bus().cop_state().reset == McuCopReset::timeout &&
              f.mcu.bus().cop_state().phase == cop_edges &&
              f.mcu.run_clock().elapsed_ticks() == start + duration &&
              f.mcu.run_clock().bus_cycles() == bus + completed &&
              f.mcu.idle_bus_cycles() == idle + (waiting ? completed : 0),
          "Natural COP timeout preempts the real bus/idle slot at its exact fractional edge");
  f.time();
  const auto adds = f.adds;
  const auto registers = f.dsp.cpu().registers();
  const auto progress = f.dsp.cpu().progress();
  const auto epoch = f.dsp.clock().native_epoch();
  require(rejects([&] { f.runtime->end_instruction(); }).contains("healthy inactive"),
          "Latched MCU timing failure cannot complete a healthy instruction boundary");
  f.begin(McuResetSource::cop);
  require(!f.runtime->faulted() && f.dsp.cpu().registers() == registers &&
              f.dsp.cpu().progress() == progress && f.dsp.clock().native_epoch() == epoch,
          "Admitting the actual COP reset changed DSP computation or retained a stale fault");
  constexpr unsigned cut_edges = 131;
  require(f.mcu.advance_system_reset(cut_edges, McuResetPad::high) == cut_edges,
          "Actual COP reset reaches its drive/synchronization boundary on real SYSCLK edges");

  require(f.mcu.advance_system_reset(reset_edges - cut_edges, McuResetPad::high) ==
              reset_edges - cut_edges,
          "The latched timeout must finish its actual native COP reset sequence");
  f.runtime->end_system_reset();
  require(f.mcu.cpu().state().pc == cop_handler && f.mcu.run_clock().native_epoch() == 2 &&
              f.mcu.run_clock().elapsed_ticks() ==
                  start + duration + reset_edges * 5ULL + vector_ticks &&
              f.adds > adds && !f.runtime->faulted() && f.dsp.clock().native_epoch() == epoch,
          "Actual FFFA COP vector differs from FFFE and preserves continuous native DSP work");
  f.time();
}

void unknown_pad() {
  Fixture f;
  f.begin(McuResetSource::external);
  f.mcu.advance_system_reset(reset_edges - 1, McuResetPad::unknown);
  const auto time = f.mcu.run_clock().elapsed_ticks();
  const auto cycles = f.cycles;
  require(
      rejects([&] { f.mcu.advance_system_reset(1, McuResetPad::unknown); }).contains("RESET pad") &&
          f.runtime->faulted() && f.cycles == cycles && f.mcu.run_clock().elapsed_ticks() == time,
      "Unknown source sampling cannot invent a reset edge or consume another DSP period");
}

void malformed_offer(const bool wrong_epoch) {
  Fixture f;
  f.begin(McuResetSource::external);
  const auto state = f.dsp.cpu().registers();
  const McuResetPeriod bad{.context = {},
                           .duration_ticks = wrong_epoch ? 5U : 4U,
                           .native_epoch = wrong_epoch ? 1U : 2U};
  require(rejects([&] {
            f.runtime->begin_reset_period(bad, pins);
          }).contains("reset offer must match its real SYSCLK edge") &&
              f.runtime->faulted() && f.cycles == 0 && f.dsp.cpu().registers() == state,
          "Wrong reset duration/epoch refuses before native DSP or MCU work");
}

void default_cop_refusal() {
  Fixture f(false, 0, false);
  f.mcu.bus().write(hc12::cop_control, 1);
  const auto start = f.mcu.run_clock().elapsed_ticks();
  const auto reason = rejects([&] {
    for (unsigned i = 0; i < cop_edges; ++i) { f.step(); }
  });
  constexpr unsigned before_timeout = cop_ticks - 10;
  require(reason.contains("Unqualified MCU/DSP COP reset deadline") && f.runtime->faulted() &&
              f.mcu.bus().cop_state().reset == McuCopReset::none &&
              f.mcu.bus().cop_state().phase == cop_edges - 2 &&
              f.mcu.run_clock().elapsed_ticks() == start + before_timeout,
          "Unqualified COP timing refuses before the timeout slot and cannot invent reset success");
  f.time();
}

void peer_failure() {
  Fixture f(false, 0, true, true);
  f.begin(McuResetSource::external);
  const auto reason =
      rejects([&] { f.mcu.advance_system_reset(reset_edges, McuResetPad::unknown); });
  constexpr unsigned completed = 9;
  constexpr unsigned hold_recovery_edges = 2;
  require(reason.contains("MCU system reset pc=") && reason.contains("pc=0xff0004") &&
              reason.contains("Y:0x002000") && reason.contains("attempted-ticks=990") &&
              f.runtime->faulted() && f.dsp.clock().faulted() && f.cycles == completed &&
              f.mcu.run_clock().reset_sysclk_edges() == completed + hold_recovery_edges &&
              f.mcu.run_clock().elapsed_ticks() ==
                  vector_ticks + ((completed + hold_recovery_edges) * 5ULL) &&
              f.dsp.clock().elapsed_ticks() == recovery_end + (completed * dsp_period) &&
              f.runtime->end_ticks() == recovery_end + ((completed + 1) * dsp_period),
          "Real reserved-Y failure during reset preserves consumed DSP/MCU prefixes and context: " +
              reason);
  require(rejects([&] { f.runtime->end_system_reset(); }).contains("healthy inactive"),
          "Failed reset execution cannot complete its pending boundary");
}

void unconsumed_reset_offer() {
  Fixture f;
  f.begin(McuResetSource::external);
  const McuResetPeriod period{.context = {}, .duration_ticks = 5, .native_epoch = 2};
  f.runtime->begin_reset_period(period, pins);
  const auto reason = rejects([&] { f.mcu.advance_system_reset(1, McuResetPad::unknown); });
  require(reason.contains("lost a consumed native period") && f.runtime->faulted() &&
              f.runtime->completed_cycles() == 0 &&
              f.dsp.clock().elapsed_ticks() == initial_common + dsp_period &&
              f.dsp.clock().hardware_reset_remaining_ticks() == dsp_period &&
              f.mcu.run_clock().reset_sysclk_edges() == 0 &&
              f.mcu.run_clock().elapsed_ticks() == vector_ticks,
          "A missing consumed reset edge cannot silently clock a second DSP period:reason=" +
              reason + " dsp-cycles=" + std::to_string(f.runtime->completed_cycles()) +
              " reset-edges=" + std::to_string(f.mcu.run_clock().reset_sysclk_edges()) +
              " mcu-ticks=" + std::to_string(f.mcu.run_clock().elapsed_ticks()));
}

bool same_cpu(const CpuState &left, const CpuState &right) {
  return std::tie(left.pc, left.d, left.x, left.y, left.sp, left.ccr) ==
         std::tie(right.pc, right.d, right.x, right.y, right.sp, right.ccr);
}

void default_reset_refusal() {
  Fixture f(false, 0, false);
  const auto mcu = f.mcu.cpu().state();
  const auto dsp = f.dsp.cpu().registers();
  require(
      rejects([&] { f.begin(McuResetSource::external); }).contains("consumed native reset cause") &&
          same_cpu(f.mcu.cpu().state(), mcu) && f.dsp.cpu().registers() == dsp,
      "Default unqualified reset contract cannot silently restart either processor");
}
}

int main(const int count, const char *const *const arguments) {
  try {
    require(count == 1 || count == 2, "Expected an optional isolated failure-case selector");
    if (count == 2) {
      const std::string selector(arguments[1]);
      if (selector == "unconsumed") {
        unconsumed_reset_offer();
      } else if (selector == "external") {
        external_reset({.sync = 3, .phase = 0});
      } else if (selector == "cop") {
        cop_timeout(false, false, 0);
      } else if (selector == "wai-cop") {
        cop_timeout(true, false, 0);
      } else if (selector == "failure") {
        peer_failure();
      } else {
        throw Fault("Unknown isolated reset-test selector");
      }
      std::cout << "Native reset control passed: " << selector << '\n';
      return 0;
    }
    unsigned cases = 0;
    for (unsigned sync = 3; sync <= 6; ++sync) {
      external_reset({.sync = sync, .phase = sync - 3});
      ++cases;
    }
    for (unsigned phase = 0; phase < 5; ++phase) {
      for (const bool waiting : {false, true}) {
        for (const bool phy : {false, true}) {
          cop_timeout(waiting, phy, phase);
          ++cases;
        }
      }
    }
    for (const auto test : {unknown_pad,
                            default_reset_refusal,
                            default_cop_refusal,
                            peer_failure,
                            unconsumed_reset_offer}) {
      test();
      ++cases;
    }
    malformed_offer(false);
    ++cases;
    malformed_offer(true);
    ++cases;
    constexpr unsigned expected_cases = 31;
    require(cases == expected_cases, "Independent complete COP/reset case census");
    std::cout << "Native MCU/dsp56300 COP/reset cases=" << cases
              << " whole-COP=20 external-resets=4 refusals=7\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
