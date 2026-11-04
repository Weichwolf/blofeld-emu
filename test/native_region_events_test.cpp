// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_dma_registers.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned extal = 12000000, timebase = 540000000, boot = 0xff0000;
constexpr std::uint64_t period = 45, reset_hold = 90, reset_recovery = 90;
// Authored FM short JMP, MOVEC #imm,SR, and short immediate R1/R2 MOVE fields.
constexpr unsigned jump = 0x0c0000, move_sr = 0x05f439, status = 0xc00000;
constexpr unsigned r1_zero = 0x310000, r2_zero = 0x320000;
constexpr unsigned r1_five = 0x310500, r2_seven = 0x320700, r2_nine = 0x320900;
constexpr unsigned foreground = 4, timer_vector = 0x54, maximum_irq_steps = 20;
constexpr unsigned idle_span = 64, active_phases = 2,
                   dma_cpu_cycles = (2 * idle_span) + active_phases;
constexpr unsigned priorities = 0xfffffe, timer_priority = 0x100;
constexpr unsigned timer_control = 0xffff8f, timer_compare_flag = 0x200000;
constexpr unsigned timer_initial = 5, timer_enable_compare_irq = 5;
constexpr unsigned dma_source = 0xffffef, dma_destination = 0xffffee;
constexpr unsigned dma_count = 0xffffed, dma_control = 0xffffec, dma_word_copy = 0x880244;
constexpr unsigned source = 0x80, destination = 0x90, first = 0x123456, second = 0x654321;
std::uint64_t assertions = 0;

void require(const bool accepted, const std::string &message) {
  ++assertions;
  if (!accepted) { throw Fault(message); }
}

struct Device {
  OwnDspMachine chip{{.extal_hz = extal,
                      .timebase_hz = timebase,
                      .switch_timing = DspClockSwitch::functional_region_boundary}};
  std::unique_ptr<OwnDspTimeline> timeline;

  explicit Device(const std::span<const unsigned> program) {
    constexpr std::array bootstrap{jump};
    chip.bus().install_rom({.start = boot, .words = bootstrap});
    chip.begin_hardware_reset(
        {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
    require(chip.advance_hardware_reset_ticks(reset_hold) == reset_hold,
            "The actual chip owns its complete reset hold");
    chip.release_hardware_reset();
    require(chip.advance_hardware_reset_ticks(reset_recovery) == reset_recovery,
            "The actual chip owns both real reset recovery periods");
    for (unsigned address = 0; address < program.size(); ++address) {
      chip.bus().write({.address = address, .value = program[address]});
    }
    timeline =
        std::make_unique<OwnDspTimeline>(chip, OwnDspAutomaticTiming::functional_native_regions, 0);
  }

  void step(const unsigned cycles = 1, const unsigned requests = 0) const {
    const auto result =
        timeline->advance_native_region(chip.clock().elapsed_ticks() + (cycles * period),
                                        OwnDspBoundary::through,
                                        {.known_requests = 15,
                                         .requests = requests,
                                         .timing = DspDmaTiming::reference_sampled_cycles,
                                         .grant = DspDmaGrant::available},
                                        256);
    require(result && result->execution.fault.kind == dsp56300::FaultKind::none,
            "The genuine native machine progresses through actual device events");
  }

  void reach(const unsigned pc) const {
    for (unsigned attempt = 0; attempt < 32 && chip.cpu().registers().pc != pc; ++attempt) {
      step();
    }
    require(chip.cpu().registers().pc == pc, "Real authored instructions reach their RAM entry");
  }
};

void test_irq_during_reservation_and_latched_vector() {
  constexpr std::array program{move_sr, status, r1_zero, r2_zero, jump | foreground};
  Device device(program);
  auto &bus = device.chip.bus();
  for (const auto store : std::array{DspStore{.address = timer_vector, .value = r1_five},
                                     DspStore{.address = timer_vector + 1, .value = r2_seven}}) {
    bus.write(store);
  }
  device.reach(foreground);
  require(device.chip.cpu().progress().instructions == 4 &&
              device.chip.cpu().progress().estimated_cycles == 7,
          "Reset-entry JMP3, MOVEC2 and two actual immediate MOVEs establish independent time");
  const auto before = device.chip.cpu().registers();
  for (const auto store : std::array{
           DspStore{.space = DspSpace::x, .address = priorities, .value = timer_priority},
           DspStore{.space = DspSpace::x, .address = timer_control - 1, .value = timer_initial},
           DspStore{.space = DspSpace::x, .address = timer_control - 2, .value = timer_initial},
           DspStore{.space = DspSpace::x,
                    .address = timer_control,
                    .value = timer_enable_compare_irq}}) {
    bus.write(store);
  }
  device.step();
  require(bus.timers().read(timer_control, timer_compare_flag) == timer_compare_flag &&
              device.chip.cpu().progress().instructions == 4 &&
              device.chip.cpu().registers().r[1] == 0,
          "The real CLK2 compare arrives during a genuine unfinished JMP, without early IRQ entry");
  device.step();
  require(device.chip.cpu().progress().instructions == 4,
          "A still unfinished three-cycle jump cannot acknowledge the pending timer");
  device.step();
  require(device.chip.cpu().progress().instructions == 5 &&
              bus.timers().read(timer_control, timer_compare_flag) == timer_compare_flag,
          "The interrupted foreground retires before actual vector admission");
  for (unsigned attempt = 0; attempt < maximum_irq_steps && device.chip.cpu().registers().r[1] != 5;
       ++attempt) {
    device.step();
  }
  require(device.chip.cpu().registers().r[1] == 5 &&
              bus.timers().read(timer_control, timer_compare_flag) == 0,
          "Real vector-pair fetch acknowledges the real flag and executes its first native word");
  bus.write({.address = timer_vector + 1, .value = r2_nine});
  for (unsigned attempt = 0; attempt < maximum_irq_steps && device.chip.cpu().registers().r[2] == 0;
       ++attempt) {
    device.step();
  }
  const auto after = device.chip.cpu().registers();
  require(after.r[2] == 7 && after.pc == foreground && after.sp == before.sp &&
              bus.read({.address = timer_vector + 1}) == r2_nine,
          "Fast return executes the genuinely latched second word despite newer vector RAM");
  std::cout << "ACTUAL_IRQ foreground-retired-first=1 real-vector-ack=1 latched-second=7\n";
}

void test_dma_during_real_native_cpu_time() {
  constexpr std::array<unsigned, 256> program{};
  Device device(program);
  device.reach(0);
  auto &bus = device.chip.bus();
  bus.write({.space = DspSpace::x, .address = source, .value = first});
  for (const auto store :
       std::array{DspStore{.space = DspSpace::x, .address = dma_source, .value = source},
                  DspStore{.space = DspSpace::x, .address = dma_destination, .value = destination},
                  DspStore{.space = DspSpace::x, .address = dma_count, .value = 0},
                  DspStore{.space = DspSpace::x, .address = dma_control, .value = dma_word_copy}}) {
    bus.write(store);
  }
  for (unsigned cycle = 0; cycle < dma::settling_cycles; ++cycle) { device.step(); }
  require(bus.has_active_dma() && bus.native_dma_quiescent(),
          "Actual DMA enable settles while real CPU instructions continue");
  const auto baseline = device.chip.cpu().progress();
  device.step(idle_span);
  require(device.chip.cpu().progress().instructions == baseline.instructions + idle_span,
          "A waiting DMA channel allows an actual 64-instruction native CPU region");
  device.step(1, 1);
  require(!bus.native_dma_quiescent(),
          "The genuine changed request consumes the actual DMA source phase");
  bus.write({.space = DspSpace::x, .address = source, .value = second});
  device.step(1, 1);
  require(bus.read({.space = DspSpace::y, .address = destination}) == first &&
              bus.read({.space = DspSpace::x, .address = source}) == second,
          "The next actual destination phase commits its original captured RAM word");
  bus.write({.space = DspSpace::y, .address = destination, .value = 0});
  device.step(idle_span, 1);
  const auto progress = device.chip.cpu().progress();
  require(bus.read({.space = DspSpace::y, .address = destination}) == 0 &&
              progress.instructions == baseline.instructions + dma_cpu_cycles &&
              progress.estimated_cycles == baseline.estimated_cycles + dma_cpu_cycles &&
              device.chip.clock().native_cycle_units() == progress.estimated_cycles &&
              device.chip.cpu().jit_statistics().helper_instructions == 0,
          "Real DMA phases overlap CPU time and held requests cannot fabricate another word");
  std::cout << "ACTUAL_DMA captured-word=" << hex(first, 6)
            << " native-cpu-cycles=130 no-invented-stall=1\n";
}
}

int main() {
  try {
    test_irq_during_reservation_and_latched_vector();
    test_dma_during_real_native_cpu_time();
    std::cout << "Native region causal events: " << assertions << " assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
