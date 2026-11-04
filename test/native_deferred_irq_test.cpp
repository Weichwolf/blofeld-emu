// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_io.hpp"
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
constexpr std::uint64_t period = 45, hold_ticks = 90, recovery_ticks = 90;
// Authored FM JMP, MOVEC #imm,SR, short immediate R1/R2 MOVE, and WAIT.
constexpr unsigned jump = 0x0c0000, move_sr = 0x05f439, status = 0xc00000;
constexpr unsigned r1_zero = 0x310000, r2_zero = 0x320000;
constexpr unsigned r1_five = 0x310500, r2_seven = 0x320700, wait = 0x000086;
constexpr unsigned foreground = 4, unknown_foreground = 0x300;
constexpr unsigned timer_vector = 0x54, priorities = 0xfffffe, timer_priority = 0x100;
constexpr unsigned timer_control = 0xffff8f, compare_flag = 0x200000;
constexpr unsigned initial_count = 5, timer_enable_compare_irq = 5;
constexpr unsigned wait_base_cycles = 10, waiting_base_cycles = 17, maximum_vector_steps = 32;
constexpr DspRunClockInputs clocks{.extal_hz = extal,
                                   .timebase_hz = timebase,
                                   .switch_timing = DspClockSwitch::functional_region_boundary};
constexpr DspDmaInputs dma_inputs{.known_requests = 15,
                                  .timing = DspDmaTiming::reference_sampled_cycles,
                                  .grant = DspDmaGrant::available};
std::uint64_t assertions = 0;

void require(const bool accepted, const std::string &message) {
  ++assertions;
  if (!accepted) { throw Fault(message); }
}

struct Device {
  OwnDspMachine chip{clocks};
  std::unique_ptr<OwnDspTimeline> timeline;

  explicit Device(const std::span<const unsigned> program) {
    constexpr std::array bootstrap{jump};
    chip.bus().install_rom({.start = boot, .words = bootstrap});
    chip.begin_hardware_reset(
        {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
    require(chip.advance_hardware_reset_ticks(hold_ticks) == hold_ticks,
            "The real chip owns the complete reset hold");
    chip.release_hardware_reset();
    require(chip.advance_hardware_reset_ticks(recovery_ticks) == recovery_ticks,
            "The real chip owns both reset recovery periods");
    for (unsigned address = 0; address < program.size(); ++address) {
      chip.bus().write({.address = address, .value = program[address]});
    }
    chip.bus().write({.address = timer_vector, .value = r1_five});
    chip.bus().write({.address = timer_vector + 1, .value = r2_seven});
    timeline =
        std::make_unique<OwnDspTimeline>(chip, OwnDspAutomaticTiming::functional_native_regions, 0);
  }

  void step(const unsigned cycles = 1) const {
    const auto result = timeline->advance_native_region(
        chip.clock().elapsed_ticks() + (cycles * period), OwnDspBoundary::through, dma_inputs, 64);
    require(result && result->execution.fault.kind == dsp56300::FaultKind::none,
            "The actual native owner progresses without fabricated foreground metadata");
  }

  void prepare_timer() {
    auto &bus = chip.bus();
    bus.write({.space = DspSpace::x, .address = priorities, .value = timer_priority});
    bus.timers().write({.address = timer_control - 1, .value = initial_count});
    bus.timers().write({.address = timer_control - 2, .value = initial_count});
    bus.timers().write({.address = timer_control, .value = timer_enable_compare_irq});
  }

  void reach_foreground() const {
    for (unsigned attempt = 0;
         attempt < maximum_vector_steps && chip.cpu().registers().pc != foreground;
         ++attempt) {
      step();
    }
    require(
        chip.cpu().registers().pc == foreground && chip.cpu().progress().instructions == 4 &&
            chip.cpu().progress().estimated_cycles == 7,
        "Real JMP3, MOVEC2 and two MOVE1 instructions establish the independent starting state");
  }

  void finish_vector() {
    for (unsigned attempt = 0; attempt < maximum_vector_steps && chip.cpu().registers().r[2] != 7;
         ++attempt) {
      step();
    }
    require(chip.cpu().registers().r[1] == 5 && chip.cpu().registers().r[2] == 7 &&
                chip.bus().timers().read(timer_control, compare_flag) == 0,
            "The genuine two-word vector executes and acknowledges its real timer source");
  }
};

void test_real_wait_wakeup() {
  constexpr std::array program{move_sr, status, r1_zero, r2_zero, wait, 0U};
  Device device(program);
  device.reach_foreground();
  device.step(wait_base_cycles);
  const auto before = device.chip.cpu().registers();
  require(device.chip.cpu().processing_state() == dsp56300::ProcessingState::waiting &&
              before.pc == foreground + 1 && device.chip.cpu().progress().instructions == 5 &&
              device.chip.cpu().progress().estimated_cycles == waiting_base_cycles,
          "The genuine ten-cycle WAIT retires and leaves the CPU waiting at the following word");
  device.prepare_timer();
  device.step();
  require(device.chip.cpu().processing_state() == dsp56300::ProcessingState::waiting &&
              device.chip.cpu().progress().instructions == 5 &&
              device.chip.bus().timers().read(timer_control, compare_flag) == compare_flag,
          "One actual idle CLK2 compare raises the timer without inventing a CPU retirement");
  device.finish_vector();
  require(device.chip.cpu().processing_state() == dsp56300::ProcessingState::running &&
              device.chip.cpu().registers().pc == foreground + 1 &&
              device.chip.cpu().registers().sp == before.sp &&
              device.chip.cpu().progress().instructions == 7,
          "Deferred real IRQ admission wakes WAIT and fast-returns without executing foreground");
  device.step();
  constexpr unsigned final_instructions = 8;
  constexpr unsigned final_base_cycles = 20;
  constexpr unsigned actual_clock_cycles = 21;
  require(device.chip.cpu().registers().pc == foreground + 2 &&
              device.chip.cpu().progress().instructions == final_instructions &&
              device.chip.cpu().progress().estimated_cycles == final_base_cycles &&
              device.chip.clock().native_cycle_units() == actual_clock_cycles,
          "The resumed real NOP retains independent instruction costs plus exactly one idle cycle");
  std::cout << "ACTUAL_WAIT deferred-IRQ=1 base-cycles=20 clock-cycles=21 resumed-NOP=1\n";
}

void test_unknown_foreground_is_not_fetched_during_irq() {
  constexpr std::array program{move_sr, status, r1_zero, r2_zero, jump | unknown_foreground};
  Device device(program);
  device.reach_foreground();
  device.prepare_timer();
  for (unsigned cycle = 0; cycle < 3; ++cycle) { device.step(); }
  require(device.chip.cpu().registers().pc == unknown_foreground &&
              !device.chip.bus().inspect_known_program(unknown_foreground) &&
              device.chip.cpu().progress().instructions == 5,
          "The real interrupted JMP reaches unknown RAM while a genuine timer source is pending");
  device.finish_vector();
  require(device.chip.cpu().registers().pc == unknown_foreground &&
              device.chip.cpu().progress().instructions == 7 && !device.chip.io().failure(),
          "Both real vector words execute before any nonexistent foreground byte is demanded");
  const auto ticks = device.chip.clock().elapsed_ticks();
  const auto result = device.timeline->advance_native_region(
      ticks + period, OwnDspBoundary::through, dma_inputs, 64);
  require(result && result->execution.fault.kind == dsp56300::FaultKind::memory_read &&
              result->execution.fault.space == dsp56300::Space::program &&
              result->execution.fault.pc == unknown_foreground &&
              result->execution.fault.address == unknown_foreground &&
              device.chip.clock().elapsed_ticks() == ticks &&
              device.chip.cpu().progress().instructions == 7 && device.timeline->faulted(),
          "Only actual post-vector fetch refuses unknown P with exact address/PC and no fake NOP");
  const auto &failure = device.chip.io().failure();
  require(failure && failure->cause && failure->pc == unknown_foreground &&
              failure->address == unknown_foreground,
          "The precise underlying unknown-memory cause survives the deferred interrupt boundary");
  std::cout << "ACTUAL_UNKNOWN_FOREGROUND vector-first=1 real-refusal-after-return=1\n";
}
}

int main() {
  try {
    test_real_wait_wakeup();
    test_unknown_foreground_is_not_fetched_during_irq();
    std::cout << "Native deferred IRQ admission: " << assertions << " assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
