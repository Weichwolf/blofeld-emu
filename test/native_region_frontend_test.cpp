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
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace {
constexpr unsigned extal = 12000000;
constexpr std::uint64_t timebase = 540000000;
constexpr std::uint64_t period = 45;
constexpr std::uint64_t reset_hold = 90;
constexpr std::uint64_t reset_recovery = 90;
constexpr unsigned boot = 0xff0000;
// Authored FM encodings: short absolute JMP and absolute X:EA MOVE to/from X0.
constexpr unsigned jump = 0x0c0000;
constexpr unsigned load_x0 = 0x44f000;
constexpr unsigned store_x0 = 0x447000;
constexpr unsigned data_address = 0x80;
constexpr unsigned bcr = 0xfffffb;
constexpr unsigned bcr_value = 0x01ffff;
constexpr unsigned port_f_control = 0xfffff7;
constexpr unsigned nop_count = 32;
constexpr unsigned store_pc = nop_count + 2;
constexpr unsigned idle_pc = nop_count + 4;
constexpr blofeld::DspRunClockInputs clocks{
    .extal_hz = extal,
    .timebase_hz = timebase,
    .switch_timing = blofeld::DspClockSwitch::functional_region_boundary};
constexpr blofeld::OwnDspResetInputs reset_inputs{
    .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2};
std::uint64_t assertions = 0;

void require(const bool accepted, const std::string &message) {
  ++assertions;
  if (!accepted) { throw blofeld::Fault(message); }
}

struct Device;
[[nodiscard]] std::optional<blofeld::OwnDspRegionResult>
advance(Device &device, unsigned cycles = 1, blofeld::DspPeripheralObserver observer = {});

struct Device {
  blofeld::OwnDspMachine chip{clocks};
  std::unique_ptr<blofeld::OwnDspTimeline> timeline;

  explicit Device(const std::span<const unsigned> program) {
    constexpr std::array bootstrap{jump};
    chip.bus().install_rom({.start = boot, .words = bootstrap});
    chip.begin_hardware_reset(reset_inputs);
    require(chip.advance_hardware_reset_ticks(reset_hold) == reset_hold,
            "The actual chip owns the complete reset hold");
    chip.release_hardware_reset();
    require(chip.advance_hardware_reset_ticks(reset_recovery) == reset_recovery,
            "The actual chip owns both recovery periods");
    require(chip.cpu().registers().pc == boot && chip.cpu().progress().instructions == 0,
            "The authored component starts at the real mode5 reset entry");
    for (unsigned address = 0; address < program.size(); ++address) {
      chip.bus().write(
          {.space = blofeld::DspSpace::p, .address = address, .value = program[address]});
    }
    timeline = std::make_unique<blofeld::OwnDspTimeline>(
        chip, blofeld::OwnDspAutomaticTiming::functional_native_regions, 0);
  }

  void reach_ram_entry() {
    for (unsigned attempt = 0; attempt < 8 && chip.cpu().registers().pc != 0; ++attempt) {
      const auto result = advance(*this);
      require(result && result->execution.fault.kind == dsp56300::FaultKind::none,
              "The actual reset-entry JMP progresses through finite clock horizons");
    }
    require(chip.cpu().registers().pc == 0 && chip.cpu().progress().instructions == 1,
            "The real bootstrap instruction, not host PC initialization, reaches authored RAM");
  }
};

std::optional<blofeld::OwnDspRegionResult>
advance(Device &device, const unsigned cycles, const blofeld::DspPeripheralObserver observer) {
  return device.timeline->advance_native_region(
      device.chip.clock().elapsed_ticks() + (cycles * period),
      blofeld::OwnDspBoundary::through,
      {.known_requests = 15,
       .timing = blofeld::DspDmaTiming::reference_sampled_cycles,
       .grant = blofeld::DspDmaGrant::available},
      64,
      observer);
}

void test_real_native_prefix_and_peripheral_completion() {
  std::vector<unsigned> program(nop_count, 0);
  program.insert(program.end(), {load_x0, data_address, store_x0, bcr, jump | idle_pc});
  Device device(program);
  device.chip.bus().write(
      {.space = blofeld::DspSpace::x, .address = data_address, .value = bcr_value});

  struct Observation {
    Device &device;
    unsigned preparing = 0;
    unsigned completed = 0;
    std::uint64_t prepared_ticks = 0;
  } observed{.device = device};

  const blofeld::DspPeripheralObserver observer{
      .context = &observed,
      .preparing =
          [](void *pointer, const unsigned pc) {
            auto &event = *static_cast<Observation *>(pointer);
            require(pc == store_pc, "The peripheral preparation uses the actual store PC");
            event.prepared_ticks = event.device.chip.clock().elapsed_ticks();
            ++event.preparing;
          },
      .completed =
          [](void *pointer, const unsigned pc) {
            auto &event = *static_cast<Observation *>(pointer);
            require(pc == store_pc && event.preparing == event.completed + 1 &&
                        event.prepared_ticks == event.device.chip.clock().elapsed_ticks(),
                    "The physical store completes once at its reserved clock boundary");
            ++event.completed;
          }};
  for (unsigned attempt = 0; attempt < 128 && observed.completed == 0; ++attempt) {
    const auto result = advance(device, 4, observer);
    require(result && result->execution.fault.kind == dsp56300::FaultKind::none,
            "Actual native regions progress without synthetic success or cycle replay");
  }
  constexpr unsigned expected_instructions = 35;
  constexpr std::uint64_t expected_cycles =
      39; // JMP3 + NOP32 + load2 + store2; DMA sampling overlaps.
  const auto cpu = device.chip.cpu().registers();
  const auto progress = device.chip.cpu().progress();
  const auto jit = device.chip.cpu().jit_statistics();
  require(observed.preparing == 1 && observed.completed == 1 && cpu.pc == idle_pc &&
              (cpu.x & dsp56300::word_mask) == bcr_value &&
              device.chip.bus().read({.space = blofeld::DspSpace::y, .address = port_f_control}) ==
                  0,
          "The real RAM load and BCR store change the actual BIU gate");
  require(progress.instructions == expected_instructions &&
              progress.estimated_cycles == expected_cycles &&
              device.chip.clock().native_cycle_units() == expected_cycles &&
              device.chip.clock().elapsed_ticks() ==
                  reset_hold + reset_recovery + (expected_cycles * period),
          "DMA sampling consumes the real instruction periods without an invented startup wait");
  require(device.chip.cpu().backend() == dsp56300::Backend::jit &&
              jit.native_alu_instructions >= nop_count && jit.helper_instructions == 0,
          "Default Core execution really uses generated native prefixes, including small slices");
  std::cout << "ACTUAL_MACHINE native=" << jit.native_alu_instructions
            << " jit-helpers=" << jit.helper_instructions << " retired=" << progress.instructions
            << " cycles=" << progress.estimated_cycles
            << " peripheral-effects=" << observed.completed << '\n';
}

void test_captured_code_mutation_and_checkpoint() {
  constexpr unsigned old_target = 32;
  constexpr unsigned new_target = 64;
  constexpr std::array program{jump | old_target};
  Device original(program);
  original.reach_ram_entry();
  const auto initial = original.chip.cpu().progress().instructions;
  require(advance(original).has_value() && original.chip.cpu().progress().instructions == initial,
          "One actual cycle reserves, but cannot retire, the three-cycle authored jump");
  const auto saved = original.timeline->checkpoint();
  original.chip.bus().write(
      {.space = blofeld::DspSpace::p, .address = 0, .value = jump | new_target});
  require(advance(original).has_value() && advance(original).has_value(),
          "The original owner consumes the remaining real reservation periods");
  require(original.chip.cpu().registers().pc == old_target &&
              original.chip.cpu().progress().instructions == 2 &&
              original.chip.bus().read({.space = blofeld::DspSpace::p, .address = 0}) ==
                  (jump | new_target),
          "Captured old code executes once without patching the newer RAM word");

  blofeld::OwnDspMachine clone(clocks);
  blofeld::OwnDspTimeline restored(clone, saved);
  clone.bus().write({.space = blofeld::DspSpace::p, .address = 0, .value = jump | new_target});
  for (unsigned cycle = 0; cycle < 2; ++cycle) {
    const auto result =
        restored.advance_native_region(clone.clock().elapsed_ticks() + period,
                                       blofeld::OwnDspBoundary::through,
                                       {.known_requests = 15,
                                        .timing = blofeld::DspDmaTiming::reference_sampled_cycles,
                                        .grant = blofeld::DspDmaGrant::available},
                                       64);
    require(result && result->execution.fault.kind == dsp56300::FaultKind::none,
            "A paired restore resumes the genuine captured reservation");
  }
  require(clone.cpu().registers().pc == old_target && clone.cpu().progress().instructions == 2 &&
              clone.cpu().registers() == original.chip.cpu().registers() &&
              clone.clock().elapsed_ticks() == original.chip.clock().elapsed_ticks(),
          "Both owners reach the independently expected original jump target and time");
}

void test_reset_cancels_inflight_work() {
  constexpr unsigned old_target = 32;
  constexpr unsigned fresh_target = 64;
  constexpr std::array program{jump | old_target};
  Device device(program);
  device.reach_ram_entry();
  require(advance(device).has_value(), "A real unfinished instruction is reserved before reset");
  device.timeline->begin_hardware_reset(reset_inputs);
  require(device.chip.cpu().registers().pc == boot &&
              device.chip.cpu().progress().instructions == 0,
          "Hardware reset cancels the original instruction and restores the native reset entry");
  require(device.timeline->advance_reset_to(device.chip.clock().elapsed_ticks() + reset_hold) ==
              reset_hold,
          "The same timeline owns the new reset hold");
  device.timeline->release_hardware_reset();
  require(device.timeline->advance_reset_to(device.chip.clock().elapsed_ticks() + reset_recovery) ==
              reset_recovery,
          "The same timeline owns reset recovery");
  device.chip.bus().write(
      {.space = blofeld::DspSpace::p, .address = 0, .value = jump | fresh_target});
  for (unsigned attempt = 0; attempt < 16 && device.chip.cpu().registers().pc != fresh_target;
       ++attempt) {
    const auto result = advance(device);
    require(result && result->execution.fault.kind == dsp56300::FaultKind::none,
            "Restart executes actual fresh code, never the cancelled old reservation");
  }
  require(device.chip.cpu().registers().pc == fresh_target &&
              device.chip.cpu().progress().instructions == 2,
          "Fresh reset-entry and RAM instructions retire with no resurrected old branch");
}

void test_cache_enabled_empty_profile_keeps_native_reservation() {
  // FM MOVEC #>$080300,SR enables CE while masking ordinary interrupts.
  constexpr unsigned movec_sr = 0x05f439;
  constexpr unsigned cache_enable = 1U << 19U;
  constexpr unsigned masked_status = cache_enable | 0x300U;
  constexpr unsigned target = 5;
  constexpr std::array program{movec_sr, masked_status, jump | target, 0U, 0U, 0U};
  Device device(program);
  device.reach_ram_entry();
  const auto setup = advance(device, 2);
  require(setup && device.chip.cpu().registers().pc == 2 &&
              (device.chip.cpu().registers().sr & cache_enable) != 0,
          "Actual MOVEC enables CE without inventing a cacheable program interval");
  const auto before = device.chip.cpu().progress();
  const auto native = device.chip.cpu().jit_statistics().native_alu_instructions;
  const auto started = device.chip.clock().elapsed_ticks();
  const auto first = advance(device);
  require(first && device.chip.cpu().progress().instructions == before.instructions &&
              device.chip.cpu().registers().pc == 2,
          "A real three-cycle JMP remains unfinished after its first native clock");
  const auto completed = advance(device, 2);
  require(completed && device.chip.cpu().registers().pc == target &&
              device.chip.cpu().progress().instructions == before.instructions + 1 &&
              device.chip.cpu().progress().estimated_cycles == before.estimated_cycles + 3 &&
              device.chip.clock().elapsed_ticks() == started + (3 * period) &&
              (device.chip.cpu().registers().sr & cache_enable) != 0,
          "The captured branch retires once after exactly three clocks with CE unchanged");
  require(device.chip.cpu().jit_statistics().native_alu_instructions == native + 1,
          "An empty-cache profile executes the actually reserved JMP natively with CE set");
}

void test_native_repeat_horizons_retain_actual_body() {
  // FM MOVEC #>value,LC and REP #3: setup costs2 and5, NOP bodies cost1.
  constexpr unsigned seed_lc = 0x05f43f;
  constexpr unsigned saved_lc = 0x357;
  constexpr unsigned repeat_three = 0x0603a0;
  constexpr unsigned body = 3;
  constexpr unsigned mutated_target = 64;
  constexpr std::array program{seed_lc, saved_lc, repeat_three, 0U, 0U};
  Device device(program);
  device.reach_ram_entry();
  const auto seeded = advance(device, 2);
  require(seeded && device.chip.cpu().registers().lc == saved_lc &&
              device.chip.cpu().registers().pc == 2,
          "Actual MOVEC seeds a known LC before entering the real REP setup");
  const auto before = device.chip.cpu().progress();
  const auto started = device.chip.clock().elapsed_ticks();
  require(advance(device, 2).has_value() &&
              device.chip.cpu().progress().instructions == before.instructions,
          "REP cannot retire before all five authentic setup periods are clocked");
  const auto setup = advance(device, 3);
  require(setup && device.chip.cpu().registers().pc == body &&
              device.chip.cpu().registers().lc == 3 &&
              device.chip.cpu().progress().instructions == before.instructions + 1 &&
              device.chip.cpu().progress().estimated_cycles == before.estimated_cycles + 5 &&
              device.chip.clock().elapsed_ticks() == started + (5 * period),
          "Split2+3 horizons retain the real five-cycle REP setup and LC/PC state");
  const auto native = device.chip.cpu().jit_statistics().native_alu_instructions;
  const auto first = advance(device);
  require(first && device.chip.cpu().registers().pc == body &&
              device.chip.cpu().registers().lc == 2,
          "The first actual repeated body remains at its retained PC with two iterations left");
  device.chip.bus().write(
      {.space = blofeld::DspSpace::p, .address = body, .value = jump | mutated_target});
  const auto last = advance(device, 2);
  require(last && device.chip.cpu().registers().pc == body + 1 &&
              device.chip.cpu().registers().lc == saved_lc &&
              device.chip.cpu().progress().instructions == before.instructions + 4 &&
              device.chip.cpu().progress().estimated_cycles == before.estimated_cycles + 8 &&
              device.chip.clock().elapsed_ticks() == started + (8 * period),
          "Two more original NOPs finish REP, restore saved LC and ignore the changed JMP body");
  require(device.chip.cpu().jit_statistics().native_alu_instructions == native + 3,
          "All three retained REP bodies execute natively across finite clock horizons");
}

void test_unknown_program_keeps_actual_fault() {
  Device device(std::span<const unsigned>{});
  device.reach_ram_entry();
  const auto result = advance(device, 4);
  require(result && result->execution.fault.kind == dsp56300::FaultKind::memory_read &&
              result->execution.fault.space == dsp56300::Space::program &&
              result->execution.fault.pc == 0 && result->execution.fault.address == 0 &&
              device.timeline->faulted() && device.chip.cpu().progress().instructions == 1,
          "Unknown lookahead performs a real owned P read and preserves the exact guest fault");
  const auto &failure = device.chip.io().failure();
  require(failure && failure->cause && failure->address == 0 && failure->pc == 0,
          "The actual bus cause survives the Core/clock owner boundary");
}
}

int main() {
  try {
    test_real_native_prefix_and_peripheral_completion();
    test_captured_code_mutation_and_checkpoint();
    test_reset_cancels_inflight_work();
    test_unknown_program_keeps_actual_fault();
    test_cache_enabled_empty_profile_keeps_native_reservation();
    test_native_repeat_horizons_retain_actual_body();
    std::cout << "Native region machine ownership: " << assertions << " assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
