// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_adapter_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_execution.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_timed_execution.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_shi.hpp"
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
constexpr std::uint64_t timebase = 540000000, period = 45, reset_ticks = 180;
constexpr unsigned boot = 0xff0000, jump = 0x0c0000;
constexpr unsigned bcr = 0xfffffb, bcr_value = 0x01ffff;
constexpr unsigned source = 0x80, sample = 0x123456, changed_sample = 0x654321;
constexpr unsigned gpio_store = 8, movep_b1 = 0x04cddd;
constexpr blofeld::DspRunClockInputs clocks{
    .extal_hz = extal,
    .timebase_hz = timebase,
    .switch_timing = blofeld::DspClockSwitch::functional_region_boundary};
constexpr blofeld::DspResetInputs reset{
    .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2};
// Own FM encodings, independently assembled. INSERT -> MOVEP requires one
// extra period; short immediate R0 -> X:(R0) requires three (FM A.2.3/A.2.4).
constexpr std::array gpio_program{
    0x56f400U, 0U, 0x44f400U, 1U, 0U, 0U, 0x0c1940U, 0x008018U, 0x04ccddU, 0U, 0U};
constexpr std::array agu_program{0x308000U, 0x44e000U, 0U, 0U};
unsigned assertions = 0;

void require(const bool condition, const std::string &message) {
  ++assertions;
  if (!condition) { throw blofeld::Fault(message); }
}

void advance(blofeld::DspMachine &chip,
             blofeld::DspExecution &timeline,
             const unsigned cycles = 1) {
  const auto result =
      timeline.advance_native_region(chip.clock().elapsed_ticks() + (cycles * period),
                                     blofeld::DspBoundary::through,
                                     {.known_requests = 15,
                                      .timing = blofeld::DspDmaTiming::reference_sampled_cycles,
                                      .grant = blofeld::DspDmaGrant::available},
                                     cycles);
  require(result && result->execution.fault.kind == dsp56300::FaultKind::none,
          "Actual native clock grant: pc=" + blofeld::hex(chip.cpu().registers().pc, 6) +
              " retired=" + std::to_string(chip.cpu().progress().instructions) +
              " fault=" + std::to_string(static_cast<unsigned>(chip.cpu().fault().kind)) +
              " address=" + blofeld::hex(chip.cpu().fault().address, 6));
}

struct Device {
  blofeld::DspMachine chip{clocks};
  std::unique_ptr<blofeld::DspExecution> timeline;

  explicit Device(
      const std::span<const unsigned> program,
      const bool configure_port_f = true,
      const blofeld::DspDependencyPolicy policy = blofeld::DspDependencyPolicy::qualified) {
    constexpr std::array bootstrap{jump};
    chip.bus().install_rom({.start = boot, .words = bootstrap});
    chip.begin_hardware_reset(reset);
    require(chip.advance_hardware_reset_ticks(reset_ticks / 2) == reset_ticks / 2,
            "The real chip completes reset hold");
    chip.release_hardware_reset();
    require(chip.advance_hardware_reset_ticks(reset_ticks / 2) == reset_ticks / 2,
            "The real chip completes reset recovery");
    for (unsigned address = 0; address < program.size(); ++address) {
      chip.bus().write(
          {.space = blofeld::DspSpace::p, .address = address, .value = program[address]});
    }
    if (configure_port_f) {
      chip.bus().write({.space = blofeld::DspSpace::x, .address = bcr, .value = bcr_value});
    }
    chip.bus().write({.space = blofeld::DspSpace::x, .address = source, .value = sample});
    auto &gpio = chip.bus().gpio(blofeld::DspPort::c);
    gpio.write({.reg = blofeld::DspGpioRegister::data, .value = 0});
    gpio.write({.reg = blofeld::DspGpioRegister::direction, .value = 1});
    gpio.write({.reg = blofeld::DspGpioRegister::data, .value = 0});
    timeline = std::make_unique<blofeld::DspExecution>(
        chip, blofeld::OwnDspAutomaticTiming::functional_native_regions, 0, std::nullopt, policy);
  }

  void retire_until(const unsigned count) {
    constexpr unsigned limit = 128;
    for (unsigned attempt = 0; attempt < limit && chip.cpu().progress().instructions < count;
         ++attempt) {
      advance(chip, *timeline);
    }
    require(chip.cpu().progress().instructions == count,
            "Only actual Core retirement reaches the authored instruction boundary");
  }

  [[nodiscard]] unsigned output() { return chip.bus().gpio(blofeld::DspPort::c).outputs().levels; }
};

void test_real_gpio_dependency_and_captured_checkpoint() {
  Device device(gpio_program);
  device.retire_until(6);
  require(device.chip.cpu().registers().pc == gpio_store && device.output() == 0 &&
              device.chip.clock().native_cycle_units() == 11,
          "Real reset JMP and authored producer complete before any GPIO write");
  advance(device.chip, *device.timeline);
  require(device.chip.cpu().progress().instructions == 6 && device.output() == 0 &&
              device.chip.clock().interlock_cycle_units() == 1 &&
              device.chip.clock().native_cycle_units() == 11,
          "The genuine dependency period advances time without retiring MOVEP or driving GPIO");
  const auto saved = device.timeline->checkpoint();
  device.chip.bus().write(
      {.space = blofeld::DspSpace::p, .address = gpio_store, .value = movep_b1});
  advance(device.chip, *device.timeline);
  require(device.output() == 1 && device.chip.cpu().progress().instructions == 7 &&
              device.chip.clock().native_cycle_units() == 12 &&
              device.chip.clock().interlock_cycle_units() == 1 &&
              device.chip.clock().elapsed_ticks() == reset_ticks + 13 * period,
          "The captured actual A1 store, not newer B1 code, commits after base plus stall time");

  blofeld::DspMachine clone(clocks);
  blofeld::DspExecution restored(clone, saved);
  clone.bus().write({.space = blofeld::DspSpace::p, .address = gpio_store, .value = movep_b1});
  advance(clone, restored);
  require(clone.bus().gpio(blofeld::DspPort::c).outputs().levels == 1 &&
              clone.cpu().registers() == device.chip.cpu().registers() &&
              clone.clock().elapsed_ticks() == device.chip.clock().elapsed_ticks() &&
              clone.clock().interlock_cycle_units() == 1,
          "Paired CPU/bus/timing restore retains the real pending word and accounted stall");

  auto other = gpio_program;
  other.at(gpio_store) = movep_b1;
  Device independent(other);
  independent.retire_until(7);
  require(independent.output() == 0 && independent.chip.clock().interlock_cycle_units() == 0 &&
              independent.chip.clock().elapsed_ticks() == reset_ticks + 12 * period &&
              independent.chip.cpu().jit_statistics().native_alu_instructions != 0,
          "A B1 consumer has no invented per-GPIO delay and retains native CPU execution");
}

void test_partial_agu_delay_restore_and_data_mutation() {
  Device device(agu_program);
  device.retire_until(2);
  require(device.chip.cpu().registers().r.front() == source,
          "The actual short MOVE establishes the independently authored address");
  advance(device.chip, *device.timeline);
  const auto saved = device.timeline->checkpoint();
  require(device.chip.cpu().progress().instructions == 2 &&
              device.chip.clock().native_cycle_units() == 4 &&
              device.chip.clock().interlock_cycle_units() == 1,
          "One of three AGU periods can finish without the actual memory read");
  device.chip.bus().write(
      {.space = blofeld::DspSpace::x, .address = source, .value = changed_sample});
  for (unsigned cycle = 0; cycle < 3; ++cycle) { advance(device.chip, *device.timeline); }
  require(device.chip.cpu().progress().instructions == 3 &&
              (device.chip.cpu().registers().x & dsp56300::word_mask) == changed_sample &&
              device.chip.clock().native_cycle_units() == 5 &&
              device.chip.clock().interlock_cycle_units() == 3 &&
              device.chip.clock().elapsed_ticks() == reset_ticks + 8 * period,
          "Two remaining stalls and one base period read actual current RAM at completion");
  blofeld::DspMachine clone(clocks);
  blofeld::DspExecution restored(clone, saved);
  clone.bus().write({.space = blofeld::DspSpace::x, .address = source, .value = changed_sample});
  for (unsigned cycle = 0; cycle < 3; ++cycle) { advance(clone, restored); }
  require(clone.cpu().registers() == device.chip.cpu().registers() &&
              clone.clock().elapsed_ticks() == device.chip.clock().elapsed_ticks() &&
              clone.clock().interlock_cycle_units() == 3,
          "A mid-stall checkpoint copies remaining clocks and dependency history together");
}

void test_reset_discards_pending_dependency() {
  Device device(agu_program);
  device.retire_until(2);
  advance(device.chip, *device.timeline);
  device.timeline->begin_hardware_reset(reset);
  require(device.chip.cpu().progress().instructions == 0 &&
              device.chip.cpu().registers().pc == boot &&
              device.chip.clock().interlock_cycle_units() == 1,
          "Reset discards the pending consumer while preserving completed lifetime stall time");
  const auto held = device.chip.clock().elapsed_ticks() + (reset_ticks / 2);
  static_cast<void>(device.timeline->advance_reset_to(held));
  device.timeline->release_hardware_reset();
  static_cast<void>(device.timeline->advance_reset_to(held + (reset_ticks / 2)));
  device.retire_until(3);
  require(device.chip.clock().interlock_cycle_units() == 4 &&
              device.chip.clock().epoch_interlock_cycle_units() == 3,
          "Fresh real execution needs all three AGU periods, with no resurrected reservation");
}

void test_real_wait_keeps_cpu_progress() {
  constexpr std::array program{0x000086U, 0U};
  Device device(program);
  device.retire_until(2);
  require(device.chip.cpu().processing_state() == dsp56300::ProcessingState::waiting,
          "The genuine WAIT opcode establishes the oscillator-running idle state");
  const auto progress = device.chip.cpu().progress();
  const auto before = device.chip.clock().elapsed_ticks();
  advance(device.chip, *device.timeline, 3);
  require(device.chip.cpu().progress() == progress &&
              device.chip.clock().elapsed_ticks() == before + 3 * period,
          "Clocked waiting ages history without a synthetic Core instruction");
}

void test_literal_bit_changes_do_not_add_dalu_status_delay() {
  constexpr unsigned literal_address = 0x20;
  constexpr unsigned move_sr_x1 = 0x0445b9;
  constexpr unsigned clear = 0x0a2000;
  constexpr unsigned change = 0x0b2000;
  constexpr unsigned word_bits = 24;
  // The actual mode5 Board reset sets FM S[1:0] and IPL: $c00300.
  constexpr unsigned expected_status = 0xc00301;
  constexpr std::array operations{0x0a2020U, clear, change, 0x0b2020U};
  for (const auto opcode : operations) {
    const std::array program{opcode, move_sr_x1, 0U};
    Device device(program);
    device.chip.bus().write(
        {.space = blofeld::DspSpace::x, .address = literal_address, .value = 1});
    device.retire_until(3);
    const auto expected = opcode == clear || opcode == change ? 0U : 1U;
    const auto registers = device.chip.cpu().registers();
    require(device.chip.bus().read({.space = blofeld::DspSpace::x, .address = literal_address}) ==
                    expected &&
                (registers.x >> word_bits) == expected_status &&
                device.chip.clock().interlock_cycle_units() == 0 &&
                device.chip.clock().elapsed_ticks() == reset_ticks + 6 * period,
            "Literal bit RMW/SR opcode=" + blofeld::hex(opcode, 6) +
                " x=" + blofeld::hex(registers.x, 12) + " sr=" + blofeld::hex(registers.sr, 6) +
                " native=" + std::to_string(device.chip.clock().native_cycle_units()) +
                " stalls=" + std::to_string(device.chip.clock().interlock_cycle_units()) +
                " ticks=" + std::to_string(device.chip.clock().elapsed_ticks()));
  }
  constexpr std::array indirect_program{0x0a6020U, 0U};
  Device indirect(indirect_program);
  indirect.retire_until(1);
  std::string refusal;
  try {
    advance(indirect.chip, *indirect.timeline);
  } catch (const blofeld::Fault &error) { refusal = error.what(); }
  require(refusal.contains("Unqualified DSP dependency instruction") &&
              indirect.chip.cpu().progress().instructions == 1,
          "Literal bit metadata never admits an unqualified register-indirect memory operand");
}

void test_real_omr_mode_bit_control() {
  constexpr unsigned jump_clear = 0x0afa03;
  constexpr unsigned jump_set = 0x0afa23;
  constexpr unsigned target = 4;
  constexpr unsigned reset_omr = 0x305;
  for (const auto opcode : {jump_clear, jump_set}) {
    const std::array program{opcode, target, 0U, 0U, 0U};
    Device device(program);
    device.retire_until(2);
    const auto expected_pc = opcode == jump_clear ? target : 2U;
    require(device.chip.cpu().registers().pc == expected_pc &&
                device.chip.cpu().registers().omr == reset_omr &&
                device.chip.clock().native_cycle_units() == 7 &&
                device.chip.clock().interlock_cycle_units() == 0 &&
                device.chip.clock().elapsed_ticks() == reset_ticks + 7 * period,
            "Real PCU mode-bit branches choose taken/untaken paths with FM base costs");
  }
  constexpr unsigned unqualified_sr = 0x0af903;
  constexpr std::array program{unqualified_sr, target, 0U, 0U, 0U};
  Device device(program);
  device.retire_until(1);
  std::string refusal;
  try {
    advance(device.chip, *device.timeline);
  } catch (const blofeld::Fault &error) { refusal = error.what(); }
  require(refusal.contains("Unqualified DSP dependency instruction") &&
              device.chip.cpu().progress().instructions == 1,
          "OMR metadata does not silently admit a direct SR bit-test interlock");
}

void test_fixed_memory_bit_control() {
  constexpr unsigned literal_address = 0x20;
  constexpr unsigned target = 4;
  constexpr unsigned jump_clear = 0x0a2080;
  constexpr unsigned jump_set = 0x0a20a0;
  constexpr unsigned reset_status = 0xc00300;
  for (const auto opcode : {jump_clear, jump_set}) {
    for (const auto value : {0U, 1U}) {
      const std::array program{opcode, target, 0U, 0U, 0U};
      Device device(program);
      device.chip.bus().write(
          {.space = blofeld::DspSpace::x, .address = literal_address, .value = value});
      device.retire_until(2);
      const bool taken = (opcode == jump_set) == (value != 0);
      require(device.chip.cpu().registers().pc == (taken ? target : 2U) &&
                  device.chip.cpu().registers().sr == reset_status &&
                  device.chip.clock().native_cycle_units() == 7 &&
                  device.chip.clock().interlock_cycle_units() == 0,
              "Actual fixed-memory bit branches preserve flags and both real control paths");
    }
  }
  // Independently assembled pp6 BCR and qq6 SHI-status tests.
  constexpr unsigned high_bcr_set = 0x0abba0;
  constexpr unsigned low_fifo_empty = 0x019191;
  for (const auto opcode : {high_bcr_set, low_fifo_empty}) {
    const std::array program{opcode, target, 0U, 0U, 0U};
    // Only reset BCR has qualified readback; configured BIU status stays unknown.
    Device device(program, false);
    device.retire_until(2);
    require(device.chip.cpu().registers().pc == target &&
                device.chip.clock().interlock_cycle_units() == 0,
            "Both actual peripheral windows are read through the genuine strict bus");
  }
}

void test_control_register_and_agu_sources() {
  constexpr unsigned clear_a = 0x200013;
  constexpr unsigned clear_ccr = 0x0000b9;
  constexpr unsigned set_ccr = 0x00fff9;
  constexpr unsigned move_sr_x1 = 0x0445b9;
  constexpr unsigned word_bits = 24;
  constexpr unsigned reset_status = 0xc00300;
  constexpr unsigned set_status = 0xc003ff;
  for (const auto opcode : {clear_ccr, set_ccr}) {
    const std::array program{clear_a, opcode, move_sr_x1, 0U};
    Device device(program);
    device.retire_until(4);
    require((device.chip.cpu().registers().x >> word_bits) ==
                    (opcode == clear_ccr ? reset_status : set_status) &&
                device.chip.clock().native_cycle_units() == 8 &&
                device.chip.clock().interlock_cycle_units() == 0,
            "Actual PCU ANDI/ORI retain independent normal-model SR values and base clocks");
  }

  constexpr unsigned move_r1_four = 0x310400;
  constexpr unsigned jump_r1 = 0x0ae180;
  constexpr std::array program{move_r1_four, jump_r1, 0U, 0U, 0U};
  Device device(program);
  device.retire_until(3);
  require(device.chip.cpu().registers().pc == 4 && device.chip.clock().native_cycle_units() == 7 &&
              device.chip.clock().interlock_cycle_units() == 3 &&
              device.chip.clock().elapsed_ticks() == reset_ticks + 10 * period,
          "A real indirect JMP consumes all three independently required AGU periods");
}

void test_real_register_loop_count() {
  constexpr unsigned load_x0 = 0x44f400;
  constexpr unsigned move_x0_a0 = 0x208800;
  constexpr unsigned loop_a0 = 0x06c800;
  constexpr unsigned endpoint = 6;
  constexpr std::array program{load_x0, 2U, move_x0_a0, loop_a0, endpoint, 0U, 0U, 0U};
  Device device(program);
  device.retire_until(8);
  constexpr unsigned loop_flag = 1U << 15;
  require(device.chip.cpu().registers().pc == endpoint + 1 &&
              device.chip.cpu().registers().lc == 0 && device.chip.cpu().registers().sp == 0 &&
              (device.chip.cpu().registers().sr & loop_flag) == 0 &&
              device.chip.clock().native_cycle_units() == 15 &&
              device.chip.clock().interlock_cycle_units() == 1 &&
              device.chip.clock().elapsed_ticks() == reset_ticks + 16 * period,
          "A0 transfer stall precedes the real two-iteration DO, stack and LF restoration");
}

void test_real_register_repeat() {
  constexpr std::array program{0x44f400U, 3U, 0x06c420U, 0U, 0U};
  Device device(program);
  device.retire_until(6);
  require(device.chip.cpu().registers().pc == 4 && device.chip.cpu().registers().lc == 0 &&
              device.chip.cpu().registers().x == 3 &&
              device.chip.clock().native_cycle_units() == 13 &&
              device.chip.clock().interlock_cycle_units() == 0,
          "Actual REP X0 repeats the real latched NOP three times and restores LC");
}

void test_real_long_store_limiter_and_status() {
  constexpr unsigned target = 0x40;
  constexpr unsigned initial_low = 0x123456;
  constexpr unsigned word_bits = 24;
  constexpr unsigned total_cycles = 17;
  for (const auto extension : {0U, 3U, 0xfeU}) {
    const bool negative = extension == 0xfe;
    const unsigned middle = negative ? 0x800000 : 0x7fffff;
    const std::array program{0x52f400U,
                             extension,
                             0x54f400U,
                             middle,
                             0x50f400U,
                             initial_low,
                             0x304000U,
                             0x485800U,
                             0x0445b9U,
                             0U};
    Device device(program);
    device.retire_until(7);
    const unsigned limited_low = negative ? 0U : dsp56300::word_mask;
    const unsigned low = extension == 0 ? initial_low : limited_low;
    const unsigned status = extension == 0 ? 0xc00300 : 0xc00340;
    const auto accumulator = (static_cast<std::uint64_t>(extension) << (2 * word_bits)) |
                             (static_cast<std::uint64_t>(middle) << word_bits) | initial_low;
    require(device.chip.bus().read({.space = blofeld::DspSpace::x, .address = target}) == middle &&
                device.chip.bus().read({.space = blofeld::DspSpace::y, .address = target}) == low &&
                device.chip.cpu().registers().r.front() == target + 1 &&
                device.chip.cpu().registers().a == accumulator &&
                (device.chip.cpu().registers().x >> word_bits) == status &&
                device.chip.clock().native_cycle_units() == 12 &&
                device.chip.clock().interlock_cycle_units() == 5 &&
                device.chip.clock().elapsed_ticks() == reset_ticks + total_cycles * period,
            "Actual L-store keeps the independent signed 48-bit limits, SR and AGU dependencies");
  }
}

void test_real_literal_calls_and_returns() {
  constexpr unsigned short_bsr = 0x050804;
  constexpr unsigned short_jsr = 0x0d0004;
  constexpr unsigned long_bsr = 0x0d1080;
  constexpr unsigned absolute_jsr = 0x0bf080;
  constexpr unsigned return_subroutine = 0x00000c;
  for (const auto opcode : {short_bsr, short_jsr, long_bsr, absolute_jsr}) {
    const bool extended = opcode == long_bsr || opcode == absolute_jsr;
    const std::array program{opcode, extended ? 4U : 0U, 0U, 0U, return_subroutine, 0U};
    Device device(program);
    device.retire_until(2);
    require(device.chip.cpu().registers().pc == 4 && device.chip.cpu().registers().sp == 1,
            "Literal calls reach the independent target through the actual PCU stack push");
    device.retire_until(3);
    const unsigned short_cycles = opcode == short_jsr ? 3U : 4U;
    const unsigned extended_cycles = opcode == long_bsr ? 5U : 4U;
    const unsigned call_cycles = extended ? extended_cycles : short_cycles;
    require(device.chip.cpu().registers().pc == (extended ? 2U : 1U) &&
                device.chip.cpu().registers().sp == 0 &&
                device.chip.clock().native_cycle_units() == 6 + call_cycles &&
                device.chip.clock().interlock_cycle_units() == 0,
            "Real RTS pops the independently expected return address and retains NXP base periods");
  }
}

void false_condition_decode_hole(const unsigned branch, const unsigned target) {
  constexpr unsigned move_b1 = 0x55f400;
  constexpr unsigned clear_a = 0x200013;
  constexpr unsigned jump_equal = 0x0ea000;
  const std::array program{move_b1, 1U, clear_a, branch | target, movep_b1, 0U, 0U};
  Device device(program);
  device.retire_until(4);
  const bool taken = branch == jump_equal;
  require(device.chip.cpu().registers().pc == (taken ? target : 4U) && device.output() == 0 &&
              device.chip.clock().native_cycle_units() == 10,
          "Actual flags select independent taken and fallthrough targets");
  const auto saved = device.timeline->checkpoint();
  advance(device.chip, *device.timeline);
  if (taken) {
    require(device.chip.cpu().progress().instructions == 5 &&
                device.output() == (target == 4 ? 1U : 0U) &&
                device.chip.clock().interlock_cycle_units() == 0,
            "A taken conditional transfer has no false-condition decoding hole");
    return;
  }
  require(device.chip.cpu().progress().instructions == 4 && device.output() == 0 &&
              device.chip.clock().native_cycle_units() == 10 &&
              device.chip.clock().interlock_cycle_units() == 1,
          "FM A.2.6.3 delays the next real GPIO instruction by one clock without retirement");
  advance(device.chip, *device.timeline);
  require(device.output() == 1 && device.chip.cpu().progress().instructions == 5 &&
              device.chip.clock().native_cycle_units() == 11,
          "The next actual GPIO effect commits only after the false-condition hole");
  blofeld::DspMachine clone(clocks);
  blofeld::DspExecution restored(clone, saved);
  advance(clone, restored);
  require(clone.cpu().progress().instructions == 4 && clone.clock().interlock_cycle_units() == 1,
          "Paired checkpoint restores the pending control-flow clock without a CPU override");
  advance(clone, restored);
  require(clone.cpu().registers() == device.chip.cpu().registers() &&
              clone.clock().elapsed_ticks() == device.chip.clock().elapsed_ticks() &&
              clone.bus().gpio(blofeld::DspPort::c).outputs().levels == 1,
          "Restored original control flow and GPIO effects retain the exact independently "
          "required period");
}

void test_real_indirect_call() {
  constexpr unsigned target = 4;
  constexpr std::array program{0x310400U, 0x0be180U, 0U, 0U, 0x00000cU, 0U};
  Device device(program);
  device.retire_until(3);
  require(device.chip.cpu().registers().pc == target && device.chip.cpu().registers().sp == 1 &&
              device.chip.clock().native_cycle_units() == 7 &&
              device.chip.clock().interlock_cycle_units() == 3,
          "The real indirect JSR consumes all independently required R1 clocks before pushing PC");
  device.retire_until(4);
  require(device.chip.cpu().registers().pc == 2 && device.chip.cpu().registers().sp == 0 &&
              device.chip.clock().native_cycle_units() == 10,
          "Actual indirect call and RTS preserve their independent return address and base costs");
}

void test_reset_discards_pending_control_period() {
  constexpr std::array program{0x200013U, 0x0e2004U, 0U, 0U, 0U};
  Device device(program);
  device.retire_until(3);
  require(device.chip.cpu().registers().pc == 2 && device.chip.clock().interlock_cycle_units() == 0,
          "The false condition leaves a real pending, not already consumed, decoding period");
  device.timeline->begin_hardware_reset(reset);
  const auto held = device.chip.clock().elapsed_ticks() + (reset_ticks / 2);
  static_cast<void>(device.timeline->advance_reset_to(held));
  device.timeline->release_hardware_reset();
  static_cast<void>(device.timeline->advance_reset_to(held + (reset_ticks / 2)));
  device.retire_until(1);
  require(
      device.chip.cpu().progress().instructions == 1 && device.chip.cpu().registers().pc == 0 &&
          device.chip.clock().epoch_interlock_cycle_units() == 0,
      "Reset control epoch: retired=" + std::to_string(device.chip.cpu().progress().instructions) +
          " pc=" + blofeld::hex(device.chip.cpu().registers().pc, 6) +
          " extra=" + std::to_string(device.chip.clock().epoch_interlock_cycle_units()));
}

void test_real_false_condition_decode_hole() {
  constexpr unsigned jump_equal = 0x0ea000;
  constexpr unsigned jump_not_equal = 0x0e2000;
  for (const auto branch : {jump_equal, jump_not_equal}) {
    for (const auto target : {4U, 6U}) { false_condition_decode_hole(branch, target); }
  }
}

void test_real_relative_register_loop() {
  // Own DOR Y1, two NOP body and count2, independently assembled.
  constexpr std::array program{0x47f400U, 2U, 0x06c710U, 3U, 0U, 0U, 0U};
  Device device(program);
  device.retire_until(7);
  require(device.chip.cpu().registers().pc == 6 && device.chip.cpu().registers().lc == 0 &&
              device.chip.cpu().registers().sp == 0 &&
              device.chip.clock().native_cycle_units() == 14 &&
              device.chip.clock().interlock_cycle_units() == 0,
          "DOR Y1 executes the real relative two-iteration loop and restores PCU state");
}

void test_real_program_memory_ports() {
  constexpr unsigned target = 0x40;
  constexpr unsigned clear_a = 0x200013;
  constexpr unsigned clear_b = 0x20001b;
  constexpr unsigned move_a1 = 0x54f400;
  constexpr unsigned move_r0 = 0x304000;
  constexpr unsigned load_x0 = 0x07e084;
  constexpr unsigned store_post = 0x07588c;
  constexpr unsigned store_absolute = 0x07708c;
  constexpr std::array load_program{move_r0, load_x0, 0U};
  Device reader(load_program);
  reader.chip.bus().write({.space = blofeld::DspSpace::p, .address = target, .value = sample});
  reader.retire_until(3);
  require(reader.chip.cpu().registers().x == sample &&
              reader.chip.cpu().registers().r.front() == target &&
              reader.chip.clock().native_cycle_units() == 10 &&
              reader.chip.clock().interlock_cycle_units() == 3,
          "Real P read preserves the independently required R0 clocks and actual source word");
  for (const auto clear : {clear_a, clear_b}) {
    const std::array post{move_a1, sample, move_r0, 0U, 0U, 0U, clear, store_post, 0U};
    Device writer(post);
    writer.chip.bus().write(
        {.space = blofeld::DspSpace::p, .address = target, .value = changed_sample});
    const auto revision = writer.chip.bus().program_revision();
    writer.retire_until(8);
    const unsigned expected = clear == clear_a ? 0U : sample;
    require(writer.chip.bus().inspect_known_program(target) == expected &&
                writer.chip.bus().program_revision() > revision &&
                writer.chip.cpu().registers().r.front() == target + 1 &&
                writer.chip.clock().native_cycle_units() == 16 &&
                writer.chip.clock().interlock_cycle_units() == (clear == clear_a ? 1U : 0U),
            "Actual postincrement P write retains data-port dependency and code invalidation");
    const std::array absolute{move_a1, sample, clear, store_absolute, target, 0U};
    Device literal(absolute);
    literal.retire_until(4);
    require(literal.chip.bus().inspect_known_program(target) == expected &&
                literal.chip.clock().native_cycle_units() == 13 &&
                literal.chip.clock().interlock_cycle_units() == 0,
            "The real absolute P write absorbs the independent earlier accumulator hazard");
  }
}

void test_real_relative_bit_control() {
  constexpr unsigned jump_clear = 0x0a2080;
  constexpr unsigned jump_set = 0x0a20a0;
  constexpr unsigned branch_clear = 0x0ca080;
  constexpr unsigned branch_set = 0x0ca0a0;
  constexpr unsigned target = 4;
  constexpr unsigned address = 0x20;
  constexpr unsigned reset_status = 0xc00300;
  for (const auto opcode : {jump_clear, jump_set, branch_clear, branch_set}) {
    for (const auto value : {0U, 1U}) {
      const std::array program{opcode, target, 0U, 0U, 0U, 0U};
      Device device(program);
      device.chip.bus().write({.space = blofeld::DspSpace::x, .address = address, .value = value});
      device.retire_until(2);
      const bool set = opcode == jump_set || opcode == branch_set;
      const bool taken = set == (value != 0);
      require(device.chip.cpu().registers().pc == (taken ? target : 2U) &&
                  device.chip.cpu().registers().sr == reset_status,
              "Actual fixed bit jumps and relative branches preserve flags and both control paths");
      advance(device.chip, *device.timeline);
      require(
          device.chip.cpu().progress().instructions == (taken ? 3U : 2U) &&
              device.chip.clock().interlock_cycle_units() == (taken ? 0U : 1U),
          "The genuine bit condition drives exactly the independent fallthrough decoding period");
    }
  }
}

unsigned bit_source_value(const dsp56300::Registers &registers, const unsigned selector) {
  constexpr unsigned word_bits = 24;
  constexpr unsigned first_accumulator = 8;
  if (selector >= first_accumulator) {
    constexpr std::array shifts{0U, 2 * word_bits, word_bits};
    const auto accumulator = (selector & 1U) != 0 ? registers.b : registers.a;
    return static_cast<unsigned>(accumulator >> shifts.at((selector - first_accumulator) / 2)) &
           dsp56300::word_mask;
  }
  constexpr unsigned first_y = 6;
  const auto pair = selector >= first_y ? registers.y : registers.x;
  return static_cast<unsigned>(pair >> ((selector & 1U) * word_bits)) & dsp56300::word_mask;
}

struct RegisterBitWindow {
  unsigned selector;
  unsigned operation;
  unsigned producer;
  unsigned gap;
  unsigned distance = 2;
};

void register_bit_window(const RegisterBitWindow window) {
  const auto [selector, operation, producer, gap, distance] = window;
  // Own NXP-assembled bit1 controls: six accumulator parts and X0/X1/Y0/Y1.
  constexpr unsigned first_accumulator = 8;
  constexpr unsigned first_high_part = 12;
  constexpr unsigned clear_a = 0x200013;
  constexpr unsigned clear_b = 0x20001b;
  constexpr unsigned move_a1 = 0x20ac00;
  constexpr unsigned move_b1 = 0x20ad00;
  constexpr unsigned set_condition = 0x20;
  const bool accumulator = selector >= first_accumulator;
  const bool matching_clear = producer == ((selector & 1U) != 0 ? clear_b : clear_a);
  const bool transfer = producer == move_a1 || producer == move_b1;
  const bool zero = accumulator && (matching_clear || (transfer && selector >= first_high_part));
  const unsigned expected_value = zero ? 0U : 2U;
  const bool taken = ((operation & set_condition) != 0) == !zero;
  const bool relative = (operation & 0xff0000U) == 0x0c0000;
  const unsigned branch_cycles = relative ? 5U : 4U;
  const unsigned dependency = accumulator && gap == 0 && (matching_clear || transfer) ? 1U : 0U;
  constexpr std::array setup{0x05f43aU, 0x305U, 0x52f400U, 2U, 0x54f400U, 2U, 0x50f400U, 2U,
                             0x53f400U, 2U,     0x55f400U, 2U, 0x51f400U, 2U, 0x44f400U, 2U,
                             0x45f400U, 0U,     0x46f400U, 2U, 0x47f400U, 2U, 0x05f439U, 0xc00301U};
  std::vector<unsigned> program(setup.begin(), setup.end());
  constexpr unsigned x1_literal = 17;
  if (!accumulator) { program.at(x1_literal) = 2; }
  constexpr unsigned quiet_periods = 8;
  constexpr unsigned setup_instructions = 12;
  program.insert(program.end(), quiet_periods, 0);
  program.push_back(producer);
  program.insert(program.end(), gap, 0);
  const auto branch_pc = static_cast<unsigned>(program.size());
  constexpr unsigned selector_shift = 8;
  program.push_back(operation | (selector << selector_shift));
  program.push_back(relative ? distance : branch_pc + distance);
  program.insert(program.end(), quiet_periods, 0);
  Device device(program);
  constexpr unsigned warmed = 1 + setup_instructions + quiet_periods;
  device.retire_until(warmed);
  const auto before = device.chip.clock().elapsed_ticks();
  const auto native = device.chip.clock().native_cycle_units();
  const auto retired_branch = warmed + 2 + gap;
  device.retire_until(retired_branch);
  const auto registers = device.chip.cpu().registers();
  const auto outcome = device.chip.cpu().instruction_outcome();
  const unsigned status = producer == clear_a || producer == clear_b ? 0xc00315U : 0xc00301U;
  require(outcome && outcome->control_transfer == taken &&
              registers.pc == branch_pc + (taken ? distance : 2U) && registers.sr == status &&
              bit_source_value(registers, selector) == expected_value &&
              device.chip.clock().native_cycle_units() == native + 1 + gap + branch_cycles &&
              device.chip.clock().interlock_cycle_units() == dependency,
          "Register bit control retains independent data, status, flow and producer clocks");
  advance(device.chip, *device.timeline);
  require(device.chip.cpu().progress().instructions == retired_branch + (taken ? 1U : 0U) &&
              device.chip.clock().interlock_cycle_units() == dependency + (taken ? 0U : 1U),
          "The genuine register condition determines the following decoding period");
  device.retire_until(retired_branch + 1);
  const auto expected_periods = 2 + gap + branch_cycles + dependency + (taken ? 0U : 1U);
  require(device.chip.clock().elapsed_ticks() == before + expected_periods * period,
          "Register bit timing includes both resource and conditional clocks exactly once");
}

void test_real_register_bit_control() {
  constexpr std::array operations{0x0ac001U, 0x0ac021U, 0x0cc081U, 0x0cc0a1U};
  constexpr unsigned clear_a = 0x200013;
  constexpr unsigned clear_b = 0x20001b;
  constexpr unsigned move_a1 = 0x20ac00;
  constexpr unsigned move_b1 = 0x20ad00;
  for (const auto operation : operations) {
    for (unsigned selector = 8; selector <= 13; ++selector) {
      const auto matching = (selector & 1U) != 0 ? clear_b : clear_a;
      const auto other = (selector & 1U) != 0 ? clear_a : clear_b;
      const auto transfer = (selector & 1U) != 0 ? move_b1 : move_a1;
      for (const auto producer : {matching, transfer, other}) {
        register_bit_window(
            {.selector = selector, .operation = operation, .producer = producer, .gap = 0});
        if (producer != other) {
          register_bit_window(
              {.selector = selector, .operation = operation, .producer = producer, .gap = 1});
        }
      }
    }
    for (unsigned selector = 4; selector <= 7; ++selector) {
      register_bit_window({.selector = selector, .operation = operation, .producer = 0, .gap = 0});
    }
    constexpr unsigned a1 = 12;
    constexpr unsigned different_target = 4;
    register_bit_window({.selector = a1,
                         .operation = operation,
                         .producer = clear_a,
                         .gap = 0,
                         .distance = different_target});
  }
}

struct RegisterLoopWindow {
  unsigned selector;
  unsigned operation;
  unsigned producer;
  unsigned gap;
};

void register_loop_window(const RegisterLoopWindow window) {
  const auto [selector, operation, producer, gap] = window;
  constexpr unsigned absolute = 0x200026;
  constexpr unsigned absolute_b = 0x20002e;
  constexpr unsigned move_a1 = 0x20ac00;
  constexpr unsigned move_b1 = 0x20ad00;
  constexpr unsigned first_accumulator = 8;
  constexpr unsigned repeat = 0x06c020;
  constexpr unsigned relative_loop = 0x06c010;
  const bool is_repeat = operation == repeat;
  const unsigned body_size = is_repeat ? 1U : 2U;
  const unsigned loop_words = is_repeat ? 1U : 2U;
  const bool matching = producer == ((selector & 1U) != 0 ? absolute_b : absolute);
  const bool transfer = producer == move_a1 || producer == move_b1;
  const unsigned dependency =
      selector >= first_accumulator && gap == 0 && (matching || transfer) ? 1U : 0U;
  constexpr std::array setup{0x05f43aU, 0x305U, 0x52f400U, 1U, 0x54f400U, 1U, 0x50f400U, 1U,
                             0x53f400U, 1U,     0x55f400U, 1U, 0x51f400U, 1U, 0x44f400U, 1U,
                             0x45f400U, 1U,     0x46f400U, 1U, 0x47f400U, 1U, 0x05f439U, 0xc00300U};
  std::vector<unsigned> program(setup.begin(), setup.end());
  constexpr unsigned quiet_periods = 8;
  constexpr unsigned setup_instructions = 12;
  program.insert(program.end(), quiet_periods, 0);
  program.push_back(producer);
  program.insert(program.end(), gap, 0);
  const auto loop_pc = static_cast<unsigned>(program.size());
  constexpr unsigned selector_shift = 8;
  program.push_back(operation | (selector << selector_shift));
  if (!is_repeat) {
    const auto last = loop_pc + loop_words + body_size - 1;
    program.push_back(operation == relative_loop ? last - loop_pc : last);
  }
  program.insert(program.end(), body_size + quiet_periods, 0);
  Device device(program);
  constexpr unsigned warmed = 1 + setup_instructions + quiet_periods;
  device.retire_until(warmed);
  const auto before = device.chip.clock().elapsed_ticks();
  const auto native = device.chip.clock().native_cycle_units();
  device.retire_until(warmed + 2 + gap);
  require(device.chip.cpu().registers().lc == 1 &&
              bit_source_value(device.chip.cpu().registers(), selector) == 1,
          "The genuine register counter initializes the actual PCU loop to one");
  device.retire_until(warmed + 3 + gap + body_size);
  const auto registers = device.chip.cpu().registers();
  constexpr unsigned loop_cycles = 5;
  const auto elapsed = 2 + gap + loop_cycles + body_size;
  const unsigned status = producer == absolute || producer == absolute_b ? 0xc00330U : 0xc00300U;
  require(registers.lc == 0 && registers.sp == 0 && registers.sr == status &&
              registers.pc == loop_pc + loop_words + body_size + 1 &&
              bit_source_value(registers, selector) == 1 &&
              device.chip.clock().interlock_cycle_units() == dependency &&
              device.chip.clock().native_cycle_units() == native + elapsed &&
              device.chip.clock().elapsed_ticks() == before + (elapsed + dependency) * period,
          "Independent DO/DOR/REP data, stack, status and resource clocks survive real execution");
}

void test_real_accumulator_loop_counters() {
  constexpr std::array operations{0x06c000U, 0x06c010U, 0x06c020U};
  constexpr unsigned absolute_a = 0x200026;
  constexpr unsigned absolute_b = 0x20002e;
  constexpr unsigned move_a1 = 0x20ac00;
  constexpr unsigned move_b1 = 0x20ad00;
  for (const auto operation : operations) {
    for (unsigned selector = 8; selector <= 13; ++selector) {
      const auto matching = (selector & 1U) != 0 ? absolute_b : absolute_a;
      const auto other = (selector & 1U) != 0 ? absolute_a : absolute_b;
      const auto transfer = (selector & 1U) != 0 ? move_b1 : move_a1;
      for (const auto producer : {matching, transfer, other}) {
        register_loop_window(
            {.selector = selector, .operation = operation, .producer = producer, .gap = 0});
        if (producer != other) {
          register_loop_window(
              {.selector = selector, .operation = operation, .producer = producer, .gap = 1});
        }
      }
    }
    for (unsigned selector = 4; selector <= 7; ++selector) {
      register_loop_window({.selector = selector, .operation = operation, .producer = 0, .gap = 0});
    }
  }
}

void test_conservative_timeline_checkpoint_and_reset() {
  constexpr unsigned increment_a = 0x000008;
  constexpr unsigned store_a0_gpio = 0x04c8dd;
  constexpr std::uint64_t serialization = 3;
  constexpr unsigned increment_status = 0xc00310;
  constexpr std::array program{increment_a, store_a0_gpio, 0U};
  Device device(program, true, blofeld::DspDependencyPolicy::conservative);
  device.retire_until(1);
  advance(device.chip, *device.timeline);
  const auto before_increment = device.timeline->checkpoint();
  require(device.chip.cpu().progress().instructions == 1 && device.chip.cpu().registers().a == 0 &&
              device.chip.clock().interlock_cycle_units() == 1 && device.output() == 0,
          "The explicit serialization period cannot retire INC or publish its later GPIO write");
  for (unsigned cycle = 0; cycle < serialization; ++cycle) {
    advance(device.chip, *device.timeline);
  }
  require(device.chip.cpu().progress().instructions == 2 && device.chip.cpu().registers().a == 1 &&
              device.chip.cpu().registers().pc == 1 &&
              device.chip.cpu().registers().sr == increment_status &&
              device.chip.clock().interlock_cycle_units() == serialization && device.output() == 0,
          "The genuine INC completes once after its three functional serialization clocks");
  blofeld::DspMachine pre_clone(clocks);
  blofeld::DspExecution pre_restored(pre_clone, before_increment);
  for (unsigned cycle = 0; cycle < serialization; ++cycle) { advance(pre_clone, pre_restored); }
  require(pre_clone.cpu().registers() == device.chip.cpu().registers() &&
              pre_clone.clock().elapsed_ticks() == device.chip.clock().elapsed_ticks() &&
              pre_clone.clock().interlock_cycle_units() == serialization,
          "Paired restore retains the conservative policy, captured INC and remaining pre-gap");

  advance(device.chip, *device.timeline);
  const auto before_gpio = device.timeline->checkpoint();
  require(device.chip.cpu().progress().instructions == 2 && device.output() == 0 &&
              device.chip.clock().interlock_cycle_units() == serialization + 1,
          "A completed unknown descriptor leaves a real pending post-gap before GPIO admission");
  for (unsigned cycle = 0; cycle < serialization; ++cycle) {
    advance(device.chip, *device.timeline);
  }
  constexpr std::uint64_t native_periods = 5;
  require(device.output() == 1 && device.chip.cpu().progress().instructions == 3 &&
              device.chip.clock().native_cycle_units() == native_periods &&
              device.chip.clock().interlock_cycle_units() == 2 * serialization &&
              device.chip.clock().elapsed_ticks() ==
                  reset_ticks + (native_periods + 2 * serialization) * period,
          "Actual A0 reaches the GPIO once after the complete pre/base/post sequence");
  blofeld::DspMachine post_clone(clocks);
  blofeld::DspExecution post_restored(post_clone, before_gpio);
  for (unsigned cycle = 0; cycle < serialization; ++cycle) { advance(post_clone, post_restored); }
  require(post_clone.cpu().registers() == device.chip.cpu().registers() &&
              post_clone.clock().elapsed_ticks() == device.chip.clock().elapsed_ticks() &&
              post_clone.bus().gpio(blofeld::DspPort::c).outputs().levels == 1,
          "A mid-post-gap checkpoint preserves the policy and delayed genuine bus effect");

  blofeld::DspMachine reset_clone(clocks);
  blofeld::DspExecution reset_restored(reset_clone, before_gpio);
  reset_restored.begin_hardware_reset(reset);
  require(
      reset_clone.cpu().registers().a == 1 && reset_clone.cpu().registers().pc == boot &&
          reset_clone.clock().epoch_interlock_cycle_units() == 0,
      "The board reset model retains uninitialized ALU registers and clears pending epoch time");
  const auto held = reset_clone.clock().elapsed_ticks() + (reset_ticks / 2);
  static_cast<void>(reset_restored.advance_reset_to(held));
  reset_restored.release_hardware_reset();
  static_cast<void>(reset_restored.advance_reset_to(held + (reset_ticks / 2)));
  constexpr unsigned limit = 32;
  for (unsigned attempt = 0; attempt < limit && reset_clone.cpu().progress().instructions < 2;
       ++attempt) {
    advance(reset_clone, reset_restored);
  }
  require(
      reset_clone.cpu().progress().instructions == 2 && reset_clone.cpu().registers().a == 2 &&
          reset_clone.clock().epoch_interlock_cycle_units() == serialization,
      "Conservative reset: retired=" + std::to_string(reset_clone.cpu().progress().instructions) +
          " a=" + blofeld::hex(reset_clone.cpu().registers().a, 14) +
          " pc=" + blofeld::hex(reset_clone.cpu().registers().pc, 6) +
          " extra=" + std::to_string(reset_clone.clock().epoch_interlock_cycle_units()));
}

void test_clocked_native_long_load_checkpoint() {
  constexpr unsigned move_r0 = 0x308000;
  constexpr unsigned load_long_a10 = 0x40d800;
  constexpr unsigned store_a0_gpio = 0x04c8dd;
  constexpr unsigned low_word = 1;
  constexpr std::uint64_t serialization = 3;
  constexpr std::array program{move_r0, load_long_a10, store_a0_gpio, 0U};
  Device device(program, true, blofeld::DspDependencyPolicy::conservative);
  device.chip.bus().write({.space = blofeld::DspSpace::y, .address = source, .value = low_word});
  device.retire_until(2);
  const auto before = device.chip.cpu().jit_statistics();
  advance(device.chip, *device.timeline);
  const auto captured = device.timeline->checkpoint();
  require(device.chip.cpu().progress().instructions == 2 && device.output() == 0 &&
              device.chip.clock().interlock_cycle_units() == 1,
          "A reserved long RAM transfer retains its first pre-gap and pending original opcode");
  for (unsigned cycle = 0; cycle < serialization; ++cycle) {
    advance(device.chip, *device.timeline);
  }
  constexpr unsigned word_bits = 24;
  const auto expected = (std::uint64_t{sample} << word_bits) | low_word;
  require(device.chip.cpu().progress().instructions == 3 &&
              device.chip.cpu().registers().a == expected &&
              device.chip.cpu().registers().r[0] == source + 1 && device.output() == 0 &&
              device.chip.clock().interlock_cycle_units() == serialization &&
              device.chip.cpu().jit_statistics().native_move_instructions ==
                  before.native_move_instructions + 1,
          "Clocked A10 RAM load actually executes natively after the full gap with one AGU update");
  blofeld::DspMachine pre_clone(clocks);
  blofeld::DspExecution pre_restored(pre_clone, captured);
  for (unsigned cycle = 0; cycle < serialization; ++cycle) { advance(pre_clone, pre_restored); }
  require(pre_clone.cpu().registers() == device.chip.cpu().registers() &&
              pre_clone.cpu().progress() == device.chip.cpu().progress() &&
              pre_clone.clock().elapsed_ticks() == device.chip.clock().elapsed_ticks(),
          "Paired checkpoint restores the captured native transfer and remaining exact clocks");
  advance(device.chip, *device.timeline);
  const auto post_gap = device.timeline->checkpoint();
  require(device.chip.cpu().progress().instructions == 3 && device.output() == 0 &&
              device.chip.clock().interlock_cycle_units() == serialization + 1,
          "Native retirement preserves the delayed genuine GPIO output and pending post-gap");
  for (unsigned cycle = 0; cycle < serialization; ++cycle) {
    advance(device.chip, *device.timeline);
  }
  constexpr std::uint64_t native_periods = 6;
  require(device.output() == low_word && device.chip.cpu().progress().instructions == 4 &&
              device.chip.clock().native_cycle_units() == native_periods &&
              device.chip.clock().interlock_cycle_units() == 2 * serialization &&
              device.chip.clock().elapsed_ticks() ==
                  reset_ticks + (native_periods + 2 * serialization) * period,
          "The actual peripheral effect follows the unchanged native/pre/post clock sequence");
  blofeld::DspMachine post_clone(clocks);
  blofeld::DspExecution post_restored(post_clone, post_gap);
  for (unsigned cycle = 0; cycle < serialization; ++cycle) { advance(post_clone, post_restored); }
  require(post_clone.cpu().registers() == device.chip.cpu().registers() &&
              post_clone.cpu().progress() == device.chip.cpu().progress() &&
              post_clone.clock().elapsed_ticks() == device.chip.clock().elapsed_ticks() &&
              post_clone.bus().gpio(blofeld::DspPort::c).outputs().levels == low_word,
          "Post-gap restoration retains the real RAM result and GPIO publication boundary");
}

void test_conservative_timeline_keeps_real_bus_fault() {
  constexpr unsigned move_r0 = 0x308000;
  constexpr unsigned load_long_a10 = 0x40e000;
  constexpr std::array program{move_r0, load_long_a10, 0U};
  Device device(program, true, blofeld::DspDependencyPolicy::conservative);
  device.retire_until(2);
  // Device initializes X:source only; the genuine second L operand in Y is unknown.
  std::string refusal;
  constexpr unsigned limit = 16;
  try {
    for (unsigned attempt = 0; attempt < limit; ++attempt) {
      advance(device.chip, *device.timeline);
    }
  } catch (const blofeld::Fault &error) { refusal = error.what(); }
  const auto fault = device.chip.cpu().fault();
  require(
      !refusal.empty() && refusal.find(blofeld::hex(source, 6)) != std::string::npos &&
          fault.kind == dsp56300::FaultKind::memory_read && fault.space == dsp56300::Space::y &&
          fault.address == source && fault.pc == 1 &&
          device.chip.cpu().progress().instructions == 2 && device.chip.cpu().registers().a == 0,
      "Conservative timing cannot hide a real unknown Y read, its PC/address or failed retirement");
}

void receive_spi_word(blofeld::DspShi &shi, const unsigned word) {
  constexpr unsigned hcsr = 0xffff91;
  constexpr unsigned spi_fifo = 0xa9;
  constexpr unsigned word_bits = 24;
  shi.observe_spi({.selected = false, .clock = false, .mosi = false});
  shi.write({.address = hcsr, .value = spi_fifo});
  shi.observe_spi({.selected = true, .clock = false, .mosi = false});
  for (unsigned index = word_bits; index != 0; --index) {
    const bool value = ((word >> (index - 1)) & 1U) != 0;
    shi.observe_spi({.selected = true, .clock = false, .mosi = value});
    shi.observe_spi({.selected = true, .clock = true, .mosi = value});
    shi.observe_spi({.selected = true, .clock = false, .mosi = value});
  }
  shi.observe_spi({.selected = false, .clock = false, .mosi = false});
}

void test_actual_spi_to_program_store() {
  constexpr unsigned code_target = 0x90;
  constexpr unsigned move_r0 = 0x309000;
  constexpr unsigned store_program = 0x009814;
  constexpr unsigned hcsr = 0xffff91;
  constexpr unsigned fifo_nonempty = 1U << 17;
  constexpr std::array program{move_r0, store_program, 0U};
  Device device(program);
  receive_spi_word(device.chip.bus().shi(), sample);
  device.retire_until(2);
  const auto revision = device.chip.bus().program_revision();
  advance(device.chip, *device.timeline);
  for (unsigned cycle = 0; cycle < 7; ++cycle) { advance(device.chip, *device.timeline); }
  require(device.chip.cpu().progress().instructions == 2 &&
              !device.chip.bus().inspect_known_program(code_target) &&
              (device.chip.bus().shi().read(hcsr) & fifo_nonempty) != 0,
          "Dependency/base reservations cannot consume FIFO or write P before completion");
  advance(device.chip, *device.timeline);
  require(device.chip.bus().inspect_known_program(code_target) == sample &&
              device.chip.bus().program_revision() > revision &&
              device.chip.cpu().registers().r.front() == code_target + 1 &&
              (device.chip.bus().shi().read(hcsr) & fifo_nonempty) == 0 &&
              device.chip.clock().native_cycle_units() == 10 &&
              device.chip.clock().interlock_cycle_units() == 3 &&
              device.chip.clock().elapsed_ticks() == reset_ticks + 13 * period,
          "Actual serial bits reach P RAM once through Core MOVEP and its real code invalidation");
}
}

int main() {
  try {
    test_real_gpio_dependency_and_captured_checkpoint();
    test_partial_agu_delay_restore_and_data_mutation();
    test_reset_discards_pending_dependency();
    test_real_wait_keeps_cpu_progress();
    test_literal_bit_changes_do_not_add_dalu_status_delay();
    test_real_omr_mode_bit_control();
    test_fixed_memory_bit_control();
    test_control_register_and_agu_sources();
    test_real_register_loop_count();
    test_real_register_repeat();
    test_real_long_store_limiter_and_status();
    test_actual_spi_to_program_store();
    test_real_literal_calls_and_returns();
    test_real_indirect_call();
    test_reset_discards_pending_control_period();
    test_real_false_condition_decode_hole();
    test_real_relative_register_loop();
    test_real_program_memory_ports();
    test_real_relative_bit_control();
    test_real_register_bit_control();
    test_real_accumulator_loop_counters();
    test_conservative_timeline_checkpoint_and_reset();
    test_clocked_native_long_load_checkpoint();
    test_conservative_timeline_keeps_real_bus_fault();
    std::cout << "Native dependency timeline: " << assertions << " assertions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
