// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

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
using blofeld::test::require;
namespace fixture = blofeld::test::checkpoint;
constexpr unsigned extal = 12000000, timebase = 540000000;
constexpr std::uint64_t period = 45, mcu_period = 60;
constexpr unsigned epoch = 1000, reset_status = 0xc00300;
constexpr unsigned mcu_oscillator = 18000000, mcu_reset_periods = 5;
constexpr unsigned entry = 0x8000, nop = 0xa7, mask = 255, protection = 0x7f0d;
constexpr unsigned security = 0x7f0f, unsecured = 0xfe;
constexpr unsigned held_ticks = 100, recovery_end = 190, exclusive_ticks = 189;
constexpr unsigned external_recovery_tick = 145, first_native_end = 235;
constexpr unsigned u_direction = 0x027a;
constexpr std::uint8_t ldab_immediate = 0xc6, stab_extended = 0x7b, clear_b = 0xc7;
constexpr std::uint8_t port_u_high = 2, port_u_low = 0x78, u_direction_low = 0x7a;

std::string rejects(const auto &operation) {
  return fixture::rejects(operation, "Expected strict reset/schedule refusal");
}

DspRunContract contract() {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = timebase,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = DspRunTiming::native_internal_estimates,
          .resets = DspResetTiming::reference_extal_hold};
}

DspHardwareResetInputs pins() {
  return {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2};
}

DspScheduleInputs scheduled(const std::uint64_t offset = epoch) {
  return {.epoch_ticks = offset,
          .observation = DspStepObservation::reference_completion_boundary,
          .resets = DspResetObservation::reference_extal_hold};
}

void initialize(DspMachine &machine) {
  blofeld::test::DspProgram code;
  for (unsigned count = 0; count < 64; ++count) { code.emit("nop"); }
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = fixture::data, .value = fixture::cookie});
  machine.begin_hardware_reset(pins());
}

DspHardwareResetState reset_state(const DspMachine &machine) {
  const auto state = machine.run_clock().hardware_reset_state();
  if (!state) { throw Fault("Expected an actual held/recovery reset record"); }
  return *state;
}

void test_partial_periods() {
  DspMachine machine(contract());
  initialize(machine);
  const auto initial = machine.checkpoint();
  for (unsigned residue = 1; residue < period; ++residue) {
    machine.restore(initial);
    const auto asserted = (2 * period) + residue;
    require(machine.advance_hardware_reset_ticks(asserted) == asserted,
            "Asserted reset retains every supplied reference tick");
    const auto held = reset_state(machine);
    require(held.asserted_extal_cycles == 2 && held.phase_ticks == residue,
            "Literal whole-period and fractional reset ledgers remain distinct");
    machine.release_hardware_reset();
    require(machine.run_clock().hardware_reset_remaining_ticks() == 2 * period &&
                machine.advance_hardware_reset_ticks(period - 1) == period - 1,
            "Recovery starts at actual release with two full required periods");
    const auto saved = machine.checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      machine.restore(saved);
      require(machine.advance_hardware_reset_ticks(epoch) == period + 1 &&
                  !machine.hardware_reset_active() &&
                  machine.run_clock().elapsed_ticks() == asserted + (2 * period) &&
                  machine.run_clock().hardware_reset_ticks() == asserted + (2 * period) &&
                  machine.run_clock().native_cycle_units() == 0 &&
                  machine.run_clock().timer_clk2_edges() == 0 &&
                  machine.cpu().state().pc == blofeld::test::boot &&
                  machine.cpu().state().sr == reset_status,
              "Fractional recovery replay retains exact time without native execution");
    }
    machine.restore(saved);
    static_cast<void>(rejects([&] { machine.advance_hardware_reset(1); }));
    require(machine.timing_faulted(),
            "Whole-period API cannot silently consume a fractional suffix");
  }
}

void test_external_events(const DspExternalOrder order) {
  DspMachine machine(contract());
  initialize(machine);
  DspScheduledRun schedule(scheduled(), machine);
  constexpr std::array offsets{1U, 47U, 99U, 100U, 145U, 190U, 191U, 235U};
  unsigned cursor = 0;
  std::vector<std::string> events;
  const DspExternalEvents external{
      .next_ticks = [&]() -> std::optional<std::uint64_t> {
        return cursor < offsets.size() ? std::optional<std::uint64_t>(epoch + offsets[cursor])
                                       : std::nullopt;
      },
      .observe =
          [&](const auto tick) {
            require(tick == epoch + offsets[cursor], "External events retain exact deadlines");
            events.push_back("E" + std::to_string(tick));
            ++cursor;
          },
      .order = order,
      .maximum_events = 16};
  require(schedule.advance_to(
              machine, epoch + held_ticks, DspScheduleBoundary::through, 16, {}, {}, external) ==
                  0 &&
              cursor == 4 && machine.run_clock().elapsed_ticks() == held_ticks &&
              schedule.next_event_ticks(machine) == epoch + external_recovery_tick,
          "External events execute under RESET without any native instruction");
  const auto held = machine.checkpoint();
  const auto held_schedule = schedule.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(held);
    cursor = 4;
    events.clear();
    schedule.restore(held_schedule, machine);
    schedule.release_hardware_reset(machine);
    require(schedule.next_event_ticks(machine) == epoch + external_recovery_tick,
            "Earlier external deadlines remain ordered before reset recovery");
    require(schedule.advance_to(
                machine, epoch + recovery_end, DspScheduleBoundary::before, 16, {}, {}, external) ==
                    0 &&
                cursor == 5 && machine.hardware_reset_active() &&
                machine.run_clock().elapsed_ticks() == exclusive_ticks,
            "Exclusive recovery endpoint cannot release execution or consume the tied event");
    require(schedule.next_event_ticks(machine) == epoch + recovery_end,
            "Pending reset completion stays at the exclusive cut");
    require(
        schedule.advance_to(
            machine, epoch + recovery_end, DspScheduleBoundary::through, 16, {}, {}, external) ==
                0 &&
            cursor == 6 && !machine.hardware_reset_active(),
        "Recovery completes exactly before any native instruction");
    const auto observe = [&](const DspScheduledStep &step) {
      events.push_back("D" + std::to_string(step.completion_ticks));
    };
    require(schedule.advance_to(machine,
                                epoch + first_native_end,
                                DspScheduleBoundary::through,
                                16,
                                observe,
                                {},
                                external) == 1 &&
                cursor == 8 && machine.run_clock().native_cycle_units() == 1 &&
                machine.run_clock().elapsed_ticks() == first_native_end &&
                schedule.completed_external_events() == 8,
            "An actual NOP completes45 ticks after the held/recovery time");
    const std::vector<std::string> dsp_first{"E1145", "E1190", "E1191", "D1235", "E1235"};
    const std::vector<std::string> external_first{"E1145", "E1190", "E1191", "E1235", "D1235"};
    require(events == (order == DspExternalOrder::dsp_before_external ? dsp_first : external_first),
            "Explicit tie contracts retain literal external/native order after reset");
  }
}

void test_contract_and_epoch_guards() {
  DspMachine machine(contract());
  initialize(machine);
  auto config = scheduled();
  config.resets = DspResetObservation::unqualified;
  static_cast<void>(rejects([&] { const DspScheduledRun invalid(config, machine); }));
  config.resets = static_cast<DspResetObservation>(mask);
  static_cast<void>(rejects([&] { const DspScheduledRun invalid(config, machine); }));
  DspScheduledRun schedule(scheduled(), machine);
  const auto before = machine.checkpoint();
  const auto saved = schedule.checkpoint(machine);
  static_cast<void>(rejects([&] { schedule.release_hardware_reset(machine); }));
  require(schedule.faulted() && machine.timing_faulted(), "Short release quarantines both owners");
  machine.restore(before);
  schedule.restore(saved, machine);
  schedule.advance_to(machine, epoch + (2 * period), DspScheduleBoundary::through, 16);
  machine.release_hardware_reset();
  static_cast<void>(rejects([&] { static_cast<void>(schedule.checkpoint(machine)); }));
  require(!machine.timing_faulted(), "Out-of-band release cannot pass schedule identity checks");
  machine.restore(before);
  schedule.restore(saved, machine);
  schedule.advance_to(machine, epoch + (2 * period), DspScheduleBoundary::through, 16);
  schedule.release_hardware_reset(machine);
  schedule.advance_to(machine, epoch + (4 * period), DspScheduleBoundary::through, 16);
  const auto native = machine.cpu().debug_state();
  schedule.advance_to(machine, epoch + (4 * period) + 1, DspScheduleBoundary::through, 16);
  const auto error = rejects([&] { schedule.begin_hardware_reset(machine, pins()); });
  require(error.contains("pending instruction time") && machine.cpu().debug_state() == native &&
              !machine.hardware_reset_active() && !schedule.faulted(),
          "Asynchronous assertion cannot invent or discard pending native effects");
  schedule.advance_to(machine, epoch + (5 * period), DspScheduleBoundary::through, 16);
  schedule.begin_hardware_reset(machine, pins());
  require(machine.hardware_reset_active() && schedule.completed_steps() == 1 &&
              machine.run_clock().native_cycle_units() == 1 &&
              machine.run_clock().elapsed_ticks() == 5 * period,
          "Exact completed native boundary admits reset without losing lifetime history");
  static_cast<void>(rejects([&] { static_cast<void>(schedule.next_completion_ticks(machine)); }));
}

void test_time_exhaustion() {
  DspMachine machine(contract());
  initialize(machine);
  machine.advance_hardware_reset_ticks(1);
  const auto before = machine.cpu().debug_state();
  static_cast<void>(rejects(
      [&] { machine.advance_hardware_reset_ticks(std::numeric_limits<std::uint64_t>::max()); }));
  require(machine.timing_faulted() && machine.run_clock().elapsed_ticks() == 1 &&
              machine.cpu().debug_state() == before,
          "Tick overflow retains the actual partial reset prefix and quarantines its owner");
  DspMachine other(contract());
  initialize(other);
  other.advance_hardware_reset(2);
  auto config = scheduled();
  config.epoch_ticks = std::numeric_limits<std::uint64_t>::max() - held_ticks;
  DspScheduledRun runner(config, other);
  const auto held = reset_state(other);
  const auto error = rejects([&] { runner.release_hardware_reset(other); });
  require(error.contains("exhausted") && reset_state(other) == held && !runner.faulted() &&
              !other.timing_faulted(),
          "Unrepresentable absolute recovery rejects before changing the held state");
}

McuFlashImage image(const unsigned padding) {
  // Own HCS12 LDAB immediate, STAB extended (PwO), NOP and CLRB opcodes.
  std::vector<std::uint8_t> code{ldab_immediate,
                                 0,
                                 stab_extended,
                                 port_u_high,
                                 port_u_low,
                                 ldab_immediate,
                                 1,
                                 stab_extended,
                                 port_u_high,
                                 u_direction_low};
  code.insert(code.end(), padding, nop);
  code.insert(code.end(), {clear_b, stab_extended, port_u_high, u_direction_low});
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  for (unsigned index = 0; index < code.size(); ++index) { bytes.at(index) = code[index]; }
  bytes[protection] = mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_contract() {
  return {.clock = {.oscillator_hz = mcu_oscillator,
                    .initial_oscillator_phase_ticks = 0,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

void attach(McuMachine &mcu, DspMachine &dsp, McuDspSchedule &schedule, std::uint64_t &release) {
  mcu.set_cycle_observer([&](const auto cycle) {
    static_cast<void>(rejects([&] { schedule.release_dsp_reset(); }));
    const bool shared =
        (cycle.address == hc12::ptu || cycle.address == u_direction) && cycle.kind == 'w';
    schedule.begin_cycle(cycle, shared);
  });
  mcu.bus().set_observer([&](const auto &event) {
    if (!event.write || (event.address != hc12::ptu && event.address != u_direction)) { return; }
    schedule.observe_write(event, [&] {
      if (event.address != u_direction) { return; }
      const auto output = mcu.bus().gpio_outputs(McuPort::u);
      if ((output.mask & 1U) != 0) {
        require((output.known & 1U) != 0 && (output.levels & 1U) == 0 &&
                    dsp.hardware_reset_active(),
                "Native MCU establishes the declared actively-low laboratory reset drive");
        return;
      }
      require(release == 0, "Own MCU release instruction occurs exactly once");
      release = schedule.end_ticks();
      schedule.release_dsp_reset();
    });
  });
}

void step(McuMachine &mcu, McuDspSchedule &schedule, const unsigned count) {
  for (unsigned index = 0; index < count; ++index) {
    require(mcu.cpu().step().instruction_executed, "Own MCU instruction executes natively");
    schedule.end_instruction();
  }
}

void test_joint_release(const unsigned padding, const McuDspTieOrder order) {
  McuMachine mcu(image(padding), BLOFELD_DEFAULT_SLA, mcu_contract());
  mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  DspMachine dsp(contract());
  initialize(dsp);
  McuDspSchedule schedule({.dsp = scheduled(mcu_reset_periods * mcu_period),
                           .shared_write_order = order,
                           .maximum_dsp_steps_per_cycle = 16},
                          mcu,
                          dsp);
  std::uint64_t release = 0;
  attach(mcu, dsp, schedule, release);
  step(mcu, schedule, padding + 6);
  const auto held = reset_state(dsp);
  const auto expected_release = (mcu_reset_periods + 11 + padding) * mcu_period;
  const auto asserted = (11 + padding) * mcu_period;
  require(release == expected_release && schedule.end_ticks() == expected_release + mcu_period &&
              mcu.run_clock().bus_cycles() == mcu_reset_periods + 12 + padding &&
              dsp.hardware_reset_active() && dsp.cpu().debug_state().native_instructions == 0 &&
              dsp.run_clock().elapsed_ticks() == asserted + mcu_period &&
              held.asserted_extal_cycles == asserted / period && held.recovery_extal_cycles == 1 &&
              held.phase_ticks == 15,
          "Literal MCU bus costs release DSP at the native write, before the trailing O slot");
  const auto own_mcu = mcu.checkpoint();
  const auto own_dsp = dsp.checkpoint();
  const auto joint = schedule.checkpoint();
  const auto mcu_before = mcu.cpu().state();
  auto expected_dsp = dsp.cpu().debug_state();
  expected_dsp.cpu.pc += 4;
  expected_dsp.native_cycles = 4;
  expected_dsp.native_instructions = 4;
  for (unsigned replay = 0; replay < 2; ++replay) {
    mcu.restore(own_mcu);
    dsp.restore(own_dsp);
    schedule.restore(joint);
    attach(mcu, dsp, schedule, release);
    step(mcu, schedule, 4);
    const auto after = mcu.cpu().state();
    require(dsp.cpu().debug_state() == expected_dsp && after.pc == mcu_before.pc + 4 &&
                after.d == mcu_before.d && after.x == mcu_before.x && after.y == mcu_before.y &&
                after.sp == mcu_before.sp && after.ccr == mcu_before.ccr,
            "Independent NOP expectations cover every observed native CPU/stack register");
    require(!schedule.faulted() && !dsp.hardware_reset_active() &&
                schedule.end_ticks() == expected_release + (5 * mcu_period) &&
                schedule.completed_steps() == 4 &&
                dsp.cpu().state().pc == blofeld::test::boot + 4 &&
                dsp.run_clock().native_cycle_units() == 4 &&
                dsp.run_clock().elapsed_ticks() == asserted + (6 * period) &&
                dsp.run_clock().hardware_reset_ticks() == asserted + (2 * period) &&
                dsp.run_clock().timer_clk2_edges() == 2 &&
                dsp.bus().read({.space = DspSpace::x, .address = fixture::data}) == fixture::cookie,
            "Joint recovery replay resumes four actual NOPs with exact lifetime time and RAM");
  }
  const auto completed = dsp.cpu().debug_state();
  static_cast<void>(rejects([&] { schedule.begin_dsp_reset(pins()); }));
  require(dsp.cpu().debug_state() == completed && !schedule.faulted(),
          "Joint reset cannot discard a partially elapsed native instruction");
  step(mcu, schedule, 1);
  schedule.begin_dsp_reset(pins());
  require(dsp.hardware_reset_active() && schedule.completed_steps() == 6,
          "Joint assertion at an exact completed boundary retains all six native steps");
  step(mcu, schedule, 2);
  schedule.release_dsp_reset();
  step(mcu, schedule, 3);
  require(!schedule.faulted() && !dsp.hardware_reset_active() && schedule.completed_steps() == 8 &&
              dsp.run_clock().native_cycle_units() == 8 &&
              dsp.cpu().state().pc == blofeld::test::boot + 2 &&
              dsp.run_clock().hardware_reset_ticks() ==
                  asserted + (4 * period) + (2 * mcu_period) &&
              dsp.run_clock().elapsed_ticks() == asserted + (12 * period) + (2 * mcu_period),
          "A second joint held reset preserves lifetime time and restarts actual native execution");
  mcu.set_cycle_observer({});
  mcu.bus().set_observer({});
}
}

namespace {
constexpr unsigned held_idle_periods = 17, held_initial_cycles = 21;
constexpr unsigned held_cut = 131, held_remaining = 64;
constexpr unsigned held_handler_epoch = 2;

constexpr std::uint64_t held_cut_ticks = 3930, held_vector_end = 6450;
constexpr std::uint64_t held_reset_ticks = 6150, held_resumed_ticks = 6330;
constexpr std::uint64_t held_total_reset_ticks = 6240, held_resumed_end = 6630;
constexpr unsigned held_cut_fraction = 15, held_end_fraction = 30;

McuFlashImage held_image(const bool waiting) {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  constexpr std::array<std::uint8_t, 16> code{
      0xcf, 0x20, 0, 0xcc, 0x12, 0x34, 0xce, 0x45, 0x67, 0xcd, 0x67, 0x89, 0x10, 0xef, 0x3e, nop};
  if (waiting) {
    for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code[i]; }
  }
  bytes[protection] = mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

void held_attach(McuMachine &mcu, McuDspSchedule &joint) {
  mcu.set_cycle_observer([&](const auto cycle) { joint.begin_cycle(cycle, false); });
  mcu.set_wait_period_observer(
      [&](const auto period_input) { joint.begin_wait_period(period_input); });
  mcu.set_reset_period_observer(
      [&](const auto reset_input) { joint.begin_reset_period(reset_input); });
}

McuDspScheduleInputs held_inputs() {
  return {.dsp = scheduled(mcu_reset_periods * mcu_period),
          .shared_write_order = McuDspTieOrder::dsp_before_mcu,
          .maximum_dsp_steps_per_cycle = 16,
          .wait_order = McuDspWaitOrder::dsp_before_mcu};
}

void test_wait_held() {
  McuMachine mcu(held_image(true), BLOFELD_DEFAULT_SLA, mcu_contract());
  mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  DspMachine dsp(contract());
  initialize(dsp);
  McuDspSchedule joint(held_inputs(), mcu, dsp);
  held_attach(mcu, joint);
  step(mcu, joint, 6);
  require(mcu.cpu().waiting() && mcu.run_clock().bus_cycles() == held_initial_cycles,
          "Independent 2+2+2+2+1+7 setup cycles reach native WAI");
  const auto cm = mcu.checkpoint();
  const auto cd = dsp.checkpoint();
  const auto cs = joint.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    mcu.restore(cm);
    dsp.restore(cd);
    joint.restore(cs);
    held_attach(mcu, joint);
    require(mcu.advance_wait(held_idle_periods) == held_idle_periods,
            "Every bounded native WAI period elapses");
    joint.end_wait();
    constexpr auto ticks =
        (held_initial_cycles - mcu_reset_periods + held_idle_periods) * mcu_period;
    require(dsp.hardware_reset_active() && dsp.run_clock().elapsed_ticks() == ticks &&
                dsp.run_clock().hardware_reset_ticks() == ticks &&
                dsp.run_clock().native_cycle_units() == 0 &&
                dsp.run_clock().timer_clk2_edges() == 0 &&
                mcu.idle_bus_cycles() == held_idle_periods,
            "Native WAI clocks only asserted reset time, never DSP instructions/timer edges");
    joint.release_dsp_reset();
    require(mcu.advance_wait(3) == 3, "Three native idle periods cover recovery and two NOPs");
    joint.end_wait();
    require(!dsp.hardware_reset_active() && dsp.run_clock().native_cycle_units() == 2 &&
                dsp.cpu().state().pc == blofeld::test::boot + 2 &&
                dsp.run_clock().elapsed_ticks() == ticks + (4 * period) &&
                dsp.run_clock().hardware_reset_ticks() == ticks + (2 * period) &&
                dsp.run_clock().timer_clk2_edges() == 1 && mcu.cpu().waiting(),
            "Native WAI continues while DSP recovers and executes exactly two NOPs");
  }
}

void test_system_reset_held() {
  McuMachine mcu(held_image(false), BLOFELD_DEFAULT_SLA, mcu_contract());
  DspMachine dsp(contract());
  initialize(dsp);
  McuDspSchedule joint(held_inputs(), mcu, dsp);
  joint.begin_system_reset(McuResetSource::external, 3, McuDspResetOrder::dsp_before_mcu);
  mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  held_attach(mcu, joint);
  require(mcu.advance_system_reset(held_cut, McuResetPad::high) == held_cut,
          "131 real reset edges precede the cut");
  const auto cm = mcu.checkpoint();
  const auto cd = dsp.checkpoint();
  const auto cs = joint.checkpoint();
  require(dsp.run_clock().elapsed_ticks() == held_cut_ticks &&
              reset_state(dsp).phase_ticks == held_cut_fraction,
          "131 OSC edges at30 common ticks retain a fractional DSP reset period");
  for (unsigned replay = 0; replay < 2; ++replay) {
    mcu.restore(cm);
    dsp.restore(cd);
    joint.restore(cs);
    held_attach(mcu, joint);
    require(mcu.advance_system_reset(held_remaining, McuResetPad::high) == held_remaining,
            "The remaining64 reset edges and native vector execute");
    joint.end_system_reset();
    require(!mcu.system_reset_active() && mcu.cpu().state().pc == entry &&
                mcu.run_clock().native_epoch() == held_handler_epoch &&
                mcu.run_clock().bus_cycles() == 2ULL * mcu_reset_periods &&
                joint.end_ticks() == held_vector_end && dsp.hardware_reset_active() &&
                dsp.run_clock().elapsed_ticks() == held_reset_ticks &&
                dsp.run_clock().hardware_reset_ticks() == held_reset_ticks &&
                dsp.run_clock().native_cycle_units() == 0 &&
                dsp.run_clock().timer_clk2_edges() == 0 &&
                reset_state(dsp).phase_ticks == held_end_fraction,
            "195 reset OSC edges plus five vector periods execute with the DSP continuously held");
    joint.release_dsp_reset();
    step(mcu, joint, 3);
    require(!dsp.hardware_reset_active() && dsp.run_clock().native_cycle_units() == 2 &&
                dsp.cpu().state().pc == blofeld::test::boot + 2 &&
                dsp.run_clock().elapsed_ticks() == held_resumed_ticks &&
                dsp.run_clock().hardware_reset_ticks() == held_total_reset_ticks &&
                dsp.run_clock().timer_clk2_edges() == 1 && joint.end_ticks() == held_resumed_end,
            "Post-vector MCU NOPs cover exact DSP recovery and native execution");
  }
}
}

int main() {
  try {
    test_partial_periods();
    test_contract_and_epoch_guards();
    test_time_exhaustion();
    test_wait_held();
    test_system_reset_held();
    for (const auto order :
         {DspExternalOrder::dsp_before_external, DspExternalOrder::external_before_dsp}) {
      test_external_events(order);
    }
    for (const unsigned padding : {0U, 1U, 2U}) {
      for (const auto order : {McuDspTieOrder::dsp_before_mcu, McuDspTieOrder::mcu_before_dsp}) {
        test_joint_release(padding, order);
      }
    }
    std::cout << "Verified44 fractional reset cuts,6 native MCU release profiles,12 joint suffix "
                 "replays,4 held-WAI/reset suffix replays, external ordering and strict "
                 "reset-boundary guards.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
