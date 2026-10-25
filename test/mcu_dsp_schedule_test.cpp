// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace {
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned common_hz = 540000000;
constexpr unsigned common_period = 90;
constexpr unsigned dsp_setup_ticks = 180;
constexpr unsigned first_attempt = 675;
constexpr unsigned first_horizon = 720;
constexpr unsigned reset_end = 450;
constexpr unsigned loaded_ticks = 70;
constexpr unsigned loaded_index = 0xffff;
constexpr unsigned reserved = 0x2000;
constexpr unsigned port_mask = 0xff;

blofeld::McuFlashImage mcu_image(const std::span<const std::uint8_t> code = {}) {
  std::array<std::uint8_t, blofeld::McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  bytes.fill(nop);
  for (unsigned index = 0; index < code.size(); ++index) { bytes.at(index) = code[index]; }
  bytes[protection] = port_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = 0x80;
  bytes.back() = 0;
  return blofeld::McuFlashImage::from_known(bytes);
}

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) { return fault.what(); }
  throw blofeld::Fault("Expected strict common-clock rejection");
}

blofeld::McuRunContract mcu_contract() {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .initial_oscillator_phase_ticks = 0,
                    .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = blofeld::McuRunMemory::unstretched_internal};
}

blofeld::DspRunContract dsp_contract(const unsigned timebase = common_hz) {
  return {.clock = {.extal_hz = oscillator_hz,
                    .timebase_hz = timebase,
                    .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
          .timing = blofeld::DspRunTiming::native_internal_estimates};
}

blofeld::McuDspScheduleInputs inputs(const std::uint64_t epoch,
                                     const blofeld::McuDspTieOrder order) {
  return {.dsp = {.epoch_ticks = epoch,
                  .observation = blofeld::DspStepObservation::reference_completion_boundary},
          .shared_write_order = order,
          .maximum_dsp_steps_per_cycle = 16};
}

void initialize(blofeld::DspMachine &dsp, const blofeld::test::DspProgram &program) {
  program.install(dsp.bus());
  dsp.bus().pll().observe_pinit(false);
  dsp.cpu().reset_spi_boot();
}

void test_fault_at_each_native_delay_slot(const unsigned consumed_slots) {
  // Own HCS12 LDX #ffff (PO); DBNE X,self (PPP), independent documented bytes/costs.
  constexpr std::array<std::uint8_t, 6> code{0xce, 0xff, 0xff, 0x04, 0x35, 0xfd};
  blofeld::McuMachine mcu(mcu_image(code), BLOFELD_DEFAULT_SLA, mcu_contract());
  blofeld::test::DspProgram program;
  program.immediate("r0", reserved);
  program.immediate("x0", 7);
  program.emit("move x0,y:(r0)");
  blofeld::DspMachine dsp(dsp_contract());
  initialize(dsp, program);
  const auto phase = consumed_slots * common_period;
  blofeld::McuDspSchedule schedule(
      inputs(reset_end + phase, blofeld::McuDspTieOrder::dsp_before_mcu), mcu, dsp);
  const auto attach = [&] {
    mcu.set_cycle_observer([&](const auto cycle) { schedule.begin_cycle(cycle, false); });
  };
  attach();
  require(mcu.cpu().step().instruction_executed, "Native setup must execute its own LDX");
  schedule.end_instruction();
  require(mcu.cpu().state().x == loaded_index && mcu.run_clock().bus_cycles() == 7,
          "Five reset periods and the literal two-period LDX precede the native delay");
  const auto native_mcu = mcu.checkpoint();
  const auto native_dsp = dsp.checkpoint();
  const auto frozen = schedule.checkpoint();
  std::string expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    mcu.restore(native_mcu);
    dsp.restore(native_dsp);
    schedule.restore(frozen);
    attach();
    const auto error = rejects([&] { static_cast<void>(mcu.cpu().step()); });
    mcu.set_cycle_observer({});
    require(schedule.faulted() && mcu.timing_faulted() && dsp.timing_faulted() &&
                mcu.run_clock().bus_cycles() == 7 + consumed_slots &&
                mcu.run_clock().elapsed_ticks() == loaded_ticks + consumed_slots * 10 &&
                mcu.run_clock().oscillator_cycles() == 14 + consumed_slots * 2 &&
                dsp.run_clock().native_cycle_units() == 4 &&
                dsp.run_clock().elapsed_ticks() == dsp_setup_ticks &&
                schedule.end_ticks() == first_horizon + phase && schedule.completed_steps() == 2,
            "Only consumed MCU slots and completed DSP instructions advance their native clocks");
    require(error.contains("Unmapped DSP memory write at Y:0x002000") &&
                error.contains("mcu-pc=0x8003") &&
                error.contains("attempted-completion=" + std::to_string(first_attempt + phase)) &&
                mcu.cpu().state().x == loaded_index - 1,
            "The actual operand fault preserves its deadline and partial native DBNE side effect");
    static_cast<void>(rejects([&] { static_cast<void>(schedule.checkpoint()); }));
    dsp.restore(native_dsp);
    static_cast<void>(rejects([&] { schedule.restore(frozen); }));
    if (replay == 0) { expected = error; }
    require(error == expected, "Joint native recovery repeats the exact interrupted-slot fault");
  }
}

void test_shared_write_order(const blofeld::McuDspTieOrder order, const bool deliver = true) {
  // Own LDAB #42 (P); STAB direct PORTB (Pw). No physical board routing is implied.
  constexpr std::array<std::uint8_t, 4> code{0xc6, 0x42, 0x5b, 0x01};
  constexpr unsigned shared_address = 1;
  constexpr unsigned shared_value = 0x42;
  blofeld::McuMachine mcu(mcu_image(code), BLOFELD_DEFAULT_SLA, mcu_contract());
  mcu.bus().set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  mcu.bus().write(blofeld::hc12::portb, 0);
  mcu.bus().write(blofeld::hc12::ddrb, port_mask);
  blofeld::test::DspProgram program;
  for (unsigned count = 0; count < 16; ++count) { program.emit("nop"); }
  blofeld::DspMachine dsp(dsp_contract());
  initialize(dsp, program);
  blofeld::McuDspSchedule schedule(inputs(reset_end, order), mcu, dsp);
  std::vector<std::string> events;
  std::uint64_t shared_end = 0;
  bool rejected_active_capture = false;
  const auto before_dsp = [&](const blofeld::DspScheduledStep &step) {
    if (!rejected_active_capture) {
      const auto error = rejects([&] { static_cast<void>(schedule.checkpoint()); });
      require(error.contains("inactive boundary"), "Active paired callbacks cannot be captured");
      rejected_active_capture = true;
    }
    events.push_back("DSP:" + std::to_string(step.completion_ticks));
  };
  mcu.set_cycle_observer([&](const auto cycle) {
    const bool shared = cycle.address == shared_address && cycle.kind == 'w';
    schedule.begin_cycle(cycle, shared, before_dsp);
  });
  mcu.bus().set_observer([&](const auto &event) {
    if (!event.write || event.address != shared_address) { return; }
    if (!deliver) { return; }
    schedule.observe_write(
        event,
        [&] {
          const auto pads = mcu.bus().observe_pads(blofeld::McuPort::b, port_mask);
          require(event.value == shared_value && pads.known == port_mask &&
                      pads.levels == shared_value,
                  "A shared signal observation follows the actual native store");
          shared_end = schedule.end_ticks();
          events.push_back("MCU:" + std::to_string(shared_end));
        },
        before_dsp);
  });
  for (unsigned step = 0; step < 2; ++step) {
    static_cast<void>(mcu.cpu().step());
    if (!deliver && step == 1) {
      const auto error = rejects([&] { schedule.end_instruction(); });
      require(error.contains("not delivered") && schedule.faulted() && !mcu.timing_faulted() &&
                  !dsp.timing_faulted() && mcu.run_clock().bus_cycles() == 8 &&
                  dsp.run_clock().native_cycle_units() == 5,
              "A dropped signal observation faults without fabricating the deferred DSP tie");
      static_cast<void>(rejects([&] { static_cast<void>(schedule.checkpoint()); }));
      mcu.set_cycle_observer({});
      mcu.bus().set_observer({});
      return;
    }
    schedule.end_instruction();
  }
  mcu.set_cycle_observer({});
  mcu.bus().set_observer({});
  constexpr unsigned write_end = 720;
  require(shared_end == write_end && schedule.ties() == 1 && mcu.run_clock().bus_cycles() == 8 &&
              dsp.run_clock().native_cycle_units() == 6,
          "Literal LDAB/STAB costs and one-unit NOPs meet at the actual write endpoint");
  std::vector<std::string> simultaneous;
  for (const auto &event : events) {
    if (event.ends_with(std::to_string(write_end))) { simultaneous.push_back(event); }
  }
  const std::vector<std::string> dsp_first{"DSP:720", "MCU:720"};
  const std::vector<std::string> mcu_first{"MCU:720", "DSP:720"};
  require(simultaneous ==
              (order == blofeld::McuDspTieOrder::dsp_before_mcu ? dsp_first : mcu_first),
          "Each explicitly selected tie order reaches the native observation exactly once");
}

void test_timebase_and_contract_guards() {
  blofeld::McuMachine mcu(mcu_image(), BLOFELD_DEFAULT_SLA, mcu_contract());
  blofeld::DspMachine slow(dsp_contract(oscillator_hz));
  const auto error = rejects([&] {
    const blofeld::McuDspSchedule bad(
        inputs(reset_end, blofeld::McuDspTieOrder::unqualified), mcu, slow);
  });
  require(error.contains("exact positive integer"), "Nonintegral common time conversion rejects");
  blofeld::DspMachine dsp(dsp_contract());
  auto config = inputs(reset_end, blofeld::McuDspTieOrder::unqualified);
  config.maximum_dsp_steps_per_cycle = 0;
  static_cast<void>(rejects([&] { const blofeld::McuDspSchedule bad(config, mcu, dsp); }));
  config.maximum_dsp_steps_per_cycle = 16;
  config.dsp.observation = blofeld::DspStepObservation::unqualified;
  static_cast<void>(rejects([&] { const blofeld::McuDspSchedule bad(config, mcu, dsp); }));
}
}

int main() {
  try {
    for (unsigned slot = 0; slot < 3; ++slot) { test_fault_at_each_native_delay_slot(slot); }
    test_shared_write_order(blofeld::McuDspTieOrder::dsp_before_mcu);
    test_shared_write_order(blofeld::McuDspTieOrder::mcu_before_dsp);
    test_shared_write_order(blofeld::McuDspTieOrder::mcu_before_dsp, false);
    test_timebase_and_contract_guards();
    std::cout
        << "Production MCU/DSP schedule, native partial-slot failures and joint restore passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
