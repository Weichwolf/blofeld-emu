// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
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
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test::fetch_schedule {
namespace {
constexpr unsigned extal = 12000000;
constexpr unsigned common_hz = 540000000;
constexpr unsigned reset_ticks = 450;
constexpr unsigned write_ticks = 720;
constexpr unsigned shared_address = 1;
constexpr unsigned shared_value = 0x42;
constexpr unsigned all_pins = 0xff;

McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  constexpr unsigned entry_hi = 0x80;
  // Independent HCS12 encodings: LDAB #42 (P), STAB direct PORTB (Pw).
  constexpr std::array<std::uint8_t, 4> code{0xc6, 0x42, 0x5b, 0x01};
  bytes.fill(nop);
  for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code[i]; }
  bytes[protection] = all_pins;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry_hi;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_contract() {
  return {.clock = {.oscillator_hz = extal,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

void prepare(DspMachine &dsp) {
  DspProgram code;
  constexpr unsigned nops = 16;
  for (unsigned count = 0; count < nops; ++count) { code.emit("nop"); }
  code.install(dsp.bus());
  dsp.bus().pll().observe_pinit(false);
  dsp.cpu().reset_spi_boot();
  static_cast<void>(
      dsp.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
}

McuDspFetchInputs inputs(const McuDspTieOrder order) {
  return {.dsp = {.extal_hz = extal,
                  .timebase_hz = common_hz,
                  .timing = DspFetchTiming::reference_cycle_order,
                  .execution = DspReferenceExecution::staged_register_boundaries,
                  .pipeline = DspFetchPipeline::automatic_memory},
          .epoch_ticks = reset_ticks,
          .shared_write_order = order,
          .maximum_dsp_cycles_per_period = 16};
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected explicit pipeline schedule rejection");
}

enum class Delivery : std::uint8_t { normal, dropped, wrong_address };

std::string_view expected_error(const McuDspTieOrder order, const Delivery delivery) {
  if (order == McuDspTieOrder::unqualified) { return "Unqualified simultaneous"; }
  if (delivery == Delivery::dropped) { return "not delivered"; }
  return "match its consumed";
}

void tie_case(const McuDspTieOrder order, const Delivery delivery = Delivery::normal) {
  McuMachine mcu(image(), BLOFELD_DEFAULT_SLA, mcu_contract());
  mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  mcu.bus().write(hc12::portb, 0);
  mcu.bus().write(hc12::ddrb, all_pins);
  DspMachine dsp;
  prepare(dsp);
  McuDspFetchSchedule schedule(inputs(order), mcu, dsp);
  std::vector<std::string> events;
  unsigned input_calls = 0;
  bool rejected_reentry = false;
  const auto cycle_inputs = [&] {
    ++input_calls;
    return fetch_clock::input();
  };
  const auto completed = [&](const DspFetchCycleResult &cycle) {
    constexpr unsigned period = 45;
    require(cycle.period_ticks == period &&
                cycle.completion_ticks == cycle.completed_cycles * period,
            "Every DSP period follows the independent reset PLL expectation");
    events.push_back("DSP:" + std::to_string(reset_ticks + cycle.completion_ticks));
    const auto error = rejects([&] { schedule.end_instruction(); });
    require(error.contains("inactive boundary"), "Active owner rejects callback reentry");
    rejected_reentry = true;
  };
  mcu.set_cycle_observer([&](const auto cycle) {
    schedule.begin_cycle(
        cycle, cycle.kind == 'w' && cycle.address == shared_address, cycle_inputs, completed);
  });
  mcu.bus().set_observer([&](const auto &event) {
    if (!event.write || event.address != shared_address || delivery == Delivery::dropped) {
      return;
    }
    auto observed = event;
    if (delivery == Delivery::wrong_address) { ++observed.address; }
    schedule.observe_write(
        observed,
        [&] {
          const auto pads = mcu.bus().observe_pads(McuPort::b, all_pins);
          require(event.value == shared_value && pads.known == all_pins &&
                      pads.levels == shared_value,
                  "The actual native write precedes its board signal observation");
          events.push_back("MCU:" + std::to_string(schedule.end_ticks()));
        },
        cycle_inputs,
        completed);
  });
  require(mcu.cpu().step().instruction_executed, "Actual LDAB executes its literal single period");
  schedule.end_instruction();
  const auto run_second = [&] {
    require(mcu.cpu().step().instruction_executed, "Actual STAB executes its literal two periods");
    schedule.end_instruction();
  };
  if (order == McuDspTieOrder::unqualified || delivery != Delivery::normal) {
    const auto error = rejects(run_second);
    require(error.contains(expected_error(order, delivery)) && schedule.faulted(),
            "Rejected wire/tie contract quarantines owner");
    require(schedule.run().completed_cycles() == 5 && input_calls == 5,
            "No rejected/deferred sixth DSP period is silently executed");
    static_cast<void>(rejects([&] { schedule.end_instruction(); }));
  } else {
    run_second();
    require(rejected_reentry && !schedule.faulted() && input_calls == 6 &&
                schedule.run().completed_cycles() == 6 && schedule.ties() == 1 &&
                schedule.end_ticks() == write_ticks && mcu.run_clock().bus_cycles() == 8,
            "Five reset,one LDAB and two STAB periods meet six actual DSP periods");
    std::vector<std::string> tied;
    for (const auto &event : events) {
      if (event.ends_with(std::to_string(write_ticks))) { tied.push_back(event); }
    }
    const std::vector<std::string> dsp_first{"DSP:720", "MCU:720"};
    const std::vector<std::string> mcu_first{"MCU:720", "DSP:720"};
    require(tied == (order == McuDspTieOrder::dsp_before_mcu ? dsp_first : mcu_first),
            "Actual shared boundary follows the explicitly selected order exactly once");
  }
  mcu.set_cycle_observer({});
  mcu.bus().set_observer({});
}

void input_case(const bool missing) {
  McuMachine mcu(image(), BLOFELD_DEFAULT_SLA, mcu_contract());
  DspMachine dsp;
  prepare(dsp);
  auto settings = inputs(McuDspTieOrder::dsp_before_mcu);
  if (!missing) { settings.maximum_dsp_cycles_per_period = 1; }
  McuDspFetchSchedule schedule(settings, mcu, dsp);
  mcu.set_cycle_observer([&](const auto cycle) {
    const McuDspFetchSchedule::CycleInputs cycle_inputs =
        missing ? McuDspFetchSchedule::CycleInputs{} : [] { return fetch_clock::input(); };
    schedule.begin_cycle(cycle, false, cycle_inputs);
  });
  const auto error = rejects([&] { static_cast<void>(mcu.cpu().step()); });
  require(error.contains(missing ? "explicit native cycle inputs" : "cycle budget exhausted") &&
              schedule.faulted() && schedule.run().completed_cycles() == (missing ? 0 : 1),
          "Missing DMA observations and exhausted budget remain explicit failures");
  mcu.set_cycle_observer({});
}

void run() {
  tie_case(McuDspTieOrder::dsp_before_mcu);
  tie_case(McuDspTieOrder::mcu_before_dsp);
  tie_case(McuDspTieOrder::unqualified);
  tie_case(McuDspTieOrder::mcu_before_dsp, Delivery::dropped);
  tie_case(McuDspTieOrder::mcu_before_dsp, Delivery::wrong_address);
  input_case(true);
  input_case(false);
}
}
}

int main() {
  try {
    blofeld::test::fetch_schedule::run();
    std::cout << "Seven native pipeline schedule timing/failure cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
