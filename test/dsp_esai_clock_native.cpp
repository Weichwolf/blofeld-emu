// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_esai.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "dsp_norm_fixture.hpp"
#include "esai_clock_fixture.hpp"
#include "esai_fixture.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld::test::esai_clock {
namespace {
using esai::rejects;
using esai::require;
constexpr unsigned primary_base = 0xffffa0;
constexpr unsigned secondary_base = 0xffff80;
constexpr unsigned mux = 0x838; // TX0, HCKT, FST and SCKT.
constexpr unsigned port_control = 31;
constexpr unsigned port_direction = 30;
constexpr unsigned extal = 12000000;
constexpr std::uint64_t common_hz = 138240000000;
constexpr std::uint64_t unit_ticks = 11520;
constexpr std::uint64_t half_period = 5625;
constexpr std::uint64_t first_unenabled_launch = 129375;
constexpr std::uint64_t first_data = 3009375;
constexpr std::uint64_t data_period = 45000;
constexpr std::uint64_t capture = first_data + (6 * data_period) + (2 * half_period) + 1;
constexpr std::uint64_t finish = first_data + (63 * data_period) + (4 * half_period);
constexpr unsigned prefix_units = 271;
constexpr unsigned jump_absolute = 0x0af080;

DspRunContract contract() {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = common_hz,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = DspRunTiming::native_internal_estimates};
}

DspProgram program(const bool second) {
  DspProgram result;
  const auto base = second ? secondary_base : primary_base;
  const std::string space = second ? "y" : "x";
  const auto put = [&](const unsigned offset, const unsigned value) {
    result.emit("movep #>" + number(value) + ',' + space + ":<<" + number(base + offset));
  };
  put(esai::control, word32);
  put(esai::clock, base_clock | clock_polarity);
  put(port_control, mux);
  put(port_direction, mux);
  put(0, esai::other);
  put(esai::control, word32 | 1U);
  // Six MOVEP cost12 units. The explicit clock starts frame0 before TE;
  // 256 NOPs place this status read/refill inside the first transmitted word.
  constexpr unsigned padding = 256;
  for (unsigned i = 0; i < padding; ++i) { result.emit("nop"); }
  result.emit("movep " + space + ":<<" + number(base + esai::status) + ",x0");
  put(0, esai::input);
  const auto loop = result.pc();
  result.emit("nop");
  result.word(jump_absolute);
  result.word(loop);
  return result;
}

struct Wire {
  std::uint64_t cursor = 0;
  std::string bits;

  DspExternalEvents
  source(DspMachine &machine, const DspEsaiInstance instance, const DspExternalOrder order) {
    return {.next_ticks = [&]() -> std::optional<std::uint64_t> { return cursor * half_period; },
            .observe =
                [&, instance](const std::uint64_t ticks) {
                  const auto previous = machine.bus().esai_clock_outputs(instance);
                  const auto native = machine.cpu().debug_state();
                  machine.bus().observe_esai_transmit(
                      instance, {.high_clock = cursor % 2 != 0, .timing = timing});
                  const auto clocks = machine.bus().esai_clock_outputs(instance);
                  if (ticks == first_unenabled_launch) {
                    require(clocks.driven == 3 && clocks.levels == 2 &&
                                machine.bus().esai_transmit_outputs(instance).driven == 0,
                            "Native muxing starts output clocks before native TE enable");
                  }
                  // TCKP1: data changes falling, the independent receiver captures rising.
                  if ((previous.known & clocks.known & 1U) != 0 && (previous.levels & 1U) == 0 &&
                      (clocks.levels & 1U) != 0) {
                    const auto output = machine.bus().esai_transmit_outputs(instance);
                    if ((output.driven & 1U) != 0) {
                      require(output.known == 1,
                              "The receiver requires an explicitly known data word");
                      bits.push_back(esai::sample(output, 0));
                      const auto expected_tick =
                          first_data + (4 * half_period) + ((bits.size() - 1) * data_period);
                      require(ticks == expected_tick,
                              "Every captured bit has an independent literal HCKT/SCKT phase");
                    }
                  }
                  require(machine.cpu().debug_state() == native,
                          "HCKT events and the receiver never execute native instructions");
                  ++cursor;
                },
            .order = order,
            .maximum_events = 2048};
  }
};

void run_native(const bool second, const DspExternalOrder order) {
  const auto instance = second ? DspEsaiInstance::secondary : DspEsaiInstance::primary;
  DspMachine machine(contract());
  const auto code = program(second);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  DspScheduledRun schedule({.observation = DspStepObservation::reference_completion_boundary},
                           machine);
  Wire wire;
  const auto source = wire.source(machine, instance, order);
  constexpr unsigned budget = 512;
  constexpr auto through = DspScheduleBoundary::through;
  schedule.advance_to(machine, capture, through, budget, {}, {}, source);
  require(
      wire.bits == "000100" && machine.cpu().state().x == esai::empty_even_frame &&
          schedule.next_completion_ticks(machine) > capture,
      "Native MOVEP sees actual TFS/TDE/TEDE; six bits precede a partial divider/word snapshot");
  const auto cpu = machine.checkpoint();
  const auto saved_schedule = schedule.checkpoint(machine);
  const auto saved_wire = wire;
  std::optional<DspDebugState> expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(cpu);
    schedule.restore(saved_schedule, machine);
    wire = saved_wire;
    schedule.advance_to(machine, finish, through, budget, {}, {}, source);
    require(
        wire.bits == word(false, true) + word(true, true) &&
            wire.cursor == finish / half_period + 1,
        "Restoring native CPU, divider, generated FST, word and receiver reproduces all64 bits");
    const auto remaining = (finish / unit_ticks) - prefix_units;
    const auto units = prefix_units + ((remaining / 5) * 5) + (remaining % 5 >= 1 ? 1U : 0U);
    require(machine.run_clock().native_cycle_units() == units &&
                machine.run_clock().elapsed_ticks() == units * unit_ticks,
            "The manual12+256+1+2 prefix and one/four-unit loop retain independent native timing");
    if (!expected) { expected = machine.cpu().debug_state(); }
    require(machine.cpu().debug_state() == *expected,
            "Partial native JMP and exact external phase restore to the same full debug state");
  }
}

void pad_contracts() {
  DspBus bus;
  constexpr auto instance = DspEsaiInstance::secondary;
  const auto put = [&](const unsigned offset, const unsigned value) {
    bus.write({.space = DspSpace::y, .address = secondary_base + offset, .value = value});
  };
  const auto observe = [&](const bool clock) {
    bus.observe_esai_transmit(instance, {.high_clock = clock, .timing = timing});
  };
  observe(false);
  put(esai::clock, base_clock);
  put(esai::control, word32 | 1U);
  put(0, esai::other);
  constexpr unsigned without_high_clock = 0x818;
  put(port_control, without_high_clock);
  put(port_direction, without_high_clock);
  require(rejects([&] { observe(true); }).contains("HCKT"),
          "A divided clock requires the actual high-clock input pad selected as a peripheral");
  put(port_control, mux);
  put(port_direction, mux);
  constexpr unsigned bit_pad = 8;
  bus.gpio(DspPort::e).set_drive({.mask = bit_pad, .levels = bit_pad});
  require(rejects([&] { observe(true); }).contains("Y:0xffff9d"),
          "Opposed external SCKT drive fails against the selected generated output");
  bus.gpio(DspPort::e).set_drive({});
  observe(true);
  observe(false);
  constexpr unsigned frame_pad = 16;
  bus.gpio(DspPort::e).set_drive({.mask = frame_pad});
  static_cast<void>(rejects([&] { observe(true); }));
  bus.gpio(DspPort::e).set_drive({});
  observe(true);
  require(bus.esai_clock_outputs(instance).levels == 3 &&
              esai::sample(bus.esai_transmit_outputs(instance), 0) == '0',
          "Rejected clock/frame contention commits neither the divider edge nor the first bit");
}

void missing_clock_recovery() {
  DspProgram code;
  const auto put = [&](const unsigned offset, const unsigned value) {
    code.emit("movep #>" + number(value) + ",x:<<" + number(primary_base + offset));
  };
  put(esai::control, word32);
  put(esai::clock, base_clock);
  put(port_control, mux);
  put(port_direction, mux);
  put(0, esai::other);
  put(esai::control, word32 | 1U);
  code.emit("movep x:<<$ffffb3,x0");
  DspMachine machine(contract());
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  for (unsigned i = 0; i < 6; ++i) { machine.cpu().step(); }
  const auto checkpoint = machine.checkpoint();
  const auto error = rejects([&] { machine.cpu().step(); });
  require(error.contains("0xff000c") && error.contains("0xffffb6") && error.contains("timing"),
          "Missing high-clock qualification faults at the actual native status read and PC");
  machine.restore(checkpoint);
  machine.bus().observe_esai_transmit(DspEsaiInstance::primary,
                                      {.high_clock = false, .timing = timing});
  machine.cpu().step();
  require(machine.cpu().state().x == 0 && machine.run_clock().native_cycle_units() == 13,
          "Trusted native restore and a frozen input baseline recover only the real reset status");
}
}

void native_clock() {
  for (const auto order :
       {DspExternalOrder::dsp_before_external, DspExternalOrder::external_before_dsp}) {
    run_native(false, order);
    run_native(true, order);
  }
  pad_contracts();
  missing_clock_recovery();
}
}
