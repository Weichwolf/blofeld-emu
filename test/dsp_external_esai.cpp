// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_external_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld::test::external {
namespace {
constexpr unsigned primary = 0xffffa0;
constexpr unsigned secondary_base = 0xffff80;
constexpr unsigned lanes = 0x21;
constexpr unsigned mux = 0x858;
constexpr unsigned first5 = 0x89abcd;
constexpr unsigned second5 = 0xfedcba;
constexpr unsigned baseline_tick = 1501;
constexpr unsigned frame_tick = baseline_tick + bit_half;
constexpr unsigned edges_per_frame = 48;
constexpr unsigned total_edges = (2 * edges_per_frame) + 2;
constexpr unsigned prefix_bits = 8;
constexpr unsigned capture_tick = frame_tick + (15 * bit_half) + 1;
constexpr unsigned end_tick = frame_tick + ((total_edges - 1) * bit_half);
constexpr unsigned jump_absolute = 0x0af080;

DspProgram diagnostic(const bool second) {
  DspProgram result;
  const std::string space = second ? "y" : "x";
  const auto base = second ? secondary_base : primary;
  const auto put = [&](const unsigned offset, const unsigned value) {
    result.emit("movep #>" + number(value) + ',' + space + ":<<" + number(base + offset));
  };
  put(esai::clock, 0);
  put(0, esai::other);
  put(5, first5);
  put(esai::control, esai::normal24 | lanes);
  constexpr unsigned port_control = 31;
  constexpr unsigned port_direction = 30;
  put(port_control, mux);
  put(port_direction, mux);
  // Literal MOVEP setup:12 units; twelve NOPs put the following writes
  // after the first serial launch at2751, inside the current shift word.
  constexpr unsigned padding = 12;
  for (unsigned i = 0; i < padding; ++i) { result.emit("nop"); }
  put(0, esai::input);
  put(5, second5);
  const auto loop = result.pc();
  result.emit("nop");
  result.word(jump_absolute); // DSP56300FM long absolute JMP: four nominal units.
  result.word(loop);
  return result;
}

struct Wire {
  unsigned cursor = 0;
  std::array<std::string, 2> serial;

  DspExternalEvents source(DspMachine &machine, const DspEsaiInstance which) {
    return {
        .next_ticks = [&]() -> std::optional<std::uint64_t> {
          if (cursor == 0) { return baseline_tick; }
          if (cursor <= total_edges) { return frame_tick + (cursor - 1) * bit_half; }
          return std::nullopt;
        },
        .observe =
            [&, which](const std::uint64_t ticks) {
              const auto before = machine.cpu().debug_state();
              if (cursor == 0) {
                machine.bus().observe_esai_transmit(which, {.clock = true, .frame_sync = false});
              } else {
                const auto edge = cursor - 1;
                const bool clock = edge % 2 != 0;
                const bool sync = edge == 0 || edge == edges_per_frame;
                machine.bus().observe_esai_transmit(which, {.clock = clock, .frame_sync = sync});
                const auto output = machine.bus().esai_transmit_outputs(which);
                if (clock && edge < 2 * edges_per_frame) {
                  serial[0].push_back(esai::sample(output, 0));
                  serial[1].push_back(esai::sample(output, 5));
                  require(output.driven == lanes && output.known == lanes,
                          "Both native data words remain fully known at each launch");
                }
                if (ticks == end_tick) {
                  require(output.driven == 0, "The final external tail releases both data pads");
                }
              }
              require(machine.cpu().debug_state() == before,
                      "Serial clocks never execute the CPU or charge native cycles");
              ++cursor;
            },
        .order = DspExternalOrder::reject_tie,
        .maximum_events = total_edges + 1};
  }
};

void native_case(const bool second) {
  const auto which = second ? DspEsaiInstance::secondary : DspEsaiInstance::primary;
  DspMachine machine(contract());
  const auto program = diagnostic(second);
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  DspScheduledRun runner({.observation = DspStepObservation::reference_completion_boundary},
                         machine);
  Wire wire;
  auto source = wire.source(machine, which);
  constexpr unsigned budget = 256;
  runner.advance_to(machine, capture_tick, through, budget, {}, {}, source);
  require(wire.serial[0] == "00010010" && wire.serial[1] == "10001001" &&
              wire.serial[0].size() == prefix_bits &&
              runner.next_completion_ticks(machine) > capture_tick &&
              source.next_ticks() == frame_tick + (16 * bit_half),
          "A non-grid mid-word checkpoint retains real initial MOVEP bits and the next clock edge");
  const auto cpu = machine.checkpoint();
  const auto schedule = runner.checkpoint(machine);
  const auto saved_wire = wire;
  std::optional<DspDebugState> expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(cpu);
    runner.restore(schedule, machine);
    wire = saved_wire;
    runner.advance_to(machine, end_tick, through, budget, {}, {}, source);
    require(wire.serial[0] == "000100100011010001010110101001011100001110011111" &&
                wire.serial[1] == "100010011010101111001101111111101101110010111010" &&
                wire.cursor == total_edges + 1 &&
                runner.completed_external_events() == total_edges + 1,
            "Independent literal serial words survive current-word native writes and exact replay");
    if (!expected) { expected = machine.cpu().debug_state(); }
    require(machine.cpu().debug_state() == *expected && !runner.faulted(),
            "Restored partial native instruction, clock cursor and serial word reproduce exactly");
    // 12 setup+12 NOP+4 buffer-write units, then one-unit NOP/four-unit JMP.
    constexpr unsigned prefix_units = 28;
    const auto remaining = (end_tick / unit_ticks) - prefix_units;
    const auto units = prefix_units + ((remaining / 5) * 5) + (remaining % 5 >= 1 ? 1U : 0U);
    require(machine.run_clock().native_cycle_units() == units &&
                machine.run_clock().elapsed_ticks() == units * unit_ticks,
            "External bit clocks leave the independent native MOVEP/NOP/JMP cost ledger intact");
  }
}
}

void native_esai() {
  native_case(false);
  native_case(true);
}
}
