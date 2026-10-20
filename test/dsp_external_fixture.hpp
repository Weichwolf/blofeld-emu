// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::external {
inline constexpr unsigned extal = 12000000;
inline constexpr unsigned timebase = 1440000000;
inline constexpr std::uint64_t unit_ticks = 120;
inline constexpr unsigned partial = 125;
inline constexpr unsigned bit_half = 625;
inline constexpr auto through = DspScheduleBoundary::through;
inline constexpr auto before = DspScheduleBoundary::before;

inline DspRunContract contract() {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = timebase,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = DspRunTiming::native_internal_estimates};
}

inline DspMachine &initialize(DspMachine &machine) {
  DspProgram program;
  constexpr unsigned count = 32;
  for (unsigned i = 0; i < count; ++i) { program.emit("nop"); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  return machine;
}

struct Fixture {
  DspMachine machine{contract()};
  DspScheduledRun runner{{.observation = DspStepObservation::reference_completion_boundary},
                         initialize(machine)};
  std::vector<std::uint64_t> deadlines{0, partial, bit_half};
  unsigned cursor = 0;
  std::vector<std::uint64_t> applied;

  DspExternalEvents source(const DspExternalOrder order = DspExternalOrder::reject_tie) {
    return {.next_ticks = [&]() -> std::optional<std::uint64_t> {
              return cursor < deadlines.size() ? std::optional{deadlines.at(cursor)} : std::nullopt;
            },
            .observe =
                [&](const std::uint64_t ticks) {
                  require(ticks == deadlines.at(cursor),
                          "Dispatch must use the pinned absolute time");
                  require(machine.run_clock().elapsed_ticks() == ticks / unit_ticks * unit_ticks,
                          "Non-tied events see exactly the earlier independent NOP completions");
                  applied.push_back(ticks);
                  ++cursor;
                },
            .order = order,
            .maximum_events = 16};
  }
};

inline std::string rejects(const auto &operation) {
  return checkpoint::rejects(operation, "Expected explicit external-event rejection");
}

void native_esai();
void paired_events();
}
