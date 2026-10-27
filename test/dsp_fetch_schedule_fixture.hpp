// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_scheduled_run.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace blofeld::test::fetch_events {
inline constexpr unsigned extal = 12000000, timebase = 540000000, nops = 128;
inline constexpr std::uint64_t period = 45;
inline constexpr unsigned cycles = 32, initial_status = 0xc00300;
inline constexpr std::uint64_t horizon = cycles * period;
inline constexpr unsigned event_budget = 64;
DspFetchScheduleInputs inputs();
DspFetchCycleInputs cycle_inputs();
DspMachine &prepare(DspMachine &machine);

inline std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected pipeline event refusal");
}

struct Source {
  std::shared_ptr<const DspFetchEventIdentity> identity =
      std::make_shared<const DspFetchEventIdentity>();
  std::vector<std::uint64_t> ticks;
  std::vector<std::string> *log = nullptr;
  unsigned cursor = 0;
  DspMachine *machine = nullptr;
  DspExternalOrder order = DspExternalOrder::external_before_dsp;
  DspFetchExternalEvents events();
};

void guards();
void pin_capture();
} // namespace blofeld::test::fetch_events
