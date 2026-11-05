// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "mcu/mcu_machine.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::fetch_events {
inline constexpr unsigned extal = 12000000, timebase = 540000000, nops = 128;
inline constexpr std::uint64_t period = 45, mcu_period = 90;
inline constexpr std::uint64_t initial_ticks = 450, reset_end = 585, first_cycle = 630,
                               horizon = 2070;
inline constexpr unsigned mcu_instructions = 18;
inline constexpr unsigned cycles = 33, initial_status = 0xc00300, boot = 0xff0000;
inline constexpr unsigned event_budget = 64;

void require(bool condition, const char *message);
McuDspRuntimeInputs inputs();
DspDmaInputs cycle_inputs();

inline std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const std::exception &error) { return error.what(); }
  throw Fault("Expected native runtime event refusal");
}

struct Source {
  std::shared_ptr<const bool> identity = std::make_shared<const bool>(true);
  std::vector<std::uint64_t> ticks;
  std::vector<std::string> log;
  unsigned cursor = 0;
  DspPeerOrder order = DspPeerOrder::peer_first;
  unsigned budget = event_budget;
  std::function<void()> on_query;
  std::function<void(std::uint64_t)> on_observe;
  DspRuntimeEvents events();
};

class Fixture {
public:
  explicit Fixture(DspPeerOrder order = DspPeerOrder::peer_first,
                   std::vector<std::uint64_t> ticks = {},
                   std::span<const std::uint32_t> program = {},
                   bool start = true);
  void connect(McuDspRuntimeInputs contract);
  void bind();
  void step();
  void run(unsigned count);
  McuMachine mcu;
  OwnDspMachine dsp;
  Source source;
  std::unique_ptr<McuDspRuntime> runtime;
  McuDspRuntime::Observer on_cycle;
  McuDspRuntime::CycleInputs provide_inputs = cycle_inputs;
};

void guards();
void pin_capture();
void boundary_checks();
}
