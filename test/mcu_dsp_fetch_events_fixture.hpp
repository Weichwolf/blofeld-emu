// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "mcu/mcu_machine.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::joint_events {
inline constexpr unsigned oscillator = 12000000, common = 540000000, entry = 0x8000;
inline constexpr std::uint64_t initial_time = 450, recovery_end = 585, period = 45, collision = 720,
                               finish = 1080;
inline constexpr unsigned pins = 0xfff, byte_mask = 0xff, marker_address = 0x1200, marker = 0x5a;

struct Event {
  char actor;
  std::uint64_t time;
  bool operator==(const Event &) const = default;
};

struct Case {
  McuWriteOrder native;
  McuWriteOrder external;
  DspPeerOrder peers;
  unsigned sample;
  std::array<char, 3> order;
};

McuFlashImage image();
McuRunContract mcu_contract();
McuDspRuntimeInputs settings(Case order);
void prepare(McuMachine &mcu, OwnDspMachine &dsp);
DspDmaInputs cycle_inputs();
dsp56300::Registers reset_registers();

inline void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

inline std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected joint MCU/DSP/external refusal");
}

struct Fixture {
  explicit Fixture(Case policy,
                   std::uint64_t at = initial_time,
                   std::vector<std::uint64_t> observations = {collision, collision + 1});
  ~Fixture();
  Fixture(const Fixture &) = delete;
  Fixture &operator=(const Fixture &) = delete;
  DspRuntimeEvents events();
  void attach();
  void step(unsigned instructions);
  void drive();
  McuMachine mcu;
  OwnDspMachine dsp{{.extal_hz = oscillator,
                     .timebase_hz = common,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary}};
  std::vector<Event> trace;
  std::vector<std::uint64_t> ticks;
  std::shared_ptr<const bool> identity = std::make_shared<const bool>(true);
  unsigned cursor = 0, levels = 0;
  const Case order;
  const std::uint64_t start;
  std::function<void()> after_event;
  std::unique_ptr<McuDspRuntime> runtime;
};

void order_cases();
void guard_cases();
void early_events();
} // namespace blofeld::test::joint_events
