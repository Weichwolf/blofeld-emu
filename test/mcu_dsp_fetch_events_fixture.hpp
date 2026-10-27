// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_scheduled_run.hpp"
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
inline constexpr std::uint64_t initial_time = 450, epoch = 540, period = 45, collision = 720,
                               finish = 900;
inline constexpr unsigned pins = 0xfff, byte_mask = 0xff, marker_address = 0x1200, marker = 0x5a;

struct Event {
  char actor;
  std::uint64_t time;
  bool operator==(const Event &) const = default;
};

struct Case {
  McuDspTieOrder native;
  McuExternalWriteOrder external;
  DspExternalOrder peers;
  unsigned sample;
  std::array<char, 3> order;
};

McuFlashImage image();
McuRunContract mcu_contract();
McuDspFetchInputs settings(Case order, std::uint64_t start);
DspMachine &prepare(McuMachine &mcu, DspMachine &dsp);

inline std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected joint MCU/DSP/external refusal");
}

struct Fixture {
  explicit Fixture(Case policy,
                   std::uint64_t at = epoch,
                   std::vector<std::uint64_t> observations = {collision, collision + 1});
  ~Fixture();
  Fixture(const Fixture &) = delete;
  Fixture &operator=(const Fixture &) = delete;
  DspFetchExternalEvents events();
  void attach();
  void step(unsigned instructions);
  void drive();
  McuMachine mcu;
  DspMachine dsp;
  std::vector<Event> trace;
  std::vector<std::uint64_t> ticks;
  std::shared_ptr<const DspFetchEventIdentity> identity =
      std::make_shared<const DspFetchEventIdentity>();
  unsigned cursor = 0, levels = 0;
  const Case order;
  const std::uint64_t start;
  std::function<void()> after_event;
  McuDspFetchSchedule schedule;
};

void order_cases();
void guard_cases();
void early_events();
} // namespace blofeld::test::joint_events
