// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_machine.hpp"
#include "coupled_boot_fixture.hpp"
#include "original_owned_clock_fixture.hpp"

#include <cstdint>
#include <filesystem>
#include <map>
#include <ostream>
#include <string>

namespace blofeld::test::scheduled {
inline constexpr unsigned common_hz = 540000000;
inline constexpr unsigned oscillator_hz = 12000000;
inline constexpr unsigned ready_delay_ticks = 5400; // Declared 10 us observation, not measured.

enum class TieOrder : std::uint8_t { unqualified, dsp_before_mcu, mcu_before_dsp };

struct Experiment {
  coupled::PadOrder pads = coupled::PadOrder::unresolved;
  TieOrder tie = TieOrder::unqualified;
  unsigned phase_ticks = 0;
  bool break_mosi = false;
};

struct DspRow {
  std::string name;
  unsigned opcode = 0;
  unsigned extension = 0;
  unsigned length = 0;
  unsigned units = 0;
  unsigned pctl = 0;
  unsigned count = 0;
};

struct LinkCheckpoint {
  DspScheduleCheckpoint schedule;
  SpiInputs previous;
  std::map<unsigned, DspRow> census;
  std::uint64_t end_tick = 0;
  unsigned writes = 0;
  unsigned frames = 0;
  unsigned ordered_pairs = 0;
  unsigned ties = 0;
  bool injected = false;
};

struct Progress {
  owned_clock::Census census;
  owned_clock::FlashPhases phases;
  owned_clock::Interrupts interrupts;
  std::uint64_t ready_tick = 0;
};

void step(McuMachine &machine, Progress &progress);
void require(bool condition, const std::string &message);
[[nodiscard]] std::uint64_t common_ticks(const McuMachine &machine);
void record_clock_write(const McuMachine &machine, const BusEvent &event, std::ostream &trace);

class ClockedLink {
public:
  ClockedLink(McuMachine &mcu,
              DspMachine &dsp,
              owned_clock::FlashPhases &phases,
              std::ostream &trace,
              Experiment experiment,
              std::uint64_t release_tick);
  void start();
  void attach();
  void detach();
  void after_mcu_step();
  [[nodiscard]] LinkCheckpoint checkpoint() const;
  void restore(const LinkCheckpoint &checkpoint);
  void write_census(const std::filesystem::path &path) const;

  [[nodiscard]] unsigned frames() const { return frames_; }

  [[nodiscard]] unsigned writes() const { return writes_; }

  [[nodiscard]] unsigned ties() const { return ties_; }

  [[nodiscard]] std::uint64_t steps() const { return scheduler_.completed_steps(); }

private:
  [[nodiscard]] SpiInputs sample() const;
  void on_cycle(Hc12BusCycle cycle);
  void on_bus_event(const BusEvent &event);
  void advance(std::uint64_t end, DspScheduleBoundary boundary);
  void on_dsp_step(const DspScheduledStep &step);
  void deliver(SpiInputs pins);
  void transition(SpiInputs pins);
  McuMachine &mcu_;
  DspMachine &dsp_;
  owned_clock::FlashPhases &phases_;
  std::ostream &trace_;
  Experiment experiment_;
  std::uint64_t release_tick_;
  DspScheduledRun scheduler_;
  SpiInputs previous_;
  std::map<unsigned, DspRow> census_;
  std::uint64_t end_tick_;
  unsigned writes_ = 0;
  unsigned frames_ = 0;
  unsigned ordered_pairs_ = 0;
  unsigned ties_ = 0;
  bool offered_ = false;
  bool injected_ = false;
  ExecutionContext context_;
  Hc12BusCycle cycle_;
};
}
