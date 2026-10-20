// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"

#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::reference {
enum class PadOrder : std::uint8_t { reject, clock_before_select, select_before_clock };
enum class InputKind : std::uint8_t { drive, phy_window, usb_decoder, timer_inputs };

struct Input {
  std::uint64_t ticks = 0;
  InputKind kind = InputKind::drive;
  McuPort port = McuPort::a;
  PinDrive drive;
  unsigned value = 0;
};

struct Loopback {
  McuPort source = McuPort::a;
  std::uint8_t source_mask = 0;
  McuPort destination = McuPort::a;
  std::uint8_t destination_mask = 0;
};

struct ReturnRoute {
  McuPort destination = McuPort::a;
  std::uint8_t mask = 0;
  bool inverted = false;
};

struct ReplayPlan {
  std::uint64_t operation = 0;
  unsigned replays = 0;
};

struct Config {
  McuRunContract mcu;
  DspRunContract dsp;
  McuDspScheduleInputs schedule;
  std::uint64_t release_ticks = 0;
  std::uint64_t operations = 0;
  bool pinit = false;
  std::uint8_t select_n = 0;
  std::uint8_t clock = 0;
  std::uint8_t mosi = 0;
  PadOrder pad_order = PadOrder::reject;
  std::optional<Loopback> loopback;
  std::optional<ReturnRoute> hreq;
  std::optional<ReturnRoute> miso;
  std::optional<ReplayPlan> checkpoint;
  std::vector<Input> initial;
  std::vector<Input> events;
};

void validate_returns(const Config &config);

[[nodiscard]] Config load_config(const std::filesystem::path &path);
[[nodiscard]] std::vector<std::uint32_t> load_rom(const std::filesystem::path &path);

struct Execution {
  const McuFlashImage &image;
  const std::vector<std::uint32_t> &rom;
  std::filesystem::path specification;
  std::filesystem::path flash_out;
  std::ostream *trace = nullptr;
  std::uint64_t instructions = 0;
};

class Board;

// Immutable, owner-bound trusted memory snapshot; not a portable state file.
class Checkpoint {
public:
  Checkpoint(const Checkpoint &) = default;
  Checkpoint &operator=(const Checkpoint &) = default;

private:
  friend class Board;
  struct Storage;
  explicit Checkpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

struct Status {
  CpuState mcu;
  DspDebugState dsp;
  std::vector<std::uint8_t> cold_nvm;
  std::uint64_t common_ticks = 0;
  std::uint64_t mcu_cycles = 0;
  std::uint64_t mcu_ticks = 0;
  std::uint64_t oscillator_cycles = 0;
  std::uint64_t idle_cycles = 0;
  std::uint64_t dsp_ticks = 0;
  std::uint64_t dsp_units = 0;
  std::uint64_t dsp_steps = 0;
  std::uint64_t instructions = 0;
  std::uint64_t operations = 0;
  std::uint64_t interrupts = 0;
  std::uint64_t writes = 0;
  std::uint64_t frames = 0;
  std::uint64_t ties = 0;
  std::uint64_t ordered_pairs = 0;
  std::size_t next_event = 0;
  bool started = false;
  bool waiting = false;
  bool held_reset = true;
  bool faulted = false;
  bool operator==(const Status &other) const;
};

// Explicit reference experiment, not a verified physical Desktop/Keyboard profile.
class Board {
public:
  Board(const Config &config, const Execution &execution);
  Board(const Board &) = delete;
  Board &operator=(const Board &) = delete;
  Board(Board &&) = delete;
  Board &operator=(Board &&) = delete;
  int run();
  void initialize();
  void advance();
  [[nodiscard]] bool finished() const;
  [[nodiscard]] Status status() const;
  [[nodiscard]] Checkpoint checkpoint() const;
  // Strong exception guarantee; restores native owners and all wiring together.
  void restore(const Checkpoint &checkpoint);

private:
  friend class Checkpoint;

  struct Identity {};

  struct Owners {
    Owners(const Config &config, const Execution &execution);
    McuMachine mcu;
    DspMachine dsp;
    std::unique_ptr<McuDspSchedule> schedule;
  };
  class Busy;

  struct Control {
    SpiInputs previous;
    ExecutionContext context;
    std::optional<std::uint64_t> return_boundary;
    std::optional<std::uint64_t> release_epoch;
    std::optional<ShiOutput> hreq_observed;
    std::optional<ShiOutput> miso_observed;
    std::uint64_t scale = 0;
    std::uint64_t instructions = 0;
    std::uint64_t operations = 0;
    std::uint64_t interrupts = 0;
    std::uint64_t writes = 0;
    std::uint64_t frames = 0;
    std::uint64_t ordered_pairs = 0;
    std::size_t next_event = 0;
    bool started = false;
    bool faulted = false;
  };

  struct Outcome {
    std::optional<std::string> fault;
    Status state;
    bool operator==(const Outcome &) const = default;
  };

  void require_idle() const;
  void require_healthy() const;
  void attach(Owners &owners);
  [[nodiscard]] Outcome finish_branch();
  [[nodiscard]] Outcome run_replays();
  void replay_marker(std::string_view phase, unsigned index);
  void flush_trace();
  [[nodiscard]] std::uint64_t ticks() const;
  [[nodiscard]] SpiInputs sample() const;
  void check_signals() const;
  void transition(SpiInputs pins);
  void update_loopback();
  void update_wiring();
  void update_return(const std::optional<ReturnRoute> &route,
                     ShiOutput output,
                     std::optional<ShiOutput> &previous,
                     std::string_view name);
  void apply(const Input &input);
  void boundary();
  void step();
  void release();
  void on_cycle(Hc12BusCycle cycle);
  void on_write(const BusEvent &event);
  void before_dsp(const DspScheduledStep &step) const;
  [[nodiscard]] DspCompletionObserver return_observer();
  void report(std::ostream &stream) const;
  void persist() const;
  const Config config_;
  const Execution execution_;
  std::shared_ptr<const Identity> identity_;
  std::unique_ptr<Owners> native_;
  Control control_;
  bool active_ = false;
};
}
