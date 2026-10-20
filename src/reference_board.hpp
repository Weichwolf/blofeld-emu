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
  std::vector<Input> initial;
  std::vector<Input> events;
};

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

// Explicit reference experiment, not a verified physical Desktop/Keyboard profile.
class Board {
public:
  Board(const Config &config, const Execution &execution);
  int run();

private:
  [[nodiscard]] std::uint64_t ticks() const;
  [[nodiscard]] SpiInputs sample() const;
  void check_signals() const;
  void transition(SpiInputs pins);
  void update_loopback();
  void apply(const Input &input);
  void boundary();
  void step();
  void release();
  void on_cycle(Hc12BusCycle cycle);
  void on_write(const BusEvent &event);
  void before_dsp(const DspScheduledStep &step) const;
  void report(std::ostream &stream) const;
  void persist() const;
  const Config &config_;
  const Execution &execution_;
  McuMachine mcu_;
  DspMachine dsp_;
  std::unique_ptr<McuDspSchedule> schedule_;
  SpiInputs previous_;
  ExecutionContext context_;
  std::uint64_t scale_ = 0;
  std::uint64_t instructions_ = 0;
  std::uint64_t operations_ = 0;
  std::uint64_t interrupts_ = 0;
  std::uint64_t writes_ = 0;
  std::uint64_t frames_ = 0;
  std::uint64_t ordered_pairs_ = 0;
  std::size_t next_event_ = 0;
};
}
