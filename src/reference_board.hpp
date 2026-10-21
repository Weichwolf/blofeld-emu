// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_esai.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_clocked_midi.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_sci.hpp"
#include "blofeld/pcm_recording.hpp"
#include "blofeld/serial_pcm.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iosfwd>
#include <memory>
#include <optional>
#include <span>
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

enum class DspInputKind : std::uint8_t { tio, esai };

struct DspInput {
  std::uint64_t ticks = 0;
  DspInputKind kind = DspInputKind::tio;
  unsigned index = 0;
  std::optional<bool> level = std::nullopt;
  bool synchronized = false;
  DspEsaiInputs esai{};
};

struct DspEsaiClock {
  std::uint64_t start_ticks = 0;
  std::uint64_t half_period_ticks = 0;
  std::uint64_t edges = 0;
  unsigned index = 0;
};

struct PcmRoute {
  unsigned instance = 0;
  unsigned lane = 0;
  std::uint64_t after_ticks = 0;
  SerialPcmFormat format;
};

struct MidiRoute {
  std::uint32_t bus_hz = 0;
  SciReceivePin pin = SciReceivePin::pr2;
  std::uint64_t after_ticks = 0;
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
  DspExternalOrder dsp_input_order = DspExternalOrder::unqualified;
  unsigned maximum_dsp_inputs_per_advance = 0;
  std::vector<DspInput> dsp_initial;
  std::vector<DspInput> dsp_events;
  std::vector<DspEsaiClock> dsp_clocks;
  std::optional<PcmRoute> pcm;
  std::optional<MidiRoute> midi;
  std::optional<SciReceiveReference> midi_reference;
};

void validate_returns(const Config &config);
void validate_midi(const Config &config);

[[nodiscard]] Config load_config(const std::filesystem::path &path);
[[nodiscard]] std::vector<std::uint32_t> load_rom(const std::filesystem::path &path);

struct AudioOutput {
  std::filesystem::path path;
  std::uint32_t sample_rate = 0;
  std::size_t maximum_frames = 0;
};

struct MidiInputWindow {
  std::uint64_t ticks = 0;
  std::size_t available = 0;
  bool started = false;
};

struct Execution {
  const McuFlashImage &image;
  const std::vector<std::uint32_t> &rom;
  std::filesystem::path specification;
  std::filesystem::path flash_out;
  std::ostream *trace = nullptr;
  std::uint64_t instructions = 0;
  std::optional<AudioOutput> audio = std::nullopt;
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
  std::size_t next_dsp_event = 0;
  std::array<std::uint64_t, 2> next_esai_edge{};
  std::array<SerialPcmPins, 2> esai_inputs{};
  std::optional<SerialPcmReceiver> pcm;
  std::optional<SerialPcmStamp> pcm_stamp;
  std::optional<SerialPcmFrame> last_pcm;
  PcmRecording recording;
  std::size_t midi_queued = 0;
  std::uint64_t midi_bytes = 0;
  bool midi_started = false;
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
  int run(const std::function<void(Board &)> &input = {});
  void initialize();
  void advance();
  [[nodiscard]] bool finished() const;
  [[nodiscard]] Status status() const;
  [[nodiscard]] MidiInputWindow midi_window() const;
  void enqueue_midi(std::span<const std::uint8_t> bytes);
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
    std::size_t next_dsp_event = 0;
    std::array<std::uint64_t, 2> next_esai_edge{};
    std::array<SerialPcmPins, 2> esai_inputs{};
    std::optional<SerialPcmReceiver> pcm;
    std::optional<SerialPcmStamp> pcm_stamp;
    std::optional<SerialPcmFrame> last_pcm;
    PcmRecording recording;
    std::optional<McuClockedMidiInput> midi;
    std::uint64_t midi_bytes = 0;
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
  [[nodiscard]] Outcome finish_branch(const std::function<void(Board &)> &input = {});
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

  struct DspOffer {
    DspInput input;
    std::optional<unsigned> clock_index = std::nullopt;
    bool collision = false;
  };

  [[nodiscard]] std::optional<DspOffer> next_dsp_input() const;
  void observe_dsp_input(std::uint64_t at);
  void apply_dsp(const DspInput &input);
  void trace_dsp_input(const DspInput &input, std::uint64_t at);
  void observe_pcm(std::uint64_t at, std::string_view source);
  [[nodiscard]] SerialPcmPins pcm_pins() const;
  void trace_pcm(SerialPcmStamp stamp,
                 SerialPcmPins pins,
                 std::string_view source,
                 const std::optional<SerialPcmFrame> &frame) const;
  [[nodiscard]] DspExternalEvents dsp_inputs();
  void boundary();
  void start_midi();
  void observe_midi(ExecutionContext context);
  void synchronize_midi();
  void step();
  void release();
  void on_cycle(Hc12BusCycle cycle);
  void on_write(const BusEvent &event);
  void before_dsp(const DspScheduledStep &step) const;
  [[nodiscard]] DspCompletionObserver return_observer();
  void report(std::ostream &stream) const;
  void persist() const;
  void save_audio() const;
  const Config config_;
  const Execution execution_;
  std::shared_ptr<const Identity> identity_;
  std::unique_ptr<Owners> native_;
  Control control_;
  bool active_ = false;
};
}
