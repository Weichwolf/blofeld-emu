// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/pcm_recording.hpp"
#include "audio/pcm_sink.hpp"
#include "audio/serial_pcm.hpp"
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "board/reference_square_clock.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp/dsp_shi.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_clocked_midi.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_sci.hpp"
#include "midi/midi_timed.hpp"
#include "storage/flash_image.hpp"

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

namespace blofeld {
class UsbipRuntime;
class VirtualDeviceState;
}

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

// Declared laboratory routing: bits0/1 select ESAI0/1 HCKT; bit2 selects DAX ACI.
// Matching divisors do not establish a physical shared clock net.
struct AudioClockRoute {
  std::uint64_t frequency_hz = 0;
  std::uint64_t epoch_ticks = 0;
  unsigned destinations = 0;
  bool initial_level = false;
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

// Free-running external wire in the common timebase; independent of MCU clocks.
struct SciMidiRoute {
  SciReceivePin pin = SciReceivePin::pr2;
  std::uint64_t after_ticks = 0;
};

struct DspMidiRoute {
  std::uint64_t after_ticks = 0;
  unsigned index = 0;
  bool synchronized = false;
};

struct DspMidiReference {
  std::uint64_t bit_ticks = 0;
  unsigned idle_bits = 0;
};

// Explicit development timing, not a hardware-verified device profile.
struct PipelineTiming {
  McuPowerOnInputs power_on;
  McuResetPad reset_pad = McuResetPad::unknown;
  McuDspFetchInputs schedule;
  DspDmaGrant dma_grant = DspDmaGrant::unknown;
};

struct Config {
  McuRunContract mcu;
  DspRunContract dsp;
  McuDspScheduleInputs schedule;
  std::optional<PipelineTiming> pipeline;
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
  std::optional<AudioClockRoute> audio_clock;
  std::optional<PcmRoute> pcm;
  std::optional<MidiRoute> midi;
  std::optional<SciMidiRoute> midi_sci_timed;
  std::optional<SciReceiveReference> midi_reference;
  std::optional<DspMidiRoute> midi_dsp;
  std::optional<DspMidiReference> midi_dsp_reference;
};

void validate_returns(const Config &config);
void validate_midi(const Config &config);
void validate_dsp_midi(const Config &config);

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
  bool configured = false;
  // External lifetime evidence, never part of modeled checkpoint control.
  std::shared_ptr<const void> owner;
  std::uint64_t restore_revision = 0;
};

struct Execution {
  const McuFlashImage &image;
  const std::vector<std::uint32_t> &rom;
  std::filesystem::path specification;
  std::filesystem::path flash_out;
  std::ostream *trace = nullptr;
  std::uint64_t instructions = 0;
  std::optional<AudioOutput> audio = std::nullopt;
  // External persistent unit owner; never rewound with modeled checkpoints.
  std::shared_ptr<VirtualDeviceState> device_state = nullptr;
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
  bool mcu_state_known = true;
  std::optional<McuResetState> mcu_reset;
  std::vector<std::uint8_t> cold_nvm;
  std::uint64_t common_ticks = 0;
  std::uint64_t mcu_cycles = 0;
  std::uint64_t mcu_ticks = 0;
  std::uint64_t oscillator_cycles = 0;
  std::uint64_t idle_cycles = 0;
  std::uint64_t dsp_ticks = 0;
  std::uint64_t dsp_units = 0;
  std::uint64_t dsp_steps = 0;
  std::uint64_t dsp_cycles = 0;
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
  std::optional<SquareClock> audio_clock;
  bool audio_clock_started = false;
  std::array<SerialPcmPins, 2> esai_inputs{};
  std::optional<SerialPcmReceiver> pcm;
  std::optional<SerialPcmStamp> pcm_stamp;
  std::optional<SerialPcmFrame> last_pcm;
  PcmRecording recording;
  std::size_t midi_queued = 0;
  std::uint64_t midi_bytes = 0;
  bool midi_started = false;
  std::uint64_t midi_sci_events = 0;
  std::uint64_t midi_sci_frames = 0;
  std::uint64_t midi_dsp_events = 0;
  std::uint64_t midi_dsp_frames = 0;
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
  // Explicit external binding; a restored model requires a fresh sink session.
  void attach_pcm_output(PcmFrameSink &sink);
  // Non-real-time input admission at an inactive native MCU operation boundary.
  void poll_usb(UsbipRuntime &runtime);
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
    std::unique_ptr<McuDspFetchSchedule> fetch;
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
    std::uint64_t dsp_retirements = 0;
    std::uint64_t ordered_pairs = 0;
    std::size_t next_event = 0;
    std::size_t next_dsp_event = 0;
    std::array<std::uint64_t, 2> next_esai_edge{};
    std::optional<SquareClock> audio_clock;
    bool audio_clock_started = false;
    std::array<SerialPcmPins, 2> esai_inputs{};
    std::optional<SerialPcmReceiver> pcm;
    std::optional<SerialPcmStamp> pcm_stamp;
    std::optional<SerialPcmFrame> last_pcm;
    PcmRecording recording;
    std::optional<McuClockedMidiInput> midi;
    std::optional<MidiTimedInput> midi_dsp;
    std::optional<MidiTimedInput> midi_sci;
    std::uint64_t midi_bytes = 0;
    bool started = false;
    bool dsp_initial_pending = false;
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
  void prepare_dsp();
  void connect_pipeline();
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
    bool midi = false;
    bool sci_midi = false;
    bool audio_clock = false;
  };

  [[nodiscard]] std::optional<DspOffer> next_dsp_input() const;
  static void select_dsp_input(std::optional<DspOffer> &selected, DspOffer candidate);
  void observe_dsp_input(std::uint64_t at);
  void prepare_audio_clock();
  void observe_audio_clock(std::uint64_t at);
  void trace_audio_clock(const AudioClockRoute &route,
                         SquareClockLevel event,
                         bool baseline,
                         std::uint64_t core_hz);
  [[nodiscard]] std::uint64_t audio_core_hz() const;
  void apply_dsp(const DspInput &input);
  void trace_dsp_input(const DspInput &input, std::uint64_t at);
  void trace_dsp_midi(std::uint64_t at);
  void observe_sci_midi(std::uint64_t at);
  void observe_pcm(std::uint64_t at, std::string_view source);
  void check_pcm_output() const;
  void retire_pcm_output() noexcept;
  void mark_fault() noexcept;
  [[nodiscard]] SerialPcmPins pcm_pins() const;
  void trace_pcm(SerialPcmStamp stamp,
                 SerialPcmPins pins,
                 std::string_view source,
                 const std::optional<SerialPcmFrame> &frame) const;
  [[nodiscard]] DspExternalEvents dsp_inputs();
  [[nodiscard]] DspFetchExternalEvents fetch_events();
  [[nodiscard]] McuDspFetchSchedule::CycleInputs fetch_inputs() const;
  [[nodiscard]] McuDspFetchSchedule::Observer fetch_observer();
  [[nodiscard]] std::uint64_t common_hz() const;
  [[nodiscard]] std::uint64_t dsp_ticks() const;
  [[nodiscard]] std::uint64_t dsp_steps() const;
  [[nodiscard]] std::uint64_t schedule_ticks() const;
  [[nodiscard]] std::uint64_t schedule_ties() const;
  [[nodiscard]] bool dsp_reset_active() const;
  [[nodiscard]] bool timing_faulted() const;
  void observe_spi_write(const BusEvent &event);
  void boundary();
  void start_midi();
  void observe_midi(ExecutionContext context);
  void synchronize_midi();
  [[nodiscard]] std::size_t midi_queue_size() const;
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
  const std::shared_ptr<const DspFetchEventIdentity> input_identity_ =
      std::make_shared<const DspFetchEventIdentity>();
  std::unique_ptr<Owners> native_;
  Control control_;
  // External transport epochs must not rewind with a modeled-state restore.
  std::uint64_t usb_restore_revision_ = 0;
  PcmFrameSink *pcm_output_ = nullptr;
  bool active_ = false;
};
}
