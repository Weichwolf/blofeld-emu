// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/pcm_sink.hpp"
#include "audio/pcm_words.hpp"
#include "audio/serial_pcm.hpp"
#include "common/bus.hpp"
#include "display/lcd_image.hpp"
#include "display/st7565_parallel.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp56300/core.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_memory.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_sci.hpp"
#include "runtime/instrument_square_clock.hpp"
#include "runtime/mcu_dsp_gpio_peer.hpp"
#include "runtime/mcu_dsp_runtime.hpp"
#include "runtime/nand_peer.hpp"
#include "storage/flash_image.hpp"
#include "storage/nand.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_image.hpp"
#include "usb/usb_host_link.hpp"
#include "usb/usb_transfer.hpp"

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
class UsbMidiHost;
class VirtualDeviceState;
}

namespace blofeld::runtime {
enum class InputKind : std::uint8_t { drive, phy_window, usb_decoder, timer_inputs };

struct Input {
  std::uint64_t ticks = 0;
  InputKind kind = InputKind::drive;
  McuPort port = McuPort::a;
  PinDrive drive{};
  unsigned value = 0;
};

struct Loopback {
  McuPort source = McuPort::a;
  std::uint8_t source_mask = 0;
  McuPort destination = McuPort::a;
  std::uint8_t destination_mask = 0;
};

// Declared laboratory routing: bits0/1 select ESAI0/1 HCKT; bit2 selects DAX ACI.
// Matching divisors do not establish a physical shared clock net.
enum class AudioClockScheduling : std::uint8_t { reference_edges, transmitter_events };

struct AudioClockRoute {
  std::uint64_t frequency_hz = 0;
  std::uint64_t epoch_ticks = 0;
  unsigned destinations = 0;
  bool initial_level = false;
  AudioClockScheduling scheduling = AudioClockScheduling::reference_edges;
};

struct PcmRoute {
  unsigned instance = 0;
  unsigned lane = 0;
  std::uint64_t after_ticks = 0;
  SerialPcmFormat format;
};

// Explicit development timing, not a hardware-verified device profile.
struct PowerOnTiming {
  McuPowerOnInputs inputs;
  McuResetPad reset_pad = McuResetPad::unknown;
};

struct NandConnection {
  NandReadContinuation continuation = NandReadContinuation::unqualified;
  // Live backing requires an existing source file and an empty in-memory image.
  // Parsed file provenance; typed in-memory fixtures need no input filename.
  std::optional<std::filesystem::path> source_path = std::nullopt;
  // Explicit held active-low input; absence does not imply write permission.
  std::optional<bool> write_protect_n = std::nullopt;
  // Explicit simulated cell physics; no block validity or command result is inferred.
  NandCellProfile cell_profile = {};
};

// Declared peripheral POR source; first native pad observation at/after release wins.
struct LcdConnection {
  std::uint64_t reset_release_ticks = 0;
};

struct Config {
  McuRunContract mcu;
  DspRunClockInputs dsp_clock;
  McuDspRuntimeInputs dsp;
  DspDmaGrant dma_grant = DspDmaGrant::unknown;
  std::optional<PowerOnTiming> power_on;
  std::optional<NandConnection> nand;
  std::optional<McuDspGpioWiring> gpio_peer;
  bool mcu_u0_release = false;
  std::optional<LcdConnection> lcd_parallel;
  std::optional<PinBiases> port_b_pulls;
  std::uint64_t release_ticks = 0;
  std::uint8_t select_n = 0;
  std::uint8_t clock = 0;
  std::uint8_t mosi = 0;
  std::optional<Loopback> loopback;
  std::vector<Input> initial;
  std::vector<Input> events;
  std::optional<AudioClockRoute> audio_clock;
  std::optional<PcmRoute> pcm;
};

[[nodiscard]] DspRunClockInputs dsp_clock_inputs(const Config &config);

[[nodiscard]] std::vector<std::uint32_t> load_rom(const std::filesystem::path &path);

struct Execution {
  const McuFlashImage &image;
  const std::vector<std::uint32_t> &rom;
  std::filesystem::path specification;
  std::ostream *trace = nullptr;
  // Exclusive persistent unit owner.
  std::shared_ptr<VirtualDeviceState> device_state = nullptr;
};

class Instrument;

struct Status {
  CpuState mcu;
  dsp56300::Registers dsp;
  bool mcu_state_known = false;
  std::uint64_t common_ticks = 0;
  std::uint64_t mcu_cycles = 0;
  std::uint64_t dsp_cycles = 0;
  std::uint64_t dsp_steps = 0;
  std::uint64_t instructions = 0;
  std::uint64_t interrupts = 0;
  bool waiting = false;
  bool held_reset = true;
  bool faulted = false;
};

// Unmodified firmware execution, device I/O and host transport integration.
class Instrument {
public:
  Instrument(Config config, Execution execution);
  Instrument(const Instrument &) = delete;
  Instrument &operator=(const Instrument &) = delete;
  Instrument(Instrument &&) = delete;
  Instrument &operator=(Instrument &&) = delete;
  void initialize();
  void advance();
  [[nodiscard]] Status status() const;

  [[nodiscard]] std::uint64_t common_ticks() const {
    require_idle();
    return ticks();
  }

  // Read-only storage inspection remains available after a runtime fault.
  [[nodiscard]] McuMemoryByte inspect_mcu(std::uint16_t address) const;
  [[nodiscard]] std::optional<std::uint32_t> inspect_dsp(DspAddress address) const;
  // Explicit external binding; a restored model requires a fresh sink session.
  void attach_pcm_output(PcmFrameSink &sink);
  // Non-real-time input admission at an inactive native MCU operation boundary.
  void poll_usb(UsbipRuntime &runtime);
  [[nodiscard]] std::vector<UsbTransferCompletion> poll_usb(UsbMidiHost &host);
  // Read-only host capture; unknown pixels retain their knownness.
  [[nodiscard]] std::optional<LcdImage> display_image(LcdPanel panel) const;

private:
  struct Identity {};

  struct Owners {
    Owners(const Config &config, const Execution &execution);
    McuMachine mcu;
    DspMachine dsp;
    std::unique_ptr<NandPeer> nand;
    std::unique_ptr<McuDspGpioPeer> gpio_peer;
    std::unique_ptr<McuDspRuntime> runtime;
  };
  class Busy;

  struct Control {
    PinDrive b_external;
    std::optional<St7565Parallel> lcd;
    std::uint64_t lcd_epoch = 0;
    SpiInputs previous;
    ExecutionContext context;
    std::optional<std::uint64_t> return_boundary;
    std::optional<std::uint64_t> release_epoch;
    std::uint64_t scale = 0;
    std::uint64_t instructions = 0;
    std::uint64_t interrupts = 0;
    std::uint64_t io_revision = 0;
    std::size_t next_event = 0;
    std::size_t next_dsp_event = 0;
    std::optional<SquareClock> audio_clock;
    bool audio_clock_started = false;
    std::array<SerialPcmPins, 2> esai_inputs{};
    std::optional<PcmWords> pcm;
    std::optional<UsbHostLink> usb_link;
    const void *usb_owner = nullptr;
    bool started = false;
    bool dsp_initial_pending = false;
    bool u0_asserted = false;
    bool faulted = false;
  };

  void require_idle() const;
  [[nodiscard]] bool prepare_usb(const void *owner);
  void require_healthy() const;
  void attach(Owners &owners);
  void prepare_dsp();
  void prepare_lcd();
  void attach_b_pulls(Owners &owners);
  void set_b_drive(PinDrive drive);
  [[nodiscard]] GpioPadSample sample_b_pulls(const Owners &owners, std::uint32_t mask) const;
  [[nodiscard]] bool b_direction_write(unsigned address, unsigned size = 1) const;
  void observe_b_direction();
  void observe_lcd(unsigned address);
  void observe_lcd_epoch();
  [[nodiscard]] LcdHostPins lcd_pins() const;
  void connect_runtime();
  [[nodiscard]] std::uint64_t ticks() const;
  [[nodiscard]] SpiInputs sample() const;
  void check_signals() const;
  void transition(SpiInputs pins);
  void update_loopback();
  void publish_io();
  [[nodiscard]] bool gpio_peer_write(unsigned address, unsigned size = 1) const;
  void update_u0_drive();
  void observe_u0_release();
  [[nodiscard]] bool u0_release_write(unsigned address, unsigned size = 1) const;
  void apply(const Input &input);

  void prepare_audio_clock();
  void observe_audio_clock(std::uint64_t at);
  [[nodiscard]] bool transmitter_clock_events() const;
  [[nodiscard]] bool batch_audio_clock() const;
  [[nodiscard]] std::optional<std::uint64_t> next_audio_clock_event() const;
  void synchronize_audio_clock(std::uint64_t at);
  void advance_audio_clock_span(std::uint64_t at);
  void trace_audio_clock(const AudioClockRoute &route, SquareClockLevel event, bool baseline) const;
  [[nodiscard]] std::uint64_t audio_core_hz() const;
  void observe_held_dsp_tio() const;
  void observe_pcm(std::uint64_t at, std::string_view source);
  void check_pcm_output() const;
  void retire_pcm_output() noexcept;
  void mark_fault() noexcept;
  void trace_pcm(const SerialPcmFrame &frame, std::string_view source) const;
  [[nodiscard]] DspRuntimeEvents runtime_events();
  [[nodiscard]] DspRuntimePeripheralEvents peripheral_events(Owners &owners);
  [[nodiscard]] McuDspRuntime::CycleInputs runtime_inputs();
  [[nodiscard]] McuDspRuntime::Observer runtime_observer();
  [[nodiscard]] std::uint64_t common_hz() const;
  [[nodiscard]] std::uint64_t dsp_ticks() const;
  [[nodiscard]] std::uint64_t dsp_steps() const;
  [[nodiscard]] std::uint64_t schedule_ticks() const;
  [[nodiscard]] bool dsp_reset_active() const;
  [[nodiscard]] bool timing_faulted() const;
  void observe_spi_write(const BusEvent &event);
  void boundary();
  void execute_until_event();
  void release(std::uint64_t at);
  void on_span(McuSpan span);
  void on_write(const BusEvent &event);
  const Config config_;
  const Execution execution_;
  std::shared_ptr<const Identity> identity_;
  const std::shared_ptr<const bool> input_identity_ = std::make_shared<const bool>(true);
  const std::shared_ptr<const PeripheralEventIdentity> clock_identity_ =
      std::make_shared<const PeripheralEventIdentity>();
  std::unique_ptr<Owners> native_;
  Control control_;
  PcmFrameSink *pcm_output_ = nullptr;
  bool active_ = false;
};
}
