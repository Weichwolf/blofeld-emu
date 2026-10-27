// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_sink.hpp"
#include "audio/serial_pcm.hpp"
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"
#include "usb/usbip_runtime.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned oscillator = 12000000, timebase = 540000000, entry = 0x9000;
constexpr unsigned nop = 0xa7, protection = 0x7f0d, security = 0x7f0f, unsecured = 0xfe;
constexpr unsigned byte_mask = 0xff, serial_pins = 0x16;
constexpr unsigned sysclk_ticks = 45, bus_ticks = 90, reset_edges = 195;
constexpr unsigned reset_end = 9225, release_tick = 450, recovery_ticks = 90;
constexpr unsigned program_words = 512, suffix = 20;

void require(const bool value, const std::string &message) {
  if (!value) { throw std::runtime_error(message); }
}

McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

reference::Config config() {
  reference::Config c;
  c.mcu = {.clock = {.oscillator_hz = oscillator,
                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
           .memory_timing = McuRunMemory::unstretched_internal};
  c.pipeline = reference::PipelineTiming{
      .power_on = {3},
      .reset_pad = McuResetPad::high,
      .schedule = {.dsp = {.extal_hz = oscillator,
                           .timebase_hz = timebase,
                           .timing = DspFetchTiming::reference_cycle_order,
                           .execution = DspReferenceExecution::staged_register_boundaries,
                           .pipeline = DspFetchPipeline::automatic_memory,
                           .resets = DspFetchResetTiming::reference_cycle_boundary},
                   .shared_write_order = McuDspTieOrder::dsp_before_mcu,
                   .maximum_dsp_cycles_per_period = 16,
                   .external_write_order = McuExternalWriteOrder::external_before_mcu,
                   .wait_order = McuDspWaitOrder::dsp_before_mcu,
                   .reset_order = McuDspResetOrder::dsp_before_mcu,
                   .initial_reset = McuDspFetchInitialReset{.mode_known = 15,
                                                            .mode_levels = 5,
                                                            .pinit = false,
                                                            .recovery_cycles = 2}},
      .dma_grant = DspDmaGrant::available};
  c.release_ticks = release_tick;
  c.operations = program_words;
  c.select_n = 2;
  c.clock = 4;
  c.mosi = 16;
  c.initial = {{.port = McuPort::b, .drive = {.mask = serial_pins, .levels = 2}},
               {.port = McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  c.dsp_input_order = DspExternalOrder::external_before_dsp;
  c.maximum_dsp_inputs_per_advance = 4;
  c.dsp_initial = {{.level = true}};
  c.dsp_events = {{.ticks = 0, .level = false},
                  {.ticks = sysclk_ticks, .level = true},
                  {.ticks = reset_end + sysclk_ticks, .level = false}};
  return c;
}

struct Fixture {
  explicit Fixture(const reference::Config &config = ::config(), const bool with_rom = true)
      : flash(image()),
        rom(with_rom ? program_words : 0),
        execution{.image = flash,
                  .rom = rom,
                  .specification = BLOFELD_DEFAULT_SLA,
                  .flash_out = {},
                  .trace = &trace,
                  .instructions = program_words},
        board(config, execution) {}

  McuFlashImage flash;
  std::vector<std::uint32_t> rom;
  std::ostringstream trace;
  reference::Execution execution;
  reference::Board board;
};

void steps(reference::Board &board, const unsigned count) {
  for (unsigned i = 0; i < count; ++i) { board.advance(); }
}

void power_on_and_forks() {
  constexpr std::array cuts{0U, 9U, 10U, 11U, 130U, 194U, 195U, 200U};
  for (const auto cut : cuts) {
    Fixture f;
    const auto zero = f.board.status();
    require(zero.common_ticks == 0 && zero.mcu_cycles == 0 && !zero.mcu_state_known &&
                zero.mcu_reset && zero.mcu_reset->edges == 0 && zero.held_reset &&
                zero.dsp_cycles == 0 && zero.dsp_steps == 0 && zero.next_dsp_event == 1,
            "Constructor consumes only explicit initial input, no CPU reset or DSP fetch");
    f.board.initialize();
    steps(f.board, cut);
    const auto saved = f.board.checkpoint();
    const auto prefix = f.board.status();
    f.trace.str({});
    f.trace.clear();
    steps(f.board, suffix);
    const auto expected = f.board.status();
    const auto trace = f.trace.str();
    for (unsigned replay = 0; replay < 2; ++replay) {
      f.board.restore(saved);
      require(f.board.status() == prefix, "Fresh-owner restore retains qualified Board state");
      f.trace.str({});
      f.trace.clear();
      steps(f.board, suffix);
      require(f.board.status() == expected && f.trace.str() == trace,
              "Native pipeline, event cursor and wiring suffix survive repeated owner replacement");
    }
  }
}

void literal_reset_and_pipeline() {
  Fixture f;
  steps(f.board, reset_edges);
  const auto state = f.board.status();
  constexpr unsigned pipeline_fill = 6;
  constexpr unsigned expected_cycles = (reset_end - release_tick - recovery_ticks) / sysclk_ticks;
  require(state.mcu_state_known && state.mcu.pc == entry && state.mcu_cycles == 5 &&
              state.common_ticks == reset_end && state.instructions == 0 &&
              state.dsp_cycles == expected_cycles &&
              state.dsp_steps == expected_cycles - pipeline_fill && state.dsp_units == 0 &&
              !state.held_reset && !state.faulted,
          "195 SYSCLK plus VfPPP, two real DSP recovery periods and seven-stage NOP retirement");
  steps(f.board, suffix);
  const auto after = f.board.status();
  require(after.mcu.pc == entry + suffix && after.mcu_cycles == 5 + suffix &&
              after.common_ticks == reset_end + suffix * bus_ticks &&
              after.dsp_cycles == expected_cycles + 2 * suffix &&
              after.dsp_steps == expected_cycles - pipeline_fill + 2 * suffix,
          "One actual MCU NOP period and two DSP pipeline periods share each 90-tick interval");
}

void midi_partial_bit_restore() {
  auto c = config();
  c.dsp_events.clear();
  c.midi_dsp = reference::DspMidiRoute{.after_ticks = release_tick};
  c.midi_dsp_reference = reference::DspMidiReference{.bit_ticks = bus_ticks, .idle_bits = 1};
  Fixture f(c);
  f.board.initialize();
  constexpr std::array<std::uint8_t, 1> first{0x90};
  constexpr std::array<std::uint8_t, 1> second{0x40};
  f.board.enqueue_midi(first);
  steps(f.board, 15);
  require(f.board.status().midi_dsp_events > 1 && f.board.status().midi_dsp_frames == 0,
          "A real timed serial frame has begun and remains partial during MCU POR");
  const auto saved = f.board.checkpoint();
  std::optional<reference::Status> expected;
  std::string trace;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.board.restore(saved);
    f.trace.str({});
    f.trace.clear();
    f.board.enqueue_midi(second);
    steps(f.board, reset_edges);
    const auto observed = f.board.status();
    require(observed.midi_bytes == 2 && observed.midi_dsp_frames == 2 && observed.midi_queued == 0,
            "Actual timed source completes both queued serial frames under the fetch owner");
    if (expected) {
      require(observed == *expected && f.trace.str() == trace,
              "Fresh-owner restore retains partial MIDI bit, queue, cursor and native suffix");
    } else {
      expected = observed;
      trace = f.trace.str();
    }
  }
}

class Sink final : public PcmFrameSink {
public:
  void accept(const SerialPcmFrame &frame) override {
    throw Fault("Unexpected unconfigured PCM frame channels=" + std::to_string(frame.channels));
  }

  void check() const override {
    if (retired) { throw Fault("Retired own PCM sink"); }
  }

  void invalidate() noexcept override { retired = true; }

  bool retired = false;
};

void external_pcm_lifetime() {
  auto c = config();
  c.pcm =
      reference::PcmRoute{.after_ticks = std::numeric_limits<std::uint64_t>::max(), .format = {}};
  Sink original;
  Sink fresh;
  Fixture f(c);
  const Fixture other(c);
  f.board.attach_pcm_output(original);
  f.board.initialize();
  const auto saved = f.board.checkpoint();
  const auto before = f.board.status();
  bool refused = false;
  try {
    f.board.restore(other.board.checkpoint());
  } catch (const Fault &) { refused = true; }
  require(refused && !original.retired && f.board.status() == before,
          "Rejected foreign restore preserves actual source and external sink");
  f.board.restore(saved);
  require(original.retired && f.board.status() == before,
          "Successful staged replacement invalidates old audio without rewinding its host session");
  f.board.attach_pcm_output(fresh);
  f.board.advance();
  require(!fresh.retired && !f.board.status().faulted,
          "Fresh audio binding can continue after a held-reset clone");
}

void external_usb_epoch() {
  Fixture f;
  steps(f.board, reset_edges);
  const UsbipRuntimeOptions options{
      .tcp = {},
      .location = {.path = "/own/usb/1-1", .busid = "1-1", .bus = 1, .device = 1},
      .after_ticks = std::numeric_limits<std::uint64_t>::max(),
      .period_ticks = 1,
      .probe_timeout_ticks = 100};
  UsbipRuntime old(options);
  f.board.poll_usb(old);
  const auto saved = f.board.checkpoint();
  f.board.advance();
  f.board.restore(saved);
  bool refused = false;
  try {
    f.board.poll_usb(old);
  } catch (const Fault &error) {
    refused = std::string(error.what()).contains("board owner/restore/time changed");
  }
  require(
      refused && old.faulted() && f.board.status().faulted,
      "Actual fetch-owner restore changes external USB revision and refuses stale host binding");
  f.board.restore(saved);
  UsbipRuntime fresh(options);
  f.board.poll_usb(fresh);
  require(!fresh.started() && !fresh.faulted() && !f.board.status().faulted,
          "Fresh USB lifecycle can wait after modeled native state is restored");
}

void missing_rom_and_reset_pad() {
  Fixture empty(config(), false);
  require(empty.board.status().dsp_cycles == 0, "Held DSP reset needs no ROM bytes");
  bool refused = false;
  try {
    steps(empty.board, reset_edges);
  } catch (const Fault &error) {
    refused = std::string(error.what()).find("ff0000") != std::string::npos;
  }
  require(refused && empty.board.status().faulted,
          "First real fetch without ROM remains a visible address-bearing failure");
  auto c = config();
  if (!c.pipeline) { throw Fault("Missing own pipeline fixture contract"); }
  c.pipeline->reset_pad = McuResetPad::unknown;
  Fixture unknown(c);
  refused = false;
  try {
    steps(unknown.board, reset_edges);
  } catch (const Fault &error) {
    refused = std::string(error.what()).find("Unknown MCU RESET pad") != std::string::npos;
  }
  require(refused && unknown.board.status().faulted,
          "MCU reset sampling needs an actual external observation, never an inferred high");
}
}

int main() {
  try {
    power_on_and_forks();
    literal_reset_and_pipeline();
    missing_rom_and_reset_pad();
    midi_partial_bit_restore();
    external_pcm_lifetime();
    external_usb_epoch();
    std::cout
        << "Board POR, automatic native pipeline, fresh-owner replay and strict inputs pass\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
