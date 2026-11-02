// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_midi_output.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "midi/midi_serial_receiver.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned oscillator = 12000000;
constexpr unsigned common_hz = 540000000;
constexpr unsigned entry = 0x9000;
constexpr unsigned flash_base = 0x8000;
constexpr unsigned protection = 0x7f0d;
constexpr unsigned security = 0x7f0f;
constexpr unsigned unsecured = 0xfe;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned nop = 0xa7;
constexpr unsigned cut_operations = 2500;
constexpr unsigned suffix_operations = 2500;
constexpr unsigned maximum_operations = 6000;
constexpr std::uint8_t pattern = 0x55;
constexpr std::uint64_t sampled_frame_span = 164160;
// Authored LDS/LDD/STD/LDAA/STAA/status-read/data-write/WAI/BRA. SCI divider12
// at the reset6MHz IPBus gives31250baud. The data byte is deliberately55.
constexpr std::array<std::uint8_t, 25> code{0xcf,    0x20, 0,    0xcc, 0,    12,   0x7c, 0,    0xc8,
                                            0x86,    8,    0x7a, 0,    0xcb, 0xb6, 0,    0xcc, 0x86,
                                            pattern, 0x7a, 0,    0xcf, 0x3e, 0x20, 0xfd};
constexpr std::array<std::uint8_t, 5> small_route{0x86, 1, 0x7a, 2, 0x57};

void require(const bool condition, const char *reason) {
  if (!condition) { throw Fault(reason); }
}

void rejects(const auto &operation, const std::string_view expected) {
  bool rejected = false;
  try {
    operation();
  } catch (const Fault &error) { rejected = std::string_view(error.what()).contains(expected); }
  require(rejected, "Required Board MIDI output rejection was absent or had the wrong context");
}

McuFlashImage image(const bool small) {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  auto *destination = bytes.begin() + (entry - flash_base);
  if (small) { destination = std::ranges::copy(small_route, destination).out; }
  std::ranges::copy(code, destination);
  return McuFlashImage::from_known(bytes);
}

reference::Config config(const std::optional<SciTransmitPin> output) {
  reference::Config c;
  c.mcu = {.clock = {.oscillator_hz = oscillator,
                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
           .memory_timing = McuRunMemory::unstretched_internal};
  c.dsp_clock = {.extal_hz = oscillator,
                 .timebase_hz = common_hz,
                 .switch_timing = DspClockSwitch::reference_cycle_boundary};
  c.dsp = {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
           .maximum_cycles_per_period = 16,
           .writes = McuWriteOrder::before_mcu,
           .external_writes = McuWriteOrder::before_mcu,
           .wait_dsp_first = true,
           .reset_dsp_first = true,
           .initial_reset = OwnDspResetInputs{
               .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  c.dma_grant = DspDmaGrant::available;
  c.power_on = reference::PowerOnTiming{.inputs = {3}, .reset_pad = McuResetPad::high};
  c.release_ticks = std::numeric_limits<std::uint64_t>::max();
  c.operations = maximum_operations;
  c.select_n = 2;
  c.clock = 4;
  c.mosi = 16;
  constexpr std::uint8_t serial_pins = 0x16;
  c.initial = {{.port = McuPort::b, .drive = {.mask = serial_pins, .levels = 2}},
               {.port = McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  c.midi_output = output;
  return c;
}

struct Fixture {
  Fixture(const reference::Config &settings, const bool small)
      : flash(image(small)),
        execution{.image = flash,
                  .rom = rom,
                  .specification = BLOFELD_DEFAULT_SLA,
                  .flash_out = {},
                  .trace = &trace,
                  .instructions = maximum_operations},
        board(settings, execution) {}

  McuFlashImage flash;
  std::vector<std::uint32_t> rom;
  std::ostringstream trace;
  reference::Execution execution;
  reference::Board board;
};

void steps(reference::Board &board, const unsigned count) {
  for (unsigned i = 0; i < count; ++i) { board.advance(); }
}

void connected(const SciTransmitPin pin) {
  const bool small = pin == SciTransmitPin::ps5;
  Fixture f(config(pin), small);
  const auto initial_window = f.board.midi_output_window();
  require(initial_window.configured && initial_window.owner && initial_window.queued_bytes == 0 &&
              !f.board.pop_midi_output(),
          "An owned configured output begins empty, with no invented response");
  steps(f.board, cut_operations);
  const auto middle = f.board.status();
  require(middle.waiting && middle.midi_output_bytes == 0 && middle.midi_output_queued == 0,
          "Native WAI retains the incomplete transmitted byte without early publication");
  const auto partial = f.board.checkpoint();
  steps(f.board, suffix_operations);
  const auto final = f.board.status();
  require(final.waiting && final.midi_output_bytes == 1 && final.midi_output_queued == 1 &&
              final.held_reset && final.dsp_steps == 0,
          "Native SCI completes the actual MCU byte during WAI while the DSP remains in reset");
  const auto queued = f.board.checkpoint();
  const auto byte = f.board.pop_midi_output();
  require(byte && byte->value == pattern &&
              byte->stop_sample_ticks - byte->start_ticks == sampled_frame_span &&
              !f.board.pop_midi_output(),
          "Real pad output has literal55, independent9.5bit sample spacing and single retirement");
  const auto before_restore = f.board.midi_output_window();
  f.board.restore(queued);
  const auto restored = f.board.midi_output_window();
  require(restored.owner == before_restore.owner &&
              restored.restore_revision > before_restore.restore_revision &&
              f.board.status() == final && f.board.pop_midi_output() == byte,
          "Queued bytes restore with the model while the external session revision advances");
  f.board.restore(partial);
  require(f.board.status() == middle, "A partial serial frame belongs to the whole Board cut");
  steps(f.board, suffix_operations);
  require(f.board.status() == final && f.board.pop_midi_output() == byte,
          "A fresh native owner replays the exact partial-frame suffix and timestamp");
  Fixture baseline(config(std::nullopt), small);
  steps(baseline.board, cut_operations + suffix_operations);
  auto observed = final;
  observed.midi_output_bytes = 0;
  observed.midi_output_queued = 0;
  require(baseline.board.status() == observed,
          "Passive capture changes no CPU/DSP/peripheral state, counters or emulated clocks");
  require(!baseline.board.midi_output_window().configured,
          "Absent output is explicitly unconfigured");
  rejects([&] { static_cast<void>(baseline.board.pop_midi_output()); }, "not configured");
}

void config_guards(const std::filesystem::path &path) {
  constexpr std::string_view base =
      "blofeld-reference-board 2\ndsp-execution reference-effects\n"
      "clock 12000000 0 12000000 540000000 reference-boundaries\n"
      "ordering dsp-before-mcu dsp-before-mcu 16\nspi-b 02 04 10 clock-before-select\n"
      "dsp-release 450 low\noperations 23\n";
  const auto write = [&](const std::string_view tail) {
    std::ofstream stream(path);
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    stream << base << tail;
  };
  for (const auto &[name, pin] :
       std::array{std::pair{"PR3", SciTransmitPin::pr3}, std::pair{"PS5", SciTransmitPin::ps5}}) {
    write(std::string("midi-output ") + name + '\n');
    require(reference::load_config(path).midi_output == pin, "Both explicit output pads parse");
  }
  write("midi-output PR2\n");
  rejects([&] { static_cast<void>(reference::load_config(path)); }, "PR3 or PS5");
  write("midi-output PR3\nmidi-output PR3\n");
  rejects([&] { static_cast<void>(reference::load_config(path)); }, "Duplicate");
  write("midi-output PR3\ninitial drive R 08 08 00\n");
  rejects([&] { static_cast<void>(reference::load_config(path)); }, "initial external drive");
  auto c = config(SciTransmitPin::pr3);
  c.hreq = reference::ReturnRoute{.destination = McuPort::r, .mask = 8};
  rejects([&] { reference::validate_midi_output(c); }, "conflicts");
  c.hreq.reset();
  c.events = {{.ticks = 1, .port = McuPort::r, .drive = {.unknown = 8}}};
  rejects([&] { reference::validate_midi_output(c); }, "timed external drive");
}
}

int main(const int argc, char **argv) {
  try {
    if (argc != 2) { throw Fault("Expected a private configuration output path"); }
    connected(SciTransmitPin::pr3);
    connected(SciTransmitPin::ps5);
    config_guards(argv[1]);
    std::cout << "Connected Board MIDI output: actual reset/SCI/WAI, both pads, partial and "
                 "queued joint restores, passive-state control and configuration guards passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
