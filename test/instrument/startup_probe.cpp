// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_sink.hpp"
#include "audio/serial_pcm.hpp"
#include "common/bus.hpp"
#include "display/lcd_image.hpp"
#include "dsp/dsp_bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/mcu_memory.hpp"
#include "storage/flash_image.hpp"
#include "storage/private_file.hpp"
#include "storage/virtual_device_state.hpp"
#include "usb_midi_stimulus.hpp"
#include "vi/vi.hpp"
#include "vi/vi_profile.hpp"

#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using Clock = std::chrono::steady_clock;
// Leave room for profiler setup and final diagnostics within the ten-second process guard.
constexpr auto host_limit = std::chrono::seconds{8};
constexpr auto report_interval = std::chrono::milliseconds{250};
constexpr double reference_rate = 44100;

class Output final : public PcmFrameSink {
public:
  explicit Output(const std::filesystem::path &path) {
    capture_.exceptions(std::ios::badbit | std::ios::failbit);
    capture_.open(path, std::ios::binary | std::ios::out | std::ios::noreplace);
  }

  void accept(const SerialPcmFrame &frame) override {
    check();
    if (frame.channels != 2) { throw Fault("Startup observation requires actual stereo PCM"); }
    // Owner-thread diagnostic capture; this sink is never an audio callback.
    static_assert(std::endian::native == std::endian::little);
    const std::array stereo{frame.samples[0], frame.samples[1]};
    const auto bytes = std::bit_cast<std::array<char, sizeof(stereo)>>(stereo);
    capture_.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    if (!first_tick_) { first_tick_ = frame.first_tick; }
    ++frames_;
    nonzero_ += frame.samples[0] != 0 || frame.samples[1] != 0 ? 1U : 0U;
  }

  void check() const override {
    if (invalid_) { throw Fault("Startup PCM output was invalidated"); }
  }

  void invalidate() noexcept override { invalid_ = true; }

  [[nodiscard]] std::uint64_t frames() const { return frames_; }

  [[nodiscard]] std::uint64_t nonzero() const { return nonzero_; }

  void finish() {
    capture_.close();
    std::cout << "audio-capture=audio.s32le channels=2 valid-bits=24 first-tick=";
    if (first_tick_) {
      std::cout << *first_tick_;
    } else {
      std::cout << "none";
    }
    std::cout << '\n';
  }

private:
  std::uint64_t frames_ = 0;
  std::uint64_t nonzero_ = 0;
  bool invalid_ = false;
  std::ofstream capture_;
  std::optional<std::uint64_t> first_tick_;
};

void report(const vi::VirtualInstrument &board,
            const Output &output,
            const Clock::time_point started,
            const std::uint64_t timebase,
            const char *phase) {
  const auto elapsed = std::chrono::duration<double>(Clock::now() - started).count();
  const auto state = board.status();
  const auto mcu_pc = state.mcu_state_known ? hex(state.mcu.pc) : std::string("unknown");
  std::cout << "phase=" << phase << " host-seconds=" << elapsed << " device-seconds="
            << static_cast<double>(state.common_ticks) / static_cast<double>(timebase)
            << " mcu-pc=" << mcu_pc << " mcu-ccr="
            << (state.mcu_state_known ? hex(state.mcu.ccr, 2) : std::string("unknown"))
            << " dsp-pc=" << hex(state.dsp.pc, 6) << " mcu-instructions=" << state.instructions
            << " dsp-instructions=" << state.dsp_steps << " interrupts=" << state.interrupts
            << " mcu-cycles=" << state.mcu_cycles << " dsp-cycles=" << state.dsp_cycles
            << " dsp-held-reset=" << state.held_reset
            << " mcu-held-reset=" << !state.mcu_state_known << " waiting=" << state.waiting
            << " faulted=" << state.faulted << " delivered-stereo-frames=" << output.frames()
            << " nonzero-frames=" << output.nonzero()
            << " frames-per-host-second=" << static_cast<double>(output.frames()) / elapsed
            << " startup-reference-rtf="
            << static_cast<double>(output.frames()) / (reference_rate * elapsed) << '\n'
            << std::flush;
}

void capture_display(const vi::VirtualInstrument &board, const PrivateDirectory &directory) {
  const auto image = board.display_image({.width = 128, .reverse_commons = true});
  if (!image) { return; }
  const auto text = encode_lcd_text(*image);
  const std::vector<std::uint8_t> bytes(text.begin(), text.end());
  directory.write("display.txt", bytes);
  for (unsigned y = 0; y < LcdImage::height; ++y) {
    for (unsigned x = 0; x < image->width(); ++x) {
      if (image->pixel(x, y) == LcdPixel::unknown) { return; }
    }
  }
  directory.write("display.pbm", encode_lcd_pbm(*image));
}

void peer_response(const vi::VirtualInstrument &instrument) {
  // Firmware 1.25 staging/storage identities, never injected response data.
  constexpr unsigned staging = 0xf000;
  constexpr unsigned received = 0x19bc;
  constexpr unsigned block_bytes = 16;
  for (unsigned index = 0; index < block_bytes; ++index) {
    const auto source = instrument.inspect_dsp({.space = DspSpace::x, .address = staging + index});
    const auto destination = instrument.inspect_mcu(static_cast<std::uint16_t>(received + index));
    std::cout << "peer-byte=" << index << " dsp=" << (source ? hex(*source, 6) : "unknown")
              << " mcu=" << (destination.value ? hex(*destination.value, 2) : "unknown") << '\n';
  }
}

int probe(const char *const *argv, const Clock::time_point started) {
  const auto profile = vi::load_profile(argv[1]);
  const auto firmware = Firmware::load(profile.firmware);
  const auto config = vi::vi_config(profile);
  if (!config.power_on || !config.pcm || !config.lcd_parallel) {
    throw Fault("Startup observation requires a power-on, PCM and display profile");
  }
  const auto rom = vi::load_rom(profile.boot_rom);
  const auto device = std::make_shared<VirtualDeviceState>(profile.device_state,
                                                           McuFlashImage::from_known(firmware.mcu));
  const std::filesystem::path directory(argv[2]);
  const PrivateDirectory output_directory(directory); // Validate before native execution.
  Output output(directory / "audio.s32le");
  test::UsbMidiStimulus midi(config.dsp_clock.timebase_hz);
  vi::VirtualInstrument board(config,
                              {.image = device->image(),
                               .rom = rom,
                               .specification = BLOFELD_DEFAULT_SLA,
                               .device_state = device});
  board.attach_pcm_output(output);
  const auto deadline = started + host_limit;
  auto next_report = started;
  bool failed = false;
  try {
    board.initialize();
    while (Clock::now() < deadline) {
      board.advance();
      midi.poll(board, output.frames() != 0);
      const auto now = Clock::now();
      if (now >= next_report) {
        report(board, output, started, config.dsp_clock.timebase_hz, "startup");
        next_report = now + report_interval;
      }
    }
  } catch (const std::exception &error) {
    failed = true;
    std::cerr << "Startup fault: " << error.what() << '\n';
  }
  const char *phase = "host-limit";
  if (failed) { phase = "fault"; }
  report(board, output, started, config.dsp_clock.timebase_hz, phase);
  midi.report();
  output.finish();
  capture_display(board, output_directory);
  if (failed) { peer_response(board); }
  // Observation is deliberately fail-closed: frames and a screen alone do not
  // prove USB-MIDI note response, SL playback or complete instrument readiness.
  std::cout << "playability=unproven scope=startup-observation\n";
  return failed ? 1 : 2;
}
}

int main(const int argc, char **argv) {
  const auto started = Clock::now();
  try {
    constexpr int arguments = 3;
    if (argc != arguments) { throw Fault("Usage: blofeld-startup-probe profile output-dir"); }
    return probe(argv, started);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
