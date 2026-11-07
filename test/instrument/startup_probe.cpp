// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_sink.hpp"
#include "audio/serial_pcm.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "display/lcd_image.hpp"
#include "firmware/firmware.hpp"
#include "storage/flash_image.hpp"
#include "storage/private_file.hpp"
#include "storage/virtual_device_state.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using Clock = std::chrono::steady_clock;
constexpr auto host_limit = std::chrono::seconds{8};
constexpr auto report_interval = std::chrono::milliseconds{250};
constexpr double reference_rate = 44100;

class Output final : public PcmFrameSink {
public:
  void accept(const SerialPcmFrame &frame) override {
    check();
    if (frame.channels != 2) { throw Fault("Startup observation requires actual stereo PCM"); }
    ++frames_;
    nonzero_ += frame.samples[0] != 0 || frame.samples[1] != 0 ? 1U : 0U;
  }

  void check() const override {
    if (invalid_) { throw Fault("Startup PCM output was invalidated"); }
  }

  void invalidate() noexcept override { invalid_ = true; }

  [[nodiscard]] std::uint64_t frames() const { return frames_; }

  [[nodiscard]] std::uint64_t nonzero() const { return nonzero_; }

private:
  std::uint64_t frames_ = 0;
  std::uint64_t nonzero_ = 0;
  bool invalid_ = false;
};

void report(const reference::Board &board,
            const Output &output,
            const Clock::time_point started,
            const std::uint64_t timebase,
            const char *phase) {
  const auto elapsed = std::chrono::duration<double>(Clock::now() - started).count();
  const auto state = board.runtime_status();
  const auto mcu_pc = state.mcu_state_known ? hex(state.mcu.pc) : std::string("unknown");
  std::cout << "phase=" << phase << " host-seconds=" << elapsed << " device-seconds="
            << static_cast<double>(state.common_ticks) / static_cast<double>(timebase)
            << " mcu-pc=" << mcu_pc << " dsp-pc=" << hex(state.dsp.pc, 6)
            << " mcu-instructions=" << state.instructions << " dsp-instructions=" << state.dsp_steps
            << " interrupts=" << state.interrupts << " mcu-cycles=" << state.mcu_cycles
            << " dsp-cycles=" << state.dsp_cycles << " dsp-held-reset=" << state.held_reset
            << " mcu-held-reset=" << !state.mcu_state_known << " waiting=" << state.waiting
            << " faulted=" << state.faulted << " delivered-stereo-frames=" << output.frames()
            << " nonzero-frames=" << output.nonzero()
            << " frames-per-host-second=" << static_cast<double>(output.frames()) / elapsed
            << " startup-reference-rtf="
            << static_cast<double>(output.frames()) / (reference_rate * elapsed) << '\n'
            << std::flush;
}

void capture_display(const reference::Board &board, const PrivateDirectory &directory) {
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

int probe(const char *const *argv, const Clock::time_point started) {
  const auto firmware = Firmware::load(argv[1]);
  const auto config = reference::load_config(argv[2]);
  if (!config.power_on || !config.pcm || !config.lcd_parallel) {
    throw Fault("Startup observation requires a power-on, PCM and display profile");
  }
  const auto rom = reference::load_rom(argv[3]);
  const auto device =
      std::make_shared<VirtualDeviceState>(argv[4], McuFlashImage::from_known(firmware.mcu));
  const std::filesystem::path directory(argv[5]);
  const PrivateDirectory output_directory(directory); // Validate before native execution.
  Output output;
  reference::Board board(config,
                         {.image = device->image(),
                          .rom = rom,
                          .specification = BLOFELD_DEFAULT_SLA,
                          .flash_out = {},
                          .instructions = std::numeric_limits<std::uint64_t>::max(),
                          .device_state = device});
  board.attach_pcm_output(output);
  const auto deadline = started + host_limit;
  auto next_report = started;
  bool failed = false;
  try {
    board.initialize();
    while (!board.finished() && Clock::now() < deadline) {
      board.advance();
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
  if (failed) {
    phase = "fault";
  } else if (board.finished()) {
    phase = "configuration-limit";
  }
  report(board, output, started, config.dsp_clock.timebase_hz, phase);
  capture_display(board, output_directory);
  // Observation is deliberately fail-closed: frames and a screen alone do not
  // prove USB-MIDI note response, SL playback or complete instrument readiness.
  std::cout << "playability=unproven scope=startup-observation\n";
  return failed ? 1 : 2;
}
}

int main(const int argc, char **argv) {
  const auto started = Clock::now();
  try {
    constexpr int arguments = 6;
    if (argc != arguments) {
      throw Fault(
          "Usage: blofeld-startup-probe firmware.zip board.bfboard boot.bin unit.bfnvm output-dir");
    }
    return probe(argv, started);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
