// SPDX-License-Identifier: GPL-3.0-only
#include "audio/consumer.hpp"
#include "audio/playback.hpp"
#include "boot_rom.hpp"
#include "display/lcd_image.hpp"
#include "display/publication.hpp"
#include "files/read.hpp"
#include "io/midi_port.hpp"
#include "options.hpp"
#include "process/configuration.hpp"
#include "process/signals.hpp"
#include "profile.hpp"
#include "render.hpp"
#include "report.hpp"
#include "storage/flash_image.hpp"
#include "storage/media_image.hpp"
#include "vm_context.hpp"

#include <chrono>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <ios>
#include <iostream>
#include <memory>
#include <ratio>
#include <span>
#include <string_view>
#include <unistd.h>
#include <utility>

namespace {

void status(const blofeld::host::audio::Playback &audio,
            const blofeld::app::AudioConsumer &consumer,
            const bool terminal) {
  const auto host = audio.status();
  const auto source = consumer.statistics();
  if (terminal) { std::cerr << "\r\033[2K"; }
  std::cerr << "PCM submitted=" << host.submitted_frames << " queue=" << host.queued_frames
            << " server-buffer=" << host.server_buffer_frames << " producer~";
  if (source.production_reserve) {
    std::cerr << std::fixed << std::setprecision(2) << *source.production_reserve << 'x';
  } else {
    std::cerr << '?';
  }
  std::cerr << " output-latency~=";
  if (host.latency_us) {
    std::cerr << std::chrono::duration<double, std::milli>(
                     std::chrono::microseconds(*host.latency_us))
                     .count()
              << "ms";
  } else {
    std::cerr << '?';
  }
  std::cerr << " underruns=" << host.underruns << '/' << source.device_underruns
            << " playing=" << (host.started ? "yes" : "no");
  if (!terminal) { std::cerr << '\n'; }
}

void publish_display(blofeld::vi::VmContext &vm, blofeld::app::DisplayPublication &publication) {
  if (const auto controller = vm.take_display()) {
    // Firmware addresses columns 0..127 and page 7 bit 7 for its logical top row.
    constexpr blofeld::vi::LcdPanel panel{.width = 128, .reverse_commons = true};
    static_cast<void>(publication.publish(blofeld::vi::capture_lcd(*controller, panel)));
  }
}

void drive(blofeld::vi::VmContext &vm,
           blofeld::host::audio::Playback &audio,
           const blofeld::app::AudioConsumer &consumer,
           const blofeld::host::Signals &signals,
           blofeld::app::DisplayPublication &display) {
  const bool terminal = ::isatty(STDERR_FILENO) != 0;
  constexpr std::chrono::milliseconds control_interval{100};
  constexpr std::chrono::milliseconds terminal_interval{250};
  constexpr std::chrono::seconds redirected_interval{1};
  vm.start();
  auto next = std::chrono::steady_clock::now();
  while (!vm.finished()) {
    publish_display(vm, display);
    vm.request_display();
    if (signals.wait(control_interval) || audio.status().failed) {
      vm.request_stop();
      break;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= next) {
      status(audio, consumer, terminal);
      next = now + (terminal ? terminal_interval : redirected_interval);
    }
  }
  if (terminal) { std::cerr << '\n'; }
}

}

int main(const int argc, char **argv) try {
  const auto launched = std::chrono::steady_clock::now();
  const blofeld::host::Signals signals;
  const auto options = blofeld::app::options(
      std::span<const char *const>(argv + 1, static_cast<std::size_t>(argc - 1)));
  const auto profile = blofeld::app::load_profile(options.profile);
  const auto display_path = blofeld::host::runtime_home() / "blofeld-emu";
  std::filesystem::create_directories(display_path);
  blofeld::app::DisplayPublication display(display_path);
  std::cerr << "Display: " << display_path << '\n';
  auto image = std::make_unique<blofeld::vi::FlashImage>(blofeld::vi::FlashImage::decode(
      blofeld::host::read_file(profile.device_state, blofeld::vi::FlashImage::file_bytes)));
  auto media = std::make_unique<blofeld::vi::MediaImage>(
      blofeld::host::read_file(profile.storage, blofeld::vi::MediaImage::max_bytes));
  auto boot = blofeld::app::load_boot_rom(profile.boot_rom, profile.boot_format);
  constexpr unsigned sample_rate = 44100;
  if (options.midi && options.wav) {
    blofeld::app::OfflineRender render(options);
    blofeld::vi::VmContext vm(std::move(image),
                              std::move(boot),
                              std::move(media),
                              render.output(),
                              blofeld::vi::MidiPort::InputMode::stream);
    return render.run(vm, signals, display, launched);
  }
  blofeld::host::audio::Playback audio(sample_rate, "blofeld-emu");
  const auto &grant = audio.grants();
  std::cerr << "Audio: s24-in-32 stereo " << grant.rate << "Hz -> " << grant.sink_format << ' '
            << grant.sink_rate << "Hz on " << grant.sink << "; buffer=" << grant.target_frames
            << " max=" << grant.maximum_frames << " minreq=";
  if (grant.minimum_request_frames) {
    std::cerr << *grant.minimum_request_frames;
  } else {
    std::cerr << '?';
  }
  std::cerr << '\n';
  if (grant.rate != grant.sink_rate) { std::cerr << "Audio backend performs rate conversion.\n"; }
  std::cerr << "MIDI is not connected; instrument readiness remains incomplete.\n";
  blofeld::app::AudioConsumer consumer(audio);
  blofeld::vi::VmContext vm(std::move(image), std::move(boot), std::move(media), consumer.output());
  drive(vm, audio, consumer, signals, display);
  const auto reports = vm.wait();
  publish_display(vm, display);
  audio.stop();
  status(audio, consumer, false);
  const bool valid = blofeld::app::report_execution(reports);
  audio.check();
  std::cerr << "Startup failed: the instrument is not playable; MIDI and remaining device "
               "contracts are incomplete.\n";
  return valid ? 2 : 1;
} catch (const std::exception &error) {

  std::cerr << "blofeld-headless: " << error.what() << '\n';
  return 1;
}
