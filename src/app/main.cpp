// SPDX-License-Identifier: GPL-3.0-only
#include "audio/audio_runtime.hpp"
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "storage/flash_image.hpp"
#include "storage/virtual_device_state.hpp"
#include "usb/usbip_runtime.hpp"
#include "vi/host/vi_display.hpp"
#include "vi/vi.hpp"
#include "vi/vi_profile.hpp"

#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string_view>

namespace {
constexpr unsigned usbip_port = 3240;
constexpr std::uint64_t usb_ready_at = 4964626296;
constexpr unsigned milliseconds_per_second = 1000;
volatile std::sig_atomic_t stopping = 0;

void stop(const int signal) {
  if (signal == SIGINT || signal == SIGTERM) { stopping = 1; }
}

std::filesystem::path default_profile(char **environment) {
  // Capture process-entry values before creating workers; no global environment API.
  std::filesystem::path config;
  std::filesystem::path home;
  constexpr std::string_view config_key = "XDG_CONFIG_HOME=";
  constexpr std::string_view home_key = "HOME=";
  for (; *environment != nullptr; ++environment) {
    const std::string_view entry(*environment);
    if (entry.starts_with(config_key)) { config = entry.substr(config_key.size()); }
    if (entry.starts_with(home_key)) { home = entry.substr(home_key.size()); }
  }
  if (!config.empty()) { return config / "blofeld-emu/default.profile"; }
  if (!home.empty()) { return home / ".config/blofeld-emu/default.profile"; }
  throw blofeld::Fault("Set XDG_CONFIG_HOME or HOME for the default profile");
}

std::filesystem::path profile_path(const int argc, char **argv) {
  if (argc == 3 && std::string_view(argv[1]) == "--profile") { return argv[2]; }
  throw blofeld::Fault("Usage: blofeld-headless [--profile file]");
}

void run(const blofeld::vi::Profile &profile) {
  using namespace blofeld;
  const auto firmware = Firmware::load(profile.firmware);
  const auto rom = vi::load_rom(profile.boot_rom);
  const auto device = std::make_shared<VirtualDeviceState>(profile.device_state,
                                                           McuFlashImage::from_known(firmware.mcu));
  const auto config = vi::vi_config(profile);
  vi::VirtualInstrument instrument(
      config, {.image = device->image(), .rom = rom, .specification = {}, .device_state = device});
  if (!config.pcm) { throw Fault("The instrument requires a PCM output format"); }
  AudioRuntime audio({.output = {.pcm = config.pcm->format,
                                 .timebase_hz = vi::timebase_hz,
                                 .sample_rate = vi::sample_rate,
                                 .server = {}}});
  instrument.attach_pcm_output(audio);
  UsbipRuntime usb(
      {.tcp = {.port = usbip_port},
       .location = {.path = "/blofeld-emu/usb/1-1", .busid = "1-1", .bus = 1, .device = 1},
       .after_ticks = usb_ready_at,
       .period_ticks = vi::timebase_hz / milliseconds_per_second,
       .probe_timeout_ticks = vi::timebase_hz * 2});
  std::optional<vi::HostDisplayPump> display;
  if (!profile.display.empty()) { display.emplace(profile.display); }
  const auto started = std::chrono::steady_clock::now();
  auto next_status = started;
  instrument.initialize();
  while (stopping == 0) {
    instrument.advance();
    instrument.poll_usb(usb);
    if (display) { display->poll(instrument); }
    const auto now = std::chrono::steady_clock::now();
    if (now < next_status) { continue; }
    const auto state = audio.status();
    const auto elapsed = std::chrono::duration<double>(now - started).count();
    std::cout << "audio-frames=" << state.written << " queued=" << state.accepted - state.written
              << " underflows=" << state.underflows << " elapsed=" << elapsed << '\n';
    next_status = now + std::chrono::seconds{1};
  }
  audio.finish();
}
}

int main(const int argc, char **argv, char **environment) {
  try {
    if (argc == 2 && std::string_view(argv[1]) == "--help") {
      std::cout << "Usage: blofeld-headless [--profile file]\n"
                   "Profile fields: firmware, boot-rom, device-state, nand, optional display.\n";
      return 0;
    }
    const auto path = argc == 1 ? default_profile(environment) : profile_path(argc, argv);
    if (std::signal(SIGINT, stop) == SIG_ERR || std::signal(SIGTERM, stop) == SIG_ERR) {
      throw blofeld::Fault("Cannot install instrument shutdown handlers");
    }
    run(blofeld::vi::load_profile(path));
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "blofeld-headless: " << error.what() << '\n';
    return 1;
  }
}
