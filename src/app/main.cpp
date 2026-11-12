// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "boot_rom.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/program.hpp"
#include "files/read.hpp"
#include "io/midi_port.hpp"
#include "options.hpp"
#if defined(BLOFELD_NATIVE_MIDI)
#include "audio/playback.hpp"
#include "display/publication.hpp"
#include "live.hpp"
#include "process/configuration.hpp"

#include <filesystem>
#endif
#include "process/signals.hpp"
#include "profile.hpp"
#include "render.hpp"
#include "storage/flash_image.hpp"
#include "storage/media_image.hpp"
#include "vm_context.hpp"

#include <chrono>
#include <cstddef>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <utility>

int main(const int argc, char **argv) try {
  const auto launched = std::chrono::steady_clock::now();
  const blofeld::host::Signals signals;
  const auto options = blofeld::app::options(
      std::span<const char *const>(argv + 1, static_cast<std::size_t>(argc - 1)));
  const auto profile = blofeld::app::load_profile(options.profile);
  auto image = std::make_unique<blofeld::vi::FlashImage>(blofeld::vi::FlashImage::decode(
      blofeld::host::read_file(profile.device_state, blofeld::vi::FlashImage::file_bytes)));
  auto media = std::make_unique<blofeld::vi::MediaImage>(
      blofeld::host::read_file(profile.storage, blofeld::vi::MediaImage::max_bytes));
  auto boot = blofeld::app::load_boot_rom(profile.boot_rom, profile.boot_format);
  const auto program = std::make_shared<blofeld::vi::dsp::Program>(
      blofeld::app::aot::load_dsp_program(profile, blofeld::vi::dsp::BootRom::document(boot)));
  if (options.midi && options.wav) {
    blofeld::app::OfflineRender render(options);
    blofeld::vi::VmContext vm(std::move(image),
                              std::move(boot),
                              std::move(media),
                              render.output(),
                              blofeld::vi::MidiPort::InputMode::stream,
                              program);
    return render.run(vm, signals, launched);
  }
#if defined(BLOFELD_NATIVE_MIDI)
  constexpr unsigned sample_rate = 44100;
  const auto display_path = blofeld::host::runtime_home() / "blofeld-emu";
  std::filesystem::create_directories(display_path);
  blofeld::app::DisplayPublication display(display_path);
  std::cerr << "Display: " << display_path << '\n';
  blofeld::host::audio::Playback audio(sample_rate, "blofeld-emu");
  blofeld::app::Live live(audio, display, options);
  blofeld::vi::VmContext vm(std::move(image),
                            std::move(boot),
                            std::move(media),
                            live.output(),
                            blofeld::vi::MidiPort::InputMode::stream,
                            program);
  return live.run(vm, signals);
#else
  std::cerr << "Native MIDI input backend is unavailable on this platform.\n";
  return 2;
#endif
} catch (const std::exception &error) {

  std::cerr << "blofeld-headless: " << error.what() << '\n';
  return 1;
}
