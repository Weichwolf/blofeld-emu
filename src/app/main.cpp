// SPDX-License-Identifier: GPL-3.0-only
#include "aot/artifact.hpp"
#include "aot/dsp.hpp"
#include "aot/mcu.hpp"
#include "audio/playback.hpp"
#include "boot_rom.hpp"
#include "display/publication.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/memory.hpp"
#include "dsp/program.hpp"
#include "dsp_memory.hpp"
#include "files/directory.hpp"
#include "files/read.hpp"
#include "io/coupling.hpp"
#include "io/midi_port.hpp"
#include "live.hpp"
#include "mcu/program.hpp"
#include "memory/faults.hpp"
#include "options.hpp"
#include "process/configuration.hpp"
#include "process/signals.hpp"
#include "profile.hpp"
#include "render.hpp"
#include "storage/flash_image.hpp"
#include "storage/media_image.hpp"
#include "vm_context.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <exception>
#include <filesystem>
#include <future>
#include <iostream>
#include <memory>
#include <span>
#include <utility>

namespace {
void commit_storage(const std::filesystem::path &path,
                    const std::span<const std::uint8_t> bytes,
                    const blofeld::host::FileRevision &revision) {
  const blofeld::host::Directory directory(std::filesystem::canonical(path.parent_path()));
  directory.commit(path.filename().native(), bytes, revision);
}
}

int main(const int argc, char **argv) try {
  const auto launched = std::chrono::steady_clock::now();
  const blofeld::host::Signals signals;
  const auto options = blofeld::app::options(
      std::span<const char *const>(argv + 1, static_cast<std::size_t>(argc - 1)));
  const auto profile = blofeld::app::load_profile(options.profile);
  const bool offline = options.midi && options.wav;
  auto boot = blofeld::app::load_boot_rom(profile.boot_rom, profile.boot_format);
  const auto mode = blofeld::app::dsp_memory_mode();
  using blofeld::host::aot::Preparation;
  const auto preparation = options.prepare ? Preparation::allow : Preparation::load_only;
  const auto loading = std::chrono::steady_clock::now();
  auto mcu = std::async(std::launch::async, [&profile, preparation] {
    return blofeld::app::aot::load_mcu_program(profile, preparation);
  });
  const auto program =
      std::make_shared<blofeld::vi::dsp::Program>(blofeld::app::aot::load_dsp_program(
          profile, blofeld::vi::dsp::BootRom::document(boot), mode, preparation));
  const auto mcu_program = std::make_shared<blofeld::vi::mcu::Program>(mcu.get());
  const auto loaded = std::chrono::steady_clock::now();
  const std::chrono::duration<double> modules = loaded - loading;
  if (options.prepare) {
    std::cerr << "AOT preparation complete in " << modules.count() << " s\n";
    return 0;
  }
  if (!offline) { std::cerr << "Native modules loaded in " << modules.count() << " s\n"; }
  blofeld::host::FileRevision device_revision;
  blofeld::host::FileRevision media_revision;
  const auto device_state = blofeld::host::read_file(profile.device_state,
                                                     blofeld::vi::FlashImage::file_bytes,
                                                     offline ? nullptr : &device_revision);
  auto image =
      std::make_unique<blofeld::vi::FlashImage>(blofeld::vi::FlashImage::decode(device_state));
  auto media = std::make_unique<blofeld::vi::MediaImage>(blofeld::host::read_file(
      profile.storage, blofeld::vi::MediaImage::max_bytes, offline ? nullptr : &media_revision));
  const auto memory =
      mode == dsp56300::CompileOptions::MemoryMode::direct
          ? std::make_unique<blofeld::app::DspMemory>(blofeld::vi::dsp::BootRom::document(boot))
          : nullptr;
  const auto dsp_storage = memory ? memory->storage() : blofeld::vi::dsp::Memory::Storage{};
  const blofeld::host::memory::Faults faults(dsp56300::Core::host_fault);
  if (offline) {
    blofeld::app::OfflineRender render(options);
    blofeld::vi::VmContext vm(std::move(image),
                              std::move(boot),
                              mcu_program,
                              std::move(media),
                              render.output(),
                              blofeld::vi::MidiPort::InputMode::stream,
                              program,
                              blofeld::vi::Coupling::default_window,
                              dsp_storage);
    return render.run(vm, signals, launched);
  }
  constexpr unsigned sample_rate = 44100;
  const auto display_path = blofeld::host::runtime_home() / "blofeld-emu";
  std::filesystem::create_directories(display_path);
  blofeld::app::DisplayPublication display(display_path);
  std::cerr << "Display: " << display_path << '\n';
  blofeld::host::audio::Playback audio(sample_rate, "blofeld-emu");
  blofeld::app::Live live(audio, display, options);
  blofeld::vi::VmContext vm(std::move(image),
                            std::move(boot),
                            mcu_program,
                            std::move(media),
                            live.output(),
                            blofeld::vi::MidiPort::InputMode::audio,
                            program,
                            blofeld::vi::Coupling::default_window,
                            dsp_storage);
  const auto result = live.run(vm, signals, launched);
  if (result.save_storage) {
    const auto storage = vm.stopped_storage();
    if (storage.device != device_state) {
      commit_storage(profile.device_state, storage.device, device_revision);
    }
    if (!storage.changed_media.empty()) {
      commit_storage(profile.storage, storage.changed_media, media_revision);
    }
  }
  return result.status;
} catch (const std::exception &error) {

  std::cerr << "blofeld-headless: " << error.what() << '\n';
  return 1;
}
