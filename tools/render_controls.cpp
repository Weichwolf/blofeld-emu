// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "aot/mcu.hpp"
#include "boot_rom.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/memory.hpp"
#include "dsp/program.hpp"
#include "dsp_memory.hpp"
#include "files/read.hpp"
#include "io/coupling.hpp"
#include "io/midi_port.hpp"
#include "io/pcm_output.hpp"
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

#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace {
class PcmTiming {
public:
  explicit PcmTiming(blofeld::app::OfflineRender &render)
      : render_(render), sink_(render.output()) {
    // Allocate before the DSP starts so recording does not grow the buffer mid-run.
    submissions_.reserve(maximum_submissions);
  }

  [[nodiscard]] blofeld::vi::PcmOutput output() noexcept {
    return {.owner = this,
            .write =
                [](void *const owner,
                   const blofeld::vi::PcmOutput::Block block,
                   const std::stop_token &stop) {
                  auto &timing = *static_cast<PcmTiming *>(owner);
                  if (!timing.sink_.write(timing.sink_.owner, block, stop)) { return false; }
                  if (timing.submissions_.size() == maximum_submissions) {
                    throw std::runtime_error("PCM timing capture reached its block limit");
                  }
                  timing.submissions_.push_back(
                      {.accepted = Clock::now(),
                       .first_frame = block.first_frame,
                       .first_tick = block.ticks.front(),
                       .last_tick = block.ticks.back(),
                       .frames = block.frames.size(),
                       .written_frames = timing.render_.written_frames()});
                  return true;
                },
            .close =
                [](void *const owner) noexcept {
                  const auto sink = static_cast<PcmTiming *>(owner)->sink_;
                  sink.close(sink.owner);
                }};
  }

  void save(std::filesystem::path path) const {
    path += ".timing.tsv";
    std::ofstream out(path);
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out << "monotonic_ns\tfirst_frame\tframes\tfirst_tick\tlast_tick\twritten_frames\n";
    for (const auto &block : submissions_) {
      const auto time =
          std::chrono::duration_cast<std::chrono::nanoseconds>(block.accepted.time_since_epoch());
      out << time.count() << '\t' << block.first_frame << '\t' << block.frames << '\t'
          << block.first_tick << '\t' << block.last_tick << '\t' << block.written_frames << '\n';
    }
    out.close();
  }

private:
  using Clock = std::chrono::steady_clock;

  struct Submission {
    Clock::time_point accepted;
    std::uint64_t first_frame, first_tick, last_tick;
    std::size_t frames;
    std::uint64_t written_frames;
  };

  static constexpr std::size_t maximum_submissions = 1U << 19U;
  const blofeld::app::OfflineRender &render_;
  blofeld::vi::PcmOutput sink_;
  std::vector<Submission> submissions_;
};

std::vector<std::uint64_t>
settings(const std::string_view name, const std::size_t count, const std::uint64_t fallback = 0) {
  auto text = blofeld::host::inherited_setting(name);
  if (text.empty()) {
    std::vector<std::uint64_t> defaults(count, fallback);
    return defaults;
  }
  std::vector<std::uint64_t> windows;
  for (;;) {
    const auto comma = text.find(',');
    const auto item = text.substr(0, comma);
    std::uint64_t window = 0;
    const auto parsed = std::from_chars(item.data(), item.data() + item.size(), window);
    if (parsed.ec != std::errc{} || parsed.ptr != item.data() + item.size()) {
      throw std::invalid_argument("Diagnostic settings require nonnegative integer lists");
    }
    windows.push_back(window);
    if (comma == std::string_view::npos) { break; }
    text.remove_prefix(comma + 1);
  }
  if (windows.size() == 1) {
    const auto window = windows.front();
    windows.assign(count, window);
  }
  if (windows.size() != count) {
    throw std::invalid_argument("Diagnostic setting count differs from workloads");
  }
  return windows;
}
}

int main(int argc, char **argv) try {
  const blofeld::host::Signals signals;
  const std::span<const char *const> arguments(argv + 1, static_cast<std::size_t>(argc - 1));
  std::vector<blofeld::app::Options> workloads;
  for (std::size_t begin = 0, end = 0; end <= arguments.size(); ++end) {
    if (end != arguments.size() && std::string_view(arguments[end]) != "--") { continue; }
    auto options = blofeld::app::options(arguments.subspan(begin, end - begin));
    if (!options.midi || !options.wav ||
        (!workloads.empty() && options.profile != workloads.front().profile)) {
      throw std::invalid_argument("Each -- separated workload requires --midi/--wav and the same "
                                  "--profile");
    }
    workloads.push_back(std::move(options));
    begin = end + 1;
  }
  const auto windows =
      settings("BLOFELD_D2", workloads.size(), blofeld::vi::Coupling::default_window);
  const bool capture_timing = blofeld::host::inherited_setting("BLOFELD_PCM_TIMING") == "1";
#if defined(BLOFELD_CENSUS)
  const auto bounds = settings("BLOFELD_CENSUS_BOUNDS", 2 * workloads.size());
#endif
  const auto profile = blofeld::app::load_profile(workloads.front().profile);
  const auto boot = blofeld::app::load_boot_rom(profile.boot_rom, profile.boot_format);
  const auto mcu_started = std::chrono::steady_clock::now();
  const auto mcu =
      std::make_shared<blofeld::vi::mcu::Program>(blofeld::app::aot::load_mcu_program(profile));
  const auto mode = blofeld::app::dsp_memory_mode();
  const blofeld::host::memory::Faults faults(dsp56300::Core::host_fault);
  const auto dsp_started = std::chrono::steady_clock::now();
  std::cerr << "MCU_AOT_SECONDS="
            << std::chrono::duration<double>(dsp_started - mcu_started).count() << '\n';
  const auto dsp = std::make_shared<blofeld::vi::dsp::Program>(blofeld::app::aot::load_dsp_program(
      profile, blofeld::vi::dsp::BootRom::document(boot), mode));
  std::cerr << "DSP_AOT_SECONDS="
            << std::chrono::duration<double>(std::chrono::steady_clock::now() - dsp_started).count()
            << '\n';
  std::size_t workload = 0;
  for (const auto &options : workloads) {
    const auto instantiated = std::chrono::steady_clock::now();
    auto image = std::make_unique<blofeld::vi::FlashImage>(blofeld::vi::FlashImage::decode(
        blofeld::host::read_file(profile.device_state, blofeld::vi::FlashImage::file_bytes)));
    auto media = std::make_unique<blofeld::vi::MediaImage>(
        blofeld::host::read_file(profile.storage, blofeld::vi::MediaImage::max_bytes));
    const auto memory =
        mode == dsp56300::CompileOptions::MemoryMode::direct
            ? std::make_unique<blofeld::app::DspMemory>(blofeld::vi::dsp::BootRom::document(boot))
            : nullptr;
    const auto storage = memory ? memory->storage() : blofeld::vi::dsp::Memory::Storage{};
    blofeld::app::OfflineRender render(options);
    std::optional<PcmTiming> timing;
    if (capture_timing) { timing.emplace(render); }
    blofeld::vi::VmContext vm(std::move(image),
                              boot,
                              mcu,
                              std::move(media),
                              timing ? timing->output() : render.output(),
                              blofeld::vi::MidiPort::InputMode::stream,
                              dsp,
                              windows[workload],
                              storage);
#if defined(BLOFELD_CENSUS)
    const auto boundary = 2 * workload;
    if (bounds[boundary] != 0 || bounds[boundary + 1] != 0) {
      vm.census().interval(bounds[boundary], bounds[boundary + 1]);
    }
#endif
    std::cerr << "CONTROL_WORKLOAD=" << ++workload << '\n';
    const auto result = render.run(vm, signals, instantiated);
#if defined(BLOFELD_CENSUS)
    vm.census().report_intervals(std::cerr);
    if (const auto *probe = vm.state_probe()) { probe->report(std::cerr); }
#endif
    if (result != 0) { return result; }
    if (timing && options.wav) { timing->save(*options.wav); }
  }
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
