// SPDX-License-Identifier: GPL-3.0-only
#include "audio/consumer.hpp"
#include "audio/playback.hpp"
#include "execution.hpp"
#include "files/read.hpp"
#include "process/signals.hpp"
#include "profile.hpp"
#include "storage/flash_image.hpp"
#include "storage/media_image.hpp"
#include "vm_context.hpp"

#include <chrono>
#include <cstddef>
#include <dsp56300/memory.hpp>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <ios>
#include <iostream>
#include <memory>
#include <ratio>
#include <stdexcept>
#include <string_view>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {
std::filesystem::path selected_profile(const int argc, char **argv) {
  if (argc == 1) { return blofeld::app::default_profile(); }
  if (argc == 3 && std::string_view(argv[1]) == "--profile") { return argv[2]; }
  throw std::invalid_argument("Usage: blofeld-headless [--profile FILE]");
}

const char *operation(const blofeld::vi::Access access) {
  switch (access) {
    case blofeld::vi::Access::read: return "read";
    case blofeld::vi::Access::write: return "write";
    case blofeld::vi::Access::fetch: return "fetch";
    case blofeld::vi::Access::bit: return "bit";
    case blofeld::vi::Access::execute: return "execute";
    case blofeld::vi::Access::reset: return "reset";
    default: return "unknown";
  }
}

std::vector<dsp56300::Word> boot_rom(const std::filesystem::path &path) {
  constexpr std::size_t bytes = 576;
  const auto input = blofeld::host::read_file(path, bytes);
  if (input.size() != bytes) { throw std::runtime_error("DSP SPI boot ROM requires 576 bytes"); }
  std::vector<dsp56300::Word> words;
  words.reserve(bytes / 3);
  for (std::size_t at = 0; at < input.size(); at += 3) {
    words.push_back((static_cast<dsp56300::Word>(input[at]) << 16U) |
                    (static_cast<dsp56300::Word>(input[at + 1]) << 8U) | input[at + 2]);
  }
  return words;
}

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
  std::cerr << " latency=";
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

void drive(blofeld::vi::VmContext &vm,
           blofeld::host::audio::Playback &audio,
           const blofeld::app::AudioConsumer &consumer,
           const blofeld::host::Signals &signals) {
  const bool terminal = ::isatty(STDERR_FILENO) != 0;
  constexpr std::chrono::milliseconds control_interval{100};
  constexpr std::chrono::milliseconds terminal_interval{250};
  constexpr std::chrono::seconds redirected_interval{1};
  vm.start();
  auto next = std::chrono::steady_clock::now();
  while (!vm.finished()) {
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
  const blofeld::host::Signals signals;
  const auto profile = blofeld::app::load_profile(selected_profile(argc, argv));
  auto image = std::make_unique<blofeld::vi::FlashImage>(blofeld::vi::FlashImage::decode(
      blofeld::host::read_file(profile.device_state, blofeld::vi::FlashImage::file_bytes)));
  auto media = std::make_unique<blofeld::vi::MediaImage>(
      blofeld::host::read_file(profile.storage, blofeld::vi::MediaImage::max_bytes));
  constexpr unsigned sample_rate = 44100;
  blofeld::host::audio::Playback audio(sample_rate, "blofeld-emu");
  const auto &grant = audio.grants();
  std::cerr << "Audio: s24-in-32 stereo " << grant.rate << "Hz -> " << grant.sink_format << ' '
            << grant.sink_rate << "Hz on " << grant.sink << "; buffer=" << grant.target_frames
            << " max=" << grant.maximum_frames << " minreq=" << grant.minimum_request_frames
            << '\n';
  if (grant.rate != grant.sink_rate) { std::cerr << "Audio server performs rate conversion.\n"; }
  std::cerr << "MIDI is not connected; instrument readiness remains incomplete.\n";
  blofeld::app::AudioConsumer consumer(audio);
  blofeld::vi::VmContext vm(
      std::move(image), boot_rom(profile.boot_rom), std::move(media), consumer.output());
  drive(vm, audio, consumer, signals);
  const auto reports = vm.wait();
  audio.stop();
  status(audio, consumer, false);
  for (std::size_t index = 0; index < reports.size(); ++index) {
    const auto &report = reports[index];
    if (report.exception) { std::rethrow_exception(report.exception); }
    const auto *name = index == 0 ? "MCU" : "DSP";
    std::cerr << name << " retired=" << report.instructions;
    if (report.failure) {
      const auto &fault = *report.failure;
      std::cerr << " " << operation(fault.operation) << " space=" << fault.space << " pc=0x"
                << std::hex << fault.pc << " address=0x" << fault.address << " value=0x"
                << fault.value << std::dec << " code=" << fault.code << ": " << fault.reason;
    }
    std::cerr << '\n';
  }
  audio.check();
  std::cerr << "Startup failed: the instrument is not playable; MIDI and remaining device "
               "contracts are incomplete.\n";
  return 2;
} catch (const std::exception &error) {

  std::cerr << "blofeld-headless: " << error.what() << '\n';
  return 1;
}
