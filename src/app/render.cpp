// SPDX-License-Identifier: GPL-3.0-only
#include "render.hpp"

#include "io/midi_port.hpp"
#include "io/pcm_output.hpp"
#include "midi/startup.hpp"
#include "options.hpp"
#include "process/signals.hpp"
#include "report.hpp"
#include "vm_context.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace blofeld::app {
namespace {
constexpr unsigned rate = 44100;
constexpr std::chrono::seconds startup_limit{10};
constexpr std::chrono::milliseconds startup_poll{1}, control_poll{10};

const std::filesystem::path &required_path(const std::optional<std::filesystem::path> &path) {
  if (!path) { throw std::invalid_argument("Offline render requires MIDI and WAV paths"); }
  return *path;
}

std::uint64_t tail_ticks(const std::uint64_t seconds) {
  if (seconds > std::numeric_limits<std::uint64_t>::max() / vi::PcmOutput::time_hz) {
    throw std::overflow_error("Render tail exceeds the instrument timeline");
  }
  return seconds * vi::PcmOutput::time_hz;
}
}

OfflineRender::OfflineRender(const Options &options)
    : wav_(required_path(options.wav), rate, true),
      midi_(required_path(options.midi)),
      tail_(tail_ticks(options.tail_seconds)) {}

vi::PcmOutput OfflineRender::output() noexcept {
  return {.owner = this,
          .write =
              [](void *owner, const vi::PcmOutput::Block block, const std::stop_token &stop) {
                auto &render = *static_cast<OfflineRender *>(owner);
                // A core may finish an instruction across an exclusive input frontier.
                // Hold its output until the producer publishes either later input or
                // the EOF/output boundary; no speculative frame reaches the file.
                auto &port = render.vm_->midi();
                if (!block.ticks.empty()) {
                  if (const auto limit = port.input_limit();
                      limit && block.ticks.back() >= *limit &&
                      !port.await_input(block.ticks.back(), stop)) {
                    return false;
                  }
                }
                const auto output = render.wav_.output();
                const auto accepted = output.write(output.owner, block, stop);
                if (render.wav_.complete()) { render.vm_->request_stop(); }
                return accepted;
              },
          .close =
              [](void *owner) noexcept {
                auto &render = *static_cast<OfflineRender *>(owner);
                const auto output = render.wav_.output();
                output.close(output.owner);
              }};
}

std::optional<std::uint64_t> OfflineRender::startup(vi::VmContext &vm,
                                                    const host::Signals &signals) {
  MidiStartup startup;
  static_cast<void>(startup.poll(vm.midi(), {}));
  vm.start();
  while (!vm.finished()) {
    if (signals.wait(startup_poll)) { break; }
    const MidiStartup::Readiness readiness{.application = vm.mcu_wait_time().has_value(),
                                           .pcm = wav_.has_pcm()};
    if (const auto origin = startup.poll(vm.midi(), readiness)) { return origin; }
    if (std::chrono::steady_clock::now() - started_ >= startup_limit) {
      std::cerr
          << "Startup failed: no original MIDI identity and valid PCM within 10 host seconds.\n";
      break;
    }
  }
  const auto observed = startup.observations();
  std::cerr << "Startup incomplete: queries=" << observed.queries
            << " MIDI-out-bytes=" << observed.output_bytes
            << " identity=" << (observed.identity ? "yes" : "no")
            << " application=" << (vm.mcu_wait_time() ? "yes" : "no")
            << " PCM=" << (wav_.has_pcm() ? "yes" : "no") << '\n';
  return {};
}

void OfflineRender::feed(vi::MidiPort &port,
                         const std::uint64_t origin,
                         const std::stop_token &stop) {
  const auto end = midi_.feed(port, origin, stop);
  if (!end) { return; }
  if (*end > std::numeric_limits<std::uint64_t>::max() - tail_) {
    throw std::overflow_error("MIDI duration and render tail exceed the instrument timeline");
  }
  // Publish the output boundary before releasing the exclusive MIDI EOF barrier.
  wav_.end_at(*end + tail_);
  if (!port.finish_input()) { throw std::runtime_error("Cannot finish offline MIDI input"); }
}

int OfflineRender::run(vi::VmContext &vm,
                       const host::Signals &signals,
                       const std::chrono::steady_clock::time_point launched) {
  if (vm_ != nullptr) { throw std::logic_error("An offline render can only run once"); }
  vm_ = &vm;
  launched_ = launched;
  started_ = std::chrono::steady_clock::now();
  const auto origin = startup(vm, signals);
  if (!origin) {
    vm.request_stop();
    const auto reports = vm.wait();
    startup_statistics();
    return report_execution(reports) ? 2 : 1;
  }
  ready_ = std::chrono::steady_clock::now();
  wav_.begin_at(*origin);
  std::cerr << "Ready: original MIDI identity and PCM; musical origin=" << *origin << " ticks\n";
  std::exception_ptr input_failure;
  std::jthread input([&](const std::stop_token &stop) {
    try {
      feed(vm.midi(), *origin, stop);
    } catch (...) {
      input_failure = std::current_exception();
      vm.request_stop();
    }
  });
  while (!vm.finished()) {
    if (signals.wait(control_poll)) {
      vm.request_stop();
      break;
    }
    while (vm.midi().read_output()) {} // Drain ordinary firmware replies, never synthesize them.
  }
  input.request_stop();
  vm.request_stop();
  const auto reports = vm.wait();
  input.join();
  finished_ = std::chrono::steady_clock::now();
  if (!report_execution(reports)) { return 1; }
  if (input_failure) { std::rethrow_exception(input_failure); }
  if (!wav_.complete()) {
    std::cerr << "Render incomplete: " << wav_.frames()
              << " original stereo frames accepted; no WAV published.\n";
    return 2;
  }
  wav_.finish();
  statistics();
  return 0;
}

void OfflineRender::startup_statistics() const {
  const auto frames = wav_.source_frames();
  const double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
  std::cerr << std::fixed << std::setprecision(3) << "Startup PCM prefix: " << frames
            << " validated stereo frames at " << rate << "Hz; host=" << seconds
            << "s frames/s=" << static_cast<double>(frames) / seconds
            << " startup-prefix-RTF=" << static_cast<double>(frames) / (rate * seconds)
            << "x; instrument not ready; no WAV published.\n";
}

void OfflineRender::statistics() const {
  const auto frames = wav_.frames();
  const double audio = static_cast<double>(frames) / rate;
  const double preparation = std::chrono::duration<double>(started_ - launched_).count();
  const double startup = std::chrono::duration<double>(ready_ - started_).count();
  const double active = std::chrono::duration<double>(finished_ - ready_).count();
  const double total = std::chrono::duration<double>(finished_ - launched_).count();
  std::cerr << std::fixed << std::setprecision(3) << "Rendered: " << frames << " stereo frames, "
            << audio << "s at " << rate << "Hz; preparation=" << preparation
            << "s startup=" << startup << "s active=" << active << "s total=" << total
            << "s; frames/s=" << static_cast<double>(frames) / active << " RTF=" << audio / active
            << "x total-RTF=" << audio / total
            << "x; device-underruns=0; MIDI deadline deviation=unmeasured\n";
}
}
