// SPDX-License-Identifier: GPL-3.0-only
#include "live.hpp"

#include "audio/consumer.hpp"
#include "audio/playback.hpp"
#include "display/lcd_image.hpp"
#include "display/publication.hpp"
#include "io/midi_port.hpp"
#include "io/pcm_output.hpp"
#include "midi/startup.hpp"
#include "process/signals.hpp"
#include "report.hpp"
#include "vm_context.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <ios>
#include <iostream>
#include <optional>
#include <ratio>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <unistd.h>

namespace blofeld::app {
namespace {
constexpr std::chrono::milliseconds startup_poll{1}, control_poll{10};
#if defined(__APPLE__)
constexpr std::chrono::microseconds input_poll{250};
#else
constexpr std::chrono::microseconds input_poll{25};
#endif

constexpr const char *yes_no(const bool value) noexcept {
  return value ? "yes" : "no";
}

}

void Live::print_audio_grants() const {
  const auto &grants = audio_.grants();
  std::cerr << "Audio: s24-in-32 stereo " << grants.rate << "Hz -> " << grants.sink_format << ' '
            << grants.sink_rate << "Hz on " << grants.sink << "; buffer=" << grants.target_frames
            << " max=" << grants.maximum_frames;
  if (grants.minimum_request_frames) { std::cerr << " quantum=" << *grants.minimum_request_frames; }
  std::cerr << '\n';
}

vi::PcmOutput Live::output() noexcept {
  return {
      .owner = this,
      .write =
          [](void *owner, const vi::PcmOutput::Block block, const std::stop_token &stop) {
            auto &live = *static_cast<Live *>(owner);
            auto &port = live.vm_->midi();
            if (!block.ticks.empty() && !port.prepare_audio(block.ticks.back(), stop)) {
              return false;
            }
            const auto output = live.consumer_.output();
            if (!output.write(output.owner, block, stop)) { return false; }
#if defined(__linux__)
            const auto quantum = live.audio_.grants().minimum_request_frames.value_or(0);
            const bool reserve = quantum != 0 && live.audio_.status().queued_frames >= quantum;
            const bool midi_quiet = live.received_ == live.received_at_pcm_ && !live.pending_ &&
                                    !port.peek_input() &&
                                    !port.peek_input(vi::MidiPort::Receiver::dsp);
            live.received_at_pcm_ = live.received_;
            live.vm_->coalesce_progress(
                midi_quiet && reserve && block.underruns == 0 &&
                std::ranges::all_of(block.frames, [](const vi::PcmOutput::Frame frame) {
                  return frame == vi::PcmOutput::Frame{};
                }));
#endif
            return true;
          },
      .close =
          [](void *owner) noexcept {
            auto &live = *static_cast<Live *>(owner);
            const auto output = live.consumer_.output();
            output.close(output.owner);
          },
#if defined(__APPLE__)
      .submit_partial =
          [](void *owner, const auto pending) noexcept {
            return static_cast<const Live *>(owner)->audio_.fills_render_quantum(pending);
          },
#endif
  };
}

void Live::publish_display() {
  if (const auto controller = vm_->take_display()) {
    constexpr vi::LcdPanel panel{.width = 128, .reverse_commons = true};
    static_cast<void>(display_.publish(vi::capture_lcd(*controller, panel)));
  }
  const auto now = std::chrono::steady_clock::now();
  if (now >= next_display_) {
    vm_->request_display();
    constexpr std::chrono::milliseconds interval{100};
    next_display_ = now + interval;
  }
}

std::optional<std::uint64_t> Live::startup(const host::Signals &signals) {
  MidiStartup startup;
  static_cast<void>(startup.poll(vm_->midi(), {}));
  const auto started = std::chrono::steady_clock::now();
  start_owners();
  while (!vm_->finished()) {
    if (signals.wait(startup_poll)) { break; }
    audio_.check();
    discard();
    control();
    const auto origin = startup.poll(vm_->midi(), this, [](void *owner) noexcept {
      const auto &live = *static_cast<Live *>(owner);
      return MidiStartup::Readiness{.application = live.vm_->mcu_wait_time().has_value(),
                                    .pcm = live.consumer_.has_pcm()};
    });
    if (origin) { return origin; }
    constexpr std::chrono::minutes guard{1};
    if (std::chrono::steady_clock::now() - started >= guard) {
      throw std::runtime_error(
          "Live startup requires original MIDI identity and PCM within one minute");
    }
  }
  const auto observed = startup.observations();
  std::cerr << "Live startup incomplete: application=" << yes_no(vm_->mcu_wait_time().has_value())
            << " pcm=" << yes_no(consumer_.has_pcm())
            << " identity-seen=" << yes_no(observed.identity) << " queries=" << observed.queries
            << " output-bytes=" << observed.output_bytes << '\n';
  return {};
}

bool Live::refill(const std::uint64_t required, const std::stop_token &stop) {
  auto &port = vm_->midi();
  if (phase_.load(std::memory_order_acquire) == Phase::booting) {
    return port.await_input(required, stop);
  }
  while (!stop.stop_requested()) {
    if (const auto known = port.input_limit(); known && *known > required) { return true; }
    audio_.check();
    const auto transfer = phase_.load(std::memory_order_acquire) == Phase::arming;
    const auto grant = transfer ? std::nullopt : consumer_.input_grant(audio_.status());
    if (grant && grant->exclusive > required && release_input(*grant, required)) { return true; }
    std::this_thread::sleep_for(input_poll);
  }
  return false;
}

void Live::status(const bool terminal) const {
  const auto host = audio_.status();
  const auto source = consumer_.statistics();
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
            << " playing=" << yes_no(host.started);
  if (!terminal) { std::cerr << '\n'; }
}

Live::Result Live::run(vi::VmContext &vm,
                       const host::Signals &signals,
                       const std::chrono::steady_clock::time_point launched) {
  if (vm_ != nullptr) { throw std::logic_error("A live session can only run once"); }
  vm_ = &vm;
  attach_audio_source();
  const auto booting = std::chrono::steady_clock::now();
  const auto origin = startup(signals);
  const auto booted = std::chrono::steady_clock::now();
  if (!origin) {
    vm.request_stop();
    const auto reports = vm.wait();
    publish_display();
    return {.status = report_execution(reports) ? 2 : 1, .save_storage = false};
  }
  consumer_.begin_at(*origin);
  const auto priming_cutoff = arm_input(*origin);
  phase_.store(Phase::arming, std::memory_order_release);
  std::cerr << "Firmware ready: original MIDI identity and PCM; musical origin=" << *origin << '\n';
  if (!vm.midi().advance_input(priming_cutoff)) {
    throw std::runtime_error("Cannot admit live priming PCM");
  }
  phase_.store(Phase::priming, std::memory_order_release);
  auto midi_output = output_worker();
  const bool terminal = ::isatty(STDERR_FILENO) != 0;
  auto next = std::chrono::steady_clock::now();
  bool announced = false;
  while (!vm.finished()) {
    control();
    if (phase_.load(std::memory_order_acquire) == Phase::playing && !announced) {
      const std::chrono::duration<double> boot = booted - booting;
      const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - launched;
      std::cerr << "Live MIDI input active: native audio started. Firmware boot=" << boot.count()
                << " s, startup=" << elapsed.count() << " s\n";
      announced = true;
    }
    discard_midi_replies();
    if (signals.wait(control_poll) || audio_.status().failed) { break; }
    const auto now = std::chrono::steady_clock::now();
    if (now >= next) {
      status(terminal);
      constexpr std::chrono::milliseconds tty_interval{250};
      constexpr std::chrono::seconds redirected_interval{1};
      next = now + (terminal ? tty_interval : redirected_interval);
    }
  }
  midi_output.request_stop();
  vm.request_stop();
  audio_.stop();
  const auto reports = vm.wait();
  if (midi_output.joinable()) { midi_output.join(); }
  publish_display();
  if (terminal) { std::cerr << '\n'; }
  status(false);
  const auto final_phase = phase_.exchange(Phase::closed);
  const bool playing = final_phase == Phase::playing;
  std::cerr << "Live MIDI bytes admitted=" << received_ << " pre-live=" << ignored_
            << " late-packets=" << late_packets_ << " without-correlation=" << uncorrelated_packets_
            << " maximum-late-ticks=" << maximum_late_ticks_
            << " clock-ready=" << yes_no(clock_ready()) << " input-active=" << yes_no(playing)
            << '\n';
  check_platform();
  if (!report_execution(reports)) {
    audio_.check();
    return {.status = 1, .save_storage = false};
  }
  audio_.check();
  return {.status = playing ? 0 : 2, .save_storage = true};
}
}
