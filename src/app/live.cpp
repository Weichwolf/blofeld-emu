// SPDX-License-Identifier: GPL-3.0-only
#include "live.hpp"

#include "audio/consumer.hpp"
#include "audio/playback.hpp"
#include "display/lcd_image.hpp"
#include "display/publication.hpp"
#include "io/midi_port.hpp"
#include "io/pcm_output.hpp"
#include "midi/input.hpp"
#include "midi/live_clock.hpp"
#include "midi/startup.hpp"
#include "midi/ump.hpp"
#include "options.hpp"
#include "process/signals.hpp"
#include "report.hpp"
#include "vm_context.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <ios>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <ratio>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <unistd.h>
#include <utility>

namespace blofeld::app {
namespace {
constexpr std::chrono::milliseconds startup_poll{1}, control_poll{10};
constexpr std::chrono::microseconds input_poll{250};
constexpr std::size_t packet_batch = 1024;

std::size_t queue_grant(const host::audio::Grants &grants) {
  std::cerr << "Audio: s24-in-32 stereo " << grants.rate << "Hz -> " << grants.sink_format << ' '
            << grants.sink_rate << "Hz on " << grants.sink << "; buffer=" << grants.target_frames
            << " max=" << grants.maximum_frames << '\n';
  if (!grants.position_history_frames || *grants.position_history_frames == 0) {
    throw std::runtime_error("Live MIDI requires qualified native audio positions/history grants");
  }
  return grants.maximum_frames;
}
}

Live::Live(host::audio::Playback &audio, DisplayPublication &display, const Options &options)
    : audio_(audio),
      display_(display),
      consumer_(audio, true),
      clock_(audio.grants().rate, queue_grant(audio.grants())),
      source_id_(options.midi_source),
      group_(options.midi_group) {
  const auto lead = std::chrono::duration<double>(static_cast<double>(clock_.lead_ticks()) /
                                                  vi::MidiPort::reference_hz);
  std::cerr << "Live MIDI: delivery allowance="
            << std::chrono::duration<double, std::milli>(
                   std::chrono::nanoseconds(LiveClock::delivery_ns))
                   .count()
            << "ms admission-lead~=" << std::chrono::duration<double, std::milli>(lead).count()
            << "ms; physical key-to-output latency is unmeasured.\n";
  connect();
}

void Live::connect() {
  auto sources = host::midi::Input::sources();
  const auto selected = std::ranges::find_if(sources, [&](const auto &source) {
    return source_id_ ? source.id == *source_id_ : sources.size() == 1;
  });
  if (selected != sources.end()) {
    input_ = std::make_unique<host::midi::Input>(
        host::midi::Input::Route{.source = selected->id, .group = group_});
    std::cerr << "MIDI input: " << selected->name << " ID=" << selected->id
              << " group=" << static_cast<unsigned>(group_) << '\n';
    active_input_.store(input_.get(), std::memory_order_release);
    return;
  }
  if (!sources_ || !std::ranges::equal(*sources_, sources, [](const auto &left, const auto &right) {
        return left.id == right.id && left.name == right.name;
      })) {
    std::cerr << "Waiting for native MIDI source; select an ID with --midi-source if ambiguous.\n";
    for (const auto &source : sources) {
      std::cerr << "MIDI source: " << source.id << ' ' << source.name << '\n';
    }
    sources_ = std::move(sources);
  }
}

void Live::control() {
  publish_display();
  const auto now = std::chrono::steady_clock::now();
  if (!input_ && now >= next_connect_) {
    next_connect_ = now + std::chrono::seconds(1);
    connect();
  }
}

vi::PcmOutput Live::output() noexcept {
  return {.owner = this,
          .write =
              [](void *owner, const vi::PcmOutput::Block block, const std::stop_token &stop) {
                auto &live = *static_cast<Live *>(owner);
                auto &port = live.vm_->midi();
                if (!block.ticks.empty()) {
                  if (const auto limit = port.input_limit();
                      limit && block.ticks.back() >= *limit &&
                      !port.await_input(block.ticks.back(), stop)) {
                    return false;
                  }
                }
                const auto output = live.consumer_.output();
                return output.write(output.owner, block, stop);
              },
          .close =
              [](void *owner) noexcept {
                auto &live = *static_cast<Live *>(owner);
                const auto output = live.consumer_.output();
                output.close(output.owner);
              }};
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

void Live::discard(const std::uint64_t now) {
  auto *input = active_input_.load(std::memory_order_acquire);
  if (input == nullptr) { return; }
  for (std::size_t count = 0; count < packet_batch; ++count) {
    if (!pending_) { pending_ = input->read(); }
    if (!pending_ || pending_->timestamp > now) { return; }
    ignored_ += pending_->size;
    pending_.reset();
  }
}

std::optional<std::uint64_t> Live::startup(const host::Signals &signals) {
  MidiStartup startup;
  static_cast<void>(startup.poll(vm_->midi(), {}));
  const auto started = std::chrono::steady_clock::now();
  vm_->start();
  while (!vm_->finished()) {
    if (signals.wait(startup_poll)) { return {}; }
    audio_.check();
    discard(host::midi::Input::now());
    control();
    const MidiStartup::Readiness readiness{.application = vm_->mcu_wait_time().has_value(),
                                           .pcm = consumer_.has_pcm()};
    if (const auto origin = startup.poll(vm_->midi(), readiness)) { return origin; }
    constexpr std::chrono::minutes guard{1};
    if (std::chrono::steady_clock::now() - started >= guard) {
      throw std::runtime_error(
          "Live startup requires original MIDI identity and PCM within one minute");
    }
  }
  return {};
}

bool Live::receive(host::midi::Input &input,
                   const LiveClock::Window window,
                   const std::uint64_t ready,
                   const std::stop_token &stop) {
  for (std::size_t count = 0; count < packet_batch && !stop.stop_requested(); ++count) {
    if (!pending_) { pending_ = input.read(); }
    if (!pending_ || pending_->timestamp > window.host_ns) { return true; }
    if (pending_->timestamp < ready) {
      ignored_ += pending_->size;
    } else {
      const auto ticks = clock_.event(pending_->timestamp);
      if (!vm_->midi().write_input(pending_->data(), ticks, stop)) {
        if (stop.stop_requested()) { return false; }
        const auto *error = vm_->midi().error();
        throw std::runtime_error(error != nullptr ? error : "Live MIDI admission failed");
      }
      received_ += pending_->size;
    }
    pending_.reset();
  }
  return false;
}

void Live::admit(const LiveClock::Window window) {
  auto &port = vm_->midi();
  const auto limit = port.input_limit();
  if (!limit) { throw std::runtime_error("Live MIDI lost its exclusive admission frontier"); }
  if (window.ticks > *limit && !port.advance_input(window.ticks)) {
    throw std::runtime_error("Live MIDI silence admission failed");
  }
  clock_.commit(window);
}

void Live::process(const LiveClock::Window window,
                   const std::uint64_t now,
                   std::optional<std::uint64_t> &ready,
                   const std::stop_token &stop) {
  auto *input = active_input_.load(std::memory_order_acquire);
  if (input == nullptr) {
    phase_.store(Phase::audio_ready, std::memory_order_release);
    admit(window);
    return;
  }
  if (!ready) {
    ready = now;
    phase_.store(Phase::playing, std::memory_order_release);
  }
  if (receive(*input, window, *ready, stop)) { admit(window); }
}

void Live::feed(const std::uint64_t origin, const std::stop_token &stop) {
  if (origin > std::numeric_limits<std::uint64_t>::max() - clock_.prime_ticks()) {
    throw std::overflow_error("Live priming timeline exhausted");
  }
  const auto initial = origin + clock_.prime_ticks();
  auto &port = vm_->midi();
  if (!port.advance_input(initial)) { throw std::runtime_error("Cannot admit live priming PCM"); }
  std::optional<std::uint64_t> ready;
  const auto started = std::chrono::steady_clock::now();
  while (!stop.stop_requested() && !vm_->finished()) {
    const auto polled = std::chrono::steady_clock::now();
    audio_.check();
    if (const auto point = consumer_.clock()) { clock_.observe(*point); }
    const auto now = host::midi::Input::now();
    if (const auto window = clock_.window(now); window && window->ticks >= initial) {
      process(*window, now, ready, stop);
    } else if (phase_.load(std::memory_order_relaxed) == Phase::priming) {
      discard(now);
      constexpr std::chrono::seconds guard{2};
      if (polled - started >= guard) {
        throw std::runtime_error("Live priming produced no qualified native audio clock");
      }
    }
    std::this_thread::sleep_until(polled + input_poll);
  }
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
            << " playing=" << (host.started ? "yes" : "no");
  if (!terminal) { std::cerr << '\n'; }
}

int Live::run(vi::VmContext &vm, const host::Signals &signals) {
  if (vm_ != nullptr) { throw std::logic_error("A live session can only run once"); }
  vm_ = &vm;
  const auto origin = startup(signals);
  if (!origin) {
    vm.request_stop();
    const auto reports = vm.wait();
    publish_display();
    std::cerr
        << "Live startup incomplete: original MIDI identity/application/PCM not established.\n";
    return report_execution(reports) ? 2 : 1;
  }
  consumer_.begin_at(*origin);
  phase_.store(Phase::priming, std::memory_order_release);
  std::cerr << "Firmware ready: original MIDI identity and PCM; musical origin=" << *origin << '\n';
  std::exception_ptr failure;
  std::jthread input([&](const std::stop_token &stop) {
    try {
      feed(*origin, stop);
    } catch (...) {
      failure = std::current_exception();
      vm.request_stop();
    }
  });
  const bool terminal = ::isatty(STDERR_FILENO) != 0;
  auto next = std::chrono::steady_clock::now();
  bool announced = false;
  while (!vm.finished()) {
    control();
    if (phase_.load(std::memory_order_acquire) == Phase::playing && !announced) {
      std::cerr << "Live MIDI input active: native audio clock established; correlation-budget="
                << clock_.uncertainty_ns() << "ns.\n";
      announced = true;
    }
    while (vm.midi().read_output()) {}
    if (signals.wait(control_poll) || audio_.status().failed) { break; }
    const auto now = std::chrono::steady_clock::now();
    if (now >= next) {
      status(terminal);
      constexpr std::chrono::milliseconds tty_interval{250};
      constexpr std::chrono::seconds redirected_interval{1};
      next = now + (terminal ? tty_interval : redirected_interval);
    }
  }
  input.request_stop();
  vm.request_stop();
  const auto reports = vm.wait();
  input.join();
  audio_.stop();
  publish_display();
  if (terminal) { std::cerr << '\n'; }
  status(false);
  const bool playing = phase_.exchange(Phase::closed) == Phase::playing;
  std::cerr << "Live MIDI bytes admitted=" << received_ << " pre-live=" << ignored_ << '\n';
  if (!report_execution(reports)) { return 1; }
  if (failure) { std::rethrow_exception(failure); }
  audio_.check();
  return playing ? 0 : 2;
}
}
