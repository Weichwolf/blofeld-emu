// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/sci.hpp"
#include "io/usb.hpp"
#include "mcu/channels.hpp"
#include "mcu/clock.hpp"
#include "mcu/timer.hpp"

#include <algorithm>
#include <cpu12/core.hpp>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <stop_token>

namespace blofeld::vi::mcu {
struct Devices {
  static constexpr std::uint8_t all_bits = 0xff;

  explicit Devices(const std::stop_token &cancellation) : stop(cancellation) {}

  const std::stop_token &stop;
  PeerPort *peer = nullptr;
  ExecutionTime *time = nullptr;
  MidiPort *midi = nullptr;
  Timer timer;
  Clock clock;
  Sci sci;
  Usb usb;
  Timer::Cycle idle = 0;
  const char *midi_error = nullptr;

  [[nodiscard]] std::optional<std::uint64_t> input_limit() const noexcept {
    std::uint64_t ticks = 0;
    if (blofeld_mcu_midi_limit(midi, ticks)) { return ticks; }
    return {};
  }

  [[nodiscard]] std::optional<MidiPort::Event> input() const noexcept {
    MidiPort::Event event;
    if (blofeld_mcu_midi_peek(midi, event)) { return event; }
    return {};
  }

  [[nodiscard]] std::optional<std::uint64_t> input_tick() const noexcept {
    if (midi == nullptr) { return {}; }
    auto next = input_limit();
    if (const auto event = input(); event && (!next || event->ticks < *next)) {
      next = event->ticks;
    }
    return next;
  }

  bool wait_input(const std::uint64_t ticks) noexcept {
    if (time != nullptr && !blofeld_mcu_time_publish(time, clock.ticks())) { return false; }
    return blofeld_mcu_midi_await(midi, ticks, stop);
  }

  bool input_ready() noexcept {
    if (const auto limit = input_limit(); limit && clock.ticks() >= *limit) {
      return wait_input(*limit);
    }
    return blofeld_mcu_midi_await(midi, clock.ticks(), stop);
  }

  bool point(const Timer::Cycle now) noexcept {
    const auto before = clock.cycles();
    const auto ticks = clock.ticks();
    const auto period = clock.bus_period();
    if (!timer.advance(now) || !clock.advance(now) || !sci.advance(now)) { return false; }
    if (midi == nullptr) { return true; }
    while (const auto output = sci.take_output()) {
      if (output->bits != 8 || output->word > all_bits || output->cycle < before ||
          output->cycle > now) {
        midi_error = "SCI MIDI output has an unsupported format or completion time";
        return false;
      }
      if (!blofeld_mcu_midi_write(midi,
                                  {.ticks = ticks + ((output->cycle - before) * period),
                                   .byte = static_cast<std::uint8_t>(output->word)})) {
        return false;
      }
    }
    while (const auto event = input()) {
      if (event->ticks > clock.ticks()) { break; }
      if (!sci.receive({.word = event->byte})) {
        midi_error = "SCI could not accept the scheduled MIDI character";
        return false;
      }
      blofeld_mcu_midi_pop(midi);
    }
    midi->publish_time(clock.ticks());
    return midi->error() == nullptr;
  }

  bool input_point(const Timer::Cycle now) noexcept {
    while (const auto limit = input_limit()) {
      if (*limit <= clock.ticks() || (*limit - clock.ticks()) % clock.bus_period() == 0) { break; }
      const auto boundary = clock.at_ticks(*limit);
      if (!boundary) { return false; }
      if (now < *boundary) { break; }
      // An unaligned exclusive frontier cannot authorize the following bus edge.
      if (!point(*boundary - 1) || !wait_input(*limit)) { return false; }
    }
    return point(now);
  }

  [[nodiscard]] std::optional<std::uint64_t> timestamp(const cpu12::Core &core) const noexcept {
    const auto cycles = core.progress().estimated_cycles;
    if (idle > std::numeric_limits<Timer::Cycle>::max() - cycles) { return {}; }
    auto projected = clock;
    if (!projected.advance(cycles + idle)) { return {}; }
    return projected.ticks();
  }

  bool advance(const cpu12::Core &core) noexcept {
    const auto cycles = core.progress().estimated_cycles;
    if (idle > std::numeric_limits<Timer::Cycle>::max() - cycles) { return false; }
    const auto now = cycles + idle;
    if (midi == nullptr) { return point(now); }
    for (;;) {
      if (!input_ready()) { return false; }
      auto next = now;
      if (const auto event = clock.deadline(); event && *event < next) { next = *event; }
      if (const auto tick = input_tick()) {
        const auto event = clock.at_ticks(*tick);
        if (!event) { return false; }
        next = std::min(next, *event);
      }
      if (!input_point(next)) { return false; }
      if (next == now) { return true; }
    }
  }

  [[nodiscard]] const char *error() const noexcept {
    if (midi_error != nullptr) { return midi_error; }
    if (midi != nullptr && midi->error() != nullptr) { return midi->error(); }
    return sci.error();
  }

  [[nodiscard]] std::optional<std::uint16_t> pending() const noexcept {
    return std::max({timer.pending(), clock.pending(), sci.pending(), usb.pending()});
  }

  [[nodiscard]] std::optional<Timer::Cycle> deadline() const noexcept {
    auto next = timer.deadline();
    const auto offer = [&next](const std::optional<Timer::Cycle> candidate) {
      if (candidate && (!next || *candidate < *next)) { next = candidate; }
    };
    offer(clock.deadline());
    offer(sci.deadline());
    if (const auto tick = input_tick()) { offer(clock.at_ticks(*tick)); }
    return next;
  }

  [[nodiscard]] bool interrupts_enabled() const noexcept {
    return peer != nullptr || timer.interrupts_enabled() || clock.interrupts_enabled() ||
           sci.interrupts_enabled() || usb.interrupts_enabled();
  }

  void activity(const cpu12::ExecutionState state) {
    switch (state) {
      case cpu12::ExecutionState::running:
        clock.activity(Clock::Activity::running);
        timer.activity(Timer::Activity::running);
        break;
      case cpu12::ExecutionState::waiting:
        if (time != nullptr) { time->publish_mcu_wait(clock.ticks()); }
        clock.activity(Clock::Activity::waiting);
        timer.activity(clock.bus_running() ? Timer::Activity::waiting : Timer::Activity::stopped);
        break;
      case cpu12::ExecutionState::stopped:
        clock.activity(Clock::Activity::stopped);
        timer.activity(Timer::Activity::stopped);
        break;
      default: throw std::logic_error("Unknown MCU execution state");
    }
    sci.activity(
        {.waiting = state == cpu12::ExecutionState::waiting, .clocked = clock.bus_running()});
  }
};
}
