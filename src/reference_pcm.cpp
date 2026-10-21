// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_esai.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/pcm_wave.hpp"
#include "blofeld/serial_pcm.hpp"
#include "reference_board.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>

namespace blofeld::reference {
namespace {
std::optional<bool> level(const unsigned mask, const unsigned known, const unsigned bits) {
  return (known & mask) == 0 ? std::nullopt : std::optional{(bits & mask) != 0};
}

const char *name(const std::optional<bool> value) {
  if (!value) { return "unknown"; }
  return *value ? "1" : "0";
}
}

SerialPcmPins Board::pcm_pins() const {
  if (!config_.pcm) { throw Fault("PCM output route is absent"); }
  const auto route = *config_.pcm;
  const bool second = route.instance == 1;
  const auto serial = second ? DspEsaiInstance::secondary : DspEsaiInstance::primary;
  auto &bus = native_->dsp.bus();
  constexpr unsigned clock_pads = 0x18;
  if ((bus.gpio(second ? DspPort::e : DspPort::c).outputs().peripheral & clock_pads) !=
      clock_pads) {
    throw Fault("PCM SCKT/FST peripheral mux is absent");
  }
  const auto clocks = bus.esai_clock_outputs(serial);
  const auto data = bus.esai_transmit_outputs(serial);
  const auto &input = control_.esai_inputs.at(route.instance);
  return {.clock = (clocks.driven & 1U) != 0 ? level(1, clocks.known, clocks.levels) : input.clock,
          .frame_sync =
              (clocks.driven & 2U) != 0 ? level(2, clocks.known, clocks.levels) : input.frame_sync,
          .data = level(1U << route.lane, data.driven & data.known, data.levels)};
}

void Board::trace_pcm(const SerialPcmStamp stamp,
                      const SerialPcmPins pins,
                      const std::string_view source,
                      const std::optional<SerialPcmFrame> &frame) const {
  if (execution_.trace == nullptr) { return; }
  auto &trace = *execution_.trace;
  trace << "PCM-PINS end=" << stamp.ticks << " order=" << stamp.order << " source=" << source
        << " clock=" << name(pins.clock) << " fs=" << name(pins.frame_sync)
        << " data=" << name(pins.data) << " mcu-pc=" << hex(native_->mcu.cpu().state().pc)
        << " dsp-pc=" << hex(native_->dsp.cpu().state().pc, 6) << '\n';
  if (!frame) { return; }
  if (!control_.pcm) { throw Fault("PCM frame has no receiver"); }
  trace << "PCM-FRAME index=" << control_.pcm->completed_frames() << " first=" << frame->first_tick
        << " first-order=" << frame->first_order << " end=" << frame->last_tick
        << " order=" << frame->last_order << " channels=" << frame->channels << " samples=";
  for (unsigned channel = 0; channel < frame->channels; ++channel) {
    if (channel != 0) { trace << ','; }
    trace << frame->samples.at(channel);
  }
  trace << '\n';
}

void Board::observe_pcm(const std::uint64_t at, const std::string_view source) {
  if (!config_.pcm || !control_.pcm || at < config_.pcm->after_ticks) { return; }
  try {
    auto stamp = SerialPcmStamp{.ticks = at};
    if (control_.pcm_stamp && control_.pcm_stamp->ticks == at) {
      if (control_.pcm_stamp->order == std::numeric_limits<std::uint64_t>::max()) {
        throw Fault("PCM same-tick observation order exhausted");
      }
      stamp.order = control_.pcm_stamp->order + 1;
    }
    const auto pins = pcm_pins();
    const auto frame = control_.pcm->observe_ordered(stamp, pins);
    control_.pcm_stamp = stamp;
    if (frame) {
      if (control_.recording) { control_.recording.append(*frame); }
      control_.last_pcm = frame;
    }
    trace_pcm(stamp, pins, source, frame);
  } catch (const Fault &error) {
    const auto route = *config_.pcm;
    constexpr unsigned primary = 0xffffa0;
    constexpr unsigned secondary = 0xffff80;
    constexpr unsigned clock_control = 22;
    const auto base = route.instance == 0 ? primary : secondary;
    const auto *const space = route.instance == 0 ? "X:" : "Y:";
    throw Fault("Reference PCM ESAI pin read clock=" + std::string(space) +
                hex(base + clock_control, 6) + " data=" + space + hex(base + route.lane, 6) +
                " at=" + std::to_string(at) + " source=" + std::string(source) +
                " mcu-pc=" + hex(native_->mcu.cpu().state().pc) +
                " dsp-pc=" + hex(native_->dsp.cpu().state().pc, 6) + ": " + error.what());
  }
}

void Board::save_audio() const {
  if (!execution_.audio) { return; }
  if (!control_.recording) { throw Fault("WAV output has no PCM recording"); }
  save_pcm_wave(execution_.audio->path,
                control_.recording,
                {.timebase_hz = native_->dsp.run_clock().timebase_hz(),
                 .sample_rate = execution_.audio->sample_rate});
}
}
