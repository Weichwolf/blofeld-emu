// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_sink.hpp"
#include "audio/pcm_wave.hpp"
#include "audio/pcm_words.hpp"
#include "audio/serial_pcm.hpp"
#include "board/reference_board.hpp"
#include "board/reference_context.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_esai_registers.hpp"

#include <cstdint>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>

namespace blofeld::reference {
void Board::trace_pcm(const SerialPcmFrame &frame, const std::string_view source) const {
  if (execution_.trace == nullptr) { return; }
  if (!control_.pcm) { throw Fault("PCM frame has no output owner"); }
  auto &trace = *execution_.trace;
  trace << "PCM-FRAME index=" << control_.pcm->completed_frames() << " first=" << frame.first_tick
        << " end=" << frame.last_tick << " channels=" << frame.channels << " source=" << source
        << " samples=";
  for (unsigned channel = 0; channel < frame.channels; ++channel) {
    if (channel != 0) { trace << ','; }
    trace << frame.samples.at(channel);
  }
  trace << '\n';
}

void Board::observe_pcm(const std::uint64_t at, const std::string_view source) {
  if (!config_.pcm || !control_.pcm) { return; }
  const auto &route = *config_.pcm;
  const auto instance = route.instance == 0 ? DspEsaiInstance::primary : DspEsaiInstance::secondary;
  try {
    const auto slot = native_->dsp.bus().esai_transmitted_slot(instance);
    if (!slot) { return; }
    std::uint64_t half_period = 0;
    if (control_.audio_clock && config_.audio_clock &&
        (config_.audio_clock->destinations & (1U << route.instance)) != 0) {
      half_period = control_.audio_clock->half_period_ticks();
    } else {
      for (const auto &clock : config_.dsp_clocks) {
        if (clock.index == route.instance) { half_period = clock.half_period_ticks; }
      }
    }
    const auto frame =
        control_.pcm->observe(*slot, route.lane, {.ticks = at, .high_half_period = half_period});
    if (at >= route.after_ticks) { control_.pcm_stamp = SerialPcmStamp{.ticks = at}; }
    if (!frame) { return; }
    if (control_.recording) { control_.recording.append(*frame); }
    if (pcm_output_ != nullptr) { pcm_output_->accept(*frame); }
    control_.last_pcm = frame;
    trace_pcm(*frame, source);
  } catch (const Fault &error) {
    const auto base = route.instance == 0 ? 0xffffa0U : 0xffff80U;
    const auto *const space = route.instance == 0 ? "X:" : "Y:";
    throw Fault("PCM transmitter word operation=read clock=" + std::string(space) +
                hex(base + esai::tccr, 6) + " data=" + space + hex(base + route.lane, 6) +
                " at=" + std::to_string(at) + " source=" + std::string(source) +
                " mcu-pc=" + mcu_pc_context(native_->mcu) +
                " dsp-pc=" + hex(native_->dsp.cpu().registers().pc, 6) + ": " + error.what());
  }
}

void Board::save_audio() const {
  if (!execution_.audio) { return; }
  if (!control_.recording) { throw Fault("WAV output has no PCM recording"); }
  save_pcm_wave(execution_.audio->path,
                control_.recording,
                {.timebase_hz = common_hz(), .sample_rate = execution_.audio->sample_rate});
}
}
