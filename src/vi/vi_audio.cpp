// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_sink.hpp"
#include "audio/pcm_words.hpp"
#include "audio/serial_pcm.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_esai_registers.hpp"
#include "vi/vi.hpp"

#include <cstdint>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>

namespace blofeld::vi {
void VirtualInstrument::attach_pcm_output(PcmFrameSink &sink) {
  require_healthy();
  if (!config_.pcm) { throw Fault("External audio requires a decoded PCM route"); }
  if (vm_.pcm_output == &sink) { throw Fault("External audio sink is already bound"); }
  sink.check();
  sink.claim_source(vm_.identity);
  retire_pcm_output();
  vm_.pcm_output = &sink;
}

void VirtualInstrument::retire_pcm_output() const noexcept {
  if (vm_.pcm_output != nullptr) { vm_.pcm_output->invalidate(); }
}

void VirtualInstrument::mark_fault() noexcept {
  vm_.faulted = true;
  retire_pcm_output();
}

void VirtualInstrument::check_pcm_output() const {
  if (vm_.pcm_output == nullptr) { return; }
  try {
    vm_.pcm_output->check();
  } catch (const Fault &error) {
    throw Fault("Reference PCM output check at=" + std::to_string(ticks()) +
                " mcu-pc=" + mcu_pc_context() + " dsp-pc=" + hex(vm_.dsp.cpu().registers().pc, 6) +
                ": " + error.what());
  }
}

void VirtualInstrument::trace_pcm(const SerialPcmFrame &frame,
                                  const std::string_view source) const {
  if (execution_.trace == nullptr) { return; }
  if (!vm_.pcm) { throw Fault("PCM frame has no output owner"); }
  const auto &pcm = *vm_.pcm;
  auto &trace = *execution_.trace;
  trace << "PCM-FRAME index=" << pcm.completed_frames() << " first=" << frame.first_tick
        << " end=" << frame.last_tick << " channels=" << frame.channels << " source=" << source
        << " samples=";
  for (unsigned channel = 0; channel < frame.channels; ++channel) {
    if (channel != 0) { trace << ','; }
    trace << frame.samples.at(channel);
  }
  trace << '\n';
}

void VirtualInstrument::observe_pcm(const std::uint64_t at, const std::string_view source) {
  if (!config_.pcm || !vm_.pcm) { return; }
  auto &pcm = *vm_.pcm;
  const auto &route = *config_.pcm;
  const auto instance = route.instance == 0 ? DspEsaiInstance::primary : DspEsaiInstance::secondary;
  try {
    const auto slot = vm_.dsp.bus().esai_transmitted_slot(instance);
    if (!slot) { return; }
    std::uint64_t half_period = 0;
    if (vm_.audio_clock && config_.audio_clock &&
        (config_.audio_clock->destinations & (1U << route.instance)) != 0) {
      half_period = vm_.audio_clock->half_period_ticks();
    }
    const auto frame =
        pcm.observe(*slot, route.lane, {.ticks = at, .high_half_period = half_period});
    if (!frame) { return; }
    if (vm_.pcm_output != nullptr) { vm_.pcm_output->accept(*frame); }
    trace_pcm(*frame, source);
  } catch (const Fault &error) {
    const auto base = route.instance == 0 ? 0xffffa0U : 0xffff80U;
    const auto *const space = route.instance == 0 ? "X:" : "Y:";
    throw Fault("PCM transmitter word operation=read clock=" + std::string(space) +
                hex(base + esai::tccr, 6) + " data=" + space + hex(base + route.lane, 6) +
                " at=" + std::to_string(at) + " source=" + std::string(source) +
                " mcu-pc=" + mcu_pc_context() + " dsp-pc=" + hex(vm_.dsp.cpu().registers().pc, 6) +
                ": " + error.what());
  }
}
}
