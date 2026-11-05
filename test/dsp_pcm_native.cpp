// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_pcm_native.hpp"

#include "audio/serial_pcm.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "dsp_pcm_program.hpp"
#include "dsp_rom_program.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <string>

namespace blofeld::test {
namespace {
constexpr unsigned extal = 12000000;
constexpr unsigned hold_periods = 2;
constexpr unsigned recovery_periods = 2;
constexpr unsigned events_per_period = 4;
constexpr auto timing = DspEsaiClockTiming::reference_reset_edges;
constexpr auto execution =
    OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts;

DspRunClockInputs clocks(const DspPcmConfig &config) {
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (config.half_period == 0 || config.observations == 0 || config.observations == maximum ||
      config.baseline >= config.start ||
      config.observations - 1 > (maximum - config.start) / config.half_period) {
    throw Fault("DSP PCM fixture requires a bounded ordered clock source");
  }
  return {.extal_hz = extal,
          .timebase_hz = config.timebase,
          .switch_timing = DspClockSwitch::reference_cycle_boundary};
}

const char *operation(const dsp56300::FaultKind kind) {
  using enum dsp56300::FaultKind;
  switch (kind) {
    case io_read:
    case memory_read: return "read";
    case io_write:
    case memory_write: return "write";
    case io_bit: return "bit";
    case none: return "none";
    case unsupported_instruction:
    case unsupported_mode:
    case stack:
    case invalid_instruction:
    case host_event: return "execute";
    default: return "invalid-operation";
  }
}
}

DspPcmRun::DspPcmRun(const DspPcmConfig config) : config_(config), machine_(clocks(config)) {
  const auto program = dsp_pcm_program(config.secondary);
  machine_.bus().install_rom({.start = DspRomProgram::boot, .words = program.words()});
  const auto instance = config.secondary ? DspEsaiInstance::secondary : DspEsaiInstance::primary;
  timeline_ = std::make_unique<OwnDspTimeline>(
      machine_,
      execution,
      0,
      OwnDspResetInputs{
          .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = recovery_periods});
  machine_.bus().observe_esai_transmit(instance, {.high_clock = false, .timing = timing});
  const auto hold = hold_periods * machine_.clock().extal_period_ticks();
  const auto recovered = hold + (recovery_periods * machine_.clock().extal_period_ticks());
  if (timeline_->advance_reset_to(hold) != hold) { throw Fault("DSP PCM fixture lost reset hold"); }
  timeline_->release_hardware_reset();
  if (timeline_->advance_reset_to(recovered) != recovered - hold ||
      machine_.clock().hardware_reset_active() || config.baseline < recovered) {
    throw Fault("DSP PCM fixture requires completed reset recovery before observations");
  }
  horizon_ = recovered;
}

std::optional<std::uint64_t> DspPcmRun::next_input() const {
  if (cursor_ == 0) { return config_.baseline; }
  if (cursor_ == config_.observations + 1) { return std::nullopt; }
  return config_.start + ((cursor_ - 1) * config_.half_period);
}

void DspPcmRun::observe(const std::uint64_t ticks) {
  const auto before = machine_.cpu().registers();
  const auto progress = machine_.cpu().progress();
  const auto clock_tick = machine_.clock().elapsed_ticks();
  const auto instance = config_.secondary ? DspEsaiInstance::secondary : DspEsaiInstance::primary;
  if (cursor_ != 0) {
    machine_.bus().observe_esai_transmit(instance,
                                         {.high_clock = cursor_ % 2 != 0, .timing = timing});
  }
  const auto clock = machine_.bus().esai_clock_outputs(instance);
  const auto data = machine_.bus().esai_transmit_outputs(instance);
  if (clock.driven != 3 || clock.known != 3) {
    throw Fault("DSP PCM native MOVEP setup did not drive both known clock outputs");
  }
  const auto frame = receiver_.observe(ticks,
                                       {.clock = (clock.levels & 1U) != 0,
                                        .frame_sync = (clock.levels & 2U) != 0,
                                        .data = (data.driven & data.known & 1U) == 0
                                                    ? std::nullopt
                                                    : std::optional{(data.levels & 1U) != 0}});
  if (frame) { frames_.push_back(*frame); }
  if (machine_.cpu().registers() != before || machine_.cpu().progress() != progress ||
      machine_.clock().elapsed_ticks() != clock_tick) {
    throw Fault("DSP PCM observation changed native CPU state or charged a clock period");
  }
  ++cursor_;
}

void DspPcmRun::drain(const std::uint64_t through) {
  while (const auto next = next_input()) {
    if (*next > through) { return; }
    if (*next < horizon_ || events_in_period_ == events_per_period) {
      throw Fault("DSP PCM fixture input moved backwards or exhausted its period budget");
    }
    observe(*next);
    ++events_in_period_;
  }
}

void DspPcmRun::check_native(const std::optional<dsp56300::InstructionProgress> &progress) const {
  if (!progress || progress->fault.kind == dsp56300::FaultKind::none) { return; }
  const auto fault = progress->fault;
  constexpr std::array spaces{"P:", "X:", "Y:"};
  std::string cause;
  if (const auto &failure = machine_.io().failure(); failure && failure->cause) {
    try {
      std::rethrow_exception(failure->cause);
    } catch (const std::exception &error) { cause = error.what(); }
  }
  throw Fault("DSP PCM native " + std::string(operation(fault.kind)) + " " +
              spaces.at(static_cast<unsigned>(fault.space)) + hex(fault.address, 6) +
              " pc=" + hex(fault.pc, 6) + " value=" + hex(fault.value, 6) +
              " tick=" + std::to_string(machine_.clock().elapsed_ticks()) + ": " + cause);
}

void DspPcmRun::advance_to(const std::uint64_t horizon, const unsigned maximum_cycles) {
  if (faulted_ || horizon < horizon_ || maximum_cycles == 0) {
    throw Fault("DSP PCM fixture requires a healthy monotonic bounded advance");
  }
  faulted_ = true;
  for (unsigned consumed = 0;; ++consumed) {
    const auto deadline = timeline_->next_deadline();
    if (deadline <= horizon && consumed == maximum_cycles) {
      throw Fault("DSP PCM fixture exceeded its native period budget");
    }
    drain(std::min(deadline, horizon));
    const auto result = timeline_->advance_one(std::min(deadline, horizon),
                                               OwnDspBoundary::through,
                                               {.cycle = cycles_,
                                                .timing = DspDmaTiming::reference_sampled_cycles,
                                                .grant = DspDmaGrant::available});
    if (!result) { break; }
    ++cycles_;
    events_in_period_ = 0;
    horizon_ = deadline;
    check_native(result->native);
  }
  horizon_ = horizon;
  faulted_ = false;
}

}
