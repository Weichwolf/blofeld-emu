// SPDX-License-Identifier: GPL-3.0-only
#include "board/reference_board.hpp"
#include "board/reference_context.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_timers.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <ostream>
#include <string>

namespace blofeld::reference {
namespace {
const char *level_name(const std::optional<bool> level) {
  if (!level.has_value()) { return "unknown"; }
  return level.value() ? "high" : "low";
}

DspEsaiInstance instance(const unsigned index) {
  if (index > 1) { throw Fault("Reference DSP ESAI index exceeds one"); }
  return index == 0 ? DspEsaiInstance::primary : DspEsaiInstance::secondary;
}
}

void Board::apply_dsp(const DspInput &input) {
  try {
    switch (input.kind) {
      case DspInputKind::tio: {
        if (input.synchronized && !input.level.has_value()) {
          throw Fault("Cannot synchronize an unknown reference DSP TIO input");
        }
        auto &timers = native_->dsp.bus().timers();
        timers.set_tio_drive(input.index, input.level);
        if (input.synchronized && input.level.has_value()) {
          timers.observe_synchronized_tio(input.index, input.level.value());
        }
        control_.held_synchronized_tio.at(input.index) =
            input.synchronized ? input.level : std::nullopt;
        break;
      }
      case DspInputKind::esai:
        native_->dsp.bus().observe_esai_transmit(instance(input.index), input.esai);
        control_.esai_inputs.at(input.index) = {.clock = input.esai.clock,
                                                .frame_sync = input.esai.frame_sync};
        break;
      default: throw Fault("Unknown reference DSP input kind");
    }
  } catch (const Fault &error) {
    const auto *const kind = input.kind == DspInputKind::tio ? "TIO" : "ESAI";
    throw Fault("Reference DSP " + std::string(kind) + " input write index=" +
                std::to_string(input.index) + " mcu-pc=" + mcu_pc_context(native_->mcu) +
                " dsp-pc=" + hex(native_->dsp.cpu().registers().pc, 6) + ": " + error.what());
  }
}

void Board::observe_held_dsp_tio() const {
  // Peripheral RESET clears the synchronizer, not the external source. Reapply
  // only its last qualified observation; a queued future MIDI edge is not a pad.
  for (std::size_t index = 0; index < control_.held_synchronized_tio.size(); ++index) {
    const auto level = control_.held_synchronized_tio.at(index);
    if (level.has_value()) {
      native_->dsp.bus().timers().observe_synchronized_tio(static_cast<unsigned>(index),
                                                           level.value());
    }
  }
}

void Board::trace_dsp_input(const DspInput &input, const std::uint64_t at) {
  if (execution_.trace == nullptr) { return; }
  auto &trace = *execution_.trace;
  if (input.kind == DspInputKind::tio) {
    trace << "DSP-INPUT requested=" << input.ticks << " actual=" << at << " tio=" << input.index
          << " level=" << level_name(input.level) << " synchronized=" << input.synchronized;
  } else {
    const auto serial = instance(input.index);
    const auto data = native_->dsp.bus().esai_transmit_outputs(serial);
    const auto clocks = native_->dsp.bus().esai_clock_outputs(serial);
    trace << "DSP-ESAI requested=" << input.ticks << " actual=" << at << " instance=" << input.index
          << " clock=" << level_name(input.esai.clock)
          << " fs=" << level_name(input.esai.frame_sync)
          << " hckt=" << level_name(input.esai.high_clock)
          << " reset-edges=" << (input.esai.timing == DspEsaiClockTiming::reference_reset_edges)
          << " tx-driven=" << hex(data.driven, 2) << " tx-known=" << hex(data.known, 2)
          << " tx-levels=" << hex(data.levels, 2) << " clocks-driven=" << hex(clocks.driven, 2)
          << " clocks-known=" << hex(clocks.known, 2) << " clocks-levels=" << hex(clocks.levels, 2);
  }
  trace << " mcu-pc=" << mcu_pc_context(native_->mcu)
        << " dsp-pc=" << hex(native_->dsp.cpu().registers().pc, 6) << '\n';
}

}
