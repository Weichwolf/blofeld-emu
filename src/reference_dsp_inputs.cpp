// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/dsp_timers.hpp"
#include "blofeld/hc12.hpp"
#include "reference_board.hpp"

#include <cstdint>
#include <optional>
#include <ostream>
#include <string>

namespace blofeld::reference {
void Board::apply_dsp(const DspInput &input) {
  if (input.synchronized && !input.level.has_value()) {
    throw Fault("Cannot synchronize an unknown reference DSP TIO input");
  }
  try {
    auto &timers = native_->dsp.bus().timers();
    timers.set_tio_drive(input.index, input.level);
    if (input.synchronized && input.level.has_value()) {
      timers.observe_synchronized_tio(input.index, input.level.value());
    }
  } catch (const Fault &error) {
    throw Fault("Reference DSP TIO input write index=" + std::to_string(input.index) +
                " mcu-pc=" + hex(native_->mcu.cpu().state().pc) +
                " dsp-pc=" + hex(native_->dsp.cpu().state().pc, 6) + ": " + error.what());
  }
}

DspExternalEvents Board::dsp_inputs() {
  if (config_.dsp_events.empty()) { return {}; }
  return {.next_ticks = [this]() -> std::optional<std::uint64_t> {
            if (control_.next_dsp_event == config_.dsp_events.size()) { return std::nullopt; }
            return config_.dsp_events.at(control_.next_dsp_event).ticks;
          },
          .observe =
              [this](const std::uint64_t at) {
                const auto &input = config_.dsp_events.at(control_.next_dsp_event);
                if (at != input.ticks) { throw Fault("DSP input source changed its event time"); }
                apply_dsp(input);
                if (execution_.trace != nullptr) {
                  *execution_.trace << "DSP-INPUT requested=" << input.ticks << " actual=" << at
                                    << " tio=" << input.index << " level=";
                  if (input.level.has_value()) {
                    *execution_.trace << (*input.level ? "high" : "low");
                  } else {
                    *execution_.trace << "unknown";
                  }
                  *execution_.trace << " synchronized=" << input.synchronized
                                    << " mcu-pc=" << hex(native_->mcu.cpu().state().pc)
                                    << " dsp-pc=" << hex(native_->dsp.cpu().state().pc, 6) << '\n';
                }
                ++control_.next_dsp_event;
              },
          .order = config_.dsp_input_order,
          .maximum_events = config_.maximum_dsp_inputs_per_advance};
}
}
