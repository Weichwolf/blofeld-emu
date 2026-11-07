// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "runtime/gpio_pads.hpp"
#include "runtime/instrument.hpp"
#include "runtime/instrument_context.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <ostream>
#include <string>

namespace blofeld::runtime {
namespace {
constexpr unsigned reset_mask = 1;
constexpr unsigned u_data = 0x278;
constexpr std::array u_registers{u_data, 0x27aU, 0x27bU, 0x27cU, 0x27dU};

}

bool Instrument::u0_release_write(const unsigned address, const unsigned size) const {
  return config_.mcu_u0_release && std::ranges::any_of(u_registers, [&](const unsigned reg) {
           return address <= reg && reg - address < size;
         });
}

void Instrument::update_u0_drive() {
  if (!config_.mcu_u0_release) { return; }
  const auto output = native_->mcu.bus().gpio_outputs(McuPort::u);
  const auto bias = native_->mcu.bus().gpio_biases(McuPort::u);
  const auto resolved = resolve_gpio_net(reset_mask,
                                         {.active = output.mask & reset_mask,
                                          .known = output.known & reset_mask,
                                          .levels = output.levels & reset_mask,
                                          .reduced = output.reduced_drive & reset_mask,
                                          .pull_up = bias.up & reset_mask,
                                          .pull_down = bias.down & reset_mask},
                                         {.pull_up = reset_mask});
  // Logical passive-source projection: drive the released input only. An active
  // MCU output remains the sole strong source; unknown local latches stay unknown.
  PinDrive input{.floating = reset_mask};
  if ((output.mask & reset_mask) == 0) {
    input = resolved.known == reset_mask
                ? PinDrive{.mask = reset_mask, .levels = static_cast<std::uint8_t>(resolved.levels)}
                : PinDrive{.unknown = reset_mask};
  }
  native_->mcu.bus().update_pin_drive(McuPort::u, reset_mask, input);
}

void Instrument::observe_u0_release() {
  update_u0_drive();
  const auto sample = native_->mcu.bus().observe_pads(McuPort::u, reset_mask);
  const auto context = [&] {
    return " at=" + hex(u_data) + " mcu-pc=" + mcu_pc_context(native_->mcu) +
           " dsp-pc=" + hex(native_->dsp.cpu().registers().pc, 6) +
           " tick=" + std::to_string(schedule_ticks());
  };
  if (sample.known != reset_mask) {
    throw Fault("Unknown MCU U0 DSP reset input operation=read" + context());
  }
  if ((sample.levels & reset_mask) == 0) {
    if (control_.release_epoch) {
      if (!config_.dsp.initial_reset) {
        throw Fault("Missing MCU U0 DSP reset inputs operation=assert" + context());
      }
      try {
        synchronize_audio_clock(schedule_ticks());
        native_->runtime->begin_dsp_reset(config_.dsp.initial_reset.value());
        if (control_.pcm) { control_.pcm->reset(); }
      } catch (const Fault &error) {
        throw Fault("MCU U0 DSP reset operation=assert" + context() + ": " + error.what());
      }
      control_.release_epoch.reset();
      control_.u0_asserted = false;
    }
    if (!control_.u0_asserted && execution_.trace != nullptr) {
      *execution_.trace << "DSP-U0-ASSERT" << context() << '\n';
    }
    control_.u0_asserted = true;
  } else if (control_.u0_asserted && !control_.release_epoch) {
    release(schedule_ticks());
  }
}
}
