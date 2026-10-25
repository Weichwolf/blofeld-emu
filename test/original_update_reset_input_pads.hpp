// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/gpio_pads.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "original_update_forward_fixture.hpp"

#include <cstdint>
#include <ostream>
#include <string>

namespace blofeld::test::reset_inputs {
constexpr unsigned q_strobe = 0x80, irq_pin = 1, idle_irq_pin = 2;
constexpr unsigned loop_pins = 12, unused_e_bias = 0x60;
constexpr unsigned e_mask = 0xff;

inline GpioNetDriver
local_driver(const PinOutputs pins, const PinBiases bias, const unsigned mask) {
  return {.active = pins.mask & mask,
          .known = pins.known & mask,
          .levels = pins.levels & mask,
          .reduced = pins.reduced_drive & mask,
          .pull_up = bias.up & mask,
          .pull_down = bias.down & mask};
}

inline GpioNetDriver one_bit(const GpioNetDriver driver, const unsigned bit) {
  return {.active = (driver.active >> bit) & 1U,
          .known = (driver.known >> bit) & 1U,
          .levels = (driver.levels >> bit) & 1U,
          .reduced = (driver.reduced >> bit) & 1U,
          .pull_up = (driver.pull_up >> bit) & 1U,
          .pull_down = (driver.pull_down >> bit) & 1U};
}

// Test-supplied Q7 observations qualify only opposed passive pulls. All strong
// conflicts/unknown outputs remain strict. E2/E3 and PJ0/IRQA share real drivers.
class BootstrapPads {
public:
  BootstrapPads(update_forward::Forward &run, const bool high) : run_(run), high_(high) {}

  void attach() {
    auto &pair = run_.pair;
    pair.mcu.bus().attach_gpio_pad_source(McuPort::q,
                                          [this](const auto bits) { return q(false, bits); });
    pair.dsp.bus().gpio(DspPort::c).attach_pad_source([this](const auto bits) {
      return q(true, bits);
    });
    pair.mcu.bus().attach_gpio_pad_source(McuPort::e, [this](const auto bits) { return e(bits); });
    pair.mcu.bus().attach_gpio_pad_source(McuPort::j, [this](const auto bits) { return j(bits); });
    pair.mcu.bus().set_pin_drive(McuPort::r, {.mask = 4, .levels = 4});
  }

  [[nodiscard]] GpioPadSample j(const unsigned bits) const {
    const auto &bus = run_.pair.mcu.bus();
    const auto local =
        local_driver(bus.gpio_outputs(McuPort::j), bus.gpio_biases(McuPort::j), bits);
    return resolve_gpio_net(bits, local, {.pull_up = bits & irq_pin});
  }

  void audit_irqa() {
    const auto sample = j(irq_pin);
    require_known_gpio_pads(sample, irq_pin);
    const char level = sample.levels == 0 ? '0' : '1';
    if (irqa_levels_.empty() || irqa_levels_.back() != level) {
      irqa_levels_ += level;
      run_.pair.trace << "RESET_INPUT_IRQA level=" << level << '\n';
    }
  }

  [[nodiscard]] unsigned bias_samples() const { return bias_samples_; }

  [[nodiscard]] const std::string &irqa_levels() const { return irqa_levels_; }

private:
  unsigned bias_samples_ = 0;
  std::string irqa_levels_;

  GpioPadSample q(const bool dsp_side, const unsigned bits) {
    auto sample = run_.pair.pads(dsp_side, bits);
    const auto &link = run_.pair.link;
    const auto idle = bits & ~(link.mcu_drive.active | link.dsp_drive.active);
    const auto opposed = idle & (link.mcu_drive.pull_up | link.dsp_drive.pull_up) &
                         (link.mcu_drive.pull_down | link.dsp_drive.pull_down) & q_strobe;
    if (opposed != 0) {
      sample.known |= opposed;
      sample.levels = (sample.levels & ~opposed) | (high_ ? opposed : 0U);
      sample.conflicts &= ~opposed;
      ++bias_samples_;
      run_.pair.trace << "RESET_INPUT_Q7 dsp=" << dsp_side << " mask=" << opposed
                      << " high=" << high_ << '\n';
    }
    return sample;
  }

  GpioPadSample e(const unsigned bits) {
    auto &pair = run_.pair;
    const auto local = local_driver(
        pair.mcu.bus().gpio_outputs(McuPort::e), pair.mcu.bus().gpio_biases(McuPort::e), e_mask);
    const auto remote = pair.dsp.bus().gpio(DspPort::e).outputs();
    constexpr auto source = update_forward::port_e_strobe;
    const unsigned active = ((remote.mask | remote.peripheral) & source) != 0 ? irq_pin : 0U;
    const unsigned known = (remote.mask & remote.known & source) != 0 ? irq_pin : 0U;
    const unsigned level = (remote.levels & source) != 0 ? known : 0U;
    require((local.active & 3U) == 0, "Reset bootstrap retains the declared MCU IRQ inputs");
    const GpioNetDriver external{.active = idle_irq_pin | active,
                                 .known = idle_irq_pin | known,
                                 .levels = idle_irq_pin | level,
                                 .pull_down = unused_e_bias};
    const auto whole = resolve_gpio_net(e_mask, local, external);
    const auto loop = resolve_gpio_net(1, one_bit(local, 2), one_bit(local, 3));
    GpioPadSample result{
        .known = (whole.known & ~loop_pins) | (loop.known != 0 ? loop_pins : 0U),
        .levels = (whole.levels & ~loop_pins) | (loop.levels != 0 ? loop_pins : 0U),
        .conflicts = (whole.conflicts & ~loop_pins) | (loop.conflicts != 0 ? loop_pins : 0U)};
    audit_irq(bits, result);
    result.known &= bits;
    result.levels &= bits;
    result.conflicts &= bits;
    return result;
  }

  void audit_irq(const unsigned bits, const GpioPadSample sample) {
    if ((bits & 3U) == 0) { return; }
    ++run_.irq_queries;
    run_.unknown_irq_queries += static_cast<unsigned>((sample.known & bits & 3U) != (bits & 3U));
    if (run_.last_irq_known != (sample.known & 3U) ||
        run_.last_irq_levels != (sample.levels & 3U)) {
      run_.last_irq_known = sample.known & 3U;
      run_.last_irq_levels = sample.levels & 3U;
      run_.pair.trace << "IRQ_PAD known=" << run_.last_irq_known
                      << " levels=" << run_.last_irq_levels << '\n';
    }
  }

  update_forward::Forward &run_;
  bool high_;
};
}
