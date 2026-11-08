// SPDX-License-Identifier: GPL-3.0-only
#include "runtime/instrument.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "runtime/instrument_context.hpp"
#include "runtime/mcu_dsp_gpio_peer.hpp"
#include "runtime/mcu_dsp_runtime.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>

namespace blofeld::runtime {
namespace {
McuMachine make_mcu(const Config &config, const Execution &execution) {
  if (config.power_on) {
    return {execution.image, execution.specification, config.mcu, config.power_on->inputs};
  }
  return {execution.image, execution.specification, config.mcu};
}

constexpr unsigned decoder_reset = 1;
constexpr unsigned decoder_suspended = 2;
constexpr unsigned decoder_enumerated = 4;
constexpr unsigned decoder_full_speed = 8;
}

Instrument::Owners::Owners(const Config &config, const Execution &execution)
    : mcu(make_mcu(config, execution)), dsp(dsp_clock_inputs(config)) {
  if (execution.device_state) { mcu.attach_flash_store(execution.device_state); }
}

Instrument::Instrument(Config config, Execution execution)
    : config_(std::move(config)),
      execution_(std::move(execution)),
      identity_(std::make_shared<const Identity>()),
      native_(std::make_unique<Owners>(config_, execution_)),
      dma_inputs_([this] {
        return DspDmaInputs{.timing = DspDmaTiming::reference_sampled_cycles,
                            .grant = config_.dma_grant};
      }),
      dsp_events_([this](const DspRuntimeCycle &cycle) { observe_dsp_cycle(cycle); }) {
  const auto common = common_hz();
  const auto local = native_->mcu.run_clock().timebase_hz();
  if (common < local || common % local != 0) { throw Fault("Invalid instrument timebase"); }
  control_.scale = common / local;
  prepare_audio_clock();
  prepare_dsp();
  if (config_.pcm) {
    if (config_.pcm->instance > 1 || config_.pcm->lane > 5) {
      throw Fault("Invalid PCM output route");
    }
    control_.pcm.emplace(config_.pcm->format, config_.pcm->after_ticks);
  }
  for (const auto &entry : config_.initial) { apply(entry); }
  attach_b_pulls(*native_);
  prepare_lcd();
  connect_runtime();
  if (config_.gpio_peer) {
    native_->gpio_peer =
        std::make_unique<McuDspGpioPeer>(native_->mcu, native_->dsp, *config_.gpio_peer);
  }
  publish_io();
  attach(*native_);
}

void Instrument::attach(Owners &owners) {
  owners.mcu.set_span_observer([this](const auto span) { on_span(span); });
  owners.mcu.set_reset_period_observer([this](const auto period) {
    native_->runtime->begin_reset_period(period, dma_inputs_, dsp_events_);
    publish_io();
  });
  owners.mcu.bus().set_observer([this](const auto &event) { on_write(event); });
}

std::uint64_t Instrument::ticks() const {
  const auto local = native_->mcu.run_clock().elapsed_ticks();
  if (control_.scale == 0 || local > std::numeric_limits<std::uint64_t>::max() / control_.scale) {
    throw Fault("Instrument time overflow");
  }
  return local * control_.scale;
}

void Instrument::apply(const Input &input) {
  switch (input.kind) {
    case InputKind::drive:
      if (config_.port_b_pulls && input.port == McuPort::b) {
        set_b_drive(input.drive);
      } else if (config_.gpio_peer && input.port == McuPort::e) {
        constexpr std::uint8_t other_e_pins = 0xfe;
        native_->mcu.bus().update_pin_drive(input.port, other_e_pins, input.drive);
      } else {
        native_->mcu.bus().set_pin_drive(input.port, input.drive);
      }
      break;
    case InputKind::phy_window: native_->mcu.bus().observe_phy({.edges = input.value}); break;
    case InputKind::usb_decoder:
      native_->mcu.bus().observe_usb_decoder(
          {.reset = (input.value & decoder_reset) != 0,
           .suspended = (input.value & decoder_suspended) != 0,
           .enumerated = (input.value & decoder_enumerated) != 0,
           .full_speed = (input.value & decoder_full_speed) != 0});
      break;
    case InputKind::timer_inputs:
      native_->mcu.set_timer_inputs(static_cast<std::uint8_t>(input.value));
      break;
    default: throw Fault("Invalid instrument input kind");
  }
}

void Instrument::release(const std::uint64_t at) {
  synchronize_audio_clock(at);
  control_.previous = sample();
  if (control_.previous.selected) { throw Fault("DSP release requires deasserted SPI SS"); }
  native_->dsp.bus().shi().observe_spi(control_.previous);
  native_->runtime->release_dsp_reset();
  control_.release_epoch = at;
  publish_io();
  if (execution_.trace != nullptr) {
    *execution_.trace << "DSP-RELEASE requested=" << config_.release_ticks << " actual=" << at
                      << " pc=" << hex(native_->dsp.cpu().registers().pc, 6) << '\n';
  }
}

void Instrument::boundary() {
  if (!native_->mcu.system_reset_active()) {
    const auto cpu = native_->mcu.cpu().state();
    control_.context = {.pc = cpu.pc, .ccr = cpu.ccr};
  }
  while (control_.next_event < config_.events.size() &&
         config_.events[control_.next_event].ticks <= ticks()) {
    const auto &event = config_.events[control_.next_event];
    apply(event);
    observe_lcd(0);
    publish_io();
    if (execution_.trace != nullptr) {
      *execution_.trace << "INPUT requested=" << event.ticks << " actual=" << ticks()
                        << " kind=" << static_cast<unsigned>(event.kind)
                        << " mcu-pc=" << mcu_pc_context(native_->mcu) << '\n';
    }
    if (control_.release_epoch) { transition(sample()); }
    ++control_.next_event;
  }
  if (config_.mcu_u0_release) {
    observe_u0_release();
  } else if (!control_.release_epoch && ticks() >= config_.release_ticks) {
    release(ticks());
  }
  check_signals();
}

void Instrument::on_span(const McuSpan span) {
  control_.context = span.context;
  native_->runtime->synchronize(dma_inputs_, dsp_events_);
  publish_io();
}

void Instrument::execute_until_event() {
  if (native_->mcu.system_reset_active()) {
    if (!config_.power_on || !native_->runtime) { throw Fault("Missing reset runtime owner"); }
    static_cast<void>(native_->mcu.advance_system_reset(1, config_.power_on->reset_pad));
    if (!native_->mcu.system_reset_active()) { native_->runtime->end_system_reset(); }
    return;
  }
  const auto local = native_->mcu.run_clock().elapsed_ticks();
  // USB transport and live input are serviced on their one-millisecond frame timeline.
  const auto frame = native_->mcu.run_clock().timebase_hz() / 1000U;
  auto horizon = ((local / frame) + 1U) * frame;
  if (control_.next_event < config_.events.size()) {
    horizon = std::min(horizon, (config_.events[control_.next_event].ticks / control_.scale) + 1U);
  }
  const auto result = native_->mcu.run_until(horizon);
  native_->runtime->end_instruction();
  control_.instructions += result.retired;
  if (result.interrupt_vector) {
    ++control_.interrupts;
    if (execution_.trace != nullptr) {
      *execution_.trace << "IRQ vector=" << hex(*result.interrupt_vector)
                        << " stack-reused=" << result.stack_reused << '\n';
    }
  }
}

}
