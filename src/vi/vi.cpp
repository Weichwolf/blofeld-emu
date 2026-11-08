// SPDX-License-Identifier: GPL-3.0-only
#include "vi/vi.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "runtime/mcu_dsp_gpio_peer.hpp"
#include "runtime/mcu_dsp_runtime.hpp"
#include "storage/private_file.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace blofeld::vi {
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

VirtualInstrument::VmContext::VmContext(const Config &config,
                                        const Execution &execution,
                                        VirtualInstrument &instrument)
    : mcu(make_mcu(config, execution)),
      dsp(dsp_clock_inputs(config)),
      dma_inputs([&config] {
        return DspDmaInputs{.timing = DspDmaTiming::reference_sampled_cycles,
                            .grant = config.dma_grant};
      }),
      dsp_events(
          [&instrument](const DspRuntimeCycle &cycle) { instrument.observe_dsp_cycle(cycle); }) {
  if (execution.device_state) { mcu.attach_flash_store(execution.device_state); }
}

VirtualInstrument::VirtualInstrument(Config config, Execution execution)
    : config_(std::move(config)),
      execution_(std::move(execution)),
      vm_(config_, execution_, *this) {
  const auto common = common_hz();
  const auto local = vm_.mcu.run_clock().timebase_hz();
  if (common < local || common % local != 0) { throw Fault("Invalid instrument timebase"); }
  vm_.scale = common / local;
  prepare_audio_clock();
  prepare_dsp();
  if (config_.pcm) {
    if (config_.pcm->instance > 1 || config_.pcm->lane > 5) {
      throw Fault("Invalid PCM output route");
    }
    vm_.pcm.emplace(config_.pcm->format, config_.pcm->after_ticks);
  }
  for (const auto &entry : config_.initial) { apply(entry); }
  attach_b_pulls(vm_);
  prepare_lcd();
  connect_runtime();
  if (config_.gpio_peer) {
    vm_.gpio_peer = std::make_unique<McuDspGpioPeer>(vm_.mcu, vm_.dsp, *config_.gpio_peer);
  }
  publish_io();
  attach(vm_);
}

std::string VirtualInstrument::mcu_pc_context() const {
  if (vm_.mcu.system_reset_active()) { return "unknown-reset"; }
  return hex(vm_.mcu.cpu().state().pc);
}

void VirtualInstrument::attach(VmContext &owners) {
  owners.mcu.set_span_observer([this](const auto span) { on_span(span); });
  owners.mcu.set_reset_period_observer([this](const auto period) {
    vm_.runtime->begin_reset_period(period, vm_.dma_inputs, vm_.dsp_events);
    publish_io();
  });
  owners.mcu.bus().set_observer([this](const auto &event) { on_write(event); });
}

std::uint64_t VirtualInstrument::ticks() const {
  const auto local = vm_.mcu.run_clock().elapsed_ticks();
  if (vm_.scale == 0 || local > std::numeric_limits<std::uint64_t>::max() / vm_.scale) {
    throw Fault("Instrument time overflow");
  }
  return local * vm_.scale;
}

void VirtualInstrument::apply(const Input &input) {
  switch (input.kind) {
    case InputKind::drive:
      if (config_.port_b_pulls && input.port == McuPort::b) {
        set_b_drive(input.drive);
      } else if (config_.gpio_peer && input.port == McuPort::e) {
        constexpr std::uint8_t other_e_pins = 0xfe;
        vm_.mcu.bus().update_pin_drive(input.port, other_e_pins, input.drive);
      } else {
        vm_.mcu.bus().set_pin_drive(input.port, input.drive);
      }
      break;
    case InputKind::phy_window: vm_.mcu.bus().observe_phy({.edges = input.value}); break;
    case InputKind::usb_decoder:
      vm_.mcu.bus().observe_usb_decoder({.reset = (input.value & decoder_reset) != 0,
                                         .suspended = (input.value & decoder_suspended) != 0,
                                         .enumerated = (input.value & decoder_enumerated) != 0,
                                         .full_speed = (input.value & decoder_full_speed) != 0});
      break;
    case InputKind::timer_inputs:
      vm_.mcu.set_timer_inputs(static_cast<std::uint8_t>(input.value));
      break;
    default: throw Fault("Invalid instrument input kind");
  }
}

void VirtualInstrument::release(const std::uint64_t at) {
  synchronize_audio_clock(at);
  vm_.previous = sample();
  if (vm_.previous.selected) { throw Fault("DSP release requires deasserted SPI SS"); }
  vm_.dsp.bus().shi().observe_spi(vm_.previous);
  vm_.runtime->release_dsp_reset();
  vm_.release_epoch = at;
  publish_io();
  if (execution_.trace != nullptr) {
    *execution_.trace << "DSP-RELEASE requested=" << config_.release_ticks << " actual=" << at
                      << " pc=" << hex(vm_.dsp.cpu().registers().pc, 6) << '\n';
  }
}

void VirtualInstrument::boundary() {
  if (!vm_.mcu.system_reset_active()) {
    const auto cpu = vm_.mcu.cpu().state();
    vm_.context = {.pc = cpu.pc, .ccr = cpu.ccr};
  }
  while (vm_.next_event < config_.events.size() &&
         config_.events[vm_.next_event].ticks <= ticks()) {
    const auto &event = config_.events[vm_.next_event];
    apply(event);
    observe_lcd(0);
    publish_io();
    if (execution_.trace != nullptr) {
      *execution_.trace << "INPUT requested=" << event.ticks << " actual=" << ticks()
                        << " kind=" << static_cast<unsigned>(event.kind)
                        << " mcu-pc=" << mcu_pc_context() << '\n';
    }
    if (vm_.release_epoch) { transition(sample()); }
    ++vm_.next_event;
  }
  if (config_.mcu_u0_release) {
    observe_u0_release();
  } else if (!vm_.release_epoch && ticks() >= config_.release_ticks) {
    release(ticks());
  }
  check_signals();
}

void VirtualInstrument::on_span(const McuSpan span) {
  vm_.context = span.context;
  vm_.runtime->synchronize(vm_.dma_inputs, vm_.dsp_events);
  publish_io();
}

void VirtualInstrument::execute_until_event() {
  if (vm_.mcu.system_reset_active()) {
    if (!config_.power_on || !vm_.runtime) { throw Fault("Missing reset runtime owner"); }
    static_cast<void>(vm_.mcu.advance_system_reset(1, config_.power_on->reset_pad));
    if (!vm_.mcu.system_reset_active()) { vm_.runtime->end_system_reset(); }
    return;
  }
  const auto local = vm_.mcu.run_clock().elapsed_ticks();
  // USB transport and live input are serviced on their one-millisecond frame timeline.
  const auto frame = vm_.mcu.run_clock().timebase_hz() / 1000U;
  auto horizon = ((local / frame) + 1U) * frame;
  if (vm_.next_event < config_.events.size()) {
    horizon = std::min(horizon, (config_.events[vm_.next_event].ticks / vm_.scale) + 1U);
  }
  const auto result = vm_.mcu.run_until(horizon);
  vm_.runtime->end_instruction();
  vm_.instructions += result.retired;
  if (result.interrupt_vector) {
    ++vm_.interrupts;
    if (execution_.trace != nullptr) {
      *execution_.trace << "IRQ vector=" << hex(*result.interrupt_vector)
                        << " stack-reused=" << result.stack_reused << '\n';
    }
  }
}

std::vector<std::uint32_t> load_rom(const std::filesystem::path &path) {
  const auto bytes = read_private_file(path, std::size_t{65536} * 3U);
  if (bytes.empty() || bytes.size() % 3 != 0) {
    throw Fault("DSP ROM requires nonempty packed big-endian 24-bit words");
  }
  std::vector<std::uint32_t> words;
  words.reserve(bytes.size() / 3);
  for (std::size_t offset = 0; offset < bytes.size(); offset += 3) {
    constexpr unsigned high_shift = 16;
    words.push_back((static_cast<std::uint32_t>(bytes[offset]) << high_shift) |
                    (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U) | bytes[offset + 2]);
  }
  return words;
}
}
