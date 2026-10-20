// SPDX-License-Identifier: GPL-3.0-only
#include "reference_board.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/flash_file.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"

#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <ostream>

namespace blofeld::reference {
namespace {
constexpr unsigned bootstrap_base = 0xff0000;
constexpr unsigned decoder_reset = 1;
constexpr unsigned decoder_suspended = 2;
constexpr unsigned decoder_enumerated = 4;
constexpr unsigned decoder_full_speed = 8;
}

Board::Board(const Config &config, const Execution &execution)
    : config_(config),
      execution_(execution),
      mcu_(execution.image, execution.specification, config.mcu),
      dsp_(config.dsp) {
  const auto common = dsp_.run_clock().timebase_hz();
  const auto local = mcu_.run_clock().timebase_hz();
  if (common < local || common % local != 0) { throw Fault("Invalid reference-board timebase"); }
  scale_ = common / local;
  if (config.release_ticks < ticks() ||
      (!config.events.empty() && config.events.front().ticks < ticks())) {
    throw Fault("Reference-board release/events precede the completed native MCU reset");
  }
  dsp_.bus().install_rom({.start = bootstrap_base, .words = execution.rom});
  dsp_.bus().pll().observe_pinit(config.pinit);
  dsp_.cpu().reset_spi_boot();
  for (const auto &entry : config.initial) { apply(entry); }
  update_loopback();
  mcu_.set_cycle_observer([this](const auto cycle) { on_cycle(cycle); });
  mcu_.set_wait_period_observer([this](const auto period) {
    check_signals();
    if (schedule_) {
      schedule_->begin_wait_period(period, [this](const auto &step) { before_dsp(step); });
    }
  });
  mcu_.bus().set_observer([this](const auto &event) { on_write(event); });
}

std::uint64_t Board::ticks() const {
  const auto local = mcu_.run_clock().elapsed_ticks();
  if (scale_ == 0 || local > std::numeric_limits<std::uint64_t>::max() / scale_) {
    throw Fault("Reference-board time overflow");
  }
  return local * scale_;
}

void Board::apply(const Input &input) {
  switch (input.kind) {
    case InputKind::drive: mcu_.bus().set_pin_drive(input.port, input.drive); break;
    case InputKind::phy_window: mcu_.bus().observe_phy({.edges = input.value}); break;
    case InputKind::usb_decoder:
      mcu_.bus().observe_usb_decoder({.reset = (input.value & decoder_reset) != 0,
                                      .suspended = (input.value & decoder_suspended) != 0,
                                      .enumerated = (input.value & decoder_enumerated) != 0,
                                      .full_speed = (input.value & decoder_full_speed) != 0});
      break;
    case InputKind::timer_inputs:
      mcu_.set_timer_inputs(static_cast<std::uint8_t>(input.value));
      break;
    default: throw Fault("Invalid reference-board input kind");
  }
}

void Board::release() {
  previous_ = sample();
  if (previous_.selected) { throw Fault("Reference-board DSP release requires deasserted SPI SS"); }
  dsp_.bus().shi().observe_spi(previous_);
  auto inputs = config_.schedule;
  inputs.dsp.epoch_ticks = ticks();
  schedule_ = std::make_unique<McuDspSchedule>(inputs, mcu_, dsp_);
  if (execution_.trace != nullptr) {
    *execution_.trace << "DSP-RELEASE requested=" << config_.release_ticks << " actual=" << ticks()
                      << " pc=" << hex(dsp_.cpu().state().pc, 6) << '\n';
  }
}

void Board::boundary() {
  const auto cpu = mcu_.cpu().state();
  context_ = {.pc = cpu.pc, .ccr = cpu.ccr};
  while (next_event_ < config_.events.size() && config_.events[next_event_].ticks <= ticks()) {
    const auto &event = config_.events[next_event_];
    apply(event);
    update_loopback();
    if (execution_.trace != nullptr) {
      *execution_.trace << "INPUT requested=" << event.ticks << " actual=" << ticks()
                        << " kind=" << static_cast<unsigned>(event.kind)
                        << " mcu-pc=" << hex(mcu_.cpu().state().pc) << '\n';
    }
    if (schedule_) { transition(sample()); }
    ++next_event_;
  }
  if (!schedule_ && ticks() >= config_.release_ticks) { release(); }
  check_signals();
}

void Board::on_cycle(const Hc12BusCycle cycle) {
  context_ = cycle.context;
  check_signals();
  if (!schedule_) { return; }
  constexpr unsigned port_b = 1;
  const bool write = cycle.kind == 'w' || cycle.kind == 'W';
  const bool shared =
      write && cycle.address && *cycle.address <= port_b && port_b - *cycle.address < cycle.size;
  schedule_->begin_cycle(cycle, shared, [this](const auto &step) { before_dsp(step); });
}

void Board::before_dsp(const DspScheduledStep &step) const {
  if (execution_.trace != nullptr) {
    *execution_.trace << "DSP end=" << step.completion_ticks
                      << " pc=" << hex(step.plan.before.cpu.pc, 6)
                      << " start=" << step.plan.start_ticks
                      << " units=" << step.plan.native_cycle_units
                      << " ticks=" << step.plan.duration_ticks << '\n';
  }
}

void Board::report(std::ostream &stream) const {
  const auto cpu = mcu_.cpu().state();
  stream << "MCU pc=" << hex(cpu.pc) << " d=" << hex(cpu.d) << " x=" << hex(cpu.x)
         << " y=" << hex(cpu.y) << " sp=" << hex(cpu.sp) << " ccr=" << hex(cpu.ccr, 2)
         << " instructions=" << instructions_ << " operations=" << operations_
         << " interrupts=" << interrupts_ << " cycles=" << mcu_.run_clock().bus_cycles()
         << " idle=" << mcu_.idle_bus_cycles() << " ticks=" << mcu_.run_clock().elapsed_ticks()
         << " timebase=" << mcu_.run_clock().timebase_hz() << '\n';
  const auto dsp = dsp_.cpu().state();
  stream << "DSP pc=" << hex(dsp.pc, 6) << " omr=" << hex(dsp.omr, 6) << " r4=" << dsp.r[4]
         << " lc=" << dsp.lc << " held-reset=" << (schedule_ == nullptr)
         << " completed-steps=" << (schedule_ ? schedule_->completed_steps() : 0)
         << " units=" << dsp_.run_clock().native_cycle_units()
         << " ticks=" << dsp_.run_clock().elapsed_ticks()
         << " timebase=" << dsp_.run_clock().timebase_hz() << " spi-writes=" << writes_
         << " frames=" << frames_ << " ordered-pairs=" << ordered_pairs_
         << " ties=" << (schedule_ ? schedule_->ties() : 0) << '\n';
}

void Board::persist() const {
  if (execution_.flash_out.empty()) { return; }
  save_flash_image(execution_.flash_out, mcu_.bus().power_off_flash_image());
  std::cout << "Saved private MCU NVM image: " << execution_.flash_out << '\n';
}

void Board::step() {
  if (mcu_.cpu().waiting() && !mcu_.cpu().pending_interrupt()) {
    static_cast<void>(mcu_.advance_wait(1));
    if (schedule_) { schedule_->end_wait(); }
  } else {
    const auto result = mcu_.cpu().step();
    if (schedule_) { schedule_->end_instruction(); }
    if (result.instruction_executed) { ++instructions_; }
    if (result.interrupt_vector) {
      ++interrupts_;
      if (execution_.trace != nullptr) {
        *execution_.trace << "IRQ vector=" << hex(*result.interrupt_vector)
                          << " stack-reused=" << result.stack_reused << '\n';
      }
    }
  }
}

int Board::run() {
  std::cout << "Explicit MCU/DSP reference experiment; physical board wiring and timing "
               "are unverified. Inputs start after native MCU reset.\n";
  try {
    boundary();
    while (operations_ < config_.operations && instructions_ < execution_.instructions) {
      step();
      ++operations_;
      if (execution_.trace != nullptr) {
        *execution_.trace << "MCU end=" << ticks() << " pc=" << hex(mcu_.cpu().state().pc)
                          << " instructions=" << instructions_ << " operations=" << operations_
                          << " idle=" << mcu_.idle_bus_cycles() << '\n';
      }
      boundary();
    }
  } catch (const Fault &fault) {
    std::cerr << "Reference-board execution stopped: " << fault.what() << '\n';
    report(std::cerr);
    if (execution_.trace != nullptr) { execution_.trace->flush(); }
    persist();
    return 2;
  }
  if (execution_.trace != nullptr) { execution_.trace->flush(); }
  std::cout << "Reference experiment budget reached; boot completion is not established.\n";
  report(std::cout);
  persist();
  return 0;
}
}
