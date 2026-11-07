// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/nand_peer.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_memory.hpp"
#include "mcu/mcu_reset.hpp"
#include "storage/flash_image.hpp"
#include "usb/usbip_runtime.hpp"
#include "usb/usbip_tcp.hpp"

#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace blofeld::reference {
class Board::Busy {
public:
  explicit Busy(Board &board) : board_(board) {
    board.require_idle();
    board_.active_ = true;
  }

  ~Busy() { board_.active_ = false; }

  Busy(const Busy &) = delete;
  Busy &operator=(const Busy &) = delete;

private:
  Board &board_;
};

void Board::require_idle() const {
  if (active_) { throw Fault("Reference-board control requires an inactive operation boundary"); }
}

void Board::require_healthy() const {
  require_idle();
  if (timing_faulted()) { throw Fault("Reference-board failure requires a fresh device instance"); }
}

bool Board::finished() const {
  return control_.operations >= config_.operations ||
         control_.instructions >= execution_.instructions;
}

void Board::initialize() {
  require_healthy();
  if (control_.started) { return; }
  const Busy guard(*this);
  try {
    check_pcm_output();
    boundary();
    control_.started = true;
  } catch (...) {
    mark_fault();
    throw;
  }
}

void Board::advance() {
  require_healthy();
  initialize();
  if (finished()) { throw Fault("Reference-board operation exceeds its execution budget"); }
  const Busy guard(*this);
  try {
    check_pcm_output();
    step();
    ++control_.operations;
    if (execution_.trace != nullptr) {
      *execution_.trace << "MCU end=" << ticks();
      const auto reset = native_->mcu.system_reset_state();
      if (reset && reset->phase != McuResetPhase::complete) {
        *execution_.trace << " held-reset=1 reset-edges=" << reset->edges;
      } else {
        *execution_.trace << " pc=" << hex(native_->mcu.cpu().state().pc);
      }
      *execution_.trace << " instructions=" << control_.instructions
                        << " operations=" << control_.operations
                        << " idle=" << native_->mcu.idle_bus_cycles() << '\n';
    }
    boundary();
  } catch (...) {
    mark_fault();
    throw;
  }
}

void Board::poll_usb(UsbipRuntime &runtime) {
  require_healthy();
  if (!control_.started || finished()) {
    throw Fault("USB/IP input requires an active reference-board execution budget");
  }
  const Busy guard(*this);
  const auto cpu = native_->mcu.cpu().state();
  const auto before_started = runtime.started();
  const auto before_published = runtime.published();
  try {
    const auto events = runtime.poll(
        native_->mcu.bus(),
        {.pc = cpu.pc, .ccr = cpu.ccr},
        {.owner = identity_, .restore_revision = usb_restore_revision_, .ticks = ticks()});
    if (execution_.trace != nullptr) {
      if (!before_started && runtime.started()) {
        *execution_.trace << "USBIP-START actual=" << ticks() << '\n';
      }
      if (!before_published && runtime.published()) {
        *execution_.trace << "USBIP-PUBLISH actual=" << ticks() << '\n';
      }
      for (const auto &event : events) {
        *execution_.trace << "USBIP-PEER actual=" << ticks() << " connection=" << event.connection
                          << " error=" << event.error << " reason=" << event.reason << '\n';
      }
    }
    if (!before_published && runtime.published()) {
      std::cout << "USBIP descriptors=ready tick=" << ticks() << '\n' << std::flush;
    }
    for (const auto &event : events) {
      if (event.error) { std::cerr << "USB/IP peer: " << event.reason << '\n'; }
    }
  } catch (...) {
    mark_fault();
    runtime.retire();
    throw;
  }
}

Status Board::status() const {
  return observe_status(true);
}

Status Board::runtime_status() const {
  return observe_status(false);
}

std::vector<McuMemoryByte> Board::inspect_mcu_memory(const std::uint16_t address,
                                                     const std::uint32_t count) const {
  require_idle();
  constexpr std::uint32_t address_space_size = 1U << 16U;
  if (count > address_space_size - address) {
    throw Fault("Reference MCU inspection range wraps at " + hex(address) +
                " count=" + std::to_string(count));
  }
  std::vector<McuMemoryByte> result;
  result.reserve(count);
  for (std::uint32_t offset = 0; offset < count; ++offset) {
    result.push_back(
        native_->mcu.bus().inspect_memory(static_cast<std::uint16_t>(address + offset)));
  }
  return result;
}

std::vector<std::optional<std::uint32_t>>
Board::inspect_dsp_memory(const DspAddress first, const std::uint32_t count) const {
  require_idle();
  constexpr std::uint32_t address_space_size = 1U << 24U;
  if (first.address >= address_space_size || count > address_space_size - first.address) {
    throw Fault("Reference DSP inspection range wraps at " + hex(first.address, 6) +
                " count=" + std::to_string(count));
  }
  std::vector<std::optional<std::uint32_t>> result;
  result.reserve(count);
  const auto &bus = native_->dsp.bus();
  for (std::uint32_t offset = 0; offset < count; ++offset) {
    result.push_back(bus.inspect_memory({.space = first.space, .address = first.address + offset}));
  }
  return result;
}

Status Board::observe_status(const bool include_cold_nvm) const {
  require_idle();
  const bool known = !native_->mcu.system_reset_active();
  return {.mcu = known ? native_->mcu.cpu().state() : CpuState{},
          .dsp = native_->dsp.cpu().registers(),
          .mcu_state_known = known,
          .mcu_reset = native_->mcu.system_reset_state(),
          .cold_nvm = include_cold_nvm
                          ? encode_flash_image(native_->mcu.bus().power_off_flash_image())
                          : std::vector<std::uint8_t>{},
          .common_ticks = ticks(),
          .mcu_cycles = native_->mcu.run_clock().bus_cycles(),
          .mcu_ticks = native_->mcu.run_clock().elapsed_ticks(),
          .oscillator_cycles = native_->mcu.run_clock().oscillator_cycles(),
          .idle_cycles = native_->mcu.idle_bus_cycles(),
          .dsp_ticks = dsp_ticks(),
          .dsp_units = native_->dsp.clock().native_cycle_units(),
          .dsp_steps = dsp_steps(),
          .dsp_cycles = native_->runtime->completed_cycles(),
          .instructions = control_.instructions,
          .operations = control_.operations,
          .interrupts = control_.interrupts,
          .writes = control_.writes,
          .frames = control_.frames,
          .ties = schedule_ties(),
          .ordered_pairs = control_.ordered_pairs,
          .lcd_cycles = control_.lcd_cycles,
          .next_event = control_.next_event,
          .next_dsp_event = control_.next_dsp_event,
          .next_esai_edge = control_.next_esai_edge,
          .audio_clock = control_.audio_clock,
          .audio_clock_started = control_.audio_clock_started,
          .esai_inputs = control_.esai_inputs,
          .pcm = control_.pcm,
          .pcm_stamp = control_.pcm_stamp,
          .last_pcm = control_.last_pcm,
          .recording = control_.recording,
          .midi_queued = midi_queue_size(),
          .midi_bytes = control_.midi_bytes,
          .midi_started = control_.midi.has_value() ||
                          ((control_.midi_dsp || control_.midi_sci) && control_.started),
          .midi_sci_events = control_.midi_sci ? control_.midi_sci->observed_events() : 0,
          .midi_sci_frames = control_.midi_sci ? control_.midi_sci->completed_frames() : 0,
          .midi_dsp_events = control_.midi_dsp ? control_.midi_dsp->observed_events() : 0,
          .midi_dsp_frames = control_.midi_dsp ? control_.midi_dsp->completed_frames() : 0,
          .midi_output_queued = control_.midi_output ? control_.midi_output->queued_bytes() : 0,
          .midi_output_bytes = control_.midi_output ? control_.midi_output->transmitted_bytes() : 0,
          .started = control_.started,
          .waiting = known && native_->mcu.cpu().waiting(),
          .held_reset = dsp_reset_active(),
          .faulted = timing_faulted()};
}

bool Status::operator==(const Status &other) const {
  return (!mcu_state_known ||
          std::tie(mcu.pc, mcu.d, mcu.x, mcu.y, mcu.sp, mcu.ccr) == std::tie(other.mcu.pc,
                                                                             other.mcu.d,
                                                                             other.mcu.x,
                                                                             other.mcu.y,
                                                                             other.mcu.sp,
                                                                             other.mcu.ccr)) &&
         std::tie(mcu_state_known,
                  mcu_reset,
                  dsp_cycles,
                  dsp,
                  cold_nvm,
                  common_ticks,
                  mcu_cycles,
                  mcu_ticks,
                  oscillator_cycles,
                  idle_cycles,
                  dsp_ticks,
                  dsp_units,
                  dsp_steps,
                  instructions,
                  operations,
                  interrupts,
                  writes,
                  frames,
                  ties,
                  ordered_pairs,
                  lcd_cycles,
                  next_event,
                  next_dsp_event,
                  next_esai_edge,
                  audio_clock,
                  audio_clock_started,
                  esai_inputs,
                  pcm,
                  pcm_stamp,
                  last_pcm,
                  recording,
                  midi_queued,
                  midi_bytes,
                  midi_started,
                  midi_sci_events,
                  midi_sci_frames,
                  midi_dsp_events,
                  midi_dsp_frames,
                  midi_output_queued,
                  midi_output_bytes,
                  started,
                  waiting,
                  held_reset,
                  faulted) == std::tie(other.mcu_state_known,
                                       other.mcu_reset,
                                       other.dsp_cycles,
                                       other.dsp,
                                       other.cold_nvm,
                                       other.common_ticks,
                                       other.mcu_cycles,
                                       other.mcu_ticks,
                                       other.oscillator_cycles,
                                       other.idle_cycles,
                                       other.dsp_ticks,
                                       other.dsp_units,
                                       other.dsp_steps,
                                       other.instructions,
                                       other.operations,
                                       other.interrupts,
                                       other.writes,
                                       other.frames,
                                       other.ties,
                                       other.ordered_pairs,
                                       other.lcd_cycles,
                                       other.next_event,
                                       other.next_dsp_event,
                                       other.next_esai_edge,
                                       other.audio_clock,
                                       other.audio_clock_started,
                                       other.esai_inputs,
                                       other.pcm,
                                       other.pcm_stamp,
                                       other.last_pcm,
                                       other.recording,
                                       other.midi_queued,
                                       other.midi_bytes,
                                       other.midi_started,
                                       other.midi_sci_events,
                                       other.midi_sci_frames,
                                       other.midi_dsp_events,
                                       other.midi_dsp_frames,
                                       other.midi_output_queued,
                                       other.midi_output_bytes,
                                       other.started,
                                       other.waiting,
                                       other.held_reset,
                                       other.faulted);
}

Board::Outcome Board::finish_branch(const std::function<void(Board &)> &input) {
  std::optional<std::string> fault;
  try {
    while (!finished()) {
      if (input) { input(*this); }
      advance();
    }
  } catch (const Fault &error) {
    mark_fault();
    // A failed live writer cannot supply a cold snapshot. Preserve the causal
    // execution fault instead of replacing it with the storage quarantine error.
    if (execution_.device_state || (native_->nand && native_->nand->durable())) { throw; }
    fault = error.what();
  }
  return {.fault = std::move(fault), .state = status()};
}

void Board::flush_trace() {
  const Busy guard(*this);
  if (execution_.trace != nullptr) {
    execution_.trace->flush();
    if (!*execution_.trace) { throw Fault("Reference-board trace output failed"); }
  }
}

}
