// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_machine.hpp"
#include "reference_board.hpp"

#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

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

struct Checkpoint::Storage {
  Storage(std::shared_ptr<const Board::Identity> owner,
          Board::Control state,
          McuCheckpoint mcu_state,
          DspCheckpoint dsp_state,
          std::optional<McuDspScheduleCheckpoint> schedule_state)
      : identity(std::move(owner)),
        control(std::move(state)),
        mcu(std::move(mcu_state)),
        dsp(std::move(dsp_state)),
        schedule(schedule_state) {}

  const std::shared_ptr<const Board::Identity> identity;
  const Board::Control control;
  const McuCheckpoint mcu;
  const DspCheckpoint dsp;
  const std::optional<McuDspScheduleCheckpoint> schedule;
};

Checkpoint::Checkpoint(std::shared_ptr<const Storage> storage) : storage_(std::move(storage)) {}

void Board::require_idle() const {
  if (active_) { throw Fault("Reference-board control requires an inactive operation boundary"); }
}

void Board::require_healthy() const {
  require_idle();
  if (control_.faulted || native_->mcu.timing_faulted() || native_->dsp.timing_faulted() ||
      (native_->schedule && native_->schedule->faulted())) {
    throw Fault("Reference-board failure requires a trusted coordinated restore");
  }
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
    boundary();
    control_.started = true;
  } catch (...) {
    control_.faulted = true;
    throw;
  }
}

void Board::advance() {
  require_healthy();
  initialize();
  if (finished()) { throw Fault("Reference-board operation exceeds its execution budget"); }
  const Busy guard(*this);
  try {
    step();
    ++control_.operations;
    if (execution_.trace != nullptr) {
      *execution_.trace << "MCU end=" << ticks() << " pc=" << hex(native_->mcu.cpu().state().pc)
                        << " instructions=" << control_.instructions
                        << " operations=" << control_.operations
                        << " idle=" << native_->mcu.idle_bus_cycles() << '\n';
    }
    boundary();
  } catch (...) {
    control_.faulted = true;
    throw;
  }
}

Checkpoint Board::checkpoint() const {
  require_healthy();
  native_->mcu.validate_run_boundary();
  if (control_.return_boundary) { throw Fault("Reference-board output observation is active"); }
  const auto schedule =
      native_->schedule ? std::optional<McuDspScheduleCheckpoint>{native_->schedule->checkpoint()}
                        : std::nullopt;
  if (static_cast<bool>(schedule) != control_.release_epoch.has_value()) {
    throw Fault("Reference-board checkpoint has an inconsistent DSP release epoch");
  }
  return Checkpoint(std::make_shared<const Checkpoint::Storage>(
      identity_, control_, native_->mcu.checkpoint(), native_->dsp.checkpoint(), schedule));
}

void Board::restore(const Checkpoint &checkpoint) {
  require_idle();
  if (!checkpoint.storage_ || checkpoint.storage_->identity != identity_) {
    throw Fault("Reference-board checkpoint belongs to another owner");
  }
  const Busy guard(*this);
  const auto &saved = *checkpoint.storage_;
  auto replacement = std::make_unique<Owners>(config_, execution_);
  replacement->mcu.restore(saved.mcu);
  replacement->dsp.restore(saved.dsp);
  if (saved.schedule) {
    if (!saved.control.release_epoch) { throw Fault("Missing saved DSP release epoch"); }
    auto inputs = config_.schedule;
    inputs.dsp.epoch_ticks = *saved.control.release_epoch;
    replacement->schedule =
        std::make_unique<McuDspSchedule>(inputs, replacement->mcu, replacement->dsp);
    replacement->schedule->restore(*saved.schedule);
  }
  attach(*replacement);
  static_assert(std::is_nothrow_copy_assignable_v<Control>);
  native_.swap(replacement);
  control_ = saved.control;
}

Status Board::status() const {
  require_idle();
  return {.mcu = native_->mcu.cpu().state(),
          .dsp = native_->dsp.cpu().debug_state(),
          .cold_nvm = encode_flash_image(native_->mcu.bus().power_off_flash_image()),
          .common_ticks = ticks(),
          .mcu_cycles = native_->mcu.run_clock().bus_cycles(),
          .mcu_ticks = native_->mcu.run_clock().elapsed_ticks(),
          .oscillator_cycles = native_->mcu.run_clock().oscillator_cycles(),
          .idle_cycles = native_->mcu.idle_bus_cycles(),
          .dsp_ticks = native_->dsp.run_clock().elapsed_ticks(),
          .dsp_units = native_->dsp.run_clock().native_cycle_units(),
          .dsp_steps = native_->schedule ? native_->schedule->completed_steps() : 0,
          .instructions = control_.instructions,
          .operations = control_.operations,
          .interrupts = control_.interrupts,
          .writes = control_.writes,
          .frames = control_.frames,
          .ties = native_->schedule ? native_->schedule->ties() : 0,
          .ordered_pairs = control_.ordered_pairs,
          .next_event = control_.next_event,
          .next_dsp_event = control_.next_dsp_event,
          .next_esai_edge = control_.next_esai_edge,
          .esai_inputs = control_.esai_inputs,
          .pcm = control_.pcm,
          .pcm_stamp = control_.pcm_stamp,
          .last_pcm = control_.last_pcm,
          .recording = control_.recording,
          .midi_queued = midi_queue_size(),
          .midi_bytes = control_.midi_bytes,
          .midi_started = control_.midi.has_value() || (control_.midi_dsp && control_.started),
          .midi_dsp_events = control_.midi_dsp ? control_.midi_dsp->observed_events() : 0,
          .midi_dsp_frames = control_.midi_dsp ? control_.midi_dsp->completed_frames() : 0,
          .started = control_.started,
          .waiting = native_->mcu.cpu().waiting(),
          .held_reset = native_->schedule == nullptr,
          .faulted = control_.faulted || native_->mcu.timing_faulted() ||
                     native_->dsp.timing_faulted() ||
                     (native_->schedule && native_->schedule->faulted())};
}

bool Status::operator==(const Status &other) const {
  return std::tie(mcu.pc, mcu.d, mcu.x, mcu.y, mcu.sp, mcu.ccr) == std::tie(other.mcu.pc,
                                                                            other.mcu.d,
                                                                            other.mcu.x,
                                                                            other.mcu.y,
                                                                            other.mcu.sp,
                                                                            other.mcu.ccr) &&
         std::tie(dsp,
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
                  next_event,
                  next_dsp_event,
                  next_esai_edge,
                  esai_inputs,
                  pcm,
                  pcm_stamp,
                  last_pcm,
                  recording,
                  midi_queued,
                  midi_bytes,
                  midi_started,
                  midi_dsp_events,
                  midi_dsp_frames,
                  started,
                  waiting,
                  held_reset,
                  faulted) == std::tie(other.dsp,
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
                                       other.next_event,
                                       other.next_dsp_event,
                                       other.next_esai_edge,
                                       other.esai_inputs,
                                       other.pcm,
                                       other.pcm_stamp,
                                       other.last_pcm,
                                       other.recording,
                                       other.midi_queued,
                                       other.midi_bytes,
                                       other.midi_started,
                                       other.midi_dsp_events,
                                       other.midi_dsp_frames,
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
    control_.faulted = true;
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

void Board::replay_marker(const std::string_view phase, const unsigned index) {
  const Busy guard(*this);
  if (execution_.trace != nullptr) {
    *execution_.trace << "REPLAY phase=" << phase << " index=" << index
                      << " operations=" << control_.operations << " end=" << ticks() << '\n';
  }
}

Board::Outcome Board::run_replays() {
  if (!config_.checkpoint) { throw Fault("Reference-board replay plan is not configured"); }
  const auto plan = *config_.checkpoint;
  while (!finished() && control_.operations < plan.operation) { advance(); }
  if (control_.operations != plan.operation) {
    throw Fault("Reference-board checkpoint was not reached within the execution budget");
  }
  const auto saved = checkpoint();
  replay_marker("begin", 0);
  const auto expected = finish_branch();
  replay_marker("end", 0);
  for (unsigned index = 0; index < plan.replays; ++index) {
    restore(saved);
    replay_marker("begin", index + 1);
    const auto observed = finish_branch();
    replay_marker("end", index + 1);
    if (observed != expected) {
      control_.faulted = true;
      throw Fault("Reference-board replay changed its visible outcome");
    }
  }
  return expected;
}
}
