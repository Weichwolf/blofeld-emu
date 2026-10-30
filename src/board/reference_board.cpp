// SPDX-License-Identifier: GPL-3.0-only
#include "board/reference_board.hpp"

#include "audio/pcm_recording.hpp"
#include "board/reference_context.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_bus.hpp"
#include "midi/midi_serial.hpp"
#include "midi/midi_timed.hpp"
#include "storage/flash_file.hpp"

#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>

namespace blofeld::reference {
namespace {
McuMachine make_mcu(const Config &config, const Execution &execution) {
  if (execution.device_state && (!execution.flash_out.empty() || config.checkpoint)) {
    throw Fault("A live device state excludes cold export and automatic RAM checkpoint replay");
  }
  if (config.pipeline) {
    return {execution.image, execution.specification, config.mcu, config.pipeline->power_on};
  }
  return {execution.image, execution.specification, config.mcu};
}

constexpr unsigned decoder_reset = 1;
constexpr unsigned decoder_suspended = 2;
constexpr unsigned decoder_enumerated = 4;
constexpr unsigned decoder_full_speed = 8;
}

Board::Owners::Owners(const Config &config, const Execution &execution)
    : mcu(make_mcu(config, execution)),
      dsp(config.pipeline ? std::nullopt : std::optional{config.dsp}) {
  if (execution.device_state) { mcu.attach_flash_store(execution.device_state); }
}

Board::Board(const Config &config, const Execution &execution)
    : config_(config),
      execution_(execution),
      identity_(std::make_shared<const Identity>()),
      native_(std::make_unique<Owners>(config_, execution_)) {
  validate_midi(config_);
  const auto common = common_hz();
  const auto local = native_->mcu.run_clock().timebase_hz();
  if (common < local || common % local != 0) { throw Fault("Invalid reference-board timebase"); }
  control_.scale = common / local;
  prepare_audio_clock();
  if (config_.midi_sci_timed) {
    if (config_.midi_reference) { native_->mcu.set_sci_receive_reference(config_.midi_reference); }
    control_.midi_sci.emplace(
        MidiWireTiming{.timebase_hz = common, .start_ticks = config_.midi_sci_timed->after_ticks});
  }
  if (config_.midi_dsp) {
    MidiWireTiming timing{.timebase_hz = common, .start_ticks = config_.midi_dsp->after_ticks};
    if (config_.midi_dsp_reference) {
      timing.reference_bit_ticks = config_.midi_dsp_reference->bit_ticks;
      timing.reference_idle_bits = config_.midi_dsp_reference->idle_bits;
    }
    control_.midi_dsp.emplace(timing);
    if (execution_.trace != nullptr) {
      *execution_.trace << "DSP-MIDI-SOURCE first=" << timing.start_ticks
                        << " baud=" << (timing.reference_bit_ticks ? 0 : MidiSerialInput::baud)
                        << " reference-bit-ticks=" << timing.reference_bit_ticks.value_or(0)
                        << " reference-idle-bits=" << timing.reference_idle_bits << '\n';
    }
  }
  const auto first_dsp = next_dsp_input();
  if (config.release_ticks < ticks() ||
      (!config.events.empty() && config.events.front().ticks < ticks()) ||
      (first_dsp && first_dsp->input.ticks < ticks())) {
    throw Fault("Reference-board release/events precede the completed native MCU reset");
  }
  prepare_dsp();
  if (config.pcm) {
    if (config.pcm->instance > 1 || config.pcm->lane > 5) {
      throw Fault("Invalid PCM output route");
    }
    control_.pcm.emplace(config.pcm->format);
  }
  if (execution.audio) {
    if (!config.pcm) { throw Fault("WAV output requires pcm-receive in the reference board"); }
    control_.recording = PcmRecording(config.pcm->format, execution.audio->maximum_frames);
  }
  for (const auto &entry : config.initial) { apply(entry); }
  connect_pipeline();
  update_wiring();
  attach(*native_);
}

void Board::attach(Owners &owners) {
  owners.mcu.set_cycle_observer([this](const auto cycle) { on_cycle(cycle); });
  if (config_.pipeline) {
    owners.mcu.set_reset_period_observer([this](const auto period) {
      observe_midi(period.context);
      native_->fetch->begin_reset_period(period, fetch_inputs(), fetch_observer());
      update_wiring();
    });
  }
  owners.mcu.set_wait_period_observer([this](const auto period) {
    observe_midi(period.context);
    check_signals();
    if (native_->fetch) {
      native_->fetch->begin_wait_period(period, fetch_inputs(), fetch_observer());
      update_wiring();
    } else if (native_->schedule) {
      native_->schedule->begin_wait_period(
          period, [this](const auto &step) { before_dsp(step); }, return_observer(), dsp_inputs());
      update_wiring();
    }
  });
  owners.mcu.bus().set_observer([this](const auto &event) { on_write(event); });
}

std::uint64_t Board::ticks() const {
  const auto local = native_->mcu.run_clock().elapsed_ticks();
  if (control_.scale == 0 || local > std::numeric_limits<std::uint64_t>::max() / control_.scale) {
    throw Fault("Reference-board time overflow");
  }
  return local * control_.scale;
}

void Board::apply(const Input &input) {
  switch (input.kind) {
    case InputKind::drive: native_->mcu.bus().set_pin_drive(input.port, input.drive); break;
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
    default: throw Fault("Invalid reference-board input kind");
  }
}

void Board::release() {
  const auto next = next_dsp_input();
  if (!native_->fetch && next && next->input.ticks < ticks()) {
    throw Fault("Timed DSP input precedes the actual DSP release boundary");
  }
  control_.previous = sample();
  if (control_.previous.selected) {
    throw Fault("Reference-board DSP release requires deasserted SPI SS");
  }
  native_->dsp.bus().shi().observe_spi(control_.previous);
  if (native_->fetch) {
    native_->fetch->release_dsp_reset();
    control_.release_epoch = ticks();
  } else {
    auto inputs = config_.schedule;
    inputs.dsp.epoch_ticks = ticks();
    control_.release_epoch = inputs.dsp.epoch_ticks;
    native_->schedule = std::make_unique<McuDspSchedule>(inputs, native_->mcu, native_->dsp);
  }
  update_wiring();
  if (execution_.trace != nullptr) {
    *execution_.trace << "DSP-RELEASE requested=" << config_.release_ticks << " actual=" << ticks()
                      << " pc=" << hex(native_->dsp.cpu().state().pc, 6) << '\n';
  }
}

void Board::boundary() {
  if (!native_->mcu.system_reset_active()) {
    const auto cpu = native_->mcu.cpu().state();
    control_.context = {.pc = cpu.pc, .ccr = cpu.ccr};
  }
  while (control_.next_event < config_.events.size() &&
         config_.events[control_.next_event].ticks <= ticks()) {
    const auto &event = config_.events[control_.next_event];
    apply(event);
    update_wiring();
    if (execution_.trace != nullptr) {
      *execution_.trace << "INPUT requested=" << event.ticks << " actual=" << ticks()
                        << " kind=" << static_cast<unsigned>(event.kind)
                        << " mcu-pc=" << mcu_pc_context(native_->mcu) << '\n';
    }
    if (control_.release_epoch) { transition(sample()); }
    ++control_.next_event;
  }
  if (!control_.release_epoch && ticks() >= config_.release_ticks) { release(); }
  start_midi();
  check_signals();
}

void Board::on_cycle(const Hc12BusCycle cycle) {
  control_.context = cycle.context;
  observe_midi(cycle.context);
  check_signals();
  if (!native_->schedule && !native_->fetch) { return; }
  constexpr unsigned port_b = 1;
  const bool write = cycle.kind == 'w' || cycle.kind == 'W';
  const bool shared =
      write && cycle.address && *cycle.address <= port_b && port_b - *cycle.address < cycle.size;
  if (native_->fetch) {
    native_->fetch->begin_cycle(cycle, shared, fetch_inputs(), fetch_observer());
    update_wiring();
    return;
  }
  native_->schedule->begin_cycle(
      cycle,
      shared,
      [this](const auto &step) { before_dsp(step); },
      return_observer(),
      dsp_inputs());
  update_wiring();
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
  const auto cpu = native_->mcu.cpu().state();
  stream << "MCU pc=" << hex(cpu.pc) << " d=" << hex(cpu.d) << " x=" << hex(cpu.x)
         << " y=" << hex(cpu.y) << " sp=" << hex(cpu.sp) << " ccr=" << hex(cpu.ccr, 2)
         << " instructions=" << control_.instructions << " operations=" << control_.operations
         << " interrupts=" << control_.interrupts
         << " cycles=" << native_->mcu.run_clock().bus_cycles()
         << " idle=" << native_->mcu.idle_bus_cycles()
         << " ticks=" << native_->mcu.run_clock().elapsed_ticks()
         << " timebase=" << native_->mcu.run_clock().timebase_hz() << '\n';
  const auto dsp = native_->dsp.cpu().state();
  stream << "DSP pc=" << hex(dsp.pc, 6) << " omr=" << hex(dsp.omr, 6) << " r4=" << dsp.r[4]
         << " lc=" << dsp.lc << " held-reset=" << dsp_reset_active()
         << " completed-steps=" << dsp_steps()
         << " units=" << (native_->fetch ? 0 : native_->dsp.run_clock().native_cycle_units())
         << " ticks=" << dsp_ticks() << " timebase=" << common_hz()
         << " pipeline-cycles=" << (native_->fetch ? native_->fetch->run().completed_cycles() : 0)
         << " spi-writes=" << control_.writes << " frames=" << control_.frames
         << " ordered-pairs=" << control_.ordered_pairs << " ties=" << schedule_ties() << '\n';
  if (!config_.dsp_events.empty()) {
    stream << "DSP-INPUTS observed=" << control_.next_dsp_event
           << " configured=" << config_.dsp_events.size() << '\n';
  }
  for (const auto &clock : config_.dsp_clocks) {
    stream << "DSP-ESAI-CLOCK instance=" << clock.index
           << " observed=" << control_.next_esai_edge.at(clock.index)
           << " configured=" << clock.edges << '\n';
  }
  if (control_.pcm) {
    stream << "PCM frames=" << control_.pcm->completed_frames()
           << " started=" << control_.pcm_stamp.has_value() << " locked=" << control_.pcm->locked()
           << " faulted=" << control_.pcm->faulted() << '\n';
  }
  if (control_.recording) {
    stream << "AUDIO recorded=" << control_.recording.frames().size()
           << " limit=" << control_.recording.capacity() << '\n';
  }
  if (config_.midi || config_.midi_dsp) {
    stream << "MIDI accepted=" << control_.midi_bytes << " queued=" << midi_queue_size()
           << " started=" << (control_.midi.has_value() || (control_.midi_dsp && control_.started));
    if (control_.midi_dsp) {
      stream << " dsp-events=" << control_.midi_dsp->observed_events()
             << " dsp-frames=" << control_.midi_dsp->completed_frames();
    }
    stream << '\n';
  }
}

void Board::persist() const {
  // A bound controller has already published every cold cut before its flags.
  // Never export stale modeled state after an uncertain writer failure.
  if (execution_.device_state) { return; }
  if (execution_.flash_out.empty()) { return; }
  save_flash_image(execution_.flash_out, native_->mcu.bus().power_off_flash_image());
  std::cout << "Saved private MCU NVM image: " << execution_.flash_out << '\n';
}

void Board::step() {
  if (native_->mcu.system_reset_active()) {
    if (!config_.pipeline || !native_->fetch) { throw Fault("Missing reset pipeline owner"); }
    static_cast<void>(native_->mcu.advance_system_reset(1, config_.pipeline->reset_pad));
    if (!native_->mcu.system_reset_active()) { native_->fetch->end_system_reset(); }
    return;
  }
  if (native_->mcu.cpu().waiting() && !native_->mcu.cpu().pending_interrupt()) {
    static_cast<void>(native_->mcu.advance_wait(1));
    if (native_->fetch) { native_->fetch->end_wait(); }
    if (native_->schedule) { native_->schedule->end_wait(); }
  } else {
    const auto result = native_->mcu.cpu().step();
    if (native_->fetch) { native_->fetch->end_instruction(); }
    if (native_->schedule) { native_->schedule->end_instruction(); }
    if (result.instruction_executed) { ++control_.instructions; }
    if (result.interrupt_vector) {
      ++control_.interrupts;
      if (execution_.trace != nullptr) {
        *execution_.trace << "IRQ vector=" << hex(*result.interrupt_vector)
                          << " stack-reused=" << result.stack_reused << '\n';
      }
    }
  }
  synchronize_midi();
}

int Board::run(const std::function<void(Board &)> &input) {
  require_healthy();
  if (input && config_.checkpoint) {
    throw Fault("External host input cannot be reread by an automatic checkpoint replay");
  }
  std::cout << "Explicit MCU/DSP reference experiment; physical board wiring and timing "
               "are unverified.\n";
  std::optional<std::string> fault;
  try {
    initialize();
    fault = config_.checkpoint ? run_replays().fault : finish_branch(input).fault;
  } catch (const Fault &error) {
    mark_fault();
    fault = error.what();
  }
  flush_trace();
  if (fault) {
    std::cerr << "Reference-board execution stopped: " << *fault << '\n';
    report(std::cerr);
    persist();
    return 2;
  }
  std::cout << "Reference experiment budget reached; boot completion is not established.\n";
  report(std::cout);
  persist();
  save_audio();
  return 0;
}
}
