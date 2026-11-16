// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/state_probe.hpp"

#include "display/port.hpp"
#include "display/st7565.hpp"
#include "io/dsp_reset.hpp"
#include "io/mcu_flash.hpp"
#include "io/midi_port.hpp"
#include "io/sci.hpp"
#include "io/usb.hpp"
#include "mcu/channels.hpp"
#include "mcu/clock.hpp"
#include "mcu/devices.hpp"
#include "mcu/io.hpp"
#include "mcu/memory.hpp"
#include "mcu/timer.hpp"

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/SHA256.h"

#include <array>
#include <atomic>
#include <cassert>
#include <concepts>
#include <cpu12/core.hpp>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ostream>
#include <span>
#include <stdexcept>
#include <string_view>
#include <type_traits>

namespace blofeld::vi::mcu {
namespace {
thread_local StateProbe *current = nullptr;
constexpr unsigned byte_bits = 8;

struct Hash {
  llvm::SHA256 value;

  void bytes(const std::span<const std::uint8_t> bytes) {
    value.update(llvm::ArrayRef(bytes.data(), bytes.size()));
  }

  template <class T> void add(const T &item) {
    if constexpr (std::integral<T> || std::is_enum_v<T>) {
      auto word = static_cast<std::uint64_t>(item);
      std::array<std::uint8_t, sizeof(word)> bytes{};
      for (auto &byte : bytes) {
        byte = static_cast<std::uint8_t>(word);
        word >>= byte_bits;
      }
      this->bytes(bytes);
    } else if constexpr (requires { item.has_value(); }) {
      add(item.has_value());
      if (item) { add(*item); }
    } else {
      add(item.size());
      for (const auto &element : item) { add(element); }
    }
  }

  template <class... T> void operator()(const T &...items) { (add(items), ...); }

  void text(const char *text) {
    add(text != nullptr);
    if (text != nullptr) {
      const std::string_view view(text);
      add(view.size());
      value.update(llvm::StringRef(view));
    }
  }
};

auto digest(const auto &write) {
  Hash hash;
  write(hash);
  return hash.value.final();
}
}

StateProbe::Scope::Scope(StateProbe *probe) {
  assert(current == nullptr);
  current = probe;
}

StateProbe::Scope::~Scope() {
  current = nullptr;
}

StateProbe *StateProbe::active() noexcept {
  return current;
}

StateProbe::Record StateProbe::capture(const cpu12::Core &core,
                                       const Memory &memory,
                                       const Devices &devices,
                                       const Io &io,
                                       const std::uint64_t instructions) {
  DspReset::State reset;
  blofeld_mcu_reset_state(&io.reset_, reset);
  const auto cpu = core.save();
  const auto ram = memory.save();
  const auto local = io.save();
  Record record{.instructions = instructions,
                .cycles = cpu.progress.estimated_cycles,
                .ticks = devices.clock.ticks(),
                .idle = devices.idle,
                .pc = cpu.state.registers.pc,
                .reset_epoch = reset.epoch,
                .reset_held = reset.held};
  record.parts[0] = digest([&](Hash &hash) {
    const auto &r = cpu.state.registers;
    const auto &f = cpu.state.failure;
    hash(r.d,
         r.x,
         r.y,
         r.sp,
         r.pc,
         r.ccr,
         cpu.state.execution,
         cpu.state.irq_delay,
         cpu.state.returned_frame,
         f.fault,
         f.retired,
         f.pc,
         f.address,
         f.value,
         f.opcode,
         f.state,
         f.access_size,
         f.read_mask,
         record.instructions,
         record.cycles,
         record.ticks,
         record.idle,
         record.reset_epoch,
         record.reset_held);
  });
  record.parts[1] = digest([&](Hash &hash) {
    hash.bytes(ram.system);
    hash.bytes(ram.system_known);
    hash.bytes(ram.queue);
    hash.bytes(ram.queue_known);
    hash.bytes(ram.mapping);
    hash.bytes(ram.pending);
    hash(ram.written);
  });
  record.parts[2] = timer_state(devices.timer);
  record.parts[3] = clock_state(devices.clock);
  record.parts[4] = sci_state(devices.sci, devices.midi_error);
  record.parts[5] = usb_state(devices.usb);
  record.parts[6] = gpio_state(local);
  record.parts[7] = flash_state(local.flash);
  record.parts[8] = lcd_state(local.display);
  return record;
}

StateProbe::Digest StateProbe::timer_state(const Timer &t) {
  return digest([&](Hash &hash) {
    hash(t.registers_,
         t.channels_,
         t.time_,
         t.counter_,
         t.pulse_count_,
         t.phase_,
         t.prescale_,
         t.pulse_control_,
         t.activity_);
  });
}

StateProbe::Digest StateProbe::clock_state(const Clock &c) {
  return digest([&](Hash &hash) {
    hash(c.time_,
         c.ticks_,
         c.rti_phase_,
         c.qualification_,
         c.usb_,
         c.divider_,
         c.flags_,
         c.select_,
         c.enable_,
         c.rti_,
         c.cop_,
         c.cop_written_,
         c.cop_wait_written_,
         c.activity_);
  });
}

StateProbe::Digest StateProbe::sci_state(const Sci &s, const char *midi_error) {
  return digest([&](Hash &hash) {
    hash(s.count_);
    for (std::size_t index = 0; index < s.count_; ++index) {
      const auto &out = s.output_[(s.head_ + index) % s.output_.size()];
      hash(out.cycle, out.word, out.bits);
    }
    const auto payload = [&](const auto &p) { hash(p.word, p.bits, p.data); };
    hash(s.transmitting_.has_value());
    if (s.transmitting_) {
      const auto &t = *s.transmitting_;
      payload(t.payload);
      hash(t.transfer, t.end, t.checked);
    }
    hash(s.receiving_.has_value());
    if (s.receiving_) {
      const auto &r = *s.receiving_;
      hash(r.frame.word, r.frame.errors, r.end);
    }
    hash(s.prepared_.has_value());
    if (s.prepared_) { payload(*s.prepared_); }
    hash(s.launch_,
         s.idle_,
         s.time_,
         s.baud_,
         s.received_,
         s.transmit_,
         s.phase_,
         s.baud_high_,
         s.control1_,
         s.control2_,
         s.control3_,
         s.status_,
         s.seen_tx_,
         s.seen_rx_,
         s.baud_pending_,
         s.generator_started_,
         s.preamble_,
         s.rx_active_,
         s.idle_armed_,
         s.activity_.waiting,
         s.activity_.clocked);
    hash.text(s.error_);
    hash.text(midi_error);
  });
}

StateProbe::Digest StateProbe::usb_state(const Usb &u) {
  return digest([&](Hash &hash) {
    for (const auto &b : u.buffers_) {
      hash.bytes(b.bytes);
      hash.bytes(b.known);
    }
    hash(u.configurations_,
         u.endpoints_,
         u.status_,
         u.interrupts_,
         u.configuration_status_,
         u.selector_,
         u.bound_buffers_,
         u.staged_high_);
  });
}

StateProbe::Digest StateProbe::gpio_state(const Io::Snapshot &local) {
  return digest([&](Hash &hash) {
    const auto &p = local.ports.state;
    for (const auto &port : p.ports) {
      hash(port.latch,
           port.known,
           port.direction,
           port.reduced,
           port.pull,
           port.polarity,
           port.external.mask,
           port.external.known,
           port.external.value,
           port.external.fallback_low);
    }
    hash(p.assignment, p.mode, p.pull, p.reduced, p.routing, p.assignment_written, p.mode_written);
    const auto &b = local.ports.boot;
    hash(b.previous, b.selected, b.pending_word, b.mode.has_value(), b.bits, b.word);
    if (b.mode) { hash(b.mode->bits, b.mode->falling, b.mode->continuous); }
    hash(local.release_reset, local.peer_strobe, local.panel_sampled);
  });
}

StateProbe::Digest StateProbe::flash_state(const McuFlash &f) {
  return digest([&](Hash &hash) {
    hash(f.divider_,
         f.security_,
         f.protection_,
         f.status_,
         f.command_,
         f.security_known_,
         f.protection_known_);
  });
}

StateProbe::Digest StateProbe::lcd_state(const DisplayPort &d) {
  return digest([&](Hash &hash) {
    const auto &c = d.controller_;
    const auto &s = c.state_;
    hash.bytes(c.bytes_);
    for (std::size_t index = 0; index < c.known_.size(); ++index) { hash(c.known_.test(index)); }
    hash(s.page,
         s.column,
         s.start_line,
         s.volume,
         s.power,
         s.static_mode,
         s.resistor,
         s.booster,
         s.display_on,
         s.reverse,
         s.adc_reverse,
         s.all_points,
         s.bias,
         s.common_reverse,
         s.static_on,
         s.read_modify_write,
         c.holder_.value,
         c.holder_.known,
         c.parameter_,
         c.saved_column_,
         c.dummy_,
         c.reset_observed_,
         d.held_.value,
         d.held_.known,
         d.enable_,
         d.active_,
         d.reading_,
         d.data_);
    hash.text(d.error_);
  });
}

void StateProbe::end_at(const std::uint64_t ticks) {
  constexpr std::uint64_t drain = MidiPort::reference_hz;
  if (ticks > std::numeric_limits<std::uint64_t>::max() - drain) {
    throw std::overflow_error("MCU state qualification end time overflows");
  }
  end_.store(ticks + drain, std::memory_order_release);
}

void StateProbe::pcm_complete() noexcept {
  pcm_done_.store(true, std::memory_order_release);
}

bool StateProbe::stopping(const std::uint64_t ticks) const noexcept {
  const auto end = end_.load(std::memory_order_acquire);
  return end != 0 && ticks >= end;
}

void StateProbe::executed(const std::uint32_t instructions, const bool replaying) noexcept {
  executed_ += instructions;
  if (replaying) { replayed_ += instructions; }
}

void StateProbe::append(Record record) {
  record.ordinal = ++count_;
  chain_ = digest([&](Hash &hash) {
    hash.bytes(chain_);
    hash(record.ordinal, record.instructions, record.cycles, record.ticks, record.idle, record.pc);
    for (const auto &part : record.parts) { hash.bytes(part); }
  });
  record.chain = chain_;
  if (!first_) { first_ = record; }
  records_[(count_ - 1) % records_.size()] = record;
}

void StateProbe::wait(Record record, const bool speculative) {
  if (stopping(record.ticks)) { ending_ = record; }
  if (!speculative) {
    append(record);
    return;
  }
  if (pending_count_ == pending_.size()) { throw std::overflow_error("MCU state probe is full"); }
  pending_[pending_count_++] = record;
}

void StateProbe::commit(const std::uint64_t instructions,
                        const cpu12::Progress progress,
                        const std::uint64_t ticks) {
  for (std::size_t index = 0; index < pending_count_; ++index) { append(pending_[index]); }
  pending_count_ = 0;
  confirmed_instructions_ = instructions;
  confirmed_cycles_ = progress.estimated_cycles;
  confirmed_ticks_ = ticks;
  if (ending_) {
    if (ending_->instructions != instructions || ending_->cycles != progress.estimated_cycles ||
        ending_->ticks != ticks || !pcm_done_.load(std::memory_order_acquire) || stop_ == nullptr) {
      throw std::logic_error(
          "MCU state qualification did not stop at its confirmed wait after PCM");
    }
    stop_->request_stop();
  }
}

void StateProbe::rollback(const std::uint64_t discarded) noexcept {
  discarded_ += discarded;
  pending_count_ = 0;
  ending_.reset();
}

void StateProbe::finish(Record record) noexcept {
  final_ = record;
}

void StateProbe::report(std::ostream &output) const {
  constexpr std::array names{"core", "ram", "timer", "clock", "sci", "usb", "gpio", "flash", "lcd"};
  output << "MCU_PROBE executed=" << executed_ << " discarded=" << discarded_
         << " replay-executed=" << replayed_
         << " confirmed-instructions=" << confirmed_instructions_
         << " confirmed-cycles=" << confirmed_cycles_ << " confirmed-ticks=" << confirmed_ticks_
         << " waits=" << count_ << " pending-waits=" << pending_count_
         << " stop-cut=" << end_.load() << " pcm-complete=" << pcm_done_.load() << '\n';
  const auto write = [&](const char *kind, const Record &record) {
    output << kind << " ordinal=" << record.ordinal << " instructions=" << record.instructions
           << " cycles=" << record.cycles << " ticks=" << record.ticks << " idle=" << record.idle
           << " pc=" << record.pc << " reset-epoch=" << record.reset_epoch
           << " reset-held=" << record.reset_held << " chain=" << llvm::toHex(record.chain, true);
    for (std::size_t index = 0; index < record.parts.size(); ++index) {
      output << ' ' << names[index] << '=' << llvm::toHex(record.parts[index], true);
    }
    output << '\n';
  };
  if (first_) { write("MCU_WAIT_FIRST", *first_); }
  const auto first = count_ > records_.size() ? count_ - records_.size() : 0;
  for (auto index = first; index < count_; ++index) {
    write("MCU_WAIT", records_[index % records_.size()]);
  }
  if (final_) { write("MCU_EXIT", *final_); }
}
}
