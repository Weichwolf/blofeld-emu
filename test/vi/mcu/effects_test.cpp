// SPDX-License-Identifier: GPL-3.0-only
#include "io/dsp_reset.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "isa.hpp"
#include "mcu/devices.hpp"
#include "mcu/effects.hpp"
#include "mcu/io.hpp"
#include "mcu/memory.hpp"
#include "mcu/output.hpp"
#include "mcu/program.hpp"
#include "mcu/timer.hpp"
#include "support/mcu.hpp"

#include <algorithm>
#include <array>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <format>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <vector>

namespace {
namespace vi = blofeld::vi;
namespace mcu = vi::mcu;
using mcu::Effects;
using Opcode = cpu12::isa::Opcode;
constexpr std::uint8_t all = 0xff, first = 0xa5, last = 0x5a, transmitted = 0x90;
constexpr std::uint32_t control = 0xffff91, receive = 0xffff94, status = control;
constexpr std::uint32_t fifo24 = 0x29, busy = 0x400000, word = 0xa591e7;
constexpr cpu12::Address entry = 0x8000, stack = 0x1fff;
constexpr cpu12::Address q_data = 0x260, q_direction = 0x262, j_data = 0x250, j_direction = 0x252;
constexpr cpu12::Address sci_baud = 0xc8, sci_control = 0xcb, sci_data = 0xcf;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void write(std::vector<cpu12::Byte> &code, const cpu12::Address address, const std::uint8_t value) {
  constexpr auto load = cpu12::isa::opcode<Opcode::ldaa_immediate>();
  constexpr auto store = cpu12::isa::opcode<Opcode::staa_extended>();
  const std::array bytes{load[0],
                         value,
                         store[0],
                         static_cast<std::uint8_t>(address >> 8U),
                         static_cast<std::uint8_t>(address & all)};
  code.insert(code.end(), bytes.begin(), bytes.end());
}

std::vector<cpu12::Byte> program() {
  std::vector<cpu12::Byte> code;
  write(code, q_direction, all);
  write(code, q_data, first);
  write(code, j_data, 1);
  write(code, j_direction, 1);
  write(code, j_data, 0);
  write(code, sci_baud, 0);
  write(code, sci_baud + 1, 1);
  write(code, sci_control, 8);
  const auto arm = cpu12::isa::program(cpu12::isa::opcode<Opcode::ldaa_extended>(),
                                       std::array<cpu12::Byte, 2>{0, 0xcc});
  code.insert(code.end(), arm.begin(), arm.end());
  write(code, sci_data, transmitted);
  write(code, 3, all);
  write(code, 1, 3);
  write(code, 1, 1);
  constexpr unsigned width = 24;
  constexpr unsigned data_bit = 0x10;
  constexpr unsigned clock_bit = 4;
  for (unsigned bit = width; bit != 0; --bit) {
    const auto data = static_cast<std::uint8_t>(1 | (((word >> (bit - 1)) & 1) * data_bit));
    write(code, 1, data);
    write(code, 1, static_cast<std::uint8_t>(data | clock_bit));
    write(code, 1, data);
  }
  write(code, 1, 3);
  write(code, q_data, last);
  constexpr auto wait_transmit = cpu12::isa::program(cpu12::isa::opcode<Opcode::ldaa_extended>(),
                                                     std::array<cpu12::Byte, 2>{0, 0xcc},
                                                     cpu12::isa::opcode<Opcode::bita_immediate>(),
                                                     std::array<cpu12::Byte, 1>{0x40},
                                                     cpu12::isa::opcode<Opcode::beq_relative>(),
                                                     std::array<cpu12::Byte, 1>{0xf9});
  code.insert(code.end(), wait_transmit.begin(), wait_transmit.end());
  code.insert(code.end(), cpu12::isa::wai.begin(), cpu12::isa::wai.end());
  return code;
}

struct Machine {
  vi::PeerPort peer;
  blofeld::test::McuImage image;
  mcu::Memory memory{*image};
  const std::stop_token stop;
  vi::MidiPort midi{vi::MidiPort::InputMode::stream};
  mcu::Devices devices{stop};
  vi::Shi shi;
  vi::DspReset reset;
  mcu::Io io{memory, *image, devices, shi, reset, stop, nullptr, nullptr, &peer};
  cpu12::Io callbacks = io.callbacks();
  std::vector<cpu12::Ram> grants;
  cpu12::Core core{cpu12::Memory::external(), &callbacks};

  Machine() {
    devices.peer = &peer;
    devices.midi = &midi;
    io.bind(core);
    require(midi.finish_input() && shi.write({.address = control, .value = fifo24}),
            "Use real configured SPI and a complete empty MIDI input stream");
  }

  void bind(const mcu::Program &compiled) {
    static_cast<void>(memory.commit_mapping());
    const auto selected = compiled.admit(memory, core.registers().pc);
    auto next = memory.regions();
    core.set_memory(cpu12::Memory::mapped(next));
    compiled.install(core, selected);
    grants.swap(next);
  }

  void run(const Effects &effects, const mcu::Timer::Cycle interval = 0) {
    constexpr unsigned maximum_sessions = 4096;
    for (unsigned session = 0; session < maximum_sessions; ++session) {
      const auto deadline = devices.deadline();
      auto until =
          deadline ? *deadline - devices.idle : std::numeric_limits<mcu::Timer::Cycle>::max() / 2;
      if (interval != 0) { until = std::min(until, core.progress().estimated_cycles + interval); }
      const auto result = core.run_until(until);
      if (result.fault != cpu12::Fault::none || !io.complete_events()) {
        const auto diagnostic = effects.diagnostic();
        throw std::runtime_error(std::format("Fused MCU fault={} PC=${:04x} address=${:04x} "
                                             "value=${:04x} effects={} ticks={}; expected={}/{}/{} "
                                             "actual={}/{}/{}; {}; {}",
                                             static_cast<unsigned>(result.fault),
                                             result.pc,
                                             result.address,
                                             result.value,
                                             static_cast<unsigned>(effects.error()),
                                             devices.clock.ticks(),
                                             static_cast<unsigned>(diagnostic.expected.kind),
                                             diagnostic.expected.ticks,
                                             diagnostic.expected.value,
                                             static_cast<unsigned>(diagnostic.actual.kind),
                                             diagnostic.actual.ticks,
                                             diagnostic.actual.value,
                                             io.error() != nullptr ? io.error() : "no device error",
                                             io.context()));
      }
      devices.activity(result.state);
      if (result.state == cpu12::ExecutionState::waiting) { return; }
    }
    throw std::runtime_error("Authored MCU output fixture did not reach WAI");
  }
};

void fused_outputs(const bool fine) {
  Machine machine;
  const auto bytes = program();
  machine.image.load(entry, bytes);
  const auto compiled = machine.image.program();
  machine.core.set_registers({.pc = entry, .sp = stack});
  machine.bind(*compiled);
  const auto cpu = machine.core.save();
  const auto memory = machine.memory.save();
  const auto devices = machine.devices.save();
  const auto io = machine.io.save();
  std::array<Effects::Entry, 4096> storage{};
  Effects effects(storage, 16);
  const mcu::Output output(effects, machine.devices, machine.core);
  machine.run(effects);
  const auto finished = machine.core.save();
  const auto finished_memory = machine.memory.save();
  const auto finished_time = machine.devices.clock.ticks();
  std::vector<Effects::Entry> finished_effects;
  for (const auto &event : effects.entries()) {
    if (event.kind != Effects::Kind::time) { finished_effects.push_back(event); }
  }
  require(effects.error() == Effects::Error::none && effects.size() != 0,
          "Fused channel calls append bounded timestamped output records");
  std::uint32_t value = 0;
  require(machine.peer.mcu_data() == vi::PeerPort::Sample{} && !machine.peer.dsp_interrupt(true) &&
              machine.shi.read(status, value) && (value & busy) == 0 &&
              !machine.shi.read(receive, value) && !machine.midi.read_output() &&
              machine.midi.time() == 0,
          "Speculative GPIO, SS, SPI word, MIDI byte and published time remain invisible");
  const auto records = std::span(storage).first(effects.size());
  const auto selected = std::ranges::find_if(records, [](const Effects::Entry &event) {
    return event.kind == Effects::Kind::select && event.value != 0;
  });
  require(selected != records.end(), "Original GPIO bit edges emit an actual SS assertion");
  const auto apply = [&](const Effects::Entry &event) {
    return mcu::deliver(event, machine.peer, machine.shi, machine.midi, machine.stop);
  };
  require(effects.apply_before(selected->ticks, apply) && machine.shi.read(status, value) &&
              (value & busy) == 0 && machine.peer.dsp_interrupt(true) == 1 &&
              machine.peer.sample_mcu_data(selected->ticks - 1) ==
                  vi::PeerPort::Sample{.mask = all, .known = all, .value = first},
          "Commit earlier real GPIO effects while withholding SS at the exact same instant");
  const auto committed_time = machine.midi.time();
  machine.memory.restore(memory);
  machine.devices.restore(devices);
  machine.io.restore(io);
  machine.core.restore(cpu);
  machine.bind(*compiled);
  require(effects.rollback(), "Discard unobserved suffix while retaining the committed prefix");
  machine.run(effects, fine ? 1 : 0);
  const auto replayed_memory = machine.memory.save();
  std::vector<Effects::Entry> replayed_effects;
  for (const auto &event : effects.entries()) {
    if (event.kind != Effects::Kind::time) { replayed_effects.push_back(event); }
  }
  require(machine.core.save() == finished && machine.devices.clock.ticks() == finished_time &&
              replayed_effects == finished_effects &&
              replayed_memory.system == finished_memory.system &&
              replayed_memory.system_known == finished_memory.system_known &&
              replayed_memory.queue == finished_memory.queue &&
              replayed_memory.queue_known == finished_memory.queue_known &&
              replayed_memory.mapping == finished_memory.mapping &&
              replayed_memory.pending == finished_memory.pending &&
              replayed_memory.written == finished_memory.written && effects.finish_replay() &&
              machine.midi.time() == committed_time && machine.peer.dsp_interrupt(true) == 1 &&
              machine.peer.sample_mcu_data(machine.devices.clock.ticks()) ==
                  vi::PeerPort::Sample{.mask = all, .known = all, .value = first},
          "Replay regenerates the committed prefix without duplicate IRQs or rewinding host time");
  const auto end = machine.devices.clock.ticks();
  require(effects.apply_before(end + 1, apply) && effects.checkpoint(end) &&
              machine.peer.mcu_data() ==
                  vi::PeerPort::Sample{.mask = all, .known = all, .value = last} &&
              machine.peer.sample_mcu_data(end) == machine.peer.mcu_data() &&
              machine.peer.dsp_interrupt(true) == 1 && machine.shi.read(receive, value) &&
              value == word && !machine.shi.read(receive, value),
          "The validated suffix delivers actual GPIO and exactly one original 24-bit word");
  const auto sent = machine.midi.read_output();
  require(sent && sent->byte == transmitted && !machine.midi.read_output() &&
              machine.midi.time() == end && effects.size() == 0,
          "Exactly one original SCI character and its validated final time reach the host");
}

void prefix_errors() {
  constexpr Effects::Entry original{.ticks = 1, .value = first, .kind = Effects::Kind::midi};
  std::array<Effects::Entry, 8> storage{};
  Effects effects(storage, 2);
  unsigned delivered = 0;
  const auto sink = [&](const Effects::Entry &) {
    ++delivered;
    return true;
  };
  require(effects.append(original) && effects.apply_before(2, sink) && effects.rollback(),
          "Retain an actually delivered byte for exact prefix comparison");
  require(!effects.append({.ticks = 1, .value = last, .kind = Effects::Kind::midi}) &&
              effects.error() == Effects::Error::prefix && delivered == 1,
          "A divergent committed prefix is a failure before another device effect");
  std::array<Effects::Entry, 8> missing_storage{};
  Effects missing(missing_storage, 2);
  require(missing.append(original) && missing.apply_before(2, sink) && missing.rollback() &&
              !missing.finish_replay() && missing.error() == Effects::Error::prefix,
          "Omitting a committed effect cannot silently finish replay");
  std::array<Effects::Entry, 8> late_storage{};
  Effects late(late_storage, 2);
  require(late.apply_before(2, sink) && !late.append(original) &&
              late.error() == Effects::Error::prefix,
          "New output cannot be inserted into already attained receiver history");
  std::array<Effects::Entry, 8> next_storage{};
  Effects next(next_storage, 2);
  constexpr Effects::Entry time{.ticks = 1, .kind = Effects::Kind::time};
  const auto before = delivered;
  require(next.append(time) && next.apply_before(2, sink) && next.checkpoint(1) &&
              next.append(time) && next.apply_before(2, sink) && delivered == before + 2,
          "The next native instruction may publish at the preceding checkpoint's cycle");
  std::array<Effects::Entry, 8> shifted_storage{};
  Effects shifted(shifted_storage, 2);
  require(shifted.append({.ticks = 2, .value = first, .kind = Effects::Kind::midi}) &&
              shifted.apply_before(3, sink) && shifted.rollback() && shifted.append(time) &&
              !shifted.append(original) && shifted.error() == Effects::Error::prefix,
          "Extra progress cannot authorize a changed timestamp for a committed MIDI byte");
  std::array<Effects::Entry, 8> progress_storage{};
  Effects progress(progress_storage, 2);
  require(progress.append({.ticks = 2, .kind = Effects::Kind::time}) &&
              progress.apply_before(3, sink) && progress.rollback() && progress.append(time) &&
              !progress.finish_replay() && progress.error() == Effects::Error::prefix,
          "Finer progress cannot omit an originally committed progress boundary");
}
}

int main() try {
  fused_outputs(false);
  fused_outputs(true);
  prefix_errors();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
