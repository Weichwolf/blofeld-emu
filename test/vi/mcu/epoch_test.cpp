// SPDX-License-Identifier: GPL-3.0-only
#include "io/peer_port.hpp"
#include "isa.hpp"
#include "mcu/epoch.hpp"
#include "mcu/input_history.hpp"

#include <algorithm>
#include <array>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <latch>
#include <span>
#include <stdexcept>
#include <thread>

namespace {
using blofeld::vi::PeerPort;
using blofeld::vi::mcu::Epoch;
using blofeld::vi::mcu::InputHistory;
using Opcode = cpu12::isa::Opcode;
constexpr cpu12::Address entry = 0x200, unknown = 0x400, handler = 0x500, vector = 0xfff0;
constexpr cpu12::Address data = 0x1000, consume = 0x1100, status = 0x1101, invalid = 0x1200;
constexpr cpu12::Address ram_first = 0x1800, stack = 0x9000;
constexpr std::uint8_t stale = 0x11, corrected = 0x5a, byte_mask = 0xff, first_byte = 0xa0;
constexpr std::uint64_t variant = 1;
constexpr std::array<cpu12::Byte, 2> spin{0x20, 0xfe};
constexpr auto code = cpu12::isa::program(cpu12::isa::nop,
                                          cpu12::isa::opcode<Opcode::ldaa_extended>(),
                                          std::array<cpu12::Byte, 2>{0x10, 0},
                                          cpu12::isa::opcode<Opcode::staa_extended>(),
                                          std::array<cpu12::Byte, 2>{0x18, 0},
                                          cpu12::isa::opcode<Opcode::ldd_extended>(),
                                          std::array<cpu12::Byte, 2>{0x11, 0},
                                          cpu12::isa::opcode<Opcode::staa_extended>(),
                                          std::array<cpu12::Byte, 2>{0x18, 1},
                                          cpu12::isa::opcode<Opcode::ldaa_extended>(),
                                          std::array<cpu12::Byte, 2>{0x10, 0},
                                          cpu12::isa::opcode<Opcode::staa_extended>(),
                                          std::array<cpu12::Byte, 2>{0x18, 2},
                                          cpu12::isa::wai,
                                          spin);
constexpr auto bad_code = cpu12::isa::program(
    cpu12::isa::opcode<Opcode::ldaa_extended>(), std::array<cpu12::Byte, 2>{0x12, 0}, spin);
constexpr auto irq_code = cpu12::isa::program(cpu12::isa::rti);
const std::array spans{cpu12::CodeSpan{.address = entry, .words = code},
                       cpu12::CodeSpan{.address = unknown, .words = bad_code},
                       cpu12::CodeSpan{.address = handler, .words = irq_code}};
constexpr std::array entries{entry, unknown, handler};
constexpr std::array layout{
    cpu12::RegionLayout{.first = entry, .size = code.size(), .writable = false},
    cpu12::RegionLayout{.first = unknown, .size = bad_code.size(), .writable = false},
    cpu12::RegionLayout{.first = handler, .size = irq_code.size(), .writable = false}};

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

struct Machine {
  PeerPort peer;
  std::latch waiting{1}, produced{1};
  std::uint64_t last_read = 0;
  Epoch::Read change;
  cpu12::Io io{.owner = this,
               .read = read,
               .write = write,
               .read_word = read_word,
               .write_word = write_word};
  cpu12::Core core{cpu12::Memory::external(), &io};
  std::array<cpu12::Ram, spans.size()> grants{};
  std::array<Epoch::Read, 32> observations{};
  Epoch epoch{observations, 8};
  std::array<Epoch::Read, 32> changes{};
  InputHistory history{changes, {stale}};
  unsigned popped = 0, low_reads = 0, unknown_reads = 0;
  bool waited = false, producer_ok = false, pending_irq = false, low_atomic = false;
  bool active_irq_rejected = false;
  std::array<std::uint8_t, cpu12::address_bytes> ram{};

  explicit Machine(const cpu12::NativeImage &image) {
    for (std::size_t index = 0; index < spans.size(); ++index) {
      const auto span = spans[index];
      std::ranges::copy(span.words, ram.begin() + span.address);
      grants[index] = {.first = span.address,
                       .bytes = std::span(ram).subspan(span.address, span.words.size()),
                       .writable = false};
    }
    ram[vector] = static_cast<std::uint8_t>(handler >> 8U);
    ram[vector + 1] = static_cast<std::uint8_t>(handler & byte_mask);
    core.set_memory(cpu12::Memory::mapped(grants));
    core.install(image, variant);
    core.set_registers({.pc = entry, .sp = stack});
    peer.dsp_data({.mask = byte_mask, .known = byte_mask, .value = 0x0d});
    require(peer.dsp_ready(true) && peer.dsp_ready(false) && peer.mcu_strobe(true) &&
                peer.mcu_strobe(false),
            "Establish actual report ownership before producing its bytes");
    peer.dsp_data({.mask = byte_mask, .known = byte_mask, .value = stale});
  }

  void produce() noexcept {
    waiting.wait();
    producer_ok = true;
    for (unsigned index = 0; index < 16; ++index) {
      const PeerPort::Sample sample{.mask = byte_mask,
                                    .known = byte_mask,
                                    .value = static_cast<std::uint8_t>(first_byte + index)};
      peer.dsp_data(sample);
      producer_ok = peer.dsp_write(sample, last_read) && producer_ok;
    }
    peer.dsp_data({});
    change = {.ticks = last_read, .value = 0, .mask = byte_mask, .signal = Epoch::Signal::data};
    produced.count_down();
  }

  static bool read(void *owner, const cpu12::Address address, std::uint8_t &value) noexcept {
    auto &self = *static_cast<Machine *>(owner);
    const auto now = self.core.progress().estimated_cycles;
    if (address == invalid) {
      ++self.unknown_reads;
      return false;
    }
    if (address == data) {
      self.last_read = now;
      const auto sample = self.history.at(now, Epoch::Signal::data);
      if (!sample) { return false; }
      value = static_cast<std::uint8_t>(*sample);
      const bool recorded = self.epoch.record(
          {.ticks = now, .value = value, .mask = byte_mask, .signal = Epoch::Signal::data});
      if (self.epoch.yield()) { self.core.request_yield(); }
      return recorded;
    }
    if (address == consume) {
      if (!self.waited) {
        self.waited = true;
        self.waiting.count_down();
        self.produced.wait();
        if (!self.producer_ok || !self.history.write(self.change)) { return false; }
        if (!self.epoch.changed(self.change)) { return self.epoch.refuse(); }
      }
      if (!self.epoch.attained(now + 1)) { return false; }
      const bool accepted = self.epoch.consume(now, [&] {
        const auto sample = self.peer.read_dsp_data({.mask = byte_mask, .ticks = now});
        if (!sample) { return false; }
        value = sample->value;
        ++self.popped;
        return true;
      });
      if (accepted) { self.core.request_yield(); }
      return accepted;
    }
    if (address == status) {
      ++self.low_reads;
      self.low_atomic = self.epoch.atomic();
      self.pending_irq = true;
      const auto attempted = self.core.interrupt({.vector = vector});
      self.active_irq_rejected = !attempted.accepted && attempted.fault == cpu12::Fault::reentrant;
      value = corrected;
      return self.epoch.record(
          {.ticks = now, .value = value, .mask = byte_mask, .signal = Epoch::Signal::mode});
    }
    value = self.ram[address];
    return true;
  }

  static bool read_word(void *owner, const cpu12::Address address, std::uint16_t &value) noexcept {
    std::uint8_t high = 0;
    std::uint8_t low = 0;
    if (!read(owner, address, high) ||
        !read(owner, static_cast<cpu12::Address>(address + 1U), low)) {
      return false;
    }
    value = static_cast<std::uint16_t>((unsigned{high} << 8U) | low);
    return true;
  }

  static bool write_word(void *owner, const cpu12::WordWrite access) noexcept {
    return write(owner,
                 {.address = access.address,
                  .value = static_cast<std::uint8_t>(access.value >> 8U)}) &&
           write(owner,
                 {.address = static_cast<cpu12::Address>(access.address + 1U),
                  .value = static_cast<std::uint8_t>(access.value & byte_mask)});
  }

  static bool write(void *owner, const cpu12::ByteWrite access) noexcept {
    auto &self = *static_cast<Machine *>(owner);
    self.ram[access.address] = access.value;
    return true;
  }
};

void native_cancellation_and_commit(const cpu12::NativeImage &image) {
  Machine machine(image);
  const auto initial = machine.core.save();
  const auto memory = machine.ram;
  std::jthread producer([&] { machine.produce(); });
  require(machine.core.run(3).retired == 3 && machine.ram[ram_first] == stale,
          "Actual LLVM code reads and stores a speculative DSP value");
  const auto failed = machine.core.run(1);
  producer.join();
  require(failed.fault == cpu12::Fault::memory_read && failed.retired == 0 &&
              machine.epoch.cancelled() && machine.epoch.conflict() == machine.last_read &&
              machine.epoch.refusal() == Epoch::Refusal::epoch_invalidated && machine.popped == 0 &&
              machine.low_reads == 0 && machine.peer.error() == nullptr,
          "A conflict discovered in the wait refuses the native access before any FIFO pop");
  require(machine.history.at(machine.last_read - 1, Epoch::Signal::data) == stale,
          "An attained future output cannot leak into a replayed earlier read");
  machine.core.restore(initial);
  machine.ram = memory;
  require(machine.epoch.rollback(), "Only the explicitly invalidated epoch can roll back");
  const auto replay = machine.core.run(4);
  require(replay.fault == cpu12::Fault::none && replay.retired == 4 &&
              machine.ram[ram_first] == 0 && machine.popped == 1 && machine.low_reads == 1 &&
              machine.low_atomic && machine.pending_irq && machine.active_irq_rejected &&
              machine.core.registers().d == ((std::uint16_t{first_byte} << 8U) | corrected),
          "Replay uses actual input and finishes both reads of one atomic LDD before yielding");
  const auto boundary = machine.core.progress().estimated_cycles;
  require(machine.epoch.attained(boundary + 1) && machine.epoch.checkpoint(boundary) &&
              machine.history.retire(boundary),
          "The checkpoint advances beyond the real FIFO consumption");
  require(machine.core.interrupt_ready(cpu12::InterruptKind::maskable),
          "The arriving IRQ becomes eligible at the first completed instruction boundary");
  const auto interrupt = machine.core.interrupt({.vector = vector});
  require(interrupt.accepted && interrupt.fault == cpu12::Fault::none &&
              machine.core.registers().pc == handler,
          "Admit the actual prepared IRQ before executing another foreground instruction");
  machine.pending_irq = false;
  const auto returned = machine.core.run(1);
  require(returned.retired == 1 && returned.fault == cpu12::Fault::none,
          "The real native RTI restores the interrupted continuation");
  machine.core.close_return_window();
  const auto resumed = machine.core.progress().estimated_cycles;
  require(machine.epoch.attained(resumed + 1) && machine.epoch.checkpoint(resumed) &&
              machine.history.retire(resumed),
          "Retain the already accepted IRQ and its real stack effects in the next checkpoint");
  const auto committed = machine.core.save();
  const auto committed_ram = machine.ram;
  require(machine.core.run(3).retired == 3 && machine.ram[ram_first + 1] == first_byte,
          "The next native instructions use the consumed byte without consuming another");
  machine.peer.dsp_data({.mask = byte_mask, .known = byte_mask, .value = corrected});
  const Epoch::Read later{.ticks = machine.last_read,
                          .value = corrected,
                          .mask = byte_mask,
                          .signal = Epoch::Signal::data};
  require(machine.history.write(later) && !machine.epoch.changed(later) &&
              machine.epoch.cancelled(),
          "A later actual DSP change invalidates only work after the consumption checkpoint");
  machine.core.restore(committed);
  machine.ram = committed_ram;
  require(machine.epoch.rollback() && machine.core.run(3).retired == 3 && machine.popped == 1 &&
              machine.ram[ram_first + 1] == first_byte && machine.ram[ram_first + 2] == corrected,
          "Restoring the later checkpoint never crosses or duplicates the destructive read");
  for (unsigned index = 1; index < 16; ++index) {
    const auto sample = machine.peer.read_dsp_data(
        {.mask = byte_mask, .ticks = machine.core.progress().estimated_cycles});
    require(sample && sample->value == first_byte + index,
            "The actual shared reply cursor retains every remaining byte exactly once");
  }
  require(machine.epoch.census().destructive == 1 && machine.epoch.census().rollbacks == 2 &&
              machine.epoch.census().cause == Epoch::Signal::data &&
              machine.epoch.census().failed[static_cast<std::size_t>(Epoch::Signal::data)] == 2 &&
              machine.peer.error() == nullptr,
          "The frontier census distinguishes actual consumption, failed validations and replay");
}

void genuine_fault(const cpu12::NativeImage &image, const bool conflict) {
  constexpr std::uint64_t source_tick = 10;
  constexpr std::uint64_t observation_tick = 20;
  Machine machine(image);
  if (conflict) {
    require(
        machine.epoch.record({.ticks = observation_tick, .value = stale, .mask = byte_mask}) &&
            !machine.epoch.changed({.ticks = source_tick, .value = corrected, .mask = byte_mask}),
        "An invalid epoch alone is not evidence that a particular bus failure was cancelled");
  }
  machine.core.set_registers({.pc = unknown, .sp = stack});
  const auto failed = machine.core.run(1);
  require(failed.fault == cpu12::Fault::memory_read && failed.address == invalid &&
              machine.epoch.cancelled() == conflict &&
              machine.epoch.refusal() == Epoch::Refusal::device &&
              machine.epoch.error() == Epoch::Error::none && machine.unknown_reads == 1,
          "A genuine unknown access has no cancellation result and remains a real Core fault");
  require(machine.core.run(1).fault == failed.fault && machine.unknown_reads == 1,
          "Unexplained failures are never retried or converted to speculative cancellation");
}

void pressure_restart() {
  std::array<Epoch::Read, 8> storage{};
  Epoch speculative(storage, 2);
  require(speculative.record({.ticks = 10, .value = 0, .mask = 1}) && !speculative.restart(4) &&
              speculative.cancelled() && speculative.conflict() == 4 && !speculative.refuse() &&
              speculative.refusal() == Epoch::Refusal::epoch_invalidated,
          "Input pressure cancels an ordinary epoch before any destructive access");
  require(speculative.rollback() && !speculative.cancelled() &&
              speculative.census().cause == Epoch::Signal::count &&
              speculative.census().pressure == 1 && speculative.census().rollbacks == 1,
          "A pressure restart retains its independent census and resumes from the checkpoint");
  Epoch atomic(storage, 2);
  require(atomic.attained(4) && atomic.consume(3, [] { return true; }) && !atomic.restart(4) &&
              !atomic.cancelled() && atomic.error() == Epoch::Error::phase,
          "Input pressure cannot discard a consumed device transaction");
}

void held_reset_boundary() {
  std::array<Epoch::Read, 8> storage{};
  Epoch empty(storage, 2);
  require(
      empty.attained(10) && empty.checkpoint_quiescent(10),
      "An externally proved quiescent source commits an empty read-set at its attained instant");
  require(empty.record({.ticks = 10, .value = 0, .mask = 1}) && empty.attained(10) &&
              !empty.changed({.ticks = 10, .value = 1, .mask = 1}) && empty.cancelled(),
          "Quiescent checkpoint does not invent knowledge about new inputs at the same instant");
  Epoch pending(storage, 2);
  require(pending.record({.ticks = 10, .value = 0, .mask = 1}) &&
              !pending.checkpoint_quiescent(10) && pending.error() == Epoch::Error::unsettled,
          "Quiescence cannot discard an outstanding speculative read");
  Epoch atomic(storage, 2);
  require(atomic.attained(10) && atomic.consume(9, [] { return true; }) &&
              !atomic.checkpoint_quiescent(10) && atomic.error() == Epoch::Error::unsettled,
          "Quiescence cannot replace confirmation after a destructive native instruction");
  Epoch changed(storage, 2);
  require(changed.attained(10) && changed.changed({.ticks = 11}) && !changed.attained(10) &&
              changed.error() == Epoch::Error::time,
          "An unchanged frontier cannot precede a source event");
}

void promoted_read_set() {
  std::array<Epoch::Read, 8> storage{};
  Epoch epoch(storage, 2);
  require(epoch.record({.ticks = 3, .value = 1, .mask = 1}) &&
              epoch.record({.ticks = 7, .value = 1, .mask = 1}) && epoch.attained(5) &&
              epoch.promote(4) && !epoch.changed({.ticks = 6, .value = 0, .mask = 1}) &&
              epoch.conflict() == 6,
          "A promoted boundary retains the speculative read set after it");
  require(epoch.rollback() && epoch.record({.ticks = 4, .value = 1, .mask = 1}) &&
              !epoch.record({.ticks = 3, .value = 1, .mask = 1}) &&
              epoch.error() == Epoch::Error::time,
          "Replay starts at the promoted boundary");
  Epoch unsettled(storage, 2);
  require(unsettled.attained(4) && !unsettled.promote(4) &&
              unsettled.error() == Epoch::Error::unsettled,
          "The receiver frontier must extend past the promoted boundary");
}

void bounded_read_set() {
  std::array<Epoch::Read, 8> storage{};
  Epoch epoch(storage, 2);
  for (std::uint64_t tick = 0; tick < 6; ++tick) {
    require(epoch.record({.ticks = tick, .value = 1, .mask = 1}), "Record actual input use");
  }
  require(epoch.yield() && epoch.attained(4) && !epoch.yield(),
          "Reserve enough slots to finish an instruction before producer backpressure");
  require(epoch.changed({.ticks = 4, .value = 3, .mask = 2}) && epoch.attained(6),
          "Changes outside the observed mask do not invalidate a retained read");
  for (std::uint64_t tick = 6; tick < 14; ++tick) {
    require(epoch.record({.ticks = tick, .value = 1, .mask = 1}),
            "Validated slots are reused in the bounded ring");
  }
  require(!epoch.record({.ticks = 14, .value = 1, .mask = 1}) &&
              epoch.error() == Epoch::Error::exhausted && !epoch.cancelled(),
          "Ignoring producer backpressure fails explicitly without dropping a read");
  std::array<Epoch::Read, 8> other{};
  Epoch unsettled(other, 2);
  unsigned calls = 0;
  require(unsettled.attained(1) && !unsettled.settled(1),
          "The attained frontier is exclusive at the destructive read boundary");
  const bool consumed = unsettled.consume(1, [&] {
    ++calls;
    return true;
  });
  require(!consumed && calls == 0 && !unsettled.cancelled() &&
              unsettled.error() == Epoch::Error::unsettled,
          "A destructive read cannot touch its device before attained DSP time");
  Epoch advanced(other, 2);
  require(advanced.attained(1) && !advanced.settled(1),
          "The receiver waits until the validated epoch advances");
  const bool allowed = advanced.attained(2) && advanced.settled(1) && advanced.consume(1, [&] {
    ++calls;
    return true;
  });
  require(allowed && calls == 1,
          "Destructive read admission uses the same validated frontier as consumption");
}
}

int main() try {
  const std::array variants{cpu12::CodeVariant{.id = variant, .code = spans, .entries = entries}};
  const std::array layouts{cpu12::VariantLayout{.id = variant, .memory = layout}};
  const std::array interrupts{cpu12::InterruptRequest{.vector = vector}};
  const auto image = cpu12::Core::compile({.variants = variants},
                                          {.memory = layouts,
                                           .interrupts = interrupts,
                                           .transitions = {},
                                           .callbacks = {},
                                           .bindings = {}});
  native_cancellation_and_commit(image);
  genuine_fault(image, false);
  genuine_fault(image, true);
  promoted_read_set();
  bounded_read_set();
  held_reset_boundary();
  pressure_restart();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
