// SPDX-License-Identifier: GPL-3.0-only
#include "original_dsp_gpio_request_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::gpio_request {
namespace {
using blofeld::test::require;
constexpr unsigned saved_request = 0x34;
constexpr unsigned deferred = 0x35;
constexpr unsigned saved_a = 0x36;
constexpr unsigned saved_r = 0x37;
constexpr unsigned saved_x = 0x38;
constexpr unsigned stamp = 0x1f;
constexpr unsigned pointer = 0x32;
constexpr unsigned reader = 0x33;
constexpr unsigned flags = 6;
constexpr unsigned x_seed = 0x123456;
constexpr unsigned r_seed = 0x321;
constexpr unsigned a_seed = 0x13579b;
constexpr unsigned status = 0xc00000;
constexpr unsigned memory_mode = 0x600085;
constexpr unsigned priority = 0x4006;
constexpr unsigned iprc = 0xffffff;
constexpr unsigned alias_offset = 0xf000;
constexpr unsigned byte_request = 0xf4;
constexpr unsigned high_bit = 0x80;
constexpr unsigned deferred_bit = 0x800000;
constexpr unsigned step_limit = 256;
constexpr unsigned poll_delay = 2;
constexpr unsigned hold_polls = 4;

void put(DspBus &bus, const unsigned address, const unsigned value) {
  bus.write({.space = DspSpace::x, .address = address, .value = value});
}

unsigned get(DspBus &bus, const unsigned address) {
  return bus.read({.space = DspSpace::x, .address = address});
}

void install(DspBus &bus, const Firmware &firmware) {
  unsigned loaded = 0;
  for (const auto &record : firmware.dsp) {
    if (record.space != 0 || (record.address != vector && record.address != entry)) { continue; }
    ++loaded;
    for (unsigned index = 0; index < record.words.size(); ++index) {
      const auto address = record.address + index;
      const bool alias = address >= 0x1000;
      bus.write({.space = alias ? DspSpace::x : DspSpace::p,
                 .address = alias ? address + alias_offset : address,
                 .value = record.words[index]});
    }
  }
  require(loaded == 2, "Install both complete original GPIO/vector records in source order");
}
}

Rig::Rig(const Firmware &firmware, Case input) : input_(input) {
  prepare(firmware);
  initial_ = machine_.cpu().state();
  machine_.bus().gpio(DspPort::c).set_drive({.mask = byte_mask, .levels = byte_request});
  machine_.bus().observe_irqa(false);
  while (machine_.cpu().state().pc != low_wait) { step(); }
  require(stats_.input_reads == 1 && machine_.cpu().state().sp == 1,
          "Actual F4 input reaches the first original poll with its genuine interrupt frame");
}

void Rig::prepare(const Firmware &firmware) {
  auto &bus = machine_.bus();
  install(bus, firmware);
  for (unsigned index = 0; index < ring_words; ++index) { put(bus, ring + index, canary ^ index); }
  for (unsigned index = 0; index < 16; ++index) {
    if (input_.failure == Failure::index_unknown && index == ((input_.first >> 2U) & 15U)) {
      continue;
    }
    put(bus, table + index, table_base + (index * table_stride));
  }
  put(bus, pointer, ring);
  put(bus, reader, ring + (ring_words / 2));
  put(bus, stamp, a_seed);
  put(bus, flags, flags_seed);
  put(bus, saved_request, request_seed);
  put(bus, deferred, deferred_seed);
  for (const auto address : {saved_a, saved_r, saved_x}) { put(bus, address, canary); }
  if (input_.failure != Failure::word_unknown && input_.kind != Kind::deferred) {
    put(bus, input_.address, input_.value);
  }
  bus.observe_irqa(true);
  DspProgram program;
  program.immediate("omr", memory_mode);
  for (unsigned index = 0; index < 3; ++index) { program.emit("nop"); }
  const auto mapped = program.pc();
  program.immediate("sr", status);
  program.immediate("a0", 0);
  program.immediate("a1", a_seed);
  program.immediate("a2", 0);
  program.immediate("x0", x_seed);
  program.immediate("r6", r_seed);
  for (unsigned index = 0; index < 8; ++index) {
    program.immediate("m" + std::to_string(index), word_mask);
  }
  program.emit("movep #>$f00,x:<<$ffffbe");
  program.emit("movep #>" + number(input_.upper) + ",x:<<$ffffbd");
  program.emit("movep #>$ff,x:<<$ffffbf");
  program.emit("movep #>" + number(priority) + ",x:<<$ffffff");
  foreground_ = program.pc();
  program.emit("nop");
  program.install(bus);
  for (unsigned steps = 0; machine_.cpu().state().pc != foreground_; ++steps) {
    require(steps < step_limit, "Own native GPIO caller must reach its declared foreground");
    if (machine_.cpu().state().pc == mapped) { bus.advance_instruction_cycles(3); }
    machine_.cpu().step();
  }
  require(get(bus, iprc) == priority && !bus.interrupt_request(0),
          "Actual native IPRC4006 setup does not fabricate any reset DMA event");
}

void Rig::observe(const DspNativeAccess &access) {
  stats_.accesses.push_back({.kind = access.kind,
                             .stage = access.stage,
                             .space = access.target.space,
                             .address = access.target.address,
                             .pc = access.pc,
                             .value = access.value,
                             .bit = access.bit});
  if (access.stage != DspNativeAccessStage::after) { return; }
  if (!access.value.has_value()) { throw Fault("Completed native access has no actual value"); }
  const auto address = access.target.address;
  const auto value = *access.value;
  if (access.kind == DspNativeAccessKind::write) {
    require(access.target.space == DspSpace::x, "Original F4 writes use X memory space");
    observe_write(address, value);
  }
  if (access.kind == DspNativeAccessKind::read && access.target.space == DspSpace::x) {
    observe_read(access, value);
  }
  if (access.kind == DspNativeAccessKind::bit) { observe_bit(access, value); }
}

void Rig::observe_write(const unsigned address, const unsigned value) {
  const bool gpio = address == control || address == direction || address == data_port;
  const bool scratch = address >= saved_request && address <= saved_x;
  require(gpio || scratch,
          "Original F4 paths write only their actual saved context/request and GPIO registers");
  if (!gpio) { return; }
  stats_.gpio.push_back({address, value});
  if (address == control && value == 0) { release_due_ = true; }
  if (address == data_port) { stats_.reply = value; }
}

void Rig::observe_read(const DspNativeAccess &access, const unsigned value) {
  const auto address = access.target.address;
  if (address == data_port) {
    const std::array expected{input_.upper | byte_request,
                              input_.upper | input_.first,
                              input_.upper | high_bit | input_.second};
    require(stats_.input_reads < expected.size() && value == expected.at(stats_.input_reads),
            "Every actual whole-port input equals the independently supplied peer phase");
    ++stats_.input_reads;
  }
  if (address == input_.address) {
    require(value == input_.value, "Selected native source read returns the own input");
    ++stats_.source_reads;
  }
  if (address >= table && address < table + 16) { ++stats_.table_reads; }
}

void Rig::observe_bit(const DspNativeAccess &access, const unsigned value) {
  if (access.target.space != DspSpace::x || access.target.address != data_port ||
      !access.bit.has_value() || access.bit->index != 7 ||
      access.bit->operation != DspBitOperation::test) {
    return;
  }
  if (access.pc == low_wait) {
    require(value == (stats_.low < poll_delay || input_.stop == Stop::low ? 1U : 0U),
            "The actual first handshake poll tests the declared high-to-low input");
    ++stats_.low;
  } else if (access.pc == high_wait) {
    require(value == (stats_.high < poll_delay || input_.stop == Stop::high ? 0U : 1U),
            "The actual second handshake poll tests the declared low-to-high input");
    ++stats_.high;
  }
}

void Rig::peer() {
  auto &port = machine_.bus().gpio(DspPort::c);
  const auto pc = machine_.cpu().state().pc;
  if (pc == low_wait && input_.failure == Failure::low_unknown) { port.set_drive({}); }
  if (pc == low_wait && stats_.low == poll_delay && input_.stop != Stop::low) {
    port.set_drive({.mask = byte_mask, .levels = input_.first});
  }
  if (pc == high_wait && stats_.high == poll_delay && input_.stop != Stop::high) {
    const bool missing = input_.failure == Failure::second_unknown;
    port.set_drive({.mask = missing ? high_bit : byte_mask,
                    .levels = missing ? high_bit : high_bit | input_.second});
  }
  if (release_due_) {
    if (input_.failure != Failure::contention) {
      port.set_drive({.floating = byte_mask});
      ++stats_.releases;
    }
    release_due_ = false;
  }
}

void Rig::step() {
  require(stats_.steps < step_limit, "Original F4 handshake must stay within its test budget");
  peer();
  const DspNativeAccessObserver observer{
      .observe = [this](const DspNativeAccess &access) { observe(access); }};
  machine_.cpu().step(observer);
  ++stats_.steps;
}

Saved Rig::save() const {
  return {.machine = machine_.checkpoint(), .stats = stats_, .release_due = release_due_};
}

void Rig::restore(const Saved &saved) {
  machine_.restore(saved.machine);
  stats_ = saved.stats;
  release_due_ = saved.release_due;
}

unsigned Rig::saved_word() const {
  if (input_.kind == Kind::multi || input_.kind == Kind::metadata) {
    return (input_.first & 15U) << 3U;
  }
  return input_.upper | input_.first;
}

unsigned Rig::expected_reply() const {
  return (saved_word() & upper_mask) | high_bit | (input_.value & byte_mask);
}

void Rig::check_guards() {
  auto &bus = machine_.bus();
  require(get(bus, pointer) == ring && get(bus, reader) == ring + (ring_words / 2) &&
              get(bus, stamp) == a_seed && get(bus, flags) == flags_seed,
          "F4 service does not publish a byte-ring word, timestamp or unrelated busy flag");
  for (unsigned index = 0; index < ring_words; ++index) {
    require(get(bus, ring + index) == (canary ^ index), "Every unrelated input ring cell survives");
  }
  require(get(bus, saved_a) == a_seed && get(bus, saved_r) == r_seed && get(bus, saved_x) == x_seed,
          "Actual original context stores retain the independently supplied caller registers");
}

void Rig::check_reply() {
  require(machine_.cpu().state() == initial_ && stats_.low == poll_delay + 1 &&
              stats_.high == poll_delay + 1 && stats_.input_reads == 3,
          "The complete native F4 path and RTI restore every foreground register after both waits");
  auto &bus = machine_.bus();
  require(get(bus, saved_request) == saved_word(), "Original request normalization is retained");
  const auto pads = bus.gpio(DspPort::c).outputs();
  const bool deferred_request = input_.kind == Kind::deferred;
  const auto expected_tag =
      deferred_request ? deferred_bit | input_.upper | high_bit | input_.second : deferred_seed;
  require(get(bus, deferred) == expected_tag && get(bus, control) == 0 &&
              get(bus, direction) == port_mask && stats_.releases == 1,
          "Original response mode, peer release and actual deferred tag retain their roles");
  if (deferred_request) {
    require(!stats_.reply && stats_.source_reads == 0 && pads.mask == port_mask &&
                pads.known == upper_mask && pads.levels == input_.upper,
            "Deferred publication cannot qualify an unknown future output latch as a reply");
  } else {
    const std::vector<std::array<unsigned, 2>> expected{
        {control, 0}, {direction, port_mask}, {data_port, expected_reply()}};
    require(stats_.gpio == expected && stats_.reply == expected_reply() &&
                stats_.source_reads == 1 && pads.mask == port_mask && pads.known == port_mask &&
                pads.levels == expected_reply() &&
                stats_.table_reads == (input_.kind == Kind::part ? 1U : 0U),
            "Actual source load, INSERT and GPIO stores drive only the independent reply byte");
  }
  check_guards();
}

void Rig::cleanup() {
  const auto before = machine_.cpu().state();
  machine_.bus().observe_irqa(true);
  machine_.bus().observe_irqa(false);
  do { step(); } while (machine_.cpu().state().pc != foreground_);
  require(machine_.cpu().state() == before && get(machine_.bus(), control) == byte_mask &&
              get(machine_.bus(), direction) == upper_mask && !machine_.bus().interrupt_request(0),
          "A subsequent real output-mode IRQ restores input mode without a host register repair");
}

void Rig::check_failure(const std::string &message, const DspState before) {
  require(input_.failure != Failure::none && message.contains("pc=" + hex(before.pc, 6)),
          "Only the declared original access with actual native PC can be an expected failure");
  const bool source = input_.failure == Failure::word_unknown;
  if (source) {
    require(before.pc == source_read, "Unknown selected data fails at the original word load");
  }
  const bool index = input_.failure == Failure::index_unknown;
  auto address = data_port;
  if (source) { address = input_.address; }
  if (index) { address = table + ((input_.first >> 2U) & 15U); }
  require(message.contains("X:" + hex(address, 6)) && !stats_.reply,
          "A missing source/pin or contention must fail before any fabricated response");
  require(message.contains(input_.failure == Failure::contention ? "contention" : "Unknown"),
          "The expected failure cannot hide another model or test invariant violation");
  const auto retained = outcome();
  bool stopped = false;
  try {
    machine_.cpu().step();
  } catch (const Fault &) { stopped = true; }
  require(stopped && outcome() == retained, "A failed original request cannot implicitly resume");
  bool rejected = false;
  try {
    static_cast<void>(machine_.checkpoint());
  } catch (const Fault &) { rejected = true; }
  require(rejected, "A failed F4 request requires coordinated owner restore before capture");
  check_guards();
}

std::optional<Saved> Rig::turnaround() {
  if (input_.stop != Stop::reply || input_.failure != Failure::none) { return std::nullopt; }
  while (!release_due_) { step(); }
  require(get(machine_.bus(), control) == 0 && get(machine_.bus(), direction) == upper_mask &&
              stats_.input_reads == 3 && stats_.releases == 0 && stats_.source_reads == 0,
          "Capture the actual disconnected-to-output transition before external peer release");
  return save();
}

void Rig::finish() {
  for (;;) {
    const auto before = machine_.cpu().state();
    if ((input_.stop == Stop::low && before.pc == low_wait && stats_.low == hold_polls) ||
        (input_.stop == Stop::high && before.pc == high_wait && stats_.high == hold_polls)) {
      require(!stats_.reply && stats_.source_reads == 0 && machine_.cpu().state().sp == 1,
              "Unreleased original waits retain their real frame without inventing a timeout");
      check_guards();
      return;
    }
    if (before.pc == foreground_) {
      require(input_.failure == Failure::none, "A declared unknown access cannot silently return");
      check_reply();
      cleanup();
      return;
    }
    try {
      step();
    } catch (const Fault &fault) {
      if (input_.failure == Failure::none) { throw; }
      check_failure(fault.what(), before);
      return;
    }
  }
}

Outcome Rig::outcome() {
  auto &bus = machine_.bus();
  const auto pads = bus.gpio(DspPort::c).outputs();
  return {.cpu = machine_.cpu().debug_state(),
          .state = {get(bus, pointer),
                    get(bus, reader),
                    get(bus, stamp),
                    get(bus, saved_request),
                    get(bus, deferred),
                    get(bus, saved_a),
                    get(bus, saved_r),
                    get(bus, saved_x)},
          .pads = {pads.mask, pads.known, pads.levels, pads.peripheral},
          .stats = stats_};
}
}
