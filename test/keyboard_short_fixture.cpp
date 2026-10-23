// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard_short_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <optional>
#include <span>
#include <string>

namespace blofeld::test::keyboard_short {
unsigned Fixture::read(const unsigned address) {
  return machine_.bus().read({.space = DspSpace::x, .address = address});
}

void Fixture::write(const unsigned address, const unsigned value) {
  machine_.bus().write({.space = DspSpace::x, .address = address, .value = value});
}

void Fixture::install(const Firmware &firmware) {
  unsigned programs = 0;
  for (const auto &record : firmware.dsp) {
    if (record.space == 0) {
      if (record.address != parameter_program && record.address != message_program) { continue; }
      ++programs;
    } else if (record.address + record.words.size() > (record.space == 1 ? x_limit : y_limit)) {
      continue;
    }
    for (unsigned i = 0; i < record.words.size(); ++i) {
      machine_.bus().write({.space = static_cast<DspSpace>(record.space),
                            .address = record.address + i,
                            .value = record.words.at(i)});
    }
  }
  require(programs == 2, "Both complete unchanged original parser program records are required");
}

Fixture::Fixture(const Firmware &firmware,
                 const unsigned route,
                 const std::optional<unsigned> missing)
    : route_(route), missing_(missing) {
  install(firmware);
  for (const auto ring : rings) {
    for (unsigned i = 0; i < ring.size; ++i) { write(ring.base + i, sentinel); }
    write(ring.producer, ring.base + ring.size - 2);
    write(ring.consumer, ring.base + ring.size - 2);
  }
  write(flags, 0);
  write(context + 4, sentinel);
  machine_.bus().write({.space = DspSpace::y, .address = other_context + 4, .value = sentinel});
  write(report, sentinel);
  write(buffer, sentinel);
  for (unsigned i = 0; i < 3; ++i) {
    if (missing != i) { write(buffer + 1 + i, i == 0 ? sentinel : 0); }
  }
  DspProgram code;
  for (unsigned i = 0; i < 8; ++i) { code.immediate("m" + std::to_string(i), word_mask); }
  code.immediate("sr", masked_status);
  const auto loop = code.pc();
  code.emit("move x:>" + number(selector) + ",r6");
  code.emit("jsr (r6)");
  complete_ = code.pc();
  code.emit("nop");
  code.word(absolute_jump);
  code.word(loop);
  code.install(machine_.bus());
  while (machine_.cpu().state().pc != loop) { machine_.cpu().step(); }
}

void Fixture::values(const unsigned u, const unsigned v) {
  u_ = u;
  v_ = v;
  if (missing_ != 1) { write(buffer + 2, u); }
  if (missing_ != 2) { write(buffer + 3, v); }
}

void Fixture::enqueue(const std::span<const unsigned> bytes) {
  const auto ring = rings.at(route_);
  auto cursor = read(ring.producer);
  require(cursor == read(ring.consumer) && bytes.size() < ring.size,
          "Own bounded producer requires a drained selected input ring");
  for (const auto byte : bytes) {
    write(cursor, byte << byte_shift);
    cursor = ring.base + ((cursor - ring.base + 1) % ring.size);
  }
  write(ring.producer, cursor);
}

void Fixture::observe(const DspNativeAccess &access) {
  if (access.stage != DspNativeAccessStage::after) { return; }
  const auto address = access.target.address;
  constexpr unsigned peripheral_base = 0xffff80;
  require(access.target.space == DspSpace::p || address < peripheral_base,
          "Original short-message rejection must not access any peripheral");
  if (access.kind == DspNativeAccessKind::read && access.target.space == DspSpace::x &&
      address > buffer && address <= buffer + 3) {
    const std::array expected{sentinel, u_, v_};
    require(events_.reads < expected.size() && address == buffer + 1 + events_.reads &&
                access.value == expected.at(events_.reads),
            "Original universal dispatcher reads all three stale words in order");
    ++events_.reads;
  }
  const bool bit_write = access.kind == DspNativeAccessKind::bit && access.bit &&
                         access.bit->operation != DspBitOperation::test;
  if (access.kind != DspNativeAccessKind::write && !bit_write) { return; }
  const bool allowed = access.target.space == DspSpace::x
                           ? address == flags || address == rings.at(route_).consumer ||
                                 address == buffer || address == kind || address == count ||
                                 (address >= context && address < context + 5)
                           : access.target.space == DspSpace::y && route_ == 1 &&
                                 address >= other_context && address < other_context + 5;
  require(allowed,
          "Unexpected original short-message write at " + hex(address, 6) +
              " pc=" + hex(access.pc, 6));
}

void Fixture::run() {
  for (unsigned steps = 0; steps < step_limit; ++steps) {
    const auto cpu = machine_.cpu().state();
    if (missing_ && !saved_ && cpu.pc == stale_pc.at(*missing_)) {
      saved_.emplace(Saved{.owner = machine_.checkpoint(), .events = events_});
    }
    if (cpu.pc == universal) {
      require(((cpu.b >> word_bits) & word_mask) == manufacturer_word && cpu.r[0] == buffer + 1,
              "Rejected manufacturer byte enters the actual universal-identity path");
      ++events_.entries;
    }
    if (cpu.pc == dispatch_return) {
      require(events_.reads == 3 && ((cpu.b >> word_bits) & word_mask) == result() &&
                  (cpu.b & word_mask) == 0 && (cpu.a & word_mask) == 0 &&
                  result() != identity_word && (cpu.sr & zero) == 0 && cpu.r[0] == buffer + 4 &&
                  cpu.r[1] == report && cpu.sp == 2,
              "Independent middle-word addition and real return frame reject identity dispatch");
      ++events_.returns;
    }
    machine_.cpu().step({.observe = [this](const auto &access) { observe(access); }});
    if (machine_.cpu().state().pc == complete_) { return; }
  }
  throw Fault("Original short-message parser exceeded its native instruction bound");
}

void Fixture::invoke(const unsigned entry) {
  write(selector, entry);
  events_ = {};
  run();
}

void Fixture::check_context(const bool completed) {
  const std::array active{single_byte,
                          completed ? idle : append,
                          (completed ? raw_end : raw_start) << byte_shift,
                          manufacturer_word,
                          sentinel};
  const std::array untouched{rings[0].entry, idle, 0U, 0U, sentinel};
  for (unsigned i = 0; i < active.size(); ++i) {
    const auto x = read(context + i);
    const auto y = machine_.bus().read({.space = DspSpace::y, .address = other_context + i});
    const bool swapped = route_ == 1 && completed;
    require(x == (swapped ? untouched.at(i) : active.at(i)) &&
                y == (swapped ? active.at(i) : untouched.at(i)),
            "Parser contexts preserve the real active/saved secondary swap state");
  }
}

void Fixture::verify(const unsigned remaining_flags) {
  const auto ring = rings.at(route_);
  require(read(ring.producer) == read(ring.consumer) && read(flags) == remaining_flags &&
              read(kind) == raw_start << byte_shift && read(count) == 1 &&
              read(buffer) == manufacturer_word && read(buffer + 1) == sentinel &&
              read(buffer + 2) == u_ && read(buffer + 3) == v_ && read(report) == sentinel &&
              machine_.cpu().state().sp == 0 && events_.entries == 1 && events_.returns == 1 &&
              events_.reads == 3,
          "Complete original rejection consumes the message and retains old tails/report state");
  check_context(true);
}

void Fixture::blocked() {
  write(flags, bulk);
  invoke(rings.at(route_).entry);
  if (route_ == 0) {
    verify(bulk);
    return;
  }
  const auto ring = rings.at(route_);
  for (unsigned poll = 0; poll < 3; ++poll) {
    if (poll != 0) { invoke(rings[0].entry); }
    require(read(flags) == (bulk | secondary) && read(ring.consumer) == ring.base &&
                read(ring.producer) == ring.base + 1 &&
                read(read(ring.consumer)) == raw_end << byte_shift && read(count) == 1 &&
                events_.entries == 0 && events_.returns == 0 && events_.reads == 0 &&
                machine_.cpu().state().sp == 0,
            "Secondary bulk F7 remains queued with its active swapped parser and no dispatch");
    check_context(false);
  }
  saved_.emplace(Saved{.owner = machine_.checkpoint(), .events = events_});
}

void Fixture::resume_blocked() {
  if (!saved_ || route_ != 1) { throw Fault("A healthy deferred secondary state is required"); }
  const auto saved = saved_.value();
  std::optional<DspDebugState> expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine_.restore(saved.owner);
    write(flags, secondary); // Explicit caller clears only its declared bulk-output condition.
    invoke(rings[0].entry);  // The retained bit4 selects secondary service without swapping again.
    verify();
    if (expected) { require(machine_.cpu().debug_state() == *expected, "Deferred suffix differs"); }
    expected = machine_.cpu().debug_state();
  }
}

void Fixture::recover_missing() {
  if (!missing_) { throw Fault("A directed unknown stale word is required"); }
  const auto missing = missing_.value();
  const auto error = rejects([&] { invoke(rings.at(route_).entry); });
  if (!saved_) { throw Fault("No healthy checkpoint precedes the missing stale word"); }
  require(events_.reads == missing &&
              error.contains("Unknown DSP memory read at X:" + hex(buffer + 1 + missing, 6)) &&
              error.contains("pc=" + hex(stale_pc.at(missing), 6)) && error.contains("omr="),
          "Original stale overread fails at the independent address/PC without invented padding");
  const auto failed = machine_.cpu().debug_state();
  require(rejects([&] { machine_.cpu().step(); }).contains("requires reset") &&
              rejects([&] {
                static_cast<void>(machine_.checkpoint());
              }).contains("completed reference step") &&
              machine_.cpu().debug_state() == failed,
          "The quarantined owner rejects retry/capture without changing native state");
  const std::array known{sentinel, u_, v_};
  const auto saved = saved_.value();
  std::optional<DspDebugState> expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine_.restore(saved.owner);
    events_ = saved.events;
    write(buffer + 1 + missing, known.at(missing));
    run();
    verify();
    if (expected) { require(machine_.cpu().debug_state() == *expected, "Known suffix differs"); }
    expected = machine_.cpu().debug_state();
  }
}

void Fixture::intact(const Firmware &firmware) {
  for (const auto &record : firmware.dsp) {
    if (record.space != 0 ||
        (record.address != parameter_program && record.address != message_program)) {
      continue;
    }
    for (unsigned i = 0; i < record.words.size(); ++i) {
      require(machine_.bus().inspect_program(record.address + i) == record.words.at(i),
              "Both entire original parser program records remain unchanged");
    }
  }
}
} // namespace blofeld::test::keyboard_short
