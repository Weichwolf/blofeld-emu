// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_workspace_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <ios>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace blofeld::test::workspace {
namespace {
constexpr std::array<std::array<unsigned, 3>, 4> ranges{
    {{1, 0, 0x3000}, {2, 0, 0x2000}, {1, 0xc000, 0x10000}, {2, 0xc000, 0x16000}}};
constexpr unsigned ram_words = 77824;
constexpr unsigned prefetch_guard = 0x2468ac;
constexpr unsigned initial_value = 0x234567;
constexpr unsigned other_x = 0x468ace;
constexpr unsigned other_y = 0x3579bd;
constexpr unsigned sound = 0x936;
constexpr unsigned scratch = 0x700;
constexpr unsigned snapshot_fallback_case = 324;
constexpr unsigned snapshot_eviction_case = 326;

bool output(const unsigned at) {
  if (at >= active && at <= active + voices) { return true; }
  if (at >= occupancy && at < owners + owner_count) { return true; }
  return at >= voice && at < voice + (voices * stride) && (at - voice) % stride == callback_offset;
}
}

void Fixture::prepare(const Case &input) {
  for (unsigned at = active; at <= active + voices; ++at) { put(at, 0); }
  put(active + voices + 1, prefetch_guard);
  for (unsigned at = occupancy; at < owners + owner_count; ++at) { put(at, 0); }
  for (unsigned i = 0; i < voices; ++i) {
    const auto base = voice + (i * stride);
    put(base + cursor_offset, 0);
    put(base + callback_offset, idle + 1);
    put(base + value_offset, initial_value);
  }
  for (const auto &[at, value] : input.writes) { put(at, value); }
  put(target_slot, input.entry);
  put(mode_slot, input.mode);
  arguments_ = {0,
                0,
                0,
                0,
                input.request,
                0,
                input.x0,
                prefetch_guard,
                other_y,
                sound,
                mask,
                other_x,
                scratch,
                input.selected};
  for (unsigned i = 0; i < arguments_.size(); ++i) { put(argument_slot + i, arguments_[i]); }
}

void Fixture::state(const char *label) const {
  const auto s = machine_.cpu().state();
  trace_ << label << std::hex << ' ' << s.pc << ' ' << s.sp << ' ' << s.stack_pc << ' ' << s.a
         << ' ' << s.b << ' ' << s.x << ' ' << s.y << ' ' << s.sr;
  for (const auto v : s.r) { trace_ << ' ' << v; }
  for (const auto v : s.n) { trace_ << ' ' << v; }
  for (const auto v : s.m) { trace_ << ' ' << v; }
  trace_ << ' ' << s.lc << ' ' << s.la << '\n';
}

void Fixture::step() {
  const auto s = machine_.cpu().state();
  require(steps_ < instruction_bound, "Workspace instruction budget exhausted");
  ++steps_;
  require((s.pc >= boot && s.pc < returned_ + 3) || (s.pc >= allocate && s.pc < module_end),
          "Execution outside original workspace/caller");
  if (s.pc < boot) { visited_.insert(s.pc); }
  if (!quiet_ && (s.pc == shift_pc || s.pc == fallback_pc)) {
    state(s.pc == shift_pc ? "SHIFT" : "RESET");
  }
  machine_.cpu().step();
  maximum_sp_ = std::max(maximum_sp_, machine_.cpu().state().sp);
}

void Fixture::until(const unsigned pc) {
  while (machine_.cpu().state().pc != pc) { step(); }
}

std::vector<std::uint32_t> Fixture::ram() {
  std::vector<std::uint32_t> result;
  result.reserve(ram_words);
  for (const auto &[space, first, end] : ranges) {
    for (unsigned at = first; at < end; ++at) {
      result.push_back(machine_.bus().read({.space = static_cast<DspSpace>(space), .address = at}));
    }
  }
  require(result.size() == ram_words, "Incomplete full mapped-RAM guard");
  return result;
}

void Fixture::compare_ram(const std::vector<std::uint32_t> &before,
                          const std::vector<std::uint32_t> &after) {
  require(before.size() == ram_words && after.size() == ram_words, "Truncated workspace RAM guard");
  std::size_t i = 0;
  for (const auto &[space, first, end] : ranges) {
    for (unsigned at = first; at < end; ++at, ++i) {
      if (space == 1 && output(at)) { continue; }
      if (before.at(i) != after.at(i)) {
        throw Fault("Workspace changed guarded RAM space=" + std::to_string(space) +
                    " address=" + hex(at, 6));
      }
    }
  }
}

void Fixture::verify(const Case &input) const {
  const auto &bus = machine_.bus();
  for (const auto &[pc, value] : source_.program) {
    require(bus.peek_program(pc) == value, "Original workspace program changed");
  }
  const auto s = machine_.cpu().state();
  require(s.pc == returned_ && s.sp == 0 && (s.sr & mode_mask) == input.mode,
          "Workspace real RTS/frame/mode differs");
  constexpr unsigned nominal_ticks = 45;
  const auto units = machine_.run_clock().native_cycle_units();
  require(units == machine_.cpu().debug_state().native_cycles &&
              machine_.run_clock().elapsed_ticks() == units * nominal_ticks,
          "Owned nominal workspace clock differs from completed native execution");
}

void Fixture::replay(const DspCheckpoint &checkpoint,
                     const DspDebugState &expected,
                     const std::vector<std::uint32_t> &memory,
                     const unsigned prefix_steps) {
  quiet_ = true;
  for (unsigned trial = 0; trial < 2; ++trial) {
    machine_.restore(checkpoint);
    steps_ = prefix_steps;
    until(returned_);
    require(machine_.cpu().debug_state() == expected && ram() == memory,
            "Joint workspace snapshot changed full CPU/stack or mapped RAM");
    constexpr unsigned nominal_ticks = 45;
    require(machine_.run_clock().native_cycle_units() == expected.native_cycles &&
                machine_.run_clock().elapsed_ticks() == expected.native_cycles * nominal_ticks,
            "Restoring workspace state changed its nominal clock prefix");
    ++replays_;
  }
  quiet_ = false;
}

void Fixture::run(const Case &input) {
  require(input.id == calls_, "Workspace case order changed");
  prepare(input);
  until(input.entry);
  steps_ = 0;
  maximum_sp_ = machine_.cpu().state().sp;
  const auto before = ram();
  trace_ << "CASE " << std::hex << input.id << '\n';
  state("ENTRY");
  std::optional<DspCheckpoint> saved;
  unsigned prefix_steps = 0;
  while (machine_.cpu().state().pc != returned_) {
    const auto pc = machine_.cpu().state().pc;
    if (!saved && ((input.id == snapshot_fallback_case && pc == fallback_pc) ||
                   (input.id == snapshot_eviction_case && pc == shift_pc))) {
      saved = machine_.checkpoint();
      prefix_steps = steps_;
      ++snapshots_;
    }
    step();
  }
  state("END");
  const auto after = ram();
  compare_ram(before, after);
  verify(input);
  trace_ << "MEMORY" << std::hex;
  for (unsigned at = active; at <= active + voices; ++at) { trace_ << ' ' << get(at); }
  for (unsigned at = occupancy; at < owners + owner_count; ++at) { trace_ << ' ' << get(at); }
  for (unsigned i = 0; i < voices; ++i) {
    trace_ << ' ' << get(voice + (i * stride) + callback_offset);
  }
  trace_ << "\nCOUNTS " << std::dec << steps_ << ' ' << maximum_sp_ << '\n';
  if (saved) {
    trace_ << "REPLAY " << std::hex << input.id << ' '
           << (input.id == snapshot_fallback_case ? fallback_pc : shift_pc) << '\n';
    replay(*saved, machine_.cpu().debug_state(), after, prefix_steps);
    verify(input);
  }
  ++calls_;
}
}
