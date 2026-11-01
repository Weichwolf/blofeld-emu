// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {
using blofeld::DspBus;
using blofeld::DspSpace;
using blofeld::Fault;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

struct Layout {
  unsigned omr;
  unsigned program_end;
  unsigned x_end;
  unsigned y_end;
};

// Independent literal expectations from DSP56371UM Rev. 2.1 tables 3-1..3-6.
constexpr std::array<Layout, 5> layouts{{
    {.omr = 0x000005, .program_end = 0x1000, .x_end = 0x12000, .y_end = 0x16000},
    {.omr = 0x600085, .program_end = 0x3000, .x_end = 0x10000, .y_end = 0x16000},
    {.omr = 0x400085, .program_end = 0x5000, .x_end = 0x10000, .y_end = 0x14000},
    {.omr = 0x200085, .program_end = 0x9000, .x_end = 0x10000, .y_end = 0x10000},
    {.omr = 0x000085, .program_end = 0xb000, .x_end = 0x10000, .y_end = 0xe000},
}};
constexpr unsigned cookie = 0x123456;
constexpr unsigned changed = 0xabcdef;
constexpr unsigned x_moved = 0x10000;
constexpr unsigned x_moved_end = 0x12000;
constexpr unsigned p_moved = 0x1000;
constexpr unsigned p_y_begin = 0x3000;
constexpr unsigned boot = 0xff0000;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned upper_data_begin = 0xc000;
constexpr unsigned default_y_end = 0x16000;
constexpr unsigned missing_rom = 0x4001;
constexpr unsigned core_priority = 0x300;
constexpr unsigned jump_absolute = 0x0af080;

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected a visible unsupported DSP memory access");
}

void select(DspBus &bus, const Layout layout) {
  bus.observe_omr(layout.omr);
  bus.advance_instruction_cycles(3);
}

void rejected_word(DspBus &bus,
                   const DspSpace space,
                   const unsigned address,
                   const std::string &reason) {
  require(rejects([&] {
            static_cast<void>(bus.read({.space = space, .address = address}));
          }).contains(reason) &&
              rejects([&] {
                bus.write({.space = space, .address = address, .value = cookie});
              }).contains(reason) &&
              rejects([&] {
                static_cast<void>(
                    bus.memory_resource({.target = {.space = space, .address = address}}));
              }).contains(reason),
          "Data, store and resource admission must retain the same strict map boundary");
}

void data_windows(DspBus &bus, const DspSpace space, const unsigned end) {
  const unsigned low_end = space == DspSpace::x ? 0x3000 : 0x2000;
  for (const auto address : {0U, low_end - 1, 0xc000U, end - 1}) {
    require(rejects([&] {
              static_cast<void>(bus.read({.space = space, .address = address}));
            }).contains("Unknown DSP memory read"),
            "Cold RAM stays unknown in every documented window");
    bus.write({.space = space, .address = address, .value = cookie ^ address});
    require(bus.read({.space = space, .address = address}) == (cookie ^ address),
            "Literal low and high RAM endpoints store independent full words");
  }
  for (const auto address : {low_end, 0x3fffU, end, 0xfeffffU}) {
    rejected_word(bus, space, address, "Unmapped DSP memory");
  }
  const std::array<std::uint32_t, 1> own_rom{changed};
  for (const auto address : {0x4000U, 0xbfffU}) {
    bus.install_rom({.space = space, .start = address, .words = own_rom});
    require(bus.read({.space = space, .address = address}) == changed &&
                rejects([&] {
                  bus.write({.space = space, .address = address});
                }).contains("Protected DSP ROM write"),
            "X/Y ROM windows remain independent of P expansion and write protected");
  }
  require(rejects([&] {
            static_cast<void>(bus.read({.space = space, .address = missing_rom}));
          }).contains("Unknown DSP memory read"),
          "An installed ROM endpoint never supplies adjacent missing contents");
}

void literal_windows() {
  for (const auto layout : layouts) {
    DspBus bus;
    bus.write({.space = DspSpace::x, .address = x_moved, .value = cookie});
    select(bus, layout);
    data_windows(bus, DspSpace::x, layout.x_end);
    data_windows(bus, DspSpace::y, layout.y_end);
    bus.write({.address = p_moved - 1, .value = changed});
    require(bus.read({.address = p_moved - 1}) == changed,
            "The original four lower P blocks remain available in every layout");
    if (layout.program_end == p_moved) {
      rejected_word(bus, DspSpace::p, p_moved, "Unmapped DSP memory");
    } else {
      require(bus.read({.address = p_moved}) == cookie &&
                  bus.memory_resource({.target = {.address = p_moved}}) ==
                      blofeld::DspMemoryResource{
                          .bank = DspSpace::x, .word = x_moved, .block_words = 8192},
              "All switched layouts retain the existing explicitly inferred X-to-P alias");
      rejected_word(bus, DspSpace::x, x_moved, "Unmapped DSP memory");
      bus.write({.address = p_y_begin - 1, .value = changed});
      if (layout.program_end > p_y_begin) {
        rejected_word(bus, DspSpace::p, p_y_begin, "Y-to-P physical relocation");
        rejected_word(bus, DspSpace::p, layout.program_end - 1, "Y-to-P physical relocation");
      }
      bus.reset();
      require(bus.read({.space = DspSpace::x, .address = x_moved_end - 1}) == changed,
              "A P write updates the retained physical X block, including after reset");
      select(bus, layout);
    }
    rejected_word(bus, DspSpace::p, layout.program_end, "Unmapped DSP memory");
    rejected_word(bus, DspSpace::p, boot - 1, "Unmapped DSP memory");
  }
}

bool between(const unsigned address, const unsigned first, const unsigned second) {
  return (address >= first && address < second) || (address >= second && address < first);
}

void transition_data(DspBus &bus, const Layout before, const Layout after) {
  constexpr std::array probes{0xdfffU, 0xe000U, 0xffffU, 0x10000U, 0x13fffU, 0x14000U, 0x15fffU};
  for (const auto address : probes) {
    const auto access = [&] { return bus.read({.space = DspSpace::y, .address = address}); };
    if (between(address, before.y_end, after.y_end)) {
      require(rejects([&] { static_cast<void>(access()); }).contains("elapsed switch cycles"),
              "A changing Y window rejects throughout the pending switch");
    } else if (address < before.y_end) {
      require(access() == (cookie ^ address),
              "Unchanged Y RAM remains available during a different window's switch");
    } else {
      require(rejects([&] { static_cast<void>(access()); }).contains("Unmapped DSP memory"),
              "An unchanged reserved Y address remains unmapped during a switch");
    }
  }
}

void all_transitions() {
  for (const auto before : layouts) {
    for (const auto after : layouts) {
      DspBus bus;
      for (unsigned address = upper_data_begin; address < default_y_end; ++address) {
        bus.write({.space = DspSpace::y, .address = address, .value = cookie ^ address});
      }
      bus.write({.space = DspSpace::x, .address = x_moved, .value = changed});
      select(bus, before);
      static_cast<void>(bus.take_code_changes());
      bus.observe_omr(after.omr);
      transition_data(bus, before, after);
      if (before.omr != after.omr) {
        require(!bus.take_code_changes().all,
                "A request does not invalidate code before the switch commits");
        if (before.program_end != p_moved && after.program_end != p_moved) {
          require(bus.read({.address = p_moved}) == changed,
                  "The shared X-to-P block remains available between switched layouts");
        }
        bus.advance_instruction_cycles(1);
        const auto saved = bus;
        bus.advance_instruction_cycles(2);
        bus = saved;
        transition_data(bus, before, after);
        bus.advance_instruction_cycles(1);
        transition_data(bus, before, after);
        bus.advance_instruction_cycles(1);
        require(bus.take_code_changes().all,
                "Every effective layout change invalidates native P dispatch at its deadline");
      }
      for (const auto address : {0xc000U, after.y_end - 1}) {
        require(bus.read({.space = DspSpace::y, .address = address}) == (cookie ^ address),
                "Literal new Y endpoints retain their initialized physical contents");
      }
      rejected_word(bus, DspSpace::y, after.y_end, "Unmapped DSP memory");
      bus.reset();
      for (unsigned address = upper_data_begin; address < default_y_end; ++address) {
        require(bus.read({.space = DspSpace::y, .address = address}) == (cookie ^ address),
                "Hidden Y words are retained, never lost or fabricated by mode changes");
      }
    }
  }
}

void ignored_and_overlapping_requests() {
  DspBus bus;
  bus.write({.space = DspSpace::x, .address = x_moved, .value = cookie});
  for (const unsigned omr : {0x000005U, 0x200005U, 0x400005U, 0x600005U}) {
    bus.observe_omr(omr);
    require(bus.read({.space = DspSpace::x, .address = x_moved}) == cookie,
            "MSW bits do not reallocate RAM when MS is clear");
    rejected_word(bus, DspSpace::p, p_moved, "Unmapped DSP memory");
  }
  bus.observe_omr(layouts.at(1).omr);
  bus.advance_instruction_cycles(2);
  bus.observe_omr(layouts.at(1).omr | core_priority);
  require(rejects([&] { bus.observe_omr(layouts.at(2).omr); }).contains("overlapping"),
          "Changing the target of an unfinished hardware switch remains unqualified");
  require(rejects([&] { bus.observe_omr(word_mask + 1); }).contains("exceeds 24 bits"),
          "An invalid OMR cannot be silently truncated into a valid mode");
  bus.advance_instruction_cycles(1);
  require(
      bus.read({.address = p_moved}) == cookie,
      "Same-map observations and rejected requests do not restart or cancel the first deadline");
}

constexpr blofeld::DspRunClockInputs clocks{.extal_hz = 12000000,
                                            .timebase_hz = 540000000,
                                            .switch_timing =
                                                blofeld::DspClockSwitch::reference_cycle_boundary};
constexpr auto timing = blofeld::OwnDspAutomaticTiming::one_effect_per_cycle;
constexpr unsigned movec_omr = 0x05f43a, period_ticks = 45, switch_cycles = 3;

struct Observation {
  dsp56300::Registers registers;
  dsp56300::Progress progress;
  std::optional<dsp56300::InstructionEffect> pending;
  std::optional<unsigned> opcode;
  std::optional<std::uint64_t> dma_cycle;
  std::uint64_t ticks = 0, cycles = 0;
  bool faulted = false;
  bool operator==(const Observation &) const = default;
};

Observation observe(blofeld::OwnDspMachine &machine, const blofeld::OwnDspTimeline &timeline) {
  const auto pending = machine.cpu().pending_instruction();
  return {.registers = machine.cpu().registers(),
          .progress = machine.cpu().progress(),
          .pending = pending ? std::optional{pending->effect} : std::nullopt,
          .opcode = pending ? pending->opcode : std::nullopt,
          .dma_cycle = machine.bus().last_dma_cycle(),
          .ticks = machine.clock().elapsed_ticks(),
          .cycles = machine.clock().native_cycle_units(),
          .faulted = timeline.faulted()};
}

void advance(blofeld::OwnDspMachine &machine, blofeld::OwnDspTimeline &timeline) {
  const auto cycle = machine.bus().last_dma_cycle();
  const auto ticks = machine.clock().elapsed_ticks();
  const auto result =
      timeline.advance_one(timeline.next_deadline(),
                           blofeld::OwnDspBoundary::through,
                           {.cycle = cycle ? *cycle + 1 : 0,
                            .known_requests = 15,
                            .timing = blofeld::DspDmaTiming::reference_sampled_cycles,
                            .grant = blofeld::DspDmaGrant::available});
  require(result && (!result->native || result->native->fault.kind == dsp56300::FaultKind::none),
          "A literal MOVEC/NOP/JMP reference cycle executes without native substitution");
  require(machine.clock().elapsed_ticks() == ticks + period_ticks,
          "Every selected reference cycle consumes one actual bypass-EXTAL period");
}

void check_switch(DspBus &bus, const bool complete) {
  if (complete) {
    require(bus.read({.address = p_moved}) == cookie,
            "The exact third elapsed reference cycle exposes the retained X-to-P word");
  } else {
    require(rejects([&] {
              static_cast<void>(bus.read({.address = p_moved}));
            }).contains("elapsed switch cycles"),
            "A pending native OMR switch refuses the changing address until its deadline");
  }
}

std::vector<Observation> suffix(blofeld::OwnDspMachine &machine,
                                blofeld::OwnDspTimeline &timeline,
                                const Layout layout,
                                const unsigned remaining) {
  std::vector<Observation> result{observe(machine, timeline)};
  auto expected = machine.cpu().registers();
  expected.pc = p_y_begin;
  expected.omr = layout.omr;
  check_switch(machine.bus(), remaining == 0);
  for (unsigned cycle = 0; cycle < remaining; ++cycle) {
    advance(machine, timeline);
    check_switch(machine.bus(), cycle + 1 == remaining);
    result.push_back(observe(machine, timeline));
  }
  constexpr unsigned bound = 128;
  constexpr unsigned instructions = 5;
  std::optional<std::string> failure;
  for (unsigned cycle = 0; cycle < bound && !failure; ++cycle) {
    try {
      advance(machine, timeline);
    } catch (const Fault &error) { failure = error.what(); }
    result.push_back(observe(machine, timeline));
  }
  const std::string reason = layout.program_end == p_y_begin ? "Unmapped DSP memory read"
                                                             : "Y-to-P physical relocation read";
  require(failure && failure->contains(reason) && failure->contains("P:0x003000") &&
              failure->contains("pc=0x003000") &&
              failure->contains("omr=" + blofeld::hex(layout.omr, 6)) && timeline.faulted(),
          "The real refused program fetch retains operation, address, PC and native OMR");
  require(machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions == instructions,
          "MOVEC, three NOPs and JMP preserve every other register before the refused fetch");
  const auto stopped = observe(machine, timeline);
  static_cast<void>(rejects([&] { advance(machine, timeline); }));
  require(observe(machine, timeline) == stopped,
          "An unsupported native mapping preserves fail-stop execution without further effects");
  return result;
}

blofeld::OwnDspTimelineCheckpoint prepare(blofeld::OwnDspMachine &machine, const Layout layout) {
  // Independent FM MOVEC immediate OMR, NOP and JMP absolute instruction fields.
  const std::array<std::uint32_t, 10> program{
      movec_omr, layout.omr, 0, 0, 0, jump_absolute, p_y_begin, 0, 0, 0};
  machine.bus().install_rom({.start = boot, .words = program});
  machine.bus().write({.space = DspSpace::x, .address = x_moved, .value = cookie});
  constexpr blofeld::OwnDspResetInputs reset{
      .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2};
  blofeld::OwnDspTimeline timeline(machine, timing, 0, reset);
  constexpr unsigned hold_ticks = 90;
  constexpr unsigned recovery_end = 180;
  constexpr unsigned bound = 32;
  require(timeline.advance_reset_to(hold_ticks) == hold_ticks, "Real native reset hold");
  timeline.release_hardware_reset();
  require(timeline.advance_reset_to(recovery_end) == recovery_end - hold_ticks,
          "Real mode-five bypass-PLL reset recovery");
  for (unsigned cycle = 0; cycle < bound && machine.cpu().registers().omr != layout.omr; ++cycle) {
    advance(machine, timeline);
  }
  require(machine.cpu().registers().omr == layout.omr,
          "Actual native MOVEC publishes the independently authored memory mode");
  check_switch(machine.bus(), false);
  return timeline.checkpoint();
}

void native_modes_and_snapshots() {
  unsigned contexts = 0;
  for (const auto layout : layouts) {
    if (layout.program_end == p_moved) { continue; }
    blofeld::OwnDspMachine initial(clocks);
    const auto start = prepare(initial, layout);
    for (unsigned cut = 0; cut <= switch_cycles; ++cut) {
      blofeld::OwnDspMachine source(clocks);
      auto owner = std::make_unique<blofeld::OwnDspTimeline>(source, start);
      for (unsigned cycle = 0; cycle < cut; ++cycle) { advance(source, *owner); }
      const auto saved = owner->checkpoint();
      const auto before = observe(source, *owner);
      const auto deadline = owner->next_deadline();
      const auto old = std::move(owner);
      owner = std::make_unique<blofeld::OwnDspTimeline>(source, saved);
      require(!old->faulted(),
              "The previous timeline is healthy before its identity is invalidated");
      static_cast<void>(rejects([&] { static_cast<void>(old->next_deadline()); }));
      require(observe(source, *owner) == before && owner->next_deadline() == deadline,
              "Restoring invalidates a healthy old owner without changing the saved boundary");
      const auto expected = suffix(source, *owner, layout, switch_cycles - cut);
      owner = std::make_unique<blofeld::OwnDspTimeline>(source, saved);
      require(observe(source, *owner) == before && owner->next_deadline() == deadline,
              "Same-owner restore recovers the native/frontend clock and switch after failure");
      require(suffix(source, *owner, layout, switch_cycles - cut) == expected,
              "Same-owner continuation repeats every real native/map effect and visible refusal");
      blofeld::OwnDspMachine fresh(clocks);
      blofeld::OwnDspTimeline replay(fresh, saved);
      require(observe(fresh, replay) == before && replay.next_deadline() == deadline,
              "Fresh-owner restore retains full native registers, staged work, DMA and time");
      require(suffix(fresh, replay, layout, switch_cycles - cut) == expected,
              "Fresh-owner continuation repeats the exact native/map/failure trace");
      ++contexts;
    }
  }
  require(contexts == 16, "All four switched maps cover each pending and committed cycle cut");
  std::cout << "Native memory-map checkpoint states=" << contexts
            << " same/fresh continuations=" << contexts * 2 << '\n';
}

}

int main() {
  try {
    literal_windows();
    all_transitions();
    ignored_and_overlapping_requests();
    native_modes_and_snapshots();
    std::cout << "Five DSP56371 window layouts and 25 transitions passed; Y-to-P order remains "
                 "explicitly unqualified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
