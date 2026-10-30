// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <vector>

namespace {
using namespace blofeld;
using test::require;
constexpr unsigned entry = 0x100, data_a = 0x400, data_b = 0x500, saved_lc = 0x33;
constexpr unsigned canary = 0x654321, padding = 24, register_bits = 24;
constexpr unsigned extal = 12000000, timebase = 540000000, period_ticks = 45;
constexpr unsigned ordinary_omr = 0x300, flags = 0xc00301;
// FM13-160/161 and Table12-13: positive REP immediate/X0/A1.
constexpr unsigned rep_immediate = 0x0600a0, rep_x0 = 0x06c420, rep_a1 = 0x06cc20;
// FM MOVE encodings: X0 -> X:(R0)+; A1 -> Y:(R0)+; B -> Y:(Rn)+.
constexpr unsigned x0_store = 0x445800, a1_store = 0x5c5800, b_store = 0x5f5800;
constexpr unsigned y_bit = 0x80000, register_shift = 8, illegal = 7;
constexpr unsigned write_x0 = 0x20a400, write_a1 = 0x20ac00, immediate_r0 = 0x60f400;
constexpr unsigned jump = 0x0af080, byte_mask = 0xff;
constexpr unsigned curve_count = 77;
enum class Pair : std::uint8_t { nops, raw, curves };
enum class Counter : std::uint8_t { immediate, x0, a1 };
enum class Gap : std::uint8_t { none, nop, pointer, three_nops, a1, x0 };
enum class Role : std::uint8_t { nop, prefix, first, second, gap };

struct Input {
  unsigned first = 1, second = 1;
  Pair pair = Pair::nops;
  Counter counter = Counter::immediate;
  Gap gap = Gap::none;
  bool mutate = false;
};

struct Event {
  Role role;
  unsigned address, decode, words = 1, count = 0, iteration = 0, wait = 0;

  [[nodiscard]] unsigned e1() const { return decode + 3 + wait; }

  [[nodiscard]] unsigned e2() const { return e1() + 1; }
};

struct Plan {
  Input input;
  std::vector<unsigned> code;
  std::vector<Event> events;
  unsigned first_body = 0, second_prefix = 0, second_body = 0, last = 0;
};

unsigned gaps(const Gap gap) {
  if (gap == Gap::none) { return 0; }
  if (gap == Gap::pointer) { return 2; }
  return gap == Gap::three_nops ? 3U : 1U;
}

unsigned encoded_count(const unsigned count) {
  return rep_immediate | ((count & byte_mask) << register_shift) | (count >> register_shift);
}

unsigned first_value(const Input sample) {
  if (sample.pair != Pair::curves) { return sample.second + (sample.gap == Gap::x0 ? 2U : 0U); }
  if (sample.counter != Counter::a1) { return test::word_mask; }
  return sample.second + (sample.gap == Gap::a1 ? 2U : 0U);
}

unsigned first_opcode(const Pair pair) {
  if (pair == Pair::nops) { return 0; }
  return pair == Pair::raw ? x0_store : a1_store;
}

unsigned gap_opcode(const Gap gap) {
  if (gap == Gap::a1) { return write_a1; }
  return gap == Gap::x0 ? write_x0 : 0U;
}

unsigned second_prefix(const Input sample) {
  if (sample.counter == Counter::immediate) { return encoded_count(sample.second); }
  return sample.counter == Counter::x0 ? rep_x0 : rep_a1;
}

unsigned second_opcode(const Input sample) {
  if (sample.pair == Pair::nops) { return 0; }
  const unsigned reg = sample.gap == Gap::pointer ? 0U : 1U;
  const unsigned opcode = sample.pair == Pair::raw ? (x0_store | y_bit) : b_store;
  return opcode | (reg << register_shift);
}

Plan plan(const Input sample) {
  Plan result{.input = sample, .code = {}, .events = {}};
  result.code = {0, encoded_count(sample.first), first_opcode(sample.pair)};
  result.first_body = entry + 2;
  result.events = {
      {.role = Role::nop, .address = entry, .decode = 3},
      {.role = Role::prefix, .address = entry + 1, .decode = 4, .count = sample.first}};
  for (unsigned index = 0; index < sample.first; ++index) {
    result.events.push_back({.role = Role::first,
                             .address = result.first_body,
                             .decode = 9 + index,
                             .count = sample.first,
                             .iteration = index});
  }
  const unsigned gap_decode = 9 + sample.first;
  if (sample.gap == Gap::pointer) {
    result.events.push_back(
        {.role = Role::gap, .address = entry + 3, .decode = gap_decode, .words = 2});
    result.code.insert(result.code.end(), {immediate_r0, data_b});
  } else {
    for (unsigned index = 0; index < gaps(sample.gap); ++index) {
      result.events.push_back(
          {.role = Role::gap, .address = entry + 3 + index, .decode = gap_decode + index});
      result.code.push_back(gap_opcode(sample.gap));
    }
  }
  result.second_prefix = entry + static_cast<unsigned>(result.code.size());
  result.second_body = result.second_prefix + 1;
  const unsigned prefix = gap_decode + gaps(sample.gap);
  const unsigned wait = sample.gap == Gap::a1 && sample.counter == Counter::a1 ? 1U : 0U;
  result.code.insert(result.code.end(), {second_prefix(sample), second_opcode(sample), 0});
  result.events.push_back({.role = Role::prefix,
                           .address = result.second_prefix,
                           .decode = prefix,
                           .count = sample.second,
                           .wait = wait});
  for (unsigned index = 0; index < sample.second; ++index) {
    result.events.push_back({.role = Role::second,
                             .address = result.second_body,
                             .decode = prefix + 5 + wait + index,
                             .count = sample.second,
                             .iteration = index});
  }
  result.events.push_back({.role = Role::nop,
                           .address = result.second_body + 1,
                           .decode = prefix + 5 + wait + sample.second});
  const auto tail = result.events.back();
  result.last = tail.e2();
  // Actual padding NOPs can enter Decode while the selected tail drains.
  // Declare each such event; none can retire inside this bounded window.
  for (unsigned clock = tail.decode + 1; clock <= result.last; ++clock) {
    result.events.push_back(
        {.role = Role::nop, .address = tail.address + clock - tail.decode, .decode = clock});
  }
  return result;
}

DspDebugState prepare(DspMachine &machine, const Plan &layout) {
  const auto sample = layout.input;
  test::DspProgram seed;
  seed.immediate("omr", ordinary_omr);
  seed.immediate("lc", saved_lc);
  seed.immediate("m0", test::word_mask);
  seed.immediate("m1", test::word_mask);
  seed.immediate("r0", data_a);
  seed.immediate("r1", data_b);
  seed.immediate("x0", sample.second + (sample.gap == Gap::x0 ? 2U : 0U));
  seed.immediate("x1", sample.second);
  seed.immediate("a0", 0);
  seed.immediate("a1",
                 sample.pair == Pair::curves ? first_value(sample)
                                             : sample.second + (sample.gap == Gap::a1 ? 2U : 0U));
  seed.immediate("a2", 0);
  for (const auto *const name : {"b0", "b1", "b2"}) { seed.immediate(name, 0); }
  seed.immediate("sr", flags);
  seed.word(jump);
  seed.word(entry);
  seed.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  test::checkpoint::until(machine, entry);
  for (unsigned index = 0; index < layout.code.size() + padding; ++index) {
    machine.bus().write({.address = entry + index,
                         .value = index < layout.code.size() ? layout.code.at(index) : 0U});
  }
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned base : {data_a, data_b}) {
      for (unsigned index = 0; index < std::max(sample.first, sample.second) + 2; ++index) {
        machine.bus().write({.space = space, .address = base + index, .value = canary});
      }
    }
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.sr == flags && before.cpu.lc == saved_lc &&
              before.cpu.r[0] == data_a && before.cpu.r[1] == data_b && before.cpu.b == 0 &&
              before.cpu.m[0] == test::word_mask && before.cpu.m[1] == test::word_mask,
          "Own native seeds establish literal PC/SR/LC/pointers/B before automatic execution");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

void early(DspState &state, const Plan &layout, const Event event, const unsigned clock) {
  const bool body = event.role == Role::first || event.role == Role::second;
  if (body && layout.input.pair != Pair::nops && clock >= event.decode + 2) {
    const unsigned reg = event.role == Role::first || layout.input.gap == Gap::pointer ? 0U : 1U;
    ++state.r.at(reg);
  }
  if (event.role != Role::gap || clock < event.e1()) { return; }
  if (layout.input.gap == Gap::pointer) { state.r[0] = data_b; }
  if (layout.input.gap == Gap::x0) {
    state.x = (state.x & ~std::uint64_t{test::word_mask}) | layout.input.second;
  }
  if (layout.input.gap == Gap::a1) {
    constexpr std::uint64_t middle_mask = std::uint64_t{test::word_mask} << register_bits;
    state.a = (state.a & ~middle_mask) | (std::uint64_t{layout.input.second} << register_bits);
  }
}

void retire(DspDebugState &state, const Event event) {
  ++state.native_instructions;
  state.native_cycles += event.role == Role::prefix ? 5U : event.words;
  state.cpu.pc = event.address + event.words;
  if (event.role == Role::prefix) { state.cpu.lc = event.count; }
  if (event.role == Role::first || event.role == Role::second) {
    const unsigned remaining = event.count - event.iteration - 1;
    state.cpu.pc = remaining == 0 ? event.address + 1 : event.address;
    state.cpu.lc = remaining == 0 ? saved_lc : remaining;
  }
}

DspDebugState expected(const DspDebugState &before, const Plan &layout, const unsigned clock) {
  auto result = before;
  for (const auto event : layout.events) {
    early(result.cpu, layout, event, clock);
    if (clock >= event.e2()) { retire(result, event); }
  }
  return result;
}

unsigned expected_cell(const Plan &layout,
                       const unsigned clock,
                       const DspSpace space,
                       const unsigned base,
                       const unsigned index) {
  unsigned wanted = canary;
  for (const auto event : layout.events) {
    if (clock < event.e1() || layout.input.pair == Pair::nops) { continue; }
    const bool first = event.role == Role::first;
    if (!first && event.role != Role::second) { continue; }
    const auto target = first && layout.input.pair == Pair::raw ? DspSpace::x : DspSpace::y;
    if (target != space || base != (first ? data_a : data_b) || index != event.iteration) {
      continue;
    }
    if (first) {
      wanted = first_value(layout.input);
    } else {
      wanted = layout.input.pair == Pair::curves ? 0U : layout.input.second;
    }
  }
  return wanted;
}

void memory(DspMachine &machine, const Plan &layout, const unsigned clock) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const unsigned base : {data_a, data_b}) {
      for (unsigned index = 0; index < std::max(layout.input.first, layout.input.second) + 2;
           ++index) {
        const auto wanted = expected_cell(layout, clock, space, base, index);
        require(machine.bus().read({.space = space, .address = base + index}) == wanted,
                "Every RAM cell and opposite-space canary follows only its independently scheduled "
                "E1 store");
      }
    }
  }
}

struct Fetches {
  std::array<unsigned, 2> f1{}, f2{};
};

void check_extensions(const DspPipelineCycle &phase, const Plan &layout, const unsigned clock) {
  std::optional<DspProgramWord> decoded;
  std::optional<DspProgramWord> retired;
  for (const auto event : layout.events) {
    if (event.words != 2) { continue; }
    const DspProgramWord extension{.address = event.address + 1, .value = data_b};
    if (clock == event.decode + 1) { decoded = extension; }
    if (clock == event.decode + 5) { retired = extension; }
  }
  require(phase.extension_decode == decoded && phase.extension_execute2 == retired,
          "FM Table2-1 separates actual extension Decode/E2 from native opcode E1/E2");
}

void check_state(const DspMachine &machine,
                 const DspDebugState &before,
                 const Plan &layout,
                 const unsigned clock) {
  const auto wanted = expected(before, layout, clock);
  if (machine.cpu().debug_state() != wanted) {
    std::cerr << "clock=" << clock << " pair=" << static_cast<unsigned>(layout.input.pair)
              << " gap=" << static_cast<unsigned>(layout.input.gap)
              << " counter=" << static_cast<unsigned>(layout.input.counter) << '\n';
  }
  if (machine.cpu().debug_state() != wanted) {
    const auto actual = machine.cpu().debug_state();
    std::cerr << "actual/wanted pc=" << actual.cpu.pc << '/' << wanted.cpu.pc
              << " lc=" << actual.cpu.lc << '/' << wanted.cpu.lc << " r0=" << actual.cpu.r[0] << '/'
              << wanted.cpu.r[0] << " r1=" << actual.cpu.r[1] << '/' << wanted.cpu.r[1]
              << " x=" << actual.cpu.x << '/' << wanted.cpu.x << " a=" << actual.cpu.a << '/'
              << wanted.cpu.a << " sr=" << actual.cpu.sr << '/' << wanted.cpu.sr
              << " instructions=" << actual.native_instructions << '/' << wanted.native_instructions
              << " cycles=" << actual.native_cycles << '/' << wanted.native_cycles << '\n';
  }
  require(machine.cpu().debug_state() == wanted,
          "Complete PC/LC/register/flags/stack/counter state matches each independent period");
}

void advance(DspFetchRun &run,
             DspMachine &machine,
             const Plan &layout,
             const DspDebugState &before,
             Fetches *fetches = nullptr) {
  const auto prior = machine.reference_fetch();
  const auto cycle =
      run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
  if (!cycle.pipeline) { throw Fault("Actual native pipeline observation is required"); }
  if (fetches != nullptr) {
    const std::array targets{layout.first_body, layout.second_body};
    for (unsigned index = 0; index < targets.size(); ++index) {
      if (cycle.issued_address == targets.at(index)) { ++fetches->f1.at(index); }
      if (cycle.bus.fetch && prior.pending_address == targets.at(index)) {
        ++fetches->f2.at(index);
      }
    }
  }
  const auto &phase = *cycle.pipeline;
  const auto clock = static_cast<unsigned>(cycle.completed_cycles);
  const bool decoded = std::ranges::any_of(
      layout.events, [clock](const Event event) { return event.decode == clock; });
  const bool retired = std::ranges::any_of(
      layout.events, [clock](const Event event) { return event.e2() == clock; });
  if (phase.decode_accepted != decoded || phase.retirement_published != retired) {
    std::cerr << "timeline clock=" << clock << " wanted-decode=" << decoded
              << " actual-decode=" << phase.decode_accepted << " wanted-retire=" << retired
              << " actual-retire=" << phase.retirement_published << " first=" << layout.input.first
              << " second=" << layout.input.second
              << " pair=" << static_cast<unsigned>(layout.input.pair)
              << " gap=" << static_cast<unsigned>(layout.input.gap)
              << " counter=" << static_cast<unsigned>(layout.input.counter) << '\n';
  }
  require(phase.decode_accepted == decoded && phase.retirement_published == retired,
          "No unscheduled Decode or native retirement escapes the independent timeline");
  for (const auto event : layout.events) {
    if (event.decode == clock) {
      require(phase.decode_accepted && phase.decode && phase.decode->address == event.address,
              "Nominal REP5/body1 and real gap widths determine every accepted Decode");
    }
    if (event.e2() == clock) {
      require(phase.retirement_published && phase.execute2 &&
                  phase.execute2->address == event.address,
              "Each genuine native E2 retains its independently specified identity and period");
    }
  }
  check_extensions(phase, layout, clock);
  check_state(machine, before, layout, clock);
  memory(machine, layout, clock);
  require(run.elapsed_ticks() == cycle.completed_cycles * period_ticks,
          "No guessed handoff stall changes the clock ledger");
  if (layout.input.mutate && cycle.bus.fetch && prior.pending_address &&
      (*prior.pending_address == layout.first_body ||
       *prior.pending_address == layout.second_body)) {
    machine.bus().write({.address = *prior.pending_address, .value = illegal});
  }
}

unsigned streams = 0, restores = 0, clones = 0;

void finish(DspFetchRun &run,
            DspMachine &machine,
            const Plan &layout,
            const DspDebugState &before) {
  while (run.completed_cycles() < layout.last) { advance(run, machine, layout, before); }
}

void one(const Input sample, const bool snapshots = false) {
  const auto layout = plan(sample);
  DspMachine machine;
  const auto before = prepare(machine, layout);
  DspFetchRun run({.extal_hz = extal,
                   .timebase_hz = timebase,
                   .timing = DspFetchTiming::reference_cycle_order,
                   .execution = DspReferenceExecution::staged_register_boundaries,
                   .pipeline = DspFetchPipeline::automatic_memory},
                  machine);
  std::vector<DspFetchCheckpoint> cuts;
  Fetches fetches;
  const unsigned last_decode = 8 + sample.first;
  while (run.completed_cycles() < layout.last) {
    advance(run, machine, layout, before, &fetches);
    const auto clock = run.completed_cycles();
    if (snapshots && clock >= last_decode && clock <= last_decode + 8) {
      cuts.push_back(run.checkpoint(machine));
    }
  }
  require(fetches.f1 == std::array{1U, 1U} && fetches.f2 == std::array{1U, 1U},
          "Each native REP body owns exactly one real F1 and F2 across all iterations");
  for (const auto &cut : cuts) {
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(cut, machine);
      finish(run, machine, layout, before);
      ++restores;
    }
    DspMachine target;
    DspFetchRun clone(cut, target);
    finish(clone, target, layout, before);
    require(target.cpu().debug_state() == machine.cpu().debug_state() &&
                clone.elapsed_ticks() == run.elapsed_ticks(),
            "Fresh native ownership preserves the independently verified handoff state and clock");
    ++clones;
  }
  ++streams;
}
}

int main() {
  try {
    for (const auto pair : {Pair::nops, Pair::raw, Pair::curves}) {
      for (const auto counter : {Counter::immediate, Counter::x0, Counter::a1}) {
        for (const auto gap :
             {Gap::none, Gap::nop, Gap::pointer, Gap::three_nops, Gap::a1, Gap::x0}) {
          for (const unsigned first : {1U, 3U}) {
            for (const unsigned second : {1U, 3U}) {
              for (const bool mutate : {false, true}) {
                one({.first = first,
                     .second = second,
                     .pair = pair,
                     .counter = counter,
                     .gap = gap,
                     .mutate = mutate},
                    first == 3 && second == 3 && mutate);
              }
            }
          }
        }
      }
    }
    one({.first = curve_count,
         .second = curve_count,
         .pair = Pair::curves,
         .gap = Gap::pointer,
         .mutate = true},
        true);
    std::cout << "PASS REP_HANDOFF streams=" << streams << " restores=" << restores
              << " clones=" << clones << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
