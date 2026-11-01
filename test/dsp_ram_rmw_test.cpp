// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace {
using namespace blofeld;
using test::esai::require;
namespace native = test::dma_test;
constexpr unsigned operand = 6, destination = 0x900, pattern = 0x654321;
constexpr unsigned canary = 0xa55aa5, word_mask = 0xffffff, limit = 64, offsets = 24;
constexpr unsigned move_r0 = 0x60f400, move_m0 = 0x05f420;
constexpr unsigned dsr = 0xffffef, ddr = 0xffffee, dco = 0xffffed, dcr = 0xffffec;
constexpr auto timing =
    OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts;
enum class Action : std::uint8_t { clear, set, change, test };
enum class Form : std::uint8_t { short_address, indirect, increment, absolute };

struct Sample {
  Action action = Action::set;
  Form form = Form::short_address;
  DspSpace space = DspSpace::x;
  unsigned bit = 13;
  bool level = false;
};

unsigned initial(const Sample sample) {
  const unsigned mask = 1U << sample.bit;
  return (pattern & ~mask) | (sample.level ? mask : 0U);
}

unsigned modified(const Sample sample) {
  const auto value = initial(sample);
  const unsigned mask = 1U << sample.bit;
  switch (sample.action) {
    case Action::clear: return value & ~mask;
    case Action::set: return value | mask;
    case Action::change: return value ^ mask;
    case Action::test: return value;
    default: throw Fault("Invalid authored bit-operation case");
  }
}

native::NativeProgram program(const Sample sample) {
  native::NativeProgram result;
  result.word(move_r0);
  result.word(operand);
  result.word(move_m0);
  result.word(word_mask);
  // DSP56300FM BCLR/BSET/BCHG/BTST: k/t select the operation,
  // S selects X/Y; AA6 and MMMRRR select the independent operand forms.
  constexpr std::array actions{0U, 0x20U, 0x10000U, 0x10020U};
  constexpr std::array forms{operand << 8U, 0x6000U, 0x5800U, 0x7000U};
  const unsigned opcode = 0x0a0000 | actions.at(static_cast<unsigned>(sample.action)) |
                          forms.at(static_cast<unsigned>(sample.form)) |
                          (sample.space == DspSpace::y ? 0x40U : 0U) | sample.bit;
  result.word(opcode);
  if (sample.form == Form::absolute) { result.word(operand); }
  for (unsigned index = 0; index < limit; ++index) { result.word(0); }
  return result;
}

void prepare(OwnDspMachine &machine, const Sample sample) {
  native::reset_native(machine, program(sample));
  native::step_native(machine);
  native::step_native(machine);
  require(machine.cpu().registers().r[0] == operand && machine.cpu().registers().m[0] == word_mask,
          "Real setup instructions establish the address and linear modifier");
  machine.bus().write({.space = sample.space, .address = operand, .value = initial(sample)});
  machine.bus().write({.space = DspSpace::x, .address = destination, .value = canary});
  machine.bus().write({.space = DspSpace::x, .address = dsr, .value = operand});
  machine.bus().write({.space = DspSpace::x, .address = ddr, .value = destination});
  machine.bus().write({.space = DspSpace::x, .address = dco});
  constexpr unsigned copy_x = 0x880240;
  machine.bus().write({.space = DspSpace::x,
                       .address = dcr,
                       .value = copy_x | (sample.space == DspSpace::y ? 1U : 0U)});
}

DspDmaInputs inputs(const unsigned cycle, const unsigned request) {
  return {.cycle = cycle,
          .known_requests = 15,
          .requests = cycle >= request ? 1U : 0U,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

struct Ledger {
  unsigned reads = 0, writes = 0, copies = 0;
  bool retired = false;
};

void observe(OwnDspMachine &machine,
             const std::optional<dsp56300::PendingInstruction> &pending,
             const OwnDspFetchCycleResult &event,
             const Sample sample,
             Ledger &ledger) {
  if (event.dma.word) {
    require(ledger.copies == 0 &&
                event.dma.word->value == (ledger.retired ? modified(sample) : initial(sample)),
            "DMA samples only the word before or after the complete native transaction");
    require(ledger.reads == 0 || ledger.retired,
            "No DMA word can complete between the native RAM sample and retirement");
    ++ledger.copies;
  }
  if (!event.native || event.native_pc != native::boot + 4) { return; }
  require(event.native->fault.kind == dsp56300::FaultKind::none,
          "The actual bit operation must execute without a fabricated result or native fault");
  // Automatic admission consumes captured F2 words and accepts the first effect
  // in the same period. Only (R0)+ starts with a separate AGU publication;
  // the other authored forms start directly with their single RAM read.
  if (!pending && event.instruction_started && sample.form != Form::increment) { ++ledger.reads; }
  if (pending && pending->effect.kind == dsp56300::EffectKind::read) { ++ledger.reads; }
  if (pending && pending->effect.kind == dsp56300::EffectKind::write) { ++ledger.writes; }
  if (event.native->retired) { ledger.retired = true; }
  require(machine.bus().read({.space = sample.space, .address = operand}) ==
              (ledger.writes != 0 ? modified(sample) : initial(sample)),
          "The RAM cell changes only at the actual native write publication");
}

void finish(OwnDspMachine &machine,
            OwnDspTimeline &timeline,
            const Sample sample,
            const unsigned request,
            unsigned cycle,
            Ledger ledger,
            const dsp56300::Registers &before) {
  for (; cycle < limit && (!ledger.retired || ledger.copies == 0); ++cycle) {
    const auto pending = machine.cpu().pending_instruction();
    const auto event = timeline.advance_one(
        timeline.next_deadline(), OwnDspBoundary::through, inputs(cycle, request));
    if (!event) { throw Fault("Every requested native/DMA period must complete"); }
    observe(machine, pending, *event, sample, ledger);
    if (event->native && event->native->retired && event->native_pc == native::boot + 4) {
      auto expected = before;
      expected.pc += sample.form == Form::absolute ? 2U : 1U;
      expected.sr = (expected.sr & ~1U) | (sample.level ? 1U : 0U);
      if (sample.form == Form::increment) { ++expected.r[0]; }
      require(machine.cpu().registers() == expected,
              "Independent bit truth, Carry, PC, AGU and all other registers match the ISA");
    }
  }
  require(ledger.retired && ledger.copies == 1 && ledger.reads == 1 &&
              ledger.writes == (sample.action == Action::test ? 0U : 1U) &&
              machine.bus().read({.space = sample.space, .address = operand}) == modified(sample) &&
              !timeline.faulted() && !machine.io().failure(),
          "One genuine RAM sample, optional store, retirement and preserved DMA request complete");
}

void overlap(const Sample sample, const unsigned request) {
  OwnDspMachine machine(native::native_clocks);
  prepare(machine, sample);
  const auto before = machine.cpu().registers();
  OwnDspTimeline timeline(machine, timing, 0);
  finish(machine, timeline, sample, request, 0, {}, before);
}

unsigned replay() {
  const Sample sample{.action = Action::change, .form = Form::increment, .level = true};
  OwnDspMachine source(native::native_clocks);
  prepare(source, sample);
  const auto before = source.cpu().registers();
  OwnDspTimeline timeline(source, timing, 0);
  Ledger ledger;
  unsigned cuts = 0;
  constexpr unsigned request = 4;
  for (unsigned cycle = 0; cycle < limit && !ledger.retired; ++cycle) {
    const auto saved = timeline.checkpoint();
    OwnDspMachine clone(native::native_clocks);
    OwnDspTimeline restored(clone, saved);
    finish(clone, restored, sample, request, cycle, ledger, before);
    ++cuts;
    const auto pending = source.cpu().pending_instruction();
    const auto event = timeline.advance_one(
        timeline.next_deadline(), OwnDspBoundary::through, inputs(cycle, request));
    if (!event) { throw Fault("The source checkpoint continuation must advance"); }
    observe(source, pending, *event, sample, ledger);
  }
  require(ledger.retired && cuts > 4,
          "Fresh-owner checkpoints include the RAM read, write, AGU and flag publications");
  return cuts;
}

void seek_read(OwnDspMachine &machine) {
  auto progress = machine.begin_fetch();
  for (unsigned index = 0; index < limit && progress.pending; ++index) {
    if (progress.pending->effect.kind == dsp56300::EffectKind::read) { return; }
    progress = machine.advance(progress.pending->ticket);
  }
  throw Fault("The native bit instruction did not request its RAM operand");
}

void direct_reservation(const bool drain) {
  const Sample sample{};
  OwnDspMachine machine(native::native_clocks);
  prepare(machine, sample);
  const auto before = machine.cpu().registers();
  require(!machine.bus().advance_dma(inputs(0, 1)), "DMA must sample a real low baseline");
  if (drain) {
    require(!machine.bus().advance_dma(inputs(1, 1)),
            "A real DMA source read precedes the indivisible CPU reservation");
  }
  seek_read(machine);
  unsigned cycle = drain ? 2U : 1U;
  for (unsigned hold = 0; hold < 3; ++hold, ++cycle) {
    auto input = inputs(cycle, 1);
    input.grant = DspDmaGrant::wait;
    const auto result =
        machine.advance_dma_native_cycle(input, OwnDspNativeCycleModel::isolated_effect);
    require(result.dma.phase && !result.dma.word && result.dma.core_stall == drain &&
                result.native.has_value() != drain,
            "A pending DMA write holds the CPU; a deferred DMA source does not");
    if (!drain) { break; }
    const auto pending = machine.cpu().pending_instruction();
    if (!pending) { throw Fault("A held native operand lost its actual pending read"); }
    require(machine.cpu().registers() == before &&
                pending->effect.kind == dsp56300::EffectKind::read,
            "External grant waits retain the exact pending RAM read and all CPU fields");
  }
  if (drain) {
    const auto result = machine.advance_dma_native_cycle(inputs(cycle++, 1),
                                                         OwnDspNativeCycleModel::isolated_effect);
    require(result.dma.word && result.dma.word->value == initial(sample) && result.dma.core_stall &&
                !result.native,
            "An already sampled DMA word completes before the CPU reads its operand");
  } else {
    ++cycle;
    for (unsigned hold = 0; hold < 3; ++hold, ++cycle) {
      const auto result = machine.advance_dma_reference_cycle(inputs(cycle, 1));
      require(result.phase && result.phase->kind == DspDmaPhaseKind::source_read && !result.word &&
                  !result.core_stall && machine.cpu().registers() == before,
              "Holding native execution after its read cannot release the instruction reservation");
    }
  }
  bool retired = false;
  for (; cycle < limit && !retired; ++cycle) {
    const auto result =
        machine.advance_dma_native_cycle(inputs(cycle, 1), OwnDspNativeCycleModel::isolated_effect);
    if (!result.native) { throw Fault("The reserved native bit effect must advance"); }
    require(!result.dma.word && result.native &&
                result.native->fault.kind == dsp56300::FaultKind::none,
            "Actual RAM write and flag publications execute while fresh DMA words remain deferred");
    retired = result.native->retired;
  }
  require(retired, "The direct owner must retire the actual bit operation");
  auto expected = before;
  ++expected.pc;
  expected.sr &= ~1U;
  require(machine.cpu().registers() == expected &&
              machine.bus().read({.space = sample.space, .address = operand}) == modified(sample),
          "The direct native owner retains independent BSET and Carry expectations");
  if (!drain) {
    require(!machine.advance_dma_reference_cycle(inputs(cycle++, 1)).word,
            "The latched request starts its real source phase after retirement");
    const auto result = machine.advance_dma_reference_cycle(inputs(cycle, 1));
    require(result.word && result.word->value == modified(sample),
            "The request retained across a held CPU observes the committed RAM word");
  }
}

void reset_transaction(const bool committed) {
  const Sample sample{};
  OwnDspMachine machine(native::native_clocks);
  prepare(machine, sample);
  seek_read(machine);
  unsigned cycle = 0;
  for (; cycle < limit; ++cycle) {
    const auto pending = machine.cpu().pending_instruction();
    if (!pending) { throw Fault("Reset cut requires an actual unretired bit instruction"); }
    const auto result = machine.advance_dma_native_cycle(inputs(cycle, limit),
                                                         OwnDspNativeCycleModel::isolated_effect);
    require(result.native && !result.native->retired, "Reset precedes actual retirement");
    const auto kind = pending->effect.kind;
    if (kind == (committed ? dsp56300::EffectKind::write : dsp56300::EffectKind::read)) { break; }
  }
  require(cycle < limit, "The requested read/write reset boundary must be reached");
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  require(
      !machine.cpu().pending_instruction() &&
          machine.bus().read({.space = sample.space, .address = operand}) ==
              (committed ? modified(sample) : initial(sample)),
      "Reset discards the unfinished transaction while retaining an already accepted RAM write");
}
}

int main() {
  try {
    unsigned cases = 0;
    constexpr std::array bits{0U, 13U, 23U};
    for (const auto action : {Action::clear, Action::set, Action::change, Action::test}) {
      for (const auto form :
           {Form::short_address, Form::indirect, Form::increment, Form::absolute}) {
        for (const auto space : {DspSpace::x, DspSpace::y}) {
          for (const auto bit : bits) {
            for (const auto level : {false, true}) {
              for (unsigned request = 1; request < offsets; ++request) {
                overlap(
                    {.action = action, .form = form, .space = space, .bit = bit, .level = level},
                    request);
                ++cases;
              }
            }
          }
        }
      }
    }
    direct_reservation(false);
    direct_reservation(true);
    reset_transaction(false);
    reset_transaction(true);
    std::cout << "RAM_RMW cases=" << cases << " fresh_checkpoint_cuts=" << replay()
              << " direct_and_reset=4 passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "RAM_RMW: " << error.what() << '\n';
    return 1;
  }
}
