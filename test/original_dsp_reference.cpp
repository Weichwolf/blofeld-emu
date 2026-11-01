// SPDX-License-Identifier: GPL-3.0-only
#include "original_dsp_reference.hpp"

#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_boot_context.hpp"

#include <array>
#include <exception>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace blofeld::test {
namespace {
constexpr unsigned clear_pc = 0x72, clear_count = 0x20000, ram_end = 0x2000;
constexpr unsigned original_omr = 0x600389, first = 0x123456, second = 0x234567;
using boot_context::capture;
using boot_context::State;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); } catch (const std::logic_error &error) {
    return error.what();
  }
  throw Fault("Expected a strict original-stage native rejection");
}

// Observation metadata counts only actually accepted data writes. The native
// pending effect, ticket and retirement remain the processor's own objects.
struct StoreProgress {
  dsp56300::InstructionProgress native;
  unsigned ordinal = 0;
  std::optional<dsp56300::InstructionEffect> completed = std::nullopt;
};

void healthy(const OwnDspMachine &machine,
             const dsp56300::InstructionProgress &progress,
             const unsigned ordinal) {
  try {
    require_original_native(machine, progress);
  } catch (const Fault &error) {
    throw Fault(std::string(error.what()) + " ordinal=" + std::to_string(ordinal));
  }
}

StoreProgress until_store(OwnDspMachine &machine, StoreProgress progress) {
  constexpr unsigned effect_bound = 64;
  for (unsigned effect = 0; effect < effect_bound; ++effect) {
    healthy(machine, progress.native, progress.ordinal);
    if (progress.native.retired) { return progress; }
    if (!progress.native.pending) { throw Fault("Original store lost its actual native effect"); }
    if (progress.native.pending->effect.kind == dsp56300::EffectKind::write) { return progress; }
    progress.native = machine.advance(progress.native.pending->ticket);
  }
  throw Fault("Original store exceeded its real native effect bound");
}

StoreProgress begin(OwnDspMachine &machine) {
  // Preserve the retained access-only observation contract explicitly. The
  // normal PCU publication contract is checked separately on the same program.
  return until_store(machine,
                     {.native = machine.begin_fetch(dsp56300::InstructionPublication::retirement)});
}

StoreProgress pending(const OwnDspMachine &machine, const unsigned ordinal) {
  const auto effect = machine.cpu().pending_instruction();
  require(machine.cpu().fault().kind == dsp56300::FaultKind::none && effect &&
              effect->effect.kind == dsp56300::EffectKind::write,
          "Original store must retain a healthy actual pending write");
  return {.native = {.pending = effect,
                     .progress = machine.cpu().progress(),
                     .fault = machine.cpu().fault()},
          .ordinal = ordinal};
}

dsp56300::EffectTicket ticket(const StoreProgress &progress) {
  if (!progress.native.pending) { throw Fault("Original store lost its actual ticket"); }
  return progress.native.pending->ticket;
}

StoreProgress advance(OwnDspMachine &machine, const StoreProgress &progress) {
  if (!progress.native.pending) { throw Fault("Original store has no pending write"); }
  require(progress.native.pending->effect.kind == dsp56300::EffectKind::write,
          "One original data-store advance requires its actual pending write");
  const auto effect = progress.native.pending->effect;
  auto result = machine.advance(ticket(progress));
  healthy(machine, result, progress.ordinal);
  return until_store(machine,
                     {.native = result, .ordinal = progress.ordinal + 1, .completed = effect});
}

void expect(const StoreProgress &progress, const DspAddress target, const unsigned ordinal) {
  if (!progress.native.pending) { throw Fault("Expected a pending original long-memory store"); }
  const auto &value = progress.native.pending->effect;
  const auto space = target.space == DspSpace::x ? dsp56300::Space::x : dsp56300::Space::y;
  require(!progress.native.retired && value.kind == dsp56300::EffectKind::write &&
              value.space == space && value.address == target.address && value.value == 0 &&
              progress.ordinal == ordinal && value.pc == clear_pc && value.omr == original_omr,
          "Loaded original REP presents exactly its independently expected next zero store");
}

void record(const std::string_view phase,
            const StoreProgress &progress,
            const OwnDspMachine &machine,
            std::ostream &trace) {
  if (!progress.native.pending) { throw Fault("Cannot record an absent original store request"); }
  const auto &value = progress.native.pending->effect;
  constexpr std::array spaces{"P:", "X:", "Y:"};
  const auto state = capture(machine);
  trace << phase << " write=" << spaces.at(static_cast<unsigned>(value.space))
        << hex(value.address, 6) << " ordinal=" << progress.ordinal << " pc=" << hex(value.pc, 6)
        << " omr=" << hex(value.omr, 6) << " r4=" << hex(state.cpu.r[4], 6)
        << " lc=" << hex(state.cpu.lc, 6) << " instructions=" << state.progress.instructions
        << " estimated-core-cycles=" << state.progress.estimated_cycles << '\n';
}

void verify_base(const State &before, const unsigned address) {
  require(before.cpu.pc == clear_pc && before.cpu.omr == original_omr &&
              before.cpu.r[4] == address && before.cpu.lc == clear_count - address &&
              before.cpu.a == 0,
          "Original stage retains its actual fractional REP count, zero source and address");
}

void mapped_pair(OwnDspMachine &machine,
                 const OwnDspCheckpoint &last_pair,
                 const State &frontier,
                 std::ostream &trace) {
  constexpr unsigned address = ram_end - 1;
  machine.restore(last_pair);
  const auto before = capture(machine);
  verify_base(before, address);
  machine.bus().write({.space = DspSpace::x, .address = address, .value = first});
  machine.bus().write({.space = DspSpace::y, .address = address, .value = second});
  auto progress = begin(machine);
  expect(progress, {.space = DspSpace::x, .address = address}, 0);
  record("last-pair-before-X", progress, machine, trace);
  require(capture(machine) == before &&
              machine.bus().read({.space = DspSpace::x, .address = address}) == first &&
              machine.bus().read({.space = DspSpace::y, .address = address}) == second,
          "Preparing the loaded original REP performs no speculative memory write");
  progress = advance(machine, progress);
  expect(progress, {.space = DspSpace::y, .address = address}, 1);
  record("last-pair-before-Y", progress, machine, trace);
  require(capture(machine) == before &&
              machine.bus().read({.space = DspSpace::x, .address = address}) == 0 &&
              machine.bus().read({.space = DspSpace::y, .address = address}) == second,
          "The real X store commits while Y and CPU retirement remain pending");
  const auto held = machine.checkpoint();
  const auto old_ticket = ticket(progress);
  const auto ordinal = progress.ordinal;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(held);
    static_cast<void>(rejected([&] { static_cast<void>(machine.advance(old_ticket)); }));
    require(machine.bus().read({.space = DspSpace::x, .address = address}) == 0 &&
                machine.bus().read({.space = DspSpace::y, .address = address}) == second,
            "Each immutable original-stage snapshot retains exactly the committed bus prefix");
    const auto outside_value = first + replay;
    machine.bus().write({.space = DspSpace::x, .address = address, .value = outside_value});
    progress = pending(machine, ordinal);
    expect(progress, {.space = DspSpace::y, .address = address}, 1);
    record("last-pair-restored-Y", progress, machine, trace);
    progress = advance(machine, progress);
    require(progress.native.retired && !progress.native.pending && progress.completed &&
                progress.completed->space == dsp56300::Space::y && progress.completed->value == 0 &&
                machine.cpu().registers().r[4] == ram_end &&
                machine.cpu().registers().lc == clear_count - ram_end &&
                capture(machine).progress.instructions == before.progress.instructions + 1 &&
                capture(machine) == frontier &&
                machine.bus().read({.space = DspSpace::x, .address = address}) == outside_value &&
                machine.bus().read({.space = DspSpace::y, .address = address}) == 0,
            "The original store retires once without replaying X or updating R4/LC twice");
    trace << "last-pair-retired replay=" << replay << " retained-X=" << hex(outside_value, 6)
          << " Y=0 r4=" << hex(ram_end, 6) << " lc=" << hex(clear_count - ram_end, 6) << '\n';
  }
}

StoreProgress pcu_prefix(OwnDspMachine &machine, const unsigned address, std::ostream &trace) {
  const auto before = capture(machine);
  auto expected = before;
  expected.cpu.r[4] = address + 1;
  auto progress = machine.begin_fetch(dsp56300::InstructionPublication::pcu_updates);
  constexpr unsigned bound = 64;
  unsigned updates = 0;
  for (unsigned count = 0; count < bound && progress.pending; ++count) {
    const auto effect = progress.pending->effect;
    if (effect.kind == dsp56300::EffectKind::write) { break; }
    const bool update = effect.kind == dsp56300::EffectKind::address_update;
    if (update) {
      require(effect.address_update && effect.address_update->index == 4 &&
                  effect.address_update->previous == address &&
                  effect.address_update->value == address + 1 && effect.address == address &&
                  capture(machine) == before,
              "The real R4 postincrement awaits its native address-publication ticket");
    }
    progress = machine.advance(progress.pending->ticket);
    healthy(machine, progress, 0);
    if (update) {
      ++updates;
      require(capture(machine) == expected,
              "Only the actual address ticket publishes R4 before the first original store");
    }
  }
  StoreProgress result{.native = progress};
  expect(result, {.space = DspSpace::x, .address = address}, 0);
  require(updates == 1 && capture(machine) == expected,
          "Default PCU publication exposes one independently expected R4 update");
  record("pcu-before-X", result, machine, trace);
  return result;
}

void pcu_path(OwnDspMachine &machine,
              const OwnDspCheckpoint &saved,
              const State &frontier,
              const bool mapped,
              std::ostream &trace) {
  machine.restore(saved);
  const auto address = mapped ? ram_end - 1 : ram_end;
  const auto before = capture(machine);
  verify_base(before, address);
  machine.bus().write({.space = DspSpace::x, .address = address, .value = first});
  if (mapped) { machine.bus().write({.space = DspSpace::y, .address = address, .value = second}); }
  auto progress = pcu_prefix(machine, address, trace);
  progress = advance(machine, progress);
  expect(progress, {.space = DspSpace::y, .address = address}, 1);
  auto published = before;
  published.cpu.r[4] = address + 1;
  require(capture(machine) == published &&
              machine.bus().read({.space = DspSpace::x, .address = address}) == 0,
          "Original X commits after R4 publication while Y and retirement remain pending");
  record("pcu-before-Y", progress, machine, trace);
  const auto held = machine.checkpoint();
  const auto old_ticket = ticket(progress);
  const auto ordinal = progress.ordinal;
  for (const bool fresh : {false, true}) {
    OwnDspMachine clone(dma_test::native_clocks);
    auto &target = fresh ? clone : machine;
    target.restore(held);
    static_cast<void>(rejected([&] { static_cast<void>(target.advance(old_ticket)); }));
    require(capture(target) == published &&
                target.bus().read({.space = DspSpace::x, .address = address}) == 0,
            "Same/fresh PCU restore retains published R4, committed X and a new pending-Y ticket");
    const auto outside = fresh ? second : first;
    target.bus().write({.space = DspSpace::x, .address = address, .value = outside});
    progress = pending(target, ordinal);
    if (mapped) {
      progress = advance(target, progress);
      require(progress.native.retired && capture(target) == frontier &&
                  target.cpu().registers().r[4] == address + 1 &&
                  target.cpu().registers().lc == clear_count - address - 1 &&
                  target.cpu().progress().instructions == before.progress.instructions + 1 &&
                  target.bus().read({.space = DspSpace::y, .address = address}) == 0,
              "Native Y/PCU/retirement finishes once without applying the published R4 twice");
    } else {
      const auto error = rejected([&] { static_cast<void>(advance(target, progress)); });
      const auto fault = target.cpu().fault();
      require(error.contains("Unmapped DSP memory write at Y:0x002000") &&
                  fault.kind == dsp56300::FaultKind::memory_write &&
                  fault.space == dsp56300::Space::y && fault.address == address &&
                  fault.value == 0 && fault.pc == clear_pc && capture(target) == published,
              "Reserved Y faults retain actual published R4 and X without retiring the REP body");
      static_cast<void>(rejected([&] { dma_test::step_native(target); }));
    }
    require(target.bus().read({.space = DspSpace::x, .address = address}) == outside,
            "Either PCU continuation preserves interposed X instead of replaying its old store");
    trace << "pcu-continued mapped=" << mapped << " fresh=" << fresh
          << " r4=" << hex(target.cpu().registers().r[4], 6)
          << " retired=" << target.cpu().progress().instructions << '\n';
  }
}

void strict_frontier(OwnDspMachine &machine,
                     const OwnDspCheckpoint &frontier,
                     std::ostream &trace) {
  machine.restore(frontier);
  const auto before = capture(machine);
  verify_base(before, ram_end);
  machine.bus().write({.space = DspSpace::x, .address = ram_end, .value = first});
  auto progress = begin(machine);
  expect(progress, {.space = DspSpace::x, .address = ram_end}, 0);
  record("frontier-before-X", progress, machine, trace);
  progress = advance(machine, progress);
  expect(progress, {.space = DspSpace::y, .address = ram_end}, 1);
  record("frontier-before-Y", progress, machine, trace);
  require(capture(machine) == before &&
              machine.bus().read({.space = DspSpace::x, .address = ram_end}) == 0,
          "The mapped X store commits before the reserved Y effect is attempted");
  const auto held = machine.checkpoint();
  const auto old_ticket = ticket(progress);
  const auto ordinal = progress.ordinal;
  std::string first_failure;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(held);
    static_cast<void>(rejected([&] { static_cast<void>(machine.advance(old_ticket)); }));
    require(machine.bus().read({.space = DspSpace::x, .address = ram_end}) == 0 &&
                capture(machine) == before,
            "Trusted restore recovers the actual pending Y effect and committed X zero");
    const auto outside_value = second + replay;
    machine.bus().write({.space = DspSpace::x, .address = ram_end, .value = outside_value});
    progress = pending(machine, ordinal);
    expect(progress, {.space = DspSpace::y, .address = ram_end}, 1);
    record("frontier-restored-Y", progress, machine, trace);
    const auto failure = rejected([&] { static_cast<void>(advance(machine, progress)); });
    require(failure.contains("Unmapped DSP memory write at Y:0x002000") &&
                failure.contains("pc=0x000072") && failure.contains("omr=0x600389") &&
                failure.contains("ordinal=1") && capture(machine) == before &&
                machine.bus().read({.space = DspSpace::x, .address = ram_end}) == outside_value,
            "Reserved Y admission fails without retiring or replaying the committed X prefix");
    if (replay == 0) { first_failure = failure; }
    require(failure == first_failure, "Both pending-Y restores produce the same native fault");
    static_cast<void>(rejected([&] { static_cast<void>(pending(machine, ordinal)); }));
    static_cast<void>(rejected([&] { dma_test::step_native(machine); }));
    // A diagnostic fault checkpoint is allowed by the native API; restoring it
    // must preserve the fault, committed X word and blocked execution.
    const auto fault = machine.cpu().fault();
    const auto failed = machine.checkpoint();
    OwnDspMachine diagnostic(dma_test::native_clocks);
    diagnostic.restore(failed);
    require(diagnostic.cpu().fault() == fault && capture(diagnostic) == before &&
                diagnostic.bus().read({.space = DspSpace::x, .address = ram_end}) == outside_value,
            "Fault checkpoints retain the real failed effect and accepted write prefix");
    static_cast<void>(rejected([&] { dma_test::step_native(diagnostic); }));
    trace << "frontier-rejected replay=" << replay << " retained-X=" << hex(outside_value, 6)
          << " error=" << failure << '\n';
  }
  machine.restore(frontier);
  require(capture(machine) == before && !machine.cpu().pending_instruction() &&
              machine.cpu().fault().kind == dsp56300::FaultKind::none,
          "The actual healthy instruction snapshot recovers after either failed branch");
}
}

void require_original_native(const OwnDspMachine &machine,
                             const dsp56300::InstructionProgress &progress) {
  if (progress.fault.kind == dsp56300::FaultKind::none) { return; }
  std::string cause = "Native original instruction fault";
  const auto failure = machine.io().failure();
  if (failure && failure->cause) {
    try {
      std::rethrow_exception(failure->cause);
    } catch (const std::exception &error) { cause = error.what(); }
  }
  throw Fault(
      cause + " pc=" + hex(progress.fault.pc, 6) + " omr=" + hex(machine.cpu().registers().omr, 6) +
      " address=" + hex(progress.fault.address, 6) + " value=" + hex(progress.fault.value, 6) +
      " space=" + std::to_string(static_cast<unsigned>(progress.fault.space)) +
      " native-kind=" + std::to_string(static_cast<unsigned>(progress.fault.kind)));
}

void verify_original_reference_access(const OwnDspCheckpoint &last_pair,
                                      const OwnDspMachine &frontier,
                                      std::ostream &trace) {
  const auto before = capture(frontier);
  const auto saved = frontier.checkpoint();
  OwnDspMachine branch(dma_test::native_clocks);
  mapped_pair(branch, last_pair, before, trace);
  strict_frontier(branch, saved, trace);
  pcu_path(branch, last_pair, before, true, trace);
  pcu_path(branch, saved, before, false, trace);
  trace << "PCU publication snapshots=2 same/fresh continuations=4\n";
  require(capture(frontier) == before, "Private branches never mutate the original firmware owner");
}

namespace boot_context {
void check_switch(const OwnDspMachine &frontier, std::ostream &trace) {
  const auto saved = frontier.checkpoint();
  const auto state = capture(frontier);
  OwnDspMachine branch(dma_test::native_clocks);
  constexpr unsigned switched_program = 0x1000;
  constexpr unsigned switch_cycles = 3;
  for (const auto elapsed : {0U, 2U, switch_cycles}) {
    branch.restore(saved);
    branch.bus().advance_instruction_cycles(elapsed);
    const DspMemoryAccess target{.target = {.address = switched_program},
                                 .operation = DspMemoryOperation::write};
    if (elapsed < switch_cycles) {
      const auto error = rejected([&] { static_cast<void>(branch.bus().memory_resource(target)); });
      require(error.contains("requires elapsed switch cycles: P:" + hex(switched_program, 6)),
              "The changing program window refuses access before the supplied model deadline");
    } else {
      const auto resource = branch.bus().memory_resource(target);
      constexpr unsigned moved_x = 0x10000;
      require(resource.bank == DspSpace::x && !resource.rom && resource.word == moved_x,
              "Three explicitly supplied switch periods commit the inferred X-to-P map");
    }
    require(capture(branch) == state, "Model switch periods do not replace native CPU state/time");
    auto progress = begin(branch);
    expect(progress, {.space = DspSpace::x, .address = ram_end}, 0);
    progress = advance(branch, progress);
    expect(progress, {.space = DspSpace::y, .address = ram_end}, 1);
    require(capture(branch) == state &&
                branch.bus().read({.space = DspSpace::x, .address = ram_end}) == 0,
            "Only the real X zero commits before the reserved Y access");
    const auto error = rejected([&] { static_cast<void>(advance(branch, progress)); });
    require(error.contains("write at Y:" + hex(ram_end, 6)) &&
                error.contains("pc=" + hex(clear_pc, 6)) &&
                error.contains("omr=" + hex(memory_mode, 6)) && capture(branch) == state,
            "The native reserved-Y failure persists before and after actual model map switching");
    trace << "switch-reference-cycles=" << elapsed << " error=" << error << '\n';
  }
  require(capture(frontier) == state, "Switch branches leave the original frontier unchanged");
}
}
}
