// SPDX-License-Identifier: GPL-3.0-only
#include "original_dsp_reference.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <cstdint>
#include <ostream>
#include <string>
#include <string_view>
#include <variant>

namespace blofeld::test {
namespace {
constexpr auto boundaries = DspReferenceExecution::access_boundaries;
constexpr unsigned clear_pc = 0x72;
constexpr unsigned clear_count = 0x20000;
constexpr unsigned ram_end = 0x2000;
constexpr unsigned original_omr = 0x600389;
constexpr unsigned first = 0x123456;
constexpr unsigned second = 0x234567;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected a strict original-stage reference rejection");
}

DspReferenceTicket ticket(const DspReferenceProgress &progress) {
  if (!progress.pending) { throw Fault("Original store lost its pending effect"); }
  return progress.pending->ticket;
}

DspReferenceProgress pending(const DspMachine &machine) {
  const auto effect = machine.reference_pending();
  if (!effect) { throw Fault("Original store must remain suspended"); }
  return {.pending = effect};
}

const DspNativeAccess &access(const DspReferenceEffect &effect) {
  const auto *value = std::get_if<DspNativeAccess>(&effect);
  if (value == nullptr) { throw Fault("Original long-memory store cannot reset peripherals"); }
  return *value;
}

void expect(const DspReferenceProgress &progress,
            const DspAddress target,
            const std::uint64_t ordinal) {
  if (!progress.pending) { throw Fault("Expected a pending original long-memory store"); }
  const auto &value = access(progress.pending->effect);
  require(!progress.retired && value.kind == DspNativeAccessKind::write &&
              value.stage == DspNativeAccessStage::before && value.target.space == target.space &&
              value.target.address == target.address && value.value == 0 &&
              value.ordinal == ordinal && value.pc == clear_pc && value.omr == original_omr,
          "Original loaded REP presents only the independently expected next zero store");
}

void record(const std::string_view phase,
            const DspReferenceProgress &progress,
            const DspMachine &machine,
            std::ostream &trace) {
  if (!progress.pending) { throw Fault("Cannot record an absent original store request"); }
  const auto &value = access(progress.pending->effect);
  constexpr std::array spaces{"P:", "X:", "Y:"};
  const auto state = machine.cpu().debug_state();
  trace << phase << " write=" << spaces.at(static_cast<unsigned>(value.target.space))
        << hex(value.target.address, 6) << " ordinal=" << value.ordinal
        << " pc=" << hex(value.pc, 6) << " omr=" << hex(value.omr, 6)
        << " r4=" << hex(state.cpu.r[4], 6) << " lc=" << hex(state.cpu.lc, 6)
        << " instructions=" << state.native_instructions << " cycles=" << state.native_cycles
        << '\n';
}

void verify_base(const DspDebugState &before, const unsigned address) {
  require(before.cpu.pc == clear_pc && before.cpu.omr == original_omr &&
              before.cpu.r[4] == address && before.cpu.lc == clear_count - address &&
              before.cpu.a == 0,
          "Original stage retains its actual fractional REP count, zero source and address");
}

void mapped_pair(DspMachine &machine,
                 const DspCheckpoint &last_pair,
                 const DspDebugState &legacy_frontier,
                 std::ostream &trace) {
  constexpr unsigned address = ram_end - 1;
  machine.restore(last_pair);
  const auto before = machine.cpu().debug_state();
  verify_base(before, address);
  machine.bus().write({.space = DspSpace::x, .address = address, .value = first});
  machine.bus().write({.space = DspSpace::y, .address = address, .value = second});
  auto progress = machine.begin_reference_instruction(boundaries);
  expect(progress, {.space = DspSpace::x, .address = address}, 0);
  record("last-pair-before-X", progress, machine, trace);
  require(machine.cpu().debug_state() == before &&
              machine.bus().read({.space = DspSpace::x, .address = address}) == first &&
              machine.bus().read({.space = DspSpace::y, .address = address}) == second,
          "Preparing the actual loaded original REP performs no speculative memory write");
  progress = machine.advance_reference_access(ticket(progress));
  expect(progress, {.space = DspSpace::y, .address = address}, 1);
  record("last-pair-before-Y", progress, machine, trace);
  require(machine.cpu().debug_state() == before &&
              machine.bus().read({.space = DspSpace::x, .address = address}) == 0 &&
              machine.bus().read({.space = DspSpace::y, .address = address}) == second,
          "The actual first store commits while Y and register retirement remain pending");
  const auto held = machine.checkpoint();
  const auto old_ticket = ticket(progress);
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(held);
    static_cast<void>(
        rejected([&] { static_cast<void>(machine.advance_reference_access(old_ticket)); }));
    require(machine.bus().read({.space = DspSpace::x, .address = address}) == 0 &&
                machine.bus().read({.space = DspSpace::y, .address = address}) == second,
            "Each immutable original-stage snapshot retains exactly the committed bus prefix");
    // Deliberate semantic-boundary mutation, not a physical clock or extra bus phase.
    const auto outside_value = first + replay;
    machine.bus().write({.space = DspSpace::x, .address = address, .value = outside_value});
    progress = pending(machine);
    expect(progress, {.space = DspSpace::y, .address = address}, 1);
    record("last-pair-restored-Y", progress, machine, trace);
    progress = machine.advance_reference_access(ticket(progress));
    require(progress.retired && !progress.pending && progress.completed &&
                access(*progress.completed).target.space == DspSpace::y &&
                access(*progress.completed).stage == DspNativeAccessStage::after &&
                access(*progress.completed).value == 0 && machine.cpu().state().r[4] == ram_end &&
                machine.cpu().state().lc == clear_count - ram_end &&
                machine.cpu().debug_state().native_instructions == before.native_instructions + 1 &&
                machine.cpu().debug_state() == legacy_frontier &&
                machine.bus().read({.space = DspSpace::x, .address = address}) == outside_value &&
                machine.bus().read({.space = DspSpace::y, .address = address}) == 0,
            "The original store retires once without replaying X or applying R4/LC updates twice");
    trace << "last-pair-retired replay=" << replay << " retained-X=" << hex(outside_value, 6)
          << " Y=0 r4=" << hex(ram_end, 6) << " lc=" << hex(clear_count - ram_end, 6) << '\n';
  }
}

void strict_frontier(DspMachine &machine, const DspCheckpoint &frontier, std::ostream &trace) {
  machine.restore(frontier);
  const auto before = machine.cpu().debug_state();
  verify_base(before, ram_end);
  machine.bus().write({.space = DspSpace::x, .address = ram_end, .value = first});
  auto progress = machine.begin_reference_instruction(boundaries);
  expect(progress, {.space = DspSpace::x, .address = ram_end}, 0);
  record("frontier-before-X", progress, machine, trace);
  progress = machine.advance_reference_access(ticket(progress));
  expect(progress, {.space = DspSpace::y, .address = ram_end}, 1);
  record("frontier-before-Y", progress, machine, trace);
  require(machine.cpu().debug_state() == before &&
              machine.bus().read({.space = DspSpace::x, .address = ram_end}) == 0,
          "The real mapped X store commits before the reserved Y effect is attempted");
  const auto held = machine.checkpoint();
  const auto old_ticket = ticket(progress);
  std::string first_failure;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(held);
    static_cast<void>(
        rejected([&] { static_cast<void>(machine.advance_reference_access(old_ticket)); }));
    require(machine.bus().read({.space = DspSpace::x, .address = ram_end}) == 0 &&
                machine.cpu().debug_state() == before,
            "Trusted restore recovers the original pending Y effect and committed X zero");
    const auto outside_value = second + replay;
    machine.bus().write({.space = DspSpace::x, .address = ram_end, .value = outside_value});
    progress = pending(machine);
    expect(progress, {.space = DspSpace::y, .address = ram_end}, 1);
    record("frontier-restored-Y", progress, machine, trace);
    const auto failure =
        rejected([&] { static_cast<void>(machine.advance_reference_access(ticket(progress))); });
    require(failure.contains("Unmapped DSP memory write at Y:0x002000") &&
                failure.contains("pc=0x000072") && failure.contains("omr=0x600389") &&
                failure.contains("ordinal=1") && machine.cpu().debug_state() == before &&
                machine.bus().read({.space = DspSpace::x, .address = ram_end}) == outside_value,
            "Actual reserved Y admission fails visibly without retiring or repeating X");
    if (replay == 0) { first_failure = failure; }
    require(failure == first_failure, "Both original pending-Y restores produce the same fault");
    static_cast<void>(rejected([&] { static_cast<void>(machine.checkpoint()); }));
    static_cast<void>(rejected([&] { static_cast<void>(machine.reference_pending()); }));
    static_cast<void>(rejected([&] { machine.cpu().step(); }));
    trace << "frontier-rejected replay=" << replay << " retained-X=" << hex(outside_value, 6)
          << " error=" << failure << '\n';
  }
  machine.restore(frontier);
  require(
      machine.cpu().debug_state() == before && !machine.reference_pending(),
      "The complete original instruction-boundary snapshot recovers after either failed branch");
}
}

void verify_original_reference_access(const DspCheckpoint &last_pair,
                                      const DspMachine &frontier,
                                      std::ostream &trace) {
  const auto before = frontier.cpu().debug_state();
  const auto saved = frontier.checkpoint();
  DspMachine branch;
  mapped_pair(branch, last_pair, before, trace);
  strict_frontier(branch, saved, trace);
  require(frontier.cpu().debug_state() == before,
          "Private reference branches never mutate the legacy firmware-run owner");
}
}
