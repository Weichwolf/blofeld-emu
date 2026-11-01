// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_reset_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

namespace {
using namespace blofeld;
using namespace test::fetch_reset;
constexpr std::uint64_t pcu_cut_ticks = 405, next_pcu_deadline = 450;
constexpr unsigned entry = 0x100, data = 0x400, high = 0x234567, low = 0x12345;
constexpr unsigned canary = 0x654321, store = 0x485c00, bound = 256;

void denied(Fixture &f, const auto &operation, const std::string &name) {
  const auto before = f.observe();
  const auto error = rejects(operation);
  require(error.contains("clocked timeline") && error.contains("operation=" + name) &&
              error.contains("pc=") && error.contains("omr=") && f.observe() == before &&
              !f.timeline->faulted() && !f.machine.clock().faulted(),
          "Clocked ownership rejects before CPU, memory, interrupt or clock mutation");
}

void partial_pcu_regression() {
  Fixture f;
  f.advance(4);
  const auto pending = f.machine.cpu().pending_instruction();
  if (!pending) { throw Fault("Actual MOVE lost its selected PCU publication"); }
  require(pending->effect.kind == dsp56300::EffectKind::pcu_update &&
              f.machine.cpu().registers().pc == boot && f.machine.cpu().registers().x == 1 &&
              f.machine.cpu().progress().instructions == 0 && f.ledger.ticks == pcu_cut_ticks,
          "The real MOVE result is published but its PC and retirement are still pending");
  denied(f, [&] { static_cast<void>(f.machine.advance(pending->ticket)); }, "native-effect");
  require(f.timeline->next_deadline() == next_pcu_deadline,
          "Refusal preserves the actual next reference period");
  f.advance(2);
  require(f.machine.cpu().registers().pc == boot + 2 &&
              f.machine.cpu().progress() ==
                  dsp56300::Progress{.instructions = 1, .estimated_cycles = 2},
          "The same pending PCU then retires through two actual owned periods");
}

void direct_running_operations() {
  Fixture f;
  denied(f, [&] { static_cast<void>(f.machine.begin_fetch()); }, "begin-fetch");
  denied(f, [&] { static_cast<void>(f.machine.advance_reference_cycle()); }, "reference-period");
  denied(
      f,
      [&] { static_cast<void>(f.machine.advance_dma_reference_cycle(inputs(f.machine))); },
      "dma-cycle");
  denied(
      f,
      [&] {
        static_cast<void>(f.machine.advance_dma_native_cycle(
            inputs(f.machine), OwnDspNativeCycleModel::isolated_effect));
      },
      "dma-cycle");
  denied(
      f,
      [&] { static_cast<void>(f.machine.admit_interrupt({.address = boot, .opcode = 0})); },
      "admit-interrupt");
  denied(f, [&] { f.machine.begin_hardware_reset(pins()); }, "reset-assert");
  denied(f, [&] { static_cast<void>(f.machine.advance_hardware_reset_ticks(1)); }, "reset-time");
  denied(f, [&] { f.machine.release_hardware_reset(); }, "reset-release");
  f.advance(6);
}

void restore_modes() {
  Fixture source;
  source.advance(4);
  const auto checkpoint = source.timeline->checkpoint();
  const auto ledger = source.ledger;
  Fixture fresh(Start::blank);
  for (auto *const f : {&source, &fresh}) {
    f->restore(checkpoint, ledger);
    const auto pending = f->machine.cpu().pending_instruction();
    if (!pending) { throw Fault("Joint restore lost the actual PCU effect"); }
    denied(*f, [&] { static_cast<void>(f->machine.advance(pending->ticket)); }, "native-effect");
    f->advance(2);
  }
  require(source.observe() == fresh.observe(),
          "Same/fresh clocked restoration preserves literal completion state and ownership");
}

void explicit_release_and_machine_restore() {
  for (const bool chip_restore : {false, true}) {
    Fixture f;
    f.advance(4);
    const auto saved = f.machine.checkpoint();
    const auto retained = f.timeline->checkpoint();
    const auto first = f.machine.cpu().pending_instruction();
    if (!first) { throw Fault("Manual continuation lost its real ticket"); }
    const auto stale = first->ticket;
    const auto before = f.machine.cpu().registers();
    const auto ticks = f.machine.clock().elapsed_ticks();
    if (chip_restore) {
      f.machine.restore(saved);
      static_cast<void>(rejects([&] { static_cast<void>(f.timeline->next_deadline()); }));
      // Timed ownership is process-local; a chip-only restore abandons it.
      static_cast<void>(rejects([&] { static_cast<void>(f.machine.advance(stale)); }));
    } else {
      // An immutable snapshot must not keep a destroyed timeline's lease alive.
      f.timeline.reset();
    }
    auto pending = f.machine.cpu().pending_instruction();
    if (!pending) { throw Fault("Bare chip lost its genuine PCU continuation"); }
    const auto pc = f.machine.advance(pending->ticket);
    auto expected = before;
    expected.pc = boot + 2;
    require(pc.fault.kind == dsp56300::FaultKind::none && !pc.retired &&
                f.machine.cpu().registers() == expected &&
                f.machine.clock().elapsed_ticks() == ticks,
            "Explicit untimed continuation publishes only the real PC without inventing clocks");
    pending = f.machine.cpu().pending_instruction();
    if (!pending) { throw Fault("Bare chip lost its actual retirement"); }
    const auto retired = f.machine.advance(pending->ticket);
    require(retired.retired && retired.fault.kind == dsp56300::FaultKind::none &&
                f.machine.cpu().registers() == expected &&
                f.machine.clock().elapsed_ticks() == ticks &&
                f.machine.cpu().progress() ==
                    dsp56300::Progress{.instructions = 1, .estimated_cycles = 2},
            "Unclocked chip use remains available after explicit release or chip-only restore");
  }
}

void held_reset_restore() {
  Fixture source;
  source.advance(4);
  source.reset();
  source.held(held_ticks, held_ticks);
  const auto saved = source.timeline->checkpoint();
  const auto ledger = source.ledger;
  Fixture fresh(Start::blank);
  for (auto *const f : {&source, &fresh}) {
    f->restore(saved, ledger);
    denied(*f, [&] { f->machine.release_hardware_reset(); }, "reset-release");
    denied(
        *f, [&] { static_cast<void>(f->machine.advance_hardware_reset_ticks(1)); }, "reset-time");
    denied(*f, [&] { f->machine.begin_hardware_reset(pins()); }, "reset-assert");
    f->timeline->release_hardware_reset();
    f->held(recovery_ticks, recovery_ticks);
    const auto release = f->timeline->checkpoint();
    f->restore(release, f->ledger);
    denied(
        *f, [&] { static_cast<void>(f->machine.advance_reference_cycle()); }, "reference-period");
    f->advance(6);
  }
  require(source.observe() == fresh.observe(),
          "Reset/recovery and fresh frontend acquisition preserve clocked admission ownership");
}

void pending_memory_store() {
  constexpr std::array<std::uint32_t, 32> program{
      0x64f400, data, 0x50f400, low, 0x54f400, high, 0x52f400, 0, jump, entry};
  Fixture f(Start::blank);
  f.machine.bus().install_rom({.space = DspSpace::p, .start = boot, .words = program});
  for (unsigned i = 0; i < 32; ++i) {
    f.machine.bus().write({.address = entry + i, .value = i == 0 ? store : 0});
  }
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    f.machine.bus().write({.space = space, .address = data, .value = canary});
  }
  f.power_on();
  bool completed = false;
  unsigned denied_writes = 0;
  for (unsigned cycle = 0; cycle < bound && !completed; ++cycle) {
    const auto pending = f.machine.cpu().pending_instruction();
    if (pending && pending->effect.kind == dsp56300::EffectKind::write &&
        pending->effect.pc == entry) {
      require(pending->effect.address == data &&
                  pending->effect.value == (denied_writes == 0 ? high : low),
              "Native long-store callbacks contain independently specified addresses/data");
      const auto x = f.machine.bus().read({.space = DspSpace::x, .address = data});
      const auto y = f.machine.bus().read({.space = DspSpace::y, .address = data});
      denied(f, [&] { static_cast<void>(f.machine.advance(pending->ticket)); }, "native-effect");
      require(f.machine.bus().read({.space = DspSpace::x, .address = data}) == x &&
                  f.machine.bus().read({.space = DspSpace::y, .address = data}) == y,
              "Rejected direct admission executes neither genuine memory write");
      ++denied_writes;
    }
    const auto result = f.step();
    const auto outcome = f.machine.cpu().instruction_outcome();
    completed = result.native && result.native->retired && outcome && outcome->address == entry;
  }
  require(completed && denied_writes == 2 && f.machine.cpu().registers().pc == entry + 1 &&
              f.machine.cpu().registers().r[4] == data + 1 &&
              f.machine.bus().read({.space = DspSpace::x, .address = data}) == high &&
              f.machine.bus().read({.space = DspSpace::y, .address = data}) == low,
          "The same two genuine stores and postincrement complete through the owned clock");
}
}

int main() {
  try {
    partial_pcu_regression();
    direct_running_operations();
    restore_modes();
    explicit_release_and_machine_restore();
    held_reset_restore();
    pending_memory_store();
    std::cout << "Clocked DSP ownership: direct effects/control, restore/release and real X/Y "
                 "stores passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
