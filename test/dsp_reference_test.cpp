// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace blofeld::test::reference {
namespace {
using native_access::fault;

void fifo_and_operand_latches() {
  constexpr unsigned initial_destination = 0x100;
  constexpr unsigned actual_destination = 0x110;
  constexpr unsigned later_destination = 0x120;
  DspMachine machine;
  DspProgram code;
  code.emit("movep #>$a9,x:<<$ffff91");
  const auto body = code.pc();
  code.emit("movep x:<<$ffff94,x:>$100");
  const auto next = code.pc();
  code.word(long_jump);
  code.word(body);
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  machine.bus().shi().observe_spi({.mosi = false});
  machine.cpu().step();
  for (const auto address : {initial_destination, actual_destination, later_destination}) {
    machine.bus().write({.space = DspSpace::x, .address = address, .value = third});
  }
  send(machine.bus().shi(), first);
  send(machine.bus().shi(), second);
  const auto before = machine.cpu().debug_state();
  auto progress = machine.begin_reference_instruction(boundaries);
  expect(progress, {.kind = DspNativeAccessKind::opcode, .target = {.address = body}, .pc = body});
  const auto opcode_ticket = ticket(progress);
  require(ticket(machine) == opcode_ticket && machine.cpu().debug_state() == before,
          "Repeated pending inspection performs no access or speculative register retirement");
  static_cast<void>(fault([&] { machine.cpu().step(); }));
  progress = machine.advance_reference_access(opcode_ticket);
  expect(progress,
         {.kind = DspNativeAccessKind::read,
          .target = {.space = DspSpace::x, .address = fifo},
          .ordinal = 1,
          .pc = body});
  require(completed(progress).stage == DspNativeAccessStage::after,
          "An admitted fetch reports its actual completion");
  static_cast<void>(
      fault([&] { static_cast<void>(machine.advance_reference_access(opcode_ticket)); }));
  const std::array changed_extension{actual_destination};
  machine.bus().install_rom({.start = body + 1, .words = changed_extension});
  progress = machine.advance_reference_access(ticket(progress));
  require(completed(progress).value == first, "The first real FIFO word is captured once");
  expect(progress,
         {.kind = DspNativeAccessKind::extension,
          .target = {.address = body + 1},
          .ordinal = 2,
          .pc = body});
  progress = machine.advance_reference_access(ticket(progress));
  require(completed(progress).value == actual_destination,
          "The actual extension read sees a change made after decoder lookahead");
  expect(progress,
         {.kind = DspNativeAccessKind::write,
          .target = {.space = DspSpace::x, .address = actual_destination},
          .value = first,
          .ordinal = 3,
          .pc = body});
  require(machine.cpu().debug_state() == before &&
              machine.bus().read({.space = DspSpace::x, .address = actual_destination}) == third,
          "The destination and architected CPU state still wait for final admission");
  const std::array changed_code{0U, later_destination};
  machine.bus().install_rom({.start = body, .words = changed_code});
  const auto held = machine.checkpoint();
  const auto old_ticket = ticket(progress);
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(held);
    static_cast<void>(
        fault([&] { static_cast<void>(machine.advance_reference_access(old_ticket)); }));
    require(machine.bus().read({.space = DspSpace::x, .address = fifo}) == second,
            "The mid-instruction snapshot contains the next FIFO word, not a replayed read");
    progress = machine.advance_reference_access(ticket(machine));
    require(progress.retired && !progress.pending && completed(progress).value == first &&
                machine.cpu().state().pc == next &&
                machine.cpu().debug_state().native_instructions == before.native_instructions + 1,
            "Exactly one instruction retires with its already captured source/extension");
    for (const auto address : {initial_destination, later_destination}) {
      require(machine.bus().read({.space = DspSpace::x, .address = address}) == third,
              "Neither speculative lookahead nor later extension mutation redirects the store");
    }
    require(machine.bus().read({.space = DspSpace::x, .address = actual_destination}) == first,
            "The actual consumed extension selects the only destination written");
    require(fault([&] {
              static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = fifo}));
            }).contains("empty"),
            "No hidden FIFO word was fabricated while the completed prefix recomputed");
    machine.cpu().step();
    machine.cpu().step();
    require(machine.cpu().state().pc == body + 1,
            "A normal revisit sees the changed NOP instead of a stale suspended MOVEP");
  }
}

void ticket_ownership() {
  DspMachine machine;
  DspProgram code;
  code.emit("nop");
  code.emit("nop");
  code.install(machine.bus());
  auto progress = machine.begin_reference_instruction(boundaries);
  const auto original = ticket(progress);
  DspReferenceTicket foreign;
  {
    DspMachine other;
    code.install(other.bus());
    foreign = ticket(other.begin_reference_instruction(boundaries));
  }
  for (const auto &invalid : {DspReferenceTicket{}, foreign}) {
    static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(invalid)); }));
  }
  require(ticket(machine) == original,
          "Invalid owner grants leave the legitimate pending access intact");
  progress = machine.advance_reference_access(original);
  require(progress.retired, "The legitimate NOP fetch retires the real instruction");
  progress = machine.begin_reference_instruction(boundaries);
  static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(original)); }));
  const auto before_reset = ticket(progress);
  machine.cpu().reset_spi_boot();
  progress = machine.begin_reference_instruction(boundaries);
  static_cast<void>(
      fault([&] { static_cast<void>(machine.advance_reference_access(before_reset)); }));
  const auto pending = machine.checkpoint();
  const auto before_restore = ticket(progress);
  machine.restore(pending);
  static_cast<void>(
      fault([&] { static_cast<void>(machine.advance_reference_access(before_restore)); }));
  require(
      machine.advance_reference_access(ticket(machine)).retired,
      "Reset/restore invalidate old tickets while a freshly queried owner grant remains usable");
}
}
}

int main() {
  using namespace blofeld::test::reference;
  try {
    fifo_and_operand_latches();
    ticket_ownership();
    core_checks();
    dma_checks();
    interrupt_checks();
    std::cout
        << "Native reference access suspension, exact effects and coordinated replay passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
