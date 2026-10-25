// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp56kEmu/assembler.h"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <variant>

namespace blofeld::test::reference {
namespace {
using native_access::fault;
constexpr unsigned timer_vector = 0x54;
constexpr unsigned timer_control = 0xffff8f;
constexpr unsigned timer_load = 0xffff8e;
constexpr unsigned timer_compare = 0xffff8d;
constexpr unsigned priorities = 0xfffffe;
constexpr unsigned both_priorities = 0x108;
constexpr unsigned shi_vector = 0x44;
constexpr unsigned unmasked_status = 0xc00000;

void raise_timer(DspMachine &machine) {
  for (const auto store : {DspStore{.space = DspSpace::x, .address = timer_control},
                           DspStore{.space = DspSpace::x, .address = priorities, .value = 0x100},
                           DspStore{.space = DspSpace::x, .address = timer_load, .value = 1},
                           DspStore{.space = DspSpace::x, .address = timer_compare, .value = 1},
                           DspStore{.space = DspSpace::x, .address = timer_control, .value = 7}}) {
    machine.bus().write(store);
  }
  machine.bus().timers().advance_clk2(1);
  require(machine.bus().interrupt_request(0).has_value(), "An actual timer source is pending");
}

void vector_words(DspMachine &machine,
                  const std::array<std::uint32_t, 2> words,
                  const unsigned base = 0) {
  for (unsigned index = 0; index < words.size(); ++index) {
    machine.bus().write({.address = base + timer_vector + index, .value = words[index]});
  }
}

void vector_moves(DspMachine &machine, const std::array<const char *, 2> text) {
  const dsp56k::Assembler assembler;
  std::array<std::uint32_t, 2> words{};
  for (unsigned index = 0; index < text.size(); ++index) {
    const auto encoded = assembler.assemble(text[index]);
    require(encoded.success() && encoded.wordCount == 1, "Own fast vector has two single words");
    words[index] = encoded.word[0];
  }
  vector_words(machine, words);
}

struct VectorExpectation {
  unsigned address = timer_vector;
  unsigned relative = timer_vector;
  unsigned level = 0;
};

DspReferenceProgress await_ack(DspMachine &machine, const VectorExpectation expected = {}) {
  const auto before = machine.cpu().debug_state();
  auto progress = machine.begin_reference_interrupt(boundaries);
  for (unsigned index = 0; index < 2; ++index) {
    expect(progress,
           {.kind = DspNativeAccessKind::vector,
            .target = {.address = expected.address + index},
            .ordinal = index,
            .pc = before.cpu.pc});
    require(machine.bus().interrupt_request(0).has_value() && machine.cpu().debug_state() == before,
            "Vector reads do not acknowledge the source or retire speculative CPU changes");
    progress = machine.advance_reference_access(ticket(progress));
  }
  if (!progress.pending) { throw Fault("Vector entry lost its pending acknowledgement"); }
  const auto *ack = std::get_if<DspInterruptAcknowledge>(&progress.pending->effect);
  if (ack == nullptr) { throw Fault("Expected an acknowledgement after both actual vector reads"); }
  require(ack->request.vector == expected.relative && ack->request.level == expected.level &&
              machine.cpu().debug_state() == before,
          "The accepted source/priority and pre-entry architectural state are retained");
  return progress;
}

void fast_stores_and_reassertion() {
  DspMachine machine;
  DspProgram code;
  code.immediate("x0", first);
  code.immediate("y0", second);
  code.immediate("r0", destination);
  code.immediate("r4", destination + 1);
  code.immediate("sr", unmasked_status);
  code.emit("nop");
  code.install(machine.bus());
  vector_moves(machine, {"move x0,x:(r0)+", "move y0,y:(r4)+"});
  for (unsigned index = 0; index < 5; ++index) { machine.cpu().step(); }
  raise_timer(machine);
  const auto before = machine.cpu().debug_state();
  auto expected = before.cpu;
  ++expected.r[0];
  ++expected.r[4];
  auto progress = await_ack(machine);
  const auto saved = machine.checkpoint();
  const auto stale = ticket(progress);
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(stale)); }));
    progress = machine.advance_reference_access(ticket(machine));
    require(!machine.bus().interrupt_request(0), "The admitted timer acknowledgement clears once");
    expect(progress,
           {.kind = DspNativeAccessKind::write,
            .target = {.space = DspSpace::x, .address = destination},
            .value = first,
            .ordinal = 2,
            .pc = before.cpu.pc});
    raise_timer(machine);
    progress = machine.advance_reference_access(ticket(progress));
    expect(progress,
           {.kind = DspNativeAccessKind::write,
            .target = {.space = DspSpace::y, .address = destination + 1},
            .value = second,
            .ordinal = 3,
            .pc = before.cpu.pc});
    require(machine.cpu().debug_state() == before,
            "The first vector's real write does not publish speculative register retirement");
    machine.bus().write({.space = DspSpace::x, .address = destination, .value = third});
    progress = machine.advance_reference_access(ticket(progress));
    const auto after = machine.cpu().debug_state();
    require(progress.retired && after.cpu == expected &&
                after.native_instructions == before.native_instructions + 2 &&
                after.stack_high == before.stack_high && after.stack_low == before.stack_low,
            "Both vector instructions retire once with independent fast-entry register/stack "
            "expectations");
    require(machine.bus().read({.space = DspSpace::x, .address = destination}) == third &&
                machine.bus().read({.space = DspSpace::y, .address = destination + 1}) == second &&
                machine.bus().interrupt_request(0).has_value(),
            "Recomputation repeats neither the completed X store nor acknowledgement of the "
            "reasserted source");
  }
}

void higher_priority_after_selection() {
  DspMachine machine;
  DspProgram code;
  code.emit("movep #>$10a9,x:<<$ffff91");
  code.immediate("sr", unmasked_status);
  code.emit("nop");
  code.install(machine.bus());
  machine.bus().shi().observe_spi({.mosi = false});
  vector_words(machine, {0U, 0U});
  machine.bus().write({.address = shi_vector});
  machine.bus().write({.address = shi_vector + 1});
  machine.cpu().step();
  machine.cpu().step();
  raise_timer(machine);
  machine.bus().write({.space = DspSpace::x, .address = priorities, .value = both_priorities});
  static_cast<void>(await_ack(machine));
  send(machine.bus().shi(), first);
  const auto higher = machine.bus().interrupt_request(0);
  require(higher && higher->vector == shi_vector && higher->level == 1,
          "Actual SHI input raises a higher-priority source after the timer vector was captured");
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    require(machine.advance_reference_access(ticket(machine)).retired,
            "FM2.3.2.6: a later higher-priority request does not replace the chosen interrupt");
    const auto pending = machine.bus().interrupt_request(0);
    require(pending && pending->vector == shi_vector && pending->level == 1,
            "Acknowledging the latched timer preserves the newer higher-priority SHI source");
    const auto shi =
        await_ack(machine, {.address = shi_vector, .relative = shi_vector, .level = 1});
    require(machine.advance_reference_access(ticket(shi)).retired &&
                machine.bus().interrupt_request(0).has_value() &&
                machine.bus().read({.space = DspSpace::x, .address = fifo}) == first,
            "Actual SHI acknowledgement neither drains the FIFO nor clears its level source");
    machine.bus().write({.space = DspSpace::x, .address = priorities, .value = 0x100});
    require(!machine.bus().interrupt_request(0),
            "Only the originally chosen timer flag was cleared");
  }
}

void fifo_vector() {
  DspMachine machine;
  DspProgram code;
  code.emit("movep #>$a9,x:<<$ffff91");
  code.immediate("r0", destination);
  code.immediate("x1", third);
  code.immediate("sr", unmasked_status);
  code.emit("nop");
  code.install(machine.bus());
  machine.bus().shi().observe_spi({.mosi = false});
  vector_moves(machine, {"movep x:<<$ffff94,x0", "move x0,x:(r0)+"});
  for (unsigned index = 0; index < 4; ++index) { machine.cpu().step(); }
  send(machine.bus().shi(), first);
  send(machine.bus().shi(), second);
  raise_timer(machine);
  const auto before = machine.cpu().debug_state();
  auto progress = await_ack(machine);
  progress = machine.advance_reference_access(ticket(progress));
  expect(progress,
         {.kind = DspNativeAccessKind::read,
          .target = {.space = DspSpace::x, .address = fifo},
          .ordinal = 2,
          .pc = before.cpu.pc});
  progress = machine.advance_reference_access(ticket(progress));
  require(completed(progress).value == first, "The vector captures one actual FIFO word");
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    require(machine.bus().read({.space = DspSpace::x, .address = fifo}) == second,
            "The checkpoint retains the next FIFO word after the vector's completed read");
    progress = machine.advance_reference_access(ticket(machine));
    const auto after = machine.cpu().debug_state();
    require(progress.retired && after.cpu.pc == before.cpu.pc &&
                after.cpu.x == (std::uint64_t{third} << word_bits) + first &&
                after.cpu.r[0] == destination + 1 &&
                after.native_instructions == before.native_instructions + 2 &&
                machine.bus().read({.space = DspSpace::x, .address = destination}) == first,
            "Second-vector dataflow uses the first vector's captured word across trusted restores");
    require(fault([&] {
              static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = fifo}));
            }).contains("empty"),
            "Vector recomputation fabricates or consumes no further FIFO data");
  }
}

void long_vectors() {
  constexpr unsigned base = 0x100;
  constexpr unsigned handler = 0x300;
  constexpr unsigned short_jsr = 0x0d0000;
  constexpr unsigned absolute_jsr = 0x0bf080;
  constexpr unsigned original_status = 0xc20455;
  constexpr unsigned cleared = 0x38f00; // FM5.4: scaling, saturation, loop and interrupt masks.
  const std::array forms{std::array{short_jsr + handler, 0U},
                         std::array{0U, short_jsr + handler},
                         std::array{absolute_jsr, handler}};
  for (unsigned form = 0; form < forms.size(); ++form) {
    DspMachine machine;
    DspProgram code;
    code.immediate("vba", base);
    code.immediate("sr", original_status);
    code.emit("nop");
    code.install(machine.bus());
    vector_words(machine, forms[form], base);
    machine.bus().write({.address = handler, .value = 4}); // FM RTI single-word opcode.
    machine.cpu().step();
    machine.cpu().step();
    raise_timer(machine);
    const auto before = machine.cpu().debug_state();
    static_cast<void>(await_ack(machine, {.address = base + timer_vector}));
    const auto saved = machine.checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      machine.restore(saved);
      vector_words(machine, {0U, 0U}, base);
      const auto progress = machine.advance_reference_access(ticket(machine));
      const auto after = machine.cpu().debug_state();
      require(progress.retired && after.cpu.pc == handler && after.cpu.sp == 1 &&
                  after.stack_high[1] == before.cpu.pc && after.stack_low[1] == original_status &&
                  after.cpu.sr == ((original_status & ~cleared) | 0x100U) &&
                  after.native_instructions == before.native_instructions + (form == 1 ? 2U : 1U),
              "Captured vector/extension words produce the literal long-entry frame, PC, mask and "
              "count");
      execute(machine);
      require(machine.cpu().state() == before.cpu,
              "An actual RTI restores the independently pinned foreground state");
    }
  }
}

void rejected_admissions() {
  DspMachine machine;
  DspProgram code;
  code.immediate("sr", unmasked_status);
  code.emit("nop");
  code.install(machine.bus());
  machine.cpu().step();
  machine.bus().write({.address = timer_vector});
  raise_timer(machine);
  const auto before = machine.cpu().debug_state();
  auto progress = machine.begin_reference_interrupt(boundaries);
  progress = machine.advance_reference_access(ticket(progress));
  const auto missing = machine.checkpoint();
  require(fault([&] {
            static_cast<void>(machine.advance_reference_access(ticket(progress)));
          }).contains("Unknown DSP memory read at P:0x000055") &&
              machine.bus().interrupt_request(0).has_value() &&
              machine.cpu().debug_state() == before,
          "A missing second vector word fails before acknowledgement and frame mutation");
  static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
  machine.restore(missing);
  machine.bus().write({.address = timer_vector + 1});
  progress = machine.advance_reference_access(ticket(machine));
  const auto ack = machine.checkpoint();
  machine.bus().write({.space = DspSpace::x, .address = timer_control});
  const auto failure =
      fault([&] { static_cast<void>(machine.advance_reference_access(ticket(progress))); });
  require(failure.contains("interrupt-acknowledge vector=0x000054") && failure.contains("pc=") &&
              failure.contains("omr=") && machine.cpu().debug_state() == before,
          "A withdrawn source rejects at its actual acknowledgement with execution context");
  machine.restore(ack);
  require(machine.advance_reference_access(ticket(machine)).retired &&
              !machine.bus().interrupt_request(0),
          "Trusted restore recovers both captured vector words and the acknowledged source");
}

void deferred_entry() {
  DspMachine machine;
  DspProgram code;
  code.immediate("sr", unmasked_status);
  code.emit("rep #2");
  code.emit("nop");
  code.emit("nop");
  code.install(machine.bus());
  vector_words(machine, {0U, 0U});
  static_cast<void>(
      fault([&] { static_cast<void>(machine.begin_reference_interrupt(boundaries)); }));
  raise_timer(machine);
  const auto masked = machine.cpu().debug_state();
  static_cast<void>(
      fault([&] { static_cast<void>(machine.begin_reference_interrupt(boundaries)); }));
  require(machine.cpu().debug_state() == masked && !machine.reference_pending(),
          "A masked source cannot start entry or mutate the healthy reference owner");
  machine.bus().write({.space = DspSpace::x, .address = timer_control});
  machine.cpu().step();
  machine.cpu().step();
  raise_timer(machine);
  const auto repeated = machine.cpu().debug_state();
  static_cast<void>(
      fault([&] { static_cast<void>(machine.begin_reference_interrupt(boundaries)); }));
  require(machine.cpu().debug_state() == repeated && !machine.reference_pending(),
          "Accepted-entry suspension respects an active native REP's interrupt deferral");
  execute(machine);
  execute(machine);
  require(finish(machine, machine.begin_reference_interrupt(boundaries)).retired,
          "The pending source enters after the actual legal REP body has completed");
}

void native_vector_failure() {
  DspMachine machine;
  DspProgram code;
  code.immediate("sr", unmasked_status);
  code.emit("nop");
  code.install(machine.bus());
  machine.cpu().step();
  vector_words(machine, {4U, 0U}); // FM2.3.2.5 excludes RTI in a fast vector.
  raise_timer(machine);
  const auto before = machine.cpu().debug_state();
  const auto progress = machine.begin_reference_interrupt(boundaries);
  const auto saved = machine.checkpoint();
  require(fault([&] {
            static_cast<void>(finish(machine, progress));
          }).contains("Unsupported reference interrupt vector instruction") &&
              !machine.bus().interrupt_request(0) &&
              machine.cpu().debug_state().native_instructions == before.native_instructions,
          "Invalid vector execution fails after its real acknowledgement without instruction "
          "retirement");
  static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
  machine.restore(saved);
  vector_words(machine, {0U, 0U});
  require(finish(machine, {.pending = machine.reference_pending()}).retired &&
              machine.cpu().state() == before.cpu,
          "Trusted prefetch restore recovers entry after repairing only the unconsumed own vector");
}
}

void interrupt_checks() {
  fast_stores_and_reassertion();
  fifo_vector();
  higher_priority_after_selection();
  long_vectors();
  rejected_admissions();
  deferred_entry();
  native_vector_failure();
}
}
