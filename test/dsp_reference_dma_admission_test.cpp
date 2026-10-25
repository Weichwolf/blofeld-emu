// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp56kEmu/assembler.h"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::DspProgram;
using blofeld::test::number;
using blofeld::test::require;
namespace ref = blofeld::test::reference;
namespace dma = blofeld::test::dma_test;
namespace banks = blofeld::test::banks;
using blofeld::test::native_access::fault;
constexpr unsigned first = 0x123456, second = 0x234567, third = 0x345678;
constexpr unsigned source = 0x660, destination = 0x900, unknown = 0x1800;
constexpr unsigned timer_control = 0xffff8f, timer_priority = 0xfffffe;
constexpr unsigned timer_match = 0x200000;
constexpr unsigned timer_vector = 0x54;
constexpr unsigned maximum_accesses = 16;

struct Script {
  std::vector<std::string> body;
  unsigned source_address = source;
  unsigned status = blofeld::test::masked_status;
  bool shi = false;
  std::optional<unsigned> source_data = first;
};

class Rig {
public:
  explicit Rig(const Script &script) {
    DspProgram program;
    program.immediate("sr", script.status);
    program.immediate("x0", second);
    for (const auto store : {DspStore{.address = dma::source0, .value = script.source_address},
                             DspStore{.address = dma::destination0, .value = destination},
                             DspStore{.address = dma::counter0},
                             DspStore{.address = dma::control0, .value = dma::word_copy}}) {
      program.emit("movep #>" + number(store.value) + ",x:<<" + number(store.address));
    }
    if (script.shi) { program.emit("movep #>$a9,x:<<$ffff91"); }
    work = program.pc();
    for (const auto &line : script.body) { program.emit(line); }
    program.emit("nop");
    program.install(machine.bus());
    if (script.source_data) {
      machine.bus().write(
          {.space = DspSpace::x, .address = script.source_address, .value = *script.source_data});
    }
    machine.bus().write({.space = DspSpace::y, .address = destination, .value = third});
    machine.bus().shi().observe_spi({.mosi = false});
    static_cast<void>(machine.advance_dma(banks::inputs({.index = 0}), banks::use()));
    for (unsigned count = 0; machine.cpu().state().pc != work && count < maximum_accesses;
         ++count) {
      machine.cpu().step();
    }
    require(machine.cpu().state().pc == work,
            "Own native caller configures the real DMA registers");
  }

  DspReferenceDmaProgress sample(const DspDmaGrant grant = DspDmaGrant::wait) {
    const auto result = machine.advance_reference_dma(
        banks::inputs({.index = next, .requests = 1, .grant = grant}), ref::ticket(machine));
    ++next;
    return result;
  }

  void seek(const DspNativeAccessKind kind) {
    static_cast<void>(machine.begin_reference_instruction(ref::boundaries));
    for (unsigned count = 0; count < maximum_accesses; ++count) {
      const auto pending = machine.reference_pending();
      if (!pending) { throw Fault("Expected an actual pending native memory effect"); }
      const auto *access = std::get_if<DspNativeAccess>(&pending.value().effect);
      if (access == nullptr) {
        throw Fault("Expected word/bit input rather than a control marker");
      }
      if (access->kind == kind) { return; }
      const auto result = sample();
      require(result.core && !result.core->retired,
              "Independent prefix accesses proceed while the DMA source waits");
    }
    throw Fault("Own caller failed to reach its bounded native access");
  }

  void execute() {
    static_cast<void>(machine.begin_reference_instruction(ref::boundaries));
    for (unsigned count = 0; count < maximum_accesses; ++count) {
      const auto result = sample();
      if (!result.core) { throw Fault("This declared independent prefix cannot stall"); }
      if (result.core.value().retired) { return; }
    }
    throw Fault("Own caller failed to retire its bounded native instruction");
  }

  DspMachine machine;
  std::uint64_t next = 1;
  unsigned work = 0;
};

void source_conflict_uses_real_ticket() {
  Rig rig({.body = {"move x0,x:>$660"}});
  rig.seek(DspNativeAccessKind::write);
  const auto before = rig.machine.cpu().debug_state();
  const auto consumed = ref::ticket(rig.machine);
  const auto result = rig.sample(DspDmaGrant::available);
  require(result.dma.phase && result.dma.phase->kind == DspDmaPhaseKind::source_read &&
              !result.dma.word && !result.dma.core_stall && result.core && result.core->retired &&
              rig.machine.bus().read({.space = DspSpace::x, .address = source}) == second &&
              rig.machine.bus().read({.space = DspSpace::y, .address = destination}) == third &&
              rig.machine.cpu().debug_state().native_instructions == before.native_instructions + 1,
          "An actual pending store wins against a fresh DMA read without caller footprint claims");
  const auto rejected = fault([&] {
    static_cast<void>(rig.machine.advance_reference_dma(
        banks::inputs({.index = rig.next, .requests = 1}), consumed));
  });
  require(rejected.contains("current owner ticket"), "A consumed ticket cannot sample DMA again");
  const auto read =
      rig.machine.advance_dma(banks::inputs({.index = rig.next, .requests = 1}), banks::use());
  require(!read.word && rig.machine.bus().read({.space = DspSpace::x, .address = dma::status}) ==
                            dma::zero_active,
          "The rejected ticket leaves the same DMA index available for a real source read");
  const auto write =
      rig.machine.advance_dma(banks::inputs({.index = rig.next + 1, .requests = 1}), banks::use());
  require(write.word && write.word->value == second &&
              rig.machine.bus().read({.space = DspSpace::y, .address = destination}) == second,
          "The delayed DMA read captures the actual preceding native store value");
}

void held_word_stalls_native_and_replays() {
  Rig rig({.body = {"move x0,x:>$660", "move #>$345678,x0", "move x0,x:>$660", "move y:>$900,x0"}});
  static_cast<void>(rig.machine.begin_reference_instruction(ref::boundaries));
  const auto captured = rig.sample(DspDmaGrant::available);
  require(captured.dma.phase && captured.dma.phase->kind == DspDmaPhaseKind::source_read &&
              !captured.dma.word && captured.core && !captured.core->retired,
          "Independent real opcode fetch and DMA source capture share the declared sample");
  const auto extension = rig.sample();
  require(extension.dma.phase && extension.dma.phase->value == first && extension.core &&
              !extension.core->retired,
          "A held DMA destination permits the independent real extension fetch");
  const auto store = rig.sample();
  require(store.core && store.core->retired && !store.dma.core_stall &&
              rig.machine.bus().read({.space = DspSpace::x, .address = source}) == second,
          "A real source overwrite cannot replace the previously captured DMA word");
  rig.execute();
  rig.execute();
  require(rig.machine.bus().read({.space = DspSpace::x, .address = source}) == third,
          "A second native overwrite remains distinct from both the held word and first store");
  rig.seek(DspNativeAccessKind::read);
  const auto before = rig.machine.cpu().debug_state();
  const auto old_ticket = ref::ticket(rig.machine);
  const auto blocked = rig.sample();
  require(blocked.dma.core_stall && !blocked.dma.word && !blocked.core &&
              ref::ticket(rig.machine) == old_ticket && rig.machine.cpu().debug_state() == before,
          "A held conflicting destination enforces the actual native stall, including global wait");
  const auto saved = rig.machine.checkpoint();
  const auto next = rig.next;
  std::optional<DspDebugState> expected;
  for (unsigned replay = 0; replay < 3; ++replay) {
    if (replay != 0) {
      rig.machine.restore(saved);
      rig.next = next;
      const auto stale = fault([&] {
        static_cast<void>(rig.machine.advance_reference_dma(
            banks::inputs({.index = rig.next, .requests = 1}), old_ticket));
      });
      require(stale.contains("current owner ticket"), "Restore retires the old native capability");
    }
    const auto written = rig.sample(DspDmaGrant::available);
    require(written.dma.word && written.dma.word->value == first && written.dma.core_stall &&
                !written.core && rig.machine.cpu().debug_state() == before &&
                rig.machine.bus().read({.space = DspSpace::y, .address = destination}) == first,
            "A completed held DMA write occupies this sample and cannot also admit the core load");
    const auto loaded = rig.sample(DspDmaGrant::available);
    require(
        !loaded.dma.phase && !loaded.dma.core_stall && loaded.core && loaded.core->retired &&
            ref::completed(*loaded.core).value == first && rig.machine.cpu().state().x == first &&
            rig.machine.bus().read({.space = DspSpace::x, .address = source}) == third &&
            rig.machine.bus().read({.space = DspSpace::x, .address = dma::status}) == dma::all_done,
        "The next sample admits the retained native load exactly once with independent data");
    if (!expected) { expected = rig.machine.cpu().debug_state(); }
    require(rig.machine.cpu().debug_state() == *expected,
            "Both trusted continuations retain full native state, held data and DMA phase");
  }
}

void validation_does_not_sample() {
  Rig rig({.body = {"move x0,x:>$660"}});
  Rig foreign({.body = {"move x0,x:>$660"}});
  rig.seek(DspNativeAccessKind::write);
  foreign.seek(DspNativeAccessKind::write);
  const auto actual = ref::ticket(rig.machine);
  for (const auto &ticket : {DspReferenceTicket{}, ref::ticket(foreign.machine)}) {
    const auto text = fault([&] {
      static_cast<void>(rig.machine.advance_reference_dma(
          banks::inputs({.index = rig.next, .requests = 1}), ticket));
    });
    require(text.contains("current owner ticket") && ref::ticket(rig.machine) == actual,
            "Default/foreign capabilities cannot advance either participant");
  }
  auto invalid = banks::inputs({.index = rig.next, .requests = 1});
  invalid.grant = DspDmaGrant::unknown;
  require(fault([&] {
            static_cast<void>(rig.machine.advance_reference_dma(invalid, actual));
          }).contains("reference cycle/grant"),
          "Invalid DMA input cannot consume a valid native capability");
  require(ref::ticket(rig.machine) == actual,
          "The same actual store and DMA index remain available after all rejected inputs");
  const auto accepted = rig.sample(DspDmaGrant::available);
  require(accepted.core && accepted.core->retired,
          "Actual admission succeeds at the same reference index after invalid inputs");
  const DspRunContract run{
      .clock = {.extal_hz = 12000000,
                .timebase_hz = 540000000,
                .switch_timing = DspClockSwitch::reference_instruction_boundary},
      .timing = DspRunTiming::native_internal_estimates};
  DspMachine clocked(run);
  require(fault([&] {
            static_cast<void>(clocked.advance_reference_dma(banks::inputs({.index = 0}), {}));
          }).contains("unclocked reference owner"),
          "This reference interface cannot bypass automatic native DMA timing rejection");
}

void unknown_native_read_retains_dma_prefix() {
  Rig rig({.body = {"move y:>$1800,x0"}});
  rig.seek(DspNativeAccessKind::read);
  const auto saved = rig.machine.checkpoint();
  const auto next = rig.next;
  const auto text = fault([&] { static_cast<void>(rig.sample(DspDmaGrant::available)); });
  require(text.contains("Unknown DSP memory read at Y:0x001800") && text.contains("pc=") &&
              text.contains("omr="),
          "Failed actual native admission retains address/PC/OMR and quarantines the core");
  auto copy = rig.machine.bus();
  const auto held =
      copy.advance_dma(banks::inputs({.index = next + 1, .requests = 1}), banks::use());
  require(held.word && held.word->value == first &&
              rig.machine.bus().read({.space = DspSpace::y, .address = destination}) == third,
          "Detached observation proves the real DMA source committed before the failed core read");
  static_cast<void>(fault([&] { static_cast<void>(rig.machine.checkpoint()); }));
  static_cast<void>(fault([&] { static_cast<void>(rig.machine.reference_pending()); }));
  static_cast<void>(fault([&] { rig.machine.cpu().step(); }));
  rig.machine.restore(saved);
  rig.next = next;
  rig.machine.bus().write({.space = DspSpace::y, .address = unknown, .value = second});
  const auto recovered = rig.sample(DspDmaGrant::available);
  require(recovered.core && recovered.core->retired && rig.machine.cpu().state().x == second &&
              !recovered.dma.word,
          "Only trusted restore plus newly supplied input recovers the failed admission");
}

void dma_failure_keeps_the_native_effect_pending() {
  Rig rig({.body = {"move x0,y:>$1800"}, .source_data = std::nullopt});
  rig.seek(DspNativeAccessKind::write);
  const auto pending = ref::ticket(rig.machine);
  const auto before = rig.machine.cpu().debug_state();
  const auto text = fault([&] { static_cast<void>(rig.sample(DspDmaGrant::available)); });
  require(text.contains("Unknown DSP memory read at X:0x000660") && text.contains("pc=") &&
              text.contains("omr=") && ref::ticket(rig.machine) == pending &&
              rig.machine.cpu().debug_state() == before,
          "A failed DMA read cannot consume or quarantine an unattempted native store");
  rig.machine.bus().write({.space = DspSpace::x, .address = source, .value = first});
  const auto admitted = rig.sample(DspDmaGrant::available);
  require(admitted.core && admitted.core->retired && !admitted.dma.word &&
              rig.machine.bus().read({.space = DspSpace::y, .address = unknown}) == second,
          "Newly supplied source data permits the same DMA index and native ticket to proceed");
}

void unqualified_geometry_and_fifo() {
  constexpr unsigned upper = 0xc000;
  Rig rig({.body = {"move x0,x:>$c400"}, .source_address = upper});
  rig.seek(DspNativeAccessKind::write);
  const auto ticket = ref::ticket(rig.machine);
  const auto text = fault([&] { static_cast<void>(rig.sample(DspDmaGrant::available)); });
  require(text.contains("Unqualified DSP core/DMA bank contention") && text.contains("pc=") &&
              text.contains("omr=") && ref::ticket(rig.machine) == ticket,
          "Same-eight-K/different-one-K arbitration cannot fabricate a safe admission");
  static_cast<void>(
      rig.machine.advance_dma(banks::inputs({.index = rig.next, .requests = 1}), banks::use()));
  ++rig.next;
  const auto independent = rig.sample();
  require(independent.core && independent.core->retired && independent.dma.phase &&
              independent.dma.phase->value == first &&
              rig.machine.bus().read({.space = DspSpace::x, .address = upper}) == first,
          "No cycle was committed by unqualified arbitration; a later distinct bank is admissible");
  Rig fifo({.body = {"movep x:<<$ffff94,x0"}, .shi = true});
  ref::send(fifo.machine.bus().shi(), first);
  ref::send(fifo.machine.bus().shi(), second);
  fifo.seek(DspNativeAccessKind::read);
  const auto pending = ref::ticket(fifo.machine);
  require(fault([&] {
            static_cast<void>(fifo.sample(DspDmaGrant::available));
          }).contains("peripheral") &&
              ref::ticket(fifo.machine) == pending,
          "Unqualified peripheral sharing rejects before DMA sampling or FIFO consumption");
  const auto admitted = fifo.machine.advance_reference_access(pending);
  require(
      admitted.retired && fifo.machine.cpu().state().x == first &&
          fifo.machine.bus().read({.space = DspSpace::x, .address = ref::fifo}) == second,
      "The real FIFO words survive rejection in order; ordinary explicit admission stays valid");
}

void control_effects_remain_explicit() {
  Rig bits({.body = {"bset #2,x:<<$ffff8f"}});
  bits.seek(DspNativeAccessKind::bit);
  require(fault([&] { static_cast<void>(bits.sample()); }).contains("indivisible native bit"),
          "An indivisible peripheral bit effect is not a fabricated word bus phase");
  require(
      bits.machine.bus().timers().read(timer_control, 4) == 0 &&
          bits.machine.advance_reference_access(ref::ticket(bits.machine)).retired &&
          bits.machine.bus().timers().read(timer_control, 4) == 4,
      "The rejected bit operation remains pending and executes only through explicit admission");
  Rig reset({.body = {"reset"}});
  static_cast<void>(reset.machine.begin_reference_instruction(ref::boundaries));
  require(reset.sample().core.has_value(), "The RESET opcode itself is a real memory fetch");
  require(fault([&] { static_cast<void>(reset.sample()); }).contains("reset or interrupt"),
          "Peripheral reset cannot be smuggled into a reference memory sample");
  require(reset.machine.bus().read({.space = DspSpace::x, .address = dma::control0}) ==
              dma::word_copy,
          "Rejected control admission cannot reset the active DMA controller");
}

void interrupt_acknowledgement_is_not_memory() {
  constexpr unsigned ordinary = 0xc00000;
  Rig rig({.body = {"nop"}, .status = ordinary});
  auto &bus = rig.machine.bus();
  bus.write({.space = DspSpace::x, .address = timer_priority, .value = 0x100});
  const dsp56k::Assembler assembler;
  const auto word = assembler.assemble("move #<7,r2");
  require(word.success() && word.wordCount == 1, "Own fast timer vector must assemble");
  bus.write({.address = timer_vector, .value = word.word[0]});
  bus.write({.address = timer_vector + 1});
  bus.timers().write({.address = timer_control - 1});
  bus.timers().write({.address = timer_control - 2});
  bus.timers().write({.address = timer_control, .value = 7});
  bus.timers().advance_clk2(1);
  static_cast<void>(rig.machine.begin_reference_interrupt(ref::boundaries));
  require(rig.sample().core.has_value() && rig.sample().core.has_value(),
          "Both actual vector words pass memory admission while the DMA source waits");
  require(fault([&] { static_cast<void>(rig.sample()); }).contains("reset or interrupt") &&
              bus.timers().read(timer_control, timer_match) == timer_match,
          "A rejected acknowledgement neither clears the timer nor samples a DMA phase");
  const auto actual = rig.machine.advance_reference_access(ref::ticket(rig.machine));
  require(
      actual.retired && rig.machine.cpu().state().r[2] == 7 &&
          bus.timers().read(timer_control, timer_match) == 0,
      "Existing explicit interrupt admission retains real acknowledgement and vector execution");
}
}

int main() {
  try {
    source_conflict_uses_real_ticket();
    held_word_stalls_native_and_replays();
    validation_does_not_sample();
    unknown_native_read_retains_dma_prefix();
    dma_failure_keeps_the_native_effect_pending();
    unqualified_geometry_and_fifo();
    control_effects_remain_explicit();
    interrupt_acknowledgement_is_not_memory();
    std::cout << "Native memory/DMA admission holds real tickets, data, failures and replay.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
