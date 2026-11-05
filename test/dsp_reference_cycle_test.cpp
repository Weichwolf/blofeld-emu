// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <string>

namespace {
using namespace blofeld;
namespace native = blofeld::test::dma_test;
namespace banks = blofeld::test::banks;
using blofeld::test::esai::rejects;
using blofeld::test::esai::require;
constexpr unsigned program = 0x100;
constexpr unsigned initial_r0 = 0x7e;
constexpr unsigned copied_r0 = 0x34;
constexpr unsigned program_source = 0x800;
constexpr unsigned program_copy = 0x88024a;
constexpr unsigned first = 0x123456;
constexpr unsigned second = 0x234567;
constexpr unsigned third = 0x345678;
constexpr unsigned bound = 32;
constexpr unsigned reset_omr = 0x305;
constexpr std::uint64_t period = 45;
constexpr auto model = OwnDspNativeCycleModel::isolated_effect;
// Independent FM MOVE/MOVEM absolute, MOVEP qq, JMP and short-immediate fields.
constexpr unsigned long_r0 = 0x60f400;
constexpr unsigned immediate_x0 = 0x44f400;
constexpr unsigned load_y_x0 = 0x4cf000;
constexpr unsigned store_x_x0 = 0x447000;
constexpr unsigned long_jump = 0x0af080;
constexpr unsigned program_load = 0x07f084;
constexpr unsigned short_move = 0x303400;
constexpr unsigned fifo_load = 0x044494;
constexpr unsigned fifo = 0xffff94;
constexpr unsigned shi_control = 0xffff91;
constexpr unsigned shi_configuration = 0xa9;

void healthy(const dsp56300::InstructionProgress &progress) {
  require(progress.fault.kind == dsp56300::FaultKind::none,
          "Actual native cycle fault pc=" + hex(progress.fault.pc, 6) +
              " address=" + hex(progress.fault.address, 6));
}

dsp56300::PendingInstruction pending(const OwnDspMachine &machine) {
  const auto value = machine.cpu().pending_instruction();
  if (!value) { throw Fault("Expected an actual pending native bus-cycle effect"); }
  return *value;
}

void finish(OwnDspMachine &machine) {
  const auto ticks = machine.clock().elapsed_ticks();
  for (unsigned effect = 0; effect < bound; ++effect) {
    const auto result = machine.advance(pending(machine).ticket);
    healthy(result);
    if (result.retired) {
      require(machine.clock().elapsed_ticks() == ticks,
              "Unclocked publication/retirement cannot invent extra reference periods");
      return;
    }
  }
  throw Fault("Actual native completion exceeded its bounded effect count");
}

void expect_data(const OwnDspMachine &machine, const dsp56300::InstructionEffect expected) {
  const auto access = pending(machine).effect;
  require(access.kind == expected.kind && access.space == expected.space &&
              access.address == expected.address && access.pc == program && access.omr == reset_omr,
          "Native occupancy comes from the actual effect space, address, direction and context");
}

void clocked(const OwnDspMachine &machine, const std::uint64_t before) {
  require(machine.clock().elapsed_ticks() == before + period,
          "Each admitted reference cycle advances one independent 45-tick period");
}

std::string failure_cause(const OwnDspMachine &machine) {
  const auto failure = machine.io().failure();
  if (!failure || !failure->cause) { throw Fault("Missing actual native I/O failure cause"); }
  try {
    std::rethrow_exception(failure->cause);
  } catch (const std::exception &error) { return error.what(); }
}

struct Fixture {
  Fixture(const std::span<const std::uint32_t> words, const banks::Transfer transfer = {}) {
    native::NativeProgram setup;
    setup.word(long_r0);
    setup.word(initial_r0);
    for (const auto store :
         {DspStore{.space = DspSpace::x, .address = native::source0, .value = transfer.source},
          DspStore{
              .space = DspSpace::x, .address = native::destination0, .value = transfer.destination},
          DspStore{.space = DspSpace::x, .address = native::counter0},
          DspStore{.space = DspSpace::x, .address = native::control0, .value = transfer.control}}) {
      setup.write(store);
    }
    setup.word(long_jump);
    setup.word(program);
    native::reset_native(machine, setup);
    for (unsigned i = 0; i < words.size(); ++i) {
      machine.bus().write({.address = program + i, .value = words[i]});
    }
    machine.bus().write({.space = DspSpace::x, .address = native::ring_start, .value = first});
    machine.bus().write({.space = DspSpace::y, .address = native::destination, .value = second});
    for (unsigned i = 0; i < 6; ++i) { native::step_native(machine); }
    require(machine.cpu().registers().pc == program && machine.cpu().registers().r[0] == initial_r0,
            "Actual reset and literal MOVEP setup configure DMA before entering the RAM program");
    static_cast<void>(machine.advance_dma_reference_cycle(banks::inputs({.index = 0})));
  }

  void start_fetch(const unsigned words) {
    fetch = std::make_unique<OwnDspFetch>(machine);
    for (unsigned i = 0; i < words; ++i) {
      fetch->issue();
      fetch->capture();
    }
    healthy(fetch->begin());
  }

  void until_data(const dsp56300::EffectKind kind) {
    healthy(machine.begin_fetch());
    for (unsigned effect = 0; effect < bound; ++effect) {
      if (pending(machine).effect.kind == kind) { return; }
      const auto step = machine.advance(pending(machine).ticket);
      healthy(step);
      require(!step.retired, "The actual instruction must reach its requested data boundary");
    }
    throw Fault("Native instruction never reached its data effect");
  }

  void complete() {
    finish(machine);
    if (fetch) { fetch->synchronize(); }
  }

  OwnDspMachine machine{native::native_clocks};
  std::unique_ptr<OwnDspFetch> fetch;
};

OwnDspDmaCycleResult native_cycle(OwnDspMachine &machine, const banks::Cycle cycle) {
  const auto before = machine.clock().elapsed_ticks();
  auto result = machine.advance_dma_native_cycle(banks::inputs(cycle), model);
  clocked(machine, before);
  return result;
}

DspDmaBusResult dma_cycle(OwnDspMachine &machine, const banks::Cycle cycle) {
  const auto before = machine.clock().elapsed_ticks();
  auto result = machine.advance_dma_reference_cycle(banks::inputs(cycle));
  clocked(machine, before);
  return result;
}

OwnDspFetchCycleResult
fetch_cycle(Fixture &fixture, const banks::Cycle cycle, const bool admit_native) {
  const auto before = fixture.machine.clock().elapsed_ticks();
  auto result = admit_native
                    ? fixture.fetch->advance_dma_capture_native(banks::inputs(cycle), model)
                    : fixture.fetch->advance_dma_capture(banks::inputs(cycle));
  clocked(fixture.machine, before);
  if (result.native && result.native->retired) { fixture.fetch->synchronize(); }
  return result;
}

void held_destination_load() {
  const std::array words{load_y_x0, native::destination};
  Fixture fixture(words);
  auto &machine = fixture.machine;
  fixture.until_data(dsp56300::EffectKind::read);
  expect_data(machine,
              {.kind = dsp56300::EffectKind::read,
               .space = dsp56300::Space::y,
               .address = native::destination});
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto captured = dma_cycle(machine, {.index = 1, .requests = 1});
  require(captured.phase && captured.phase->kind == DspDmaPhaseKind::source_read && !captured.word,
          "Actual DMA captures its source before the selected native load");
  const auto held = native_cycle(machine, {.index = 2, .requests = 1, .grant = DspDmaGrant::wait});
  require(held.dma.core_stall && !held.dma.word && !held.native &&
              machine.cpu().registers() == before && machine.cpu().progress() == progress,
          "A held DMA destination prevents the actual native read and all CPU publication");
  Fixture foreign(words);
  foreign.until_data(dsp56300::EffectKind::read);
  const auto stale = pending(foreign.machine).ticket;

  const auto ticks = machine.clock().elapsed_ticks();
  static_cast<void>(rejects([&] { static_cast<void>(machine.advance(stale)); }));
  require(machine.clock().elapsed_ticks() == ticks && machine.bus().last_dma_cycle() == 2,
          "Foreign effect capabilities cannot sample another DMA cycle");
  machine.bus().write({.space = DspSpace::x, .address = native::ring_start, .value = third});
  const auto written = native_cycle(machine, {.index = 3, .requests = 1});
  require(written.dma.word && written.dma.word->value == first && written.dma.core_stall &&
              !written.native && machine.cpu().registers() == before &&
              machine.cpu().progress() == progress,
          "Actual destination commit preserves the latched source and still holds the native CPU");
  const auto loaded = native_cycle(machine, {.index = 4});
  require(!loaded.dma.core_stall && loaded.native && !loaded.native->retired &&
              machine.cpu().progress() == progress,
          "A later native data read is an actual effect, separate from result "
          "publication/retirement");
  fixture.complete();
  auto expected = before;
  expected.x = first;
  expected.pc = program + 2;
  require(machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions == progress.instructions + 1,
          "Independent complete registers and one retirement observe the committed DMA word");
}

void native_store_wins_source() {
  const std::array words{immediate_x0, second, store_x_x0, native::ring_start};
  Fixture fixture(words);
  auto &machine = fixture.machine;
  native::step_native(machine);
  fixture.until_data(dsp56300::EffectKind::write);
  const auto actual = pending(machine).effect;
  require(actual.kind == dsp56300::EffectKind::write && actual.space == dsp56300::Space::x &&
              actual.address == native::ring_start && actual.value == second &&
              actual.pc == program + 2,
          "The actual pending native store supplies its own address and source value");
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto stored = native_cycle(machine, {.index = 1, .requests = 1});
  require(
      stored.dma.phase && stored.dma.phase->kind == DspDmaPhaseKind::source_read &&
          !stored.dma.word && !stored.dma.core_stall && stored.native && !stored.native->retired &&
          machine.bus().read({.space = DspSpace::x, .address = native::ring_start}) == second,
      "A genuine native source-bank store defers DMA's source capture and writes its real value");
  static_cast<void>(dma_cycle(machine, {.index = 2, .requests = 1}));
  machine.bus().write({.space = DspSpace::x, .address = native::ring_start, .value = third});
  const auto copied = dma_cycle(machine, {.index = 3, .requests = 1});
  require(copied.word && copied.word->value == second &&
              machine.bus().read({.space = DspSpace::y, .address = native::destination}) ==
                  second &&
              machine.cpu().progress() == progress,
          "DMA retains the newer core source across later RAM mutation while CPU retirement "
          "remains held");
  fixture.complete();
  auto expected = before;
  expected.pc = program + 4;
  require(machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions == progress.instructions + 1,
          "Native store publishes only its independent sequential PC and single completion");
}

void fetch_wait_and_admit() {
  const std::array words{0U, 0U};
  Fixture fixture(words,
                  {.source = program_source, .destination = program, .control = program_copy});
  auto &machine = fixture.machine;
  machine.bus().write({.address = program_source, .value = short_move});
  fixture.fetch = std::make_unique<OwnDspFetch>(machine);
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto ticks = machine.clock().elapsed_ticks();
  const auto issued =
      fixture.fetch->advance_dma_pipeline(banks::inputs({.index = 1, .requests = 1}), false, model);
  clocked(machine, ticks);
  require(issued.dma.phase && issued.dma.phase->kind == DspDmaPhaseKind::source_read &&
              issued.issued_address == program && !issued.captured && !issued.native,
          "Actual DMA source capture coexists with F1 issue, without a fictitious P read");
  const auto held =
      fetch_cycle(fixture, {.index = 2, .requests = 1, .grant = DspDmaGrant::wait}, false);
  require(held.dma.core_stall && !held.captured && fixture.fetch->view().used == 0 &&
              machine.cpu().registers() == before && machine.cpu().progress() == progress,
          "A conflicting held DMA destination prevents the real F2 sample");

  const auto committed = fetch_cycle(fixture, {.index = 3, .requests = 1}, false);
  require(committed.dma.word && committed.dma.word->value == short_move &&
              committed.dma.core_stall && !committed.captured && fixture.fetch->view().used == 0,
          "The actual DMA destination write cannot also sample the conflicting P word");
  const auto fetched = fetch_cycle(fixture, {.index = 4}, false);
  require(fetched.captured && fixture.fetch->view().words.front().data == short_move &&
              machine.cpu().registers() == before && machine.cpu().progress() == progress,
          "A later F2 reads DMA's actual replacement before any native instruction starts");
  healthy(fixture.fetch->begin());
  require(pending(machine).effect.kind == dsp56300::EffectKind::move_update,
          "An already captured ordinary opcode needs CPU publication, not another P read");
  const auto published =
      fixture.fetch->advance_dma_pipeline(banks::inputs({.index = 5}), true, model);
  require(published.native && !published.native->retired && !published.dma.core_stall,
          "Captured instruction publication has no fabricated extra program-bus access");
  fixture.complete();
  auto expected = before;
  expected.pc = program + 1;
  expected.r[0] = copied_r0;
  require(machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions == progress.instructions + 1,
          "Actual publication and retirement execute the captured DMA-written word exactly once");
}

void simultaneous_fetch_and_redirect() {
  const std::array words{long_jump, program + 2, 0U};
  Fixture fixture(words);
  fixture.start_fetch(2);
  fixture.fetch->issue();
  const auto published = fixture.machine.advance(pending(fixture.machine).ticket);
  require(!published.retired &&
              pending(fixture.machine).effect.kind == dsp56300::EffectKind::retire,
          "A real JMP target publication leaves actual retirement pending");
  const auto before = fixture.machine.cpu().progress();
  const auto result = fetch_cycle(fixture, {.index = 1}, true);
  const auto view = fixture.fetch->view();
  require(result.native && result.native->retired && result.captured && view.used == 0 &&
              !view.pending_address && view.next_address == program + 2 &&
              fixture.machine.cpu().progress().instructions == before.instructions + 1,
          "Same-cursor JMP completion discards the younger word actually captured in that cycle");
}

void dual_port_preflight() {
  constexpr unsigned operand = 0x500;
  const std::array words{program_load, operand, 0U};
  Fixture fixture(words);
  auto &machine = fixture.machine;
  machine.bus().write({.address = operand, .value = first});
  fixture.start_fetch(2);
  expect_data(
      machine,
      {.kind = dsp56300::EffectKind::read, .space = dsp56300::Space::program, .address = operand});
  fixture.fetch->issue();
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto view = fixture.fetch->view();
  const auto ticks = machine.clock().elapsed_ticks();
  const auto error = rejects([&] { static_cast<void>(fetch_cycle(fixture, {.index = 1}, true)); });
  require(error.contains("overlap read P:") && error.contains("pc=0x000100") &&
              error.contains("omr=0x000305") && machine.cpu().registers() == before &&
              machine.cpu().progress() == progress && fixture.fetch->view() == view &&
              machine.clock().elapsed_ticks() == ticks && machine.bus().last_dma_cycle() == 0,
          "Unqualified simultaneous P reads reject before request sampling, clocks or CPU effects");
  static_cast<void>(fetch_cycle(fixture, {.index = 1}, false));
  const auto loaded = fixture.fetch->advance_dma_pipeline(banks::inputs({.index = 2}), true, model);
  require(loaded.native && !loaded.native->retired,
          "Separate native P-data admission remains genuine");
  fixture.complete();
  auto expected = before;
  expected.pc = program + 2;
  expected.x = first;
  require(
      machine.cpu().registers() == expected &&
          machine.cpu().progress().instructions == progress.instructions + 1,
      "Preflight refusal preserves the original instruction and its independently expected result");
}

void failed_required_read_keeps_dma_prefix() {
  constexpr unsigned unknown = 0x20;
  const std::array words{load_y_x0, unknown};
  Fixture fixture(words);
  auto &machine = fixture.machine;
  fixture.start_fetch(2);
  fixture.fetch->issue();
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto result = fetch_cycle(fixture, {.index = 1, .requests = 1}, true);
  const auto failure = machine.io().failure();
  const auto actual_fault = result.native ? result.native->fault : dsp56300::Fault{};
  require(result.native && result.native->fault.kind == dsp56300::FaultKind::memory_read &&
              result.native->fault.space == dsp56300::Space::y &&
              result.native->fault.address == unknown && result.native->fault.pc == program,
          "Required native data fault kind=" +
              std::to_string(static_cast<unsigned>(actual_fault.kind)) +
              " address=" + hex(actual_fault.address, 6) + " pc=" + hex(actual_fault.pc, 6));
  require(failure && failure->cause && failure->address == unknown && failure->pc == program,
          "Native failure preserves its real board I/O context");
  require(machine.cpu().registers() == before && machine.cpu().progress() == progress,
          "Failed native data cannot publish registers or retirement");
  require(result.dma.phase && result.dma.phase->kind == DspDmaPhaseKind::source_read &&
              result.captured && !fixture.fetch->view().words.front().data,
          "Earlier actual DMA and absent-value F2 remain an accepted prefix");
  require(machine.bus().access_peripheral_bit({.space = DspSpace::x,
                                               .address = native::status,
                                               .bit = 8,
                                               .operation = DspBitOperation::test}) == true &&
              machine.bus().read({.space = DspSpace::y, .address = native::destination}) == second,
          "Accepted DMA source remains active while its destination is still unchanged");
  require(failure_cause(machine).contains("Unknown DSP memory read at Y:0x000020"),
          "Failure preserves its actual bus operation/address");
  const auto ticks = machine.clock().elapsed_ticks();
  static_cast<void>(rejects(
      [&] { static_cast<void>(fixture.fetch->advance_dma_capture(banks::inputs({.index = 2}))); }));
  require(machine.clock().elapsed_ticks() == ticks && machine.bus().last_dma_cycle() == 1,
          "Faulted execution cannot silently advance another reference cycle");
  Fixture known_fixture(words);
  auto &known = known_fixture.machine;
  known_fixture.start_fetch(2);
  known_fixture.fetch->issue();
  const auto known_before = known.cpu().registers();
  const auto known_progress = known.cpu().progress();
  known.bus().write({.space = DspSpace::y, .address = unknown, .value = third});
  known.bus().write({.address = program + 2});
  known.bus().write({.address = program, .value = short_move});
  const auto loaded = fetch_cycle(known_fixture, {.index = 1, .requests = 1}, true);
  require(loaded.native && loaded.native->fault.kind == dsp56300::FaultKind::none &&
              !loaded.native->retired && loaded.captured,
          "The fresh owner accepts its actual data read and separately issued F2 address");
  known_fixture.complete();
  auto expected = known_before;
  expected.pc = program + 2;
  expected.x = third;
  require(known.cpu().registers() == expected &&
              known.cpu().progress().instructions == known_progress.instructions + 1,
          "Captured original load executes despite changed code and uses explicitly known data");
}

void protocol_preflight() {
  const std::array words{0U, 0U};
  Fixture fixture(words);
  fixture.start_fetch(1);
  Fixture foreign(words);
  foreign.start_fetch(1);
  const auto stale = pending(foreign.machine).ticket;
  const auto before = fixture.machine.cpu().registers();
  const auto progress = fixture.machine.cpu().progress();
  const auto ticks = fixture.machine.clock().elapsed_ticks();
  static_cast<void>(rejects([&] {
    static_cast<void>(fixture.fetch->advance_dma_capture_native(
        banks::inputs({.index = 1, .requests = 1}), OwnDspNativeCycleModel::unqualified));
  }));
  static_cast<void>(rejects([&] {
    static_cast<void>(
        fixture.fetch->advance_dma_capture(banks::inputs({.index = 1, .requests = 1})));
  }));
  static_cast<void>(rejects([&] { static_cast<void>(fixture.machine.advance(stale)); }));
  require(fixture.machine.cpu().registers() == before &&
              fixture.machine.cpu().progress() == progress &&
              fixture.machine.clock().elapsed_ticks() == ticks &&
              fixture.machine.bus().last_dma_cycle() == 0,
          "Unqualified timing and missing F1 reject before DMA request sampling or native effects");
  fixture.complete();
}

void controller_bit_with_fetch() {
  // FM13-88: JSET #8,X:fffff4,absolute target; the target equals sequential PC.
  constexpr unsigned status_test = 0x0ab4a8;
  for (const bool requested : {false, true}) {
    const std::array words{status_test, program + 2, 0U};
    Fixture fixture(words);
    fixture.start_fetch(2);
    expect_data(fixture.machine,
                {.kind = dsp56300::EffectKind::bit,
                 .space = dsp56300::Space::x,
                 .address = native::status});
    fixture.fetch->issue();
    auto expected = fixture.machine.cpu().registers();
    const auto result = fetch_cycle(fixture, {.index = 1, .requests = requested ? 1U : 0U}, true);
    require(result.native && result.native->fault.kind == dsp56300::FaultKind::none &&
                result.captured && !result.dma.core_stall &&
                result.dma.phase.has_value() == requested &&
                fixture.machine.bus().access_peripheral_bit({.space = DspSpace::x,
                                                             .address = native::status,
                                                             .bit = 8,
                                                             .operation = DspBitOperation::test}) ==
                    requested,
            "DMA-first sampling exposes its actual selected activity bit to a genuine native test "
            "alongside F2");
    fixture.complete();
    expected.pc = program + 2;
    const auto outcome = fixture.machine.cpu().instruction_outcome();
    require(fixture.machine.cpu().registers() == expected && outcome &&
                outcome->control_transfer == requested &&
                fixture.fetch->view().used == (requested ? 0U : 1U),
            "Actual DMA state determines same-PC branch/refill versus retained F2 without a host "
            "condition");
  }
}

void controller_write_preflight() {
  // FM BCLR #8,X:fffff4; unlike the read-only status test this RMW stays unqualified.
  constexpr unsigned status_clear = 0x0ab408;
  const std::array words{status_clear, 0U};
  Fixture fixture(words);
  fixture.start_fetch(1);
  fixture.fetch->issue();
  const auto before = fixture.machine.cpu().registers();
  const auto ticks = fixture.machine.clock().elapsed_ticks();
  const auto view = fixture.fetch->view();
  const auto error =
      rejects([&] { static_cast<void>(fetch_cycle(fixture, {.index = 1, .requests = 1}, true)); });
  require(error.contains("overlap") && error.contains("0xfffff4") &&
              fixture.machine.cpu().registers() == before &&
              fixture.machine.clock().elapsed_ticks() == ticks &&
              fixture.machine.bus().last_dma_cycle() == 0 && fixture.fetch->view() == view,
          "Status RMW cannot inherit the pure status-test/F2 qualification or sample DMA requests");
}

void peripheral_preflight() {
  const std::array words{fifo_load, 0U};
  Fixture fixture(words);
  auto &machine = fixture.machine;
  auto &shi = machine.bus().shi();
  shi.observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = shi_control, .value = shi_configuration});
  constexpr unsigned bits = 24;
  shi.observe_spi({.selected = true, .mosi = false});
  for (unsigned bit = bits; bit != 0; --bit) {
    const bool level = ((first >> (bit - 1)) & 1U) != 0;
    shi.observe_spi({.selected = true, .clock = true, .mosi = level});
    shi.observe_spi({.selected = true, .clock = false, .mosi = level});
  }
  shi.observe_spi({.mosi = false});
  fixture.start_fetch(1);
  fixture.fetch->issue();
  const auto ticks = machine.clock().elapsed_ticks();
  const auto error =
      rejects([&] { static_cast<void>(fetch_cycle(fixture, {.index = 1, .requests = 1}, true)); });
  require(error.contains("pc=0x000100") && error.contains("omr=0x000305") &&
              error.contains("0xffff94") && machine.clock().elapsed_ticks() == ticks &&
              machine.bus().last_dma_cycle() == 0 &&
              machine.bus().read({.space = DspSpace::x, .address = fifo}) == first,
          "Unqualified FIFO/F2 overlap with active DMA refuses before consuming data or sampling "
          "requests");
}

void clocked_owner_guard() {
  auto unsupported = native::native_clocks;
  unsupported.switch_timing = DspClockSwitch::reference_instruction_boundary;
  OwnDspMachine machine(unsupported);
  native::NativeProgram code;
  code.word(0);
  native::reset_native(machine, code);
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto ticks = machine.clock().elapsed_ticks();
  static_cast<void>(rejects([&] {
    static_cast<void>(machine.advance_dma_reference_cycle(banks::inputs({.index = 0})));
  }));
  require(machine.cpu().registers() == before && machine.cpu().progress() == progress &&
              machine.clock().elapsed_ticks() == ticks && !machine.bus().last_dma_cycle(),
          "Explicit composed cycles cannot override an instruction-boundary clock contract");
  native::step_native(machine);
  require(
      machine.cpu().progress().instructions == progress.instructions + 1 &&
          machine.clock().elapsed_ticks() == ticks,
      "Refused clock ownership preserves genuine native execution without inventing clock time");
}
}

int main() {
  try {
    held_destination_load();
    native_store_wins_source();
    fetch_wait_and_admit();
    simultaneous_fetch_and_redirect();
    dual_port_preflight();
    failed_required_read_keeps_dma_prefix();
    protocol_preflight();
    controller_bit_with_fetch();
    controller_write_preflight();
    peripheral_preflight();
    clocked_owner_guard();
    std::cout << "Native DMA/F2 cycle, publication and actual controller-status checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
