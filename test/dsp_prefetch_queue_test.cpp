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
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
namespace native = blofeld::test::dma_test;
namespace banks = blofeld::test::banks;
using blofeld::test::esai::rejects;
using blofeld::test::esai::require;
constexpr unsigned program = 0x100;
constexpr unsigned source = 0x800;
constexpr unsigned program_copy = 0x88024a;
constexpr unsigned initial_r0 = 0x7e;
constexpr unsigned old_r0 = 0x12;
constexpr unsigned new_r0 = 0x34;
constexpr unsigned first = 0x123456;
constexpr unsigned second = 0x234567;
constexpr unsigned changed_extension = 0x654321;
constexpr unsigned fifo = 0xffff94;
constexpr unsigned shi_control = 0xffff91;
constexpr unsigned shi_configuration = 0xa9;
constexpr unsigned shi_idle_empty = 0x8200; // UM HCSR HIDLE and HTDE reset status bits.
constexpr unsigned word_bits = 24;
constexpr unsigned bound = 32;
// Independent FM MOVE/MOVEP/MOVEM fields, absolute-extension JMP and short JSR/RTS.
constexpr unsigned long_r0 = 0x60f400;
constexpr unsigned long_r1 = 0x61f400;
constexpr unsigned short_old = 0x301200;
constexpr unsigned short_new = 0x303400;
constexpr unsigned fifo_load = 0x044494;
constexpr unsigned program_load = 0x07f084;
constexpr unsigned long_jump = 0x0af080;
constexpr unsigned short_call = 0x0d0104;
constexpr unsigned return_opcode = 0x00000c;

struct Replacement {
  unsigned address;
  unsigned value;
};

dsp56300::PendingInstruction pending(const OwnDspMachine &machine) {
  const auto result = machine.cpu().pending_instruction();
  if (!result) { throw Fault("Expected an actual native queued instruction effect"); }
  return *result;
}

void healthy(const dsp56300::InstructionProgress &progress) {
  require(progress.fault.kind == dsp56300::FaultKind::none,
          "Actual native queue instruction fault pc=" + hex(progress.fault.pc, 6) +
              " address=" + hex(progress.fault.address, 6));
}

void finish(OwnDspMachine &machine, dsp56300::InstructionProgress progress) {
  healthy(progress);
  for (unsigned effect = 0; effect < bound && !progress.retired && progress.pending; ++effect) {
    progress = machine.advance(progress.pending->ticket);
    healthy(progress);
  }
  require(progress.retired, "The actual native instruction retires within its effect bound");
}

void finish(OwnDspMachine &machine, OwnDspFetch &fetch) {
  finish(machine,
         {.pending = pending(machine),
          .progress = machine.cpu().progress(),
          .fault = machine.cpu().fault()});
  fetch.synchronize();
}

void execute(OwnDspMachine &machine, OwnDspFetch &fetch) {
  const auto before = machine.cpu().progress().instructions;
  const auto ticks = machine.clock().elapsed_ticks();
  finish(machine, fetch.begin());
  fetch.synchronize();
  require(machine.cpu().progress().instructions == before + 1 &&
              machine.clock().elapsed_ticks() == ticks,
          "One actual instruction retires without inventing clock periods");
}

void fetch_word(OwnDspMachine &machine, OwnDspFetch &fetch) {
  const auto before = fetch.view();
  const auto cpu = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto ticks = machine.clock().elapsed_ticks();
  fetch.issue();
  require(fetch.view().used == before.used && fetch.view().pending_address == before.next_address,
          "F1 generates its next address without sampling memory");
  fetch.capture();
  const auto sampled = fetch.view();
  require(sampled.used == before.used + 1 && !sampled.pending_address &&
              sampled.words.at(before.used).address == before.next_address &&
              machine.cpu().registers() == cpu && machine.cpu().progress() == progress &&
              machine.clock().elapsed_ticks() == ticks,
          "Actual F2 samples one P word without executing or clocking the held CPU");
}

struct Fixture {
  Fixture(const std::span<const std::uint32_t> words, const Replacement replacement) {
    native::NativeProgram setup;
    setup.word(long_r0);
    setup.word(initial_r0);
    for (const auto store :
         {DspStore{.space = DspSpace::x, .address = native::source0, .value = source},
          DspStore{
              .space = DspSpace::x, .address = native::destination0, .value = replacement.address},
          DspStore{.space = DspSpace::x, .address = native::counter0},
          DspStore{.space = DspSpace::x, .address = native::control0, .value = program_copy}}) {
      setup.write(store);
    }
    setup.word(long_jump);
    setup.word(program);
    native::reset_native(machine, setup);
    for (unsigned i = 0; i < words.size(); ++i) {
      machine.bus().write({.address = program + i, .value = words[i]});
    }
    machine.bus().write({.address = source, .value = replacement.value});
    static_cast<void>(native::dma(machine, banks::inputs({.index = 0}), banks::use()));
    for (unsigned i = 0; i < 6; ++i) { native::step_native(machine); }
    require(machine.cpu().registers().pc == program && machine.cpu().registers().r[0] == initial_r0,
            "Actual reset and literal MOVEP instructions configure P-to-P DMA before RAM entry");
    fetch = std::make_unique<OwnDspFetch>(machine);
  }

  void read_words(const std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) { fetch_word(machine, *fetch); }
  }

  void overwrite(const Replacement replacement) {
    const auto before = machine.cpu().registers();
    const auto progress = machine.cpu().progress();
    const auto ticks = machine.clock().elapsed_ticks();
    const auto read =
        native::dma(machine, banks::inputs({.index = 1, .requests = 1}), banks::use());
    const auto write =
        native::dma(machine, banks::inputs({.index = 2, .requests = 1}), banks::use());
    require(read.phase && read.phase->kind == DspDmaPhaseKind::source_read && write.word &&
                write.word->value == replacement.value &&
                machine.bus().read({.address = replacement.address}) == replacement.value &&
                machine.cpu().registers() == before && machine.cpu().progress() == progress &&
                machine.clock().elapsed_ticks() == ticks,
            "Real standalone DMA reads and writes while the native CPU and clock remain held");
  }

  OwnDspMachine machine{native::native_clocks};
  std::unique_ptr<OwnDspFetch> fetch;
};

void send(DspShi &shi, const unsigned value) {
  shi.observe_spi({.selected = true, .mosi = false});
  for (unsigned bit = word_bits; bit != 0; --bit) {
    const bool level = ((value >> (bit - 1)) & 1U) != 0;
    shi.observe_spi({.selected = true, .clock = true, .mosi = level});
    shi.observe_spi({.selected = true, .clock = false, .mosi = level});
  }
  shi.observe_spi({.mosi = false});
}

void overlapped_dma(const bool extension_word, const bool before_capture) {
  const std::array words{fifo_load, short_old, long_r1, first, 0U};
  const Replacement replacement{.address = program + (extension_word ? 3U : 1U),
                                .value = extension_word ? changed_extension : short_new};
  Fixture fixture(words, replacement);
  auto &machine = fixture.machine;
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = shi_control, .value = shi_configuration});
  send(machine.bus().shi(), first);
  send(machine.bus().shi(), second);
  fixture.read_words(1);
  healthy(fixture.fetch->begin());
  const auto held = pending(machine);
  require(held.effect.kind == dsp56300::EffectKind::read &&
              held.effect.space == dsp56300::Space::x && held.effect.address == fifo &&
              held.effect.pc == program,
          "Actual native MOVEP waits at the real FIFO operand while F1/F2 continue");
  while (fixture.fetch->view().next_address < replacement.address) { fixture.read_words(1); }
  if (before_capture) {
    fixture.fetch->issue();
  } else {
    fixture.read_words(1);
  }
  fixture.overwrite(replacement);
  if (before_capture) { fixture.fetch->capture(); }
  while (fixture.fetch->view().next_address < program + words.size()) { fixture.read_words(1); }
  auto expected = machine.cpu().registers();
  const auto before = machine.cpu().progress();
  finish(machine, *fixture.fetch);
  expected.pc = program + 1;
  expected.x = first;
  require(machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions == before.instructions + 1 &&
              machine.bus().read({.space = DspSpace::x, .address = fifo}) == second,
          "Exactly the admitted native FIFO read consumes the first real serial word");
  const auto moves = machine.cpu().progress();
  execute(machine, *fixture.fetch);
  execute(machine, *fixture.fetch);
  expected.r[0] = !extension_word && before_capture ? new_r0 : old_r0;
  expected.r[1] = extension_word && before_capture ? changed_extension : first;
  expected.pc = program + 4;
  require(machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions == moves.instructions + 2 &&
              machine.cpu().progress().estimated_cycles == moves.estimated_cycles + 3 &&
              fixture.fetch->view().used == 1,
          "Literal full registers, two retirements and three base units distinguish real F2 time");
  execute(machine, *fixture.fetch);
  ++expected.pc;
  require(machine.cpu().registers() == expected && fixture.fetch->view().used == 0,
          "The queued zero executes as a real NOP after both earlier instructions");
}

void branch_redirects() {
  for (const bool fallthrough_target : {false, true}) {
    const auto destination = program + (fallthrough_target ? 2U : 4U);
    const std::array words{long_jump, destination, 0U, 0U, 0U, 0U};
    Fixture fixture(words, {.address = destination, .value = short_new});
    fixture.read_words(5);
    fixture.fetch->issue();
    healthy(fixture.fetch->begin());

    auto expected = fixture.machine.cpu().registers();
    fixture.overwrite({.address = destination, .value = short_new});
    finish(fixture.machine, *fixture.fetch);
    const auto fetch = fixture.fetch->view();
    expected.pc = destination;
    require(fixture.machine.cpu().registers() == expected && fetch.active && fetch.used == 0 &&
                !fetch.pending_address && fetch.next_address == destination &&
                fetch.consume_address == destination,
            "Actual JMP flushes queued F2 and pending F1, including a fallthrough target");
    fixture.read_words(1);
    execute(fixture.machine, *fixture.fetch);
    expected.pc = destination + 1;
    expected.r[0] = new_r0;
    require(fixture.machine.cpu().registers() == expected,
            "Fresh branch-target F2 observes the actual DMA-written instruction");
  }
}

void capacity_and_protocol() {
  const std::array<std::uint32_t, dsp_program_fetch_capacity + 1> words{};
  Fixture fixture(words, {.address = program, .value = 0});
  auto &machine = fixture.machine;
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  static_cast<void>(rejects([&] { static_cast<void>(fixture.fetch->begin()); }));
  static_cast<void>(rejects([&] { fixture.fetch->capture(); }));
  fixture.fetch->issue();
  const auto issued = fixture.fetch->view();
  static_cast<void>(rejects([&] { fixture.fetch->issue(); }));
  require(fixture.fetch->view() == issued && machine.cpu().registers() == before &&
              machine.cpu().progress() == progress && !machine.cpu().pending_instruction(),
          "Starvation, missing F1 and duplicate issue preserve CPU and outstanding address");
  fixture.fetch->capture();
  fixture.read_words(dsp_program_fetch_capacity - 1);
  const auto full = fixture.fetch->view();
  static_cast<void>(rejects([&] { fixture.fetch->issue(); }));
  static_cast<void>(rejects([&] { const OwnDspFetch competing(machine); }));
  static_cast<void>(rejects([&] { static_cast<void>(machine.begin_fetch()); }));
  static_cast<void>(
      rejects([&] { static_cast<void>(machine.admit_interrupt({.address = program})); }));
  require(fixture.fetch->view() == full && machine.cpu().registers() == before &&
              machine.cpu().progress() == progress,
          "Full capacity and competing CPU/vector ownership cannot replace actual F2 words");
  execute(machine, *fixture.fetch);
  require(fixture.fetch->view().used == dsp_program_fetch_capacity - 1,
          "One actual NOP consumes precisely one queue entry");
  fixture.fetch->issue();
  require(fixture.fetch->view().pending_address == program + dsp_program_fetch_capacity,
          "Released capacity permits another real F1 before reset");
  healthy(fixture.fetch->begin());
  const auto ticket = pending(machine).ticket;
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  static_cast<void>(rejects([&] { static_cast<void>(fixture.fetch->view()); }));
  require(!machine.cpu().pending_instruction() && machine.cpu().progress().instructions == 0,
          "Hardware reset discards pending CPU work and invalidates the old queue owner");
  constexpr unsigned hold = 90;
  require(machine.advance_hardware_reset_ticks(hold) == hold,
          "Reset assertion has actual hold time");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(hold) == hold,
          "Reset recovery has actual hold time");
  static_cast<void>(rejects([&] { static_cast<void>(machine.advance(ticket)); }));
  OwnDspFetch fresh(machine);
  fetch_word(machine, fresh);
  require(fresh.view().used == 1 && fresh.view().words.front() ==
                                        DspProgramWord{.address = native::boot, .value = long_r0},
          "A fresh post-reset owner samples the actual retained boot ROM");
}

void unknown_read_and_failure() {
  const std::array words{0U};
  for (const bool rewrite : {false, true}) {
    Fixture fixture(words, {.address = program, .value = 0});
    auto &machine = fixture.machine;
    fixture.read_words(1);
    fixture.fetch->issue();
    const auto before = machine.cpu().registers();
    const auto progress = machine.cpu().progress();
    fixture.fetch->capture();
    const auto unknown = fixture.fetch->view();
    require(unknown.used == 2 && unknown.words[1].address == program + 1 &&
                !unknown.words[1].data && machine.cpu().registers() == before &&
                machine.cpu().progress() == progress,
            "Actual F2 retains an explicitly absent value without executing it");
    if (rewrite) { machine.bus().write({.address = program + 1}); }
    execute(machine, *fixture.fetch);
    auto expected = before;
    expected.pc = program + 1;
    const auto error = rejects([&] { static_cast<void>(fixture.fetch->begin()); });
    require(error.contains("Unknown DSP program sample read P:0x000101") &&
                error.contains("pc=0x000101") && error.contains("omr=0x000305") &&
                machine.cpu().registers() == expected &&
                machine.cpu().progress() ==
                    dsp56300::Progress{.instructions = progress.instructions + 1,
                                       .estimated_cycles = progress.estimated_cycles + 1},
            "Unknown consumption fails contextually despite later writes to backing RAM");
  }
  Fixture known(words, {.address = program, .value = 0});
  known.read_words(1);
  known.fetch->issue();
  known.machine.bus().write({.address = program + 1});
  known.fetch->capture();
  require(known.fetch->view().words[1] == DspProgramWord{.address = program + 1},
          "Explicit memory initialization before F2 supplies a genuinely captured zero opcode");
  execute(known.machine, *known.fetch);
  execute(known.machine, *known.fetch);
  require(known.machine.cpu().registers().pc == program + 2,
          "The fresh owner executes both actual captured NOPs");
}

void program_data_is_not_prefetch() {
  const std::array words{program_load, program + 2, 0U};
  Fixture fixture(words, {.address = program + 2, .value = short_new});
  auto &machine = fixture.machine;
  fixture.read_words(words.size());
  fixture.overwrite({.address = program + 2, .value = short_new});
  auto expected = machine.cpu().registers();
  healthy(fixture.fetch->begin());
  const auto access = pending(machine);
  require(access.effect.kind == dsp56300::EffectKind::read &&
              access.effect.space == dsp56300::Space::program &&
              access.effect.address == program + 2 && access.effect.pc == program &&
              fixture.fetch->view().used == 1,
          "The captured two-word MOVEM leaves a real P-data read independent of its queued NOP");
  static_cast<void>(rejects([&] { static_cast<void>(machine.begin_fetch()); }));
  finish(machine, *fixture.fetch);
  expected.pc = program + 2;
  expected.x = short_new;
  require(machine.cpu().registers() == expected && fixture.fetch->view().used == 1,
          "Actual program-data read sees DMA replacement without replacing the old opcode sample");
  execute(machine, *fixture.fetch);
  ++expected.pc;
  require(machine.cpu().registers() == expected && fixture.fetch->view().used == 0,
          "The same address's queued zero remains a NOP with all other registers unchanged");
}

void repeat_control() {
  constexpr unsigned repeat_three = 0x0603a0;
  const std::array words{repeat_three, short_old, 0U};
  Fixture fixture(words, {.address = program, .value = 0});
  auto &machine = fixture.machine;
  fixture.read_words(words.size());
  auto expected = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  execute(machine, *fixture.fetch);
  execute(machine, *fixture.fetch);
  require(machine.cpu().instruction_fetch().latched_opcode == short_old &&
              fixture.fetch->view().used == 1,
          "REP holds its actual executed body while preserving the younger queued NOP");
  machine.bus().write({.address = program + 1, .value = short_new});
  static_cast<void>(rejects([&] { fixture.fetch->issue(); }));
  execute(machine, *fixture.fetch);
  execute(machine, *fixture.fetch);
  execute(machine, *fixture.fetch);
  expected.pc = program + 3;
  expected.r[0] = old_r0;
  require(machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions == progress.instructions + 5 &&
              !machine.cpu().instruction_fetch().latched_opcode && fixture.fetch->view().used == 0,
          "Three genuine REP bodies retain the original opcode and restore the original LC");
}

void wait_control() {
  constexpr unsigned wait_opcode = 0x86;
  const std::array words{wait_opcode, 0U};
  Fixture fixture(words, {.address = program, .value = 0});
  fixture.read_words(words.size());
  auto expected = fixture.machine.cpu().registers();
  execute(fixture.machine, *fixture.fetch);
  expected.pc = program + 1;
  const auto queue = fixture.fetch->view();
  const auto progress = fixture.machine.cpu().progress();
  static_cast<void>(rejects([&] { static_cast<void>(fixture.fetch->begin()); }));
  require(
      fixture.machine.cpu().registers() == expected &&
          fixture.machine.cpu().processing_state() == dsp56300::ProcessingState::waiting &&
          fixture.machine.cpu().progress() == progress && fixture.fetch->view() == queue,
      "Actual WAIT retires into waiting state; another admission cannot execute its queued NOP");
}

void reset_control() {
  constexpr unsigned reset_opcode = 0x84;
  const std::array words{reset_opcode, 0U};
  Fixture fixture(words, {.address = program, .value = 0});
  auto &machine = fixture.machine;
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = shi_control, .value = shi_configuration});
  require(machine.bus().read({.space = DspSpace::x, .address = native::control0}) == program_copy &&
              machine.bus().read({.space = DspSpace::x, .address = shi_control}) ==
                  (shi_configuration | shi_idle_empty),
          "Real DMA and SHI registers start programmed before guest RESET");
  fixture.read_words(words.size());
  auto expected = machine.cpu().registers();
  auto progress = fixture.fetch->begin();
  require(progress.pending && progress.pending->effect.kind == dsp56300::EffectKind::reset &&
              progress.pending->effect.pc == program &&
              progress.pending->effect.value == reset_opcode,
          "Native guest RESET offers its actual derivative-peripheral action before completion");
  finish(machine, progress);
  fixture.fetch->synchronize();
  expected.pc = program + 1;
  require(machine.cpu().registers() == expected &&
              machine.bus().read({.space = DspSpace::x, .address = native::control0}) == 0 &&
              machine.bus().read({.space = DspSpace::x, .address = shi_control}) ==
                  shi_idle_empty &&
              fixture.fetch->view().used == 1 &&
              machine.bus().read({.address = program}) == reset_opcode,
          "Real RESET clears derivative DMA/SHI while retaining CPU, P-RAM and queued NOP");
  execute(machine, *fixture.fetch);
}

void conditional_flow() {
  constexpr unsigned jne = 0x0e2104;
  constexpr unsigned jeq = 0x0ea104;
  constexpr unsigned zero = 4;
  for (const bool taken : {false, true}) {
    const std::array words{taken ? jne : jeq, 0U, 0U, 0U, 0U};
    Fixture fixture(words, {.address = program, .value = 0});
    fixture.read_words(words.size());
    require((fixture.machine.cpu().registers().sr & zero) == 0,
            "Literal Z=0 selects JNE and rejects JEQ");
    healthy(fixture.fetch->begin());

    auto expected = fixture.machine.cpu().registers();
    finish(fixture.machine, *fixture.fetch);
    expected.pc = program + (taken ? 4U : 1U);
    const auto fetch = fixture.fetch->view();
    require(fixture.machine.cpu().registers() == expected &&
                fetch.used == (taken ? 0U : words.size() - 1) &&
                fetch.next_address == program + (taken ? 4U : words.size()),
            "Actual taken Jcc discards younger samples; untaken fallthrough preserves them");
  }
}

struct ConditionalCase {
  bool taken;
  bool bit_operand;
};

void conditional_dma_continuation(Fixture &fixture,
                                  const ConditionalCase scenario,
                                  const Replacement replacement) {
  constexpr unsigned operand = 0x20;
  auto &machine = fixture.machine;
  auto expected = machine.cpu().registers();
  fixture.overwrite(replacement);
  if (scenario.bit_operand) {
    const auto read = pending(machine);
    require(read.effect.kind == dsp56300::EffectKind::read &&
                read.effect.space == dsp56300::Space::x && read.effect.address == operand &&
                read.effect.pc == program,
            "The actual pending RAM-bit read still owns its original execution context");
    const auto accepted = machine.advance(read.ticket);
    healthy(accepted);
    machine.bus().write(
        {.space = DspSpace::x, .address = operand, .value = scenario.taken ? 0U : 1U});
    finish(machine, accepted);
    fixture.fetch->synchronize();
  } else {
    finish(machine, *fixture.fetch);
  }
  expected.pc = replacement.address;
  if (!scenario.bit_operand && scenario.taken) {
    expected.sp = expected.sc = 1;
    expected.stack[1] = (std::uint64_t{replacement.address} << word_bits) | expected.sr;
  }
  const auto outcome = machine.cpu().instruction_outcome();
  const auto fetch = fixture.fetch->view();
  const auto actual = machine.cpu().registers();
  require(
      actual == expected && outcome && outcome->control_transfer == scenario.taken &&
          fetch.used == (scenario.taken ? 0U : 3U) &&
          fetch.pending_address.has_value() != scenario.taken,
      "Accepted native RAM input or call condition controls same-PC retention, never later RAM: "
      "taken=" +
          std::string(scenario.taken ? "true" : "false") +
          " bit=" + (scenario.bit_operand ? "true" : "false") + " pc=" + hex(actual.pc, 6) +
          " sr=" + hex(actual.sr, 6) + " sp=" + hex(actual.sp, 2) + " sc=" + hex(actual.sc, 2) +
          " frame=" + hex(actual.stack[1], 12) + " expected-frame=" + hex(expected.stack[1], 12) +
          " queued=" + std::to_string(fetch.used) +
          " redirect=" + (outcome && outcome->control_transfer ? "true" : "false"));
  if (scenario.taken) { fixture.read_words(1); }
  execute(machine, *fixture.fetch);
  ++expected.pc;
  if (scenario.taken) { expected.r[0] = new_r0; }
  require(machine.cpu().registers() == expected &&
              fixture.fetch->view().used == (scenario.taken ? 0U : 2U),
          "The following opcode sees either actual DMA refetch or its original queued NOP");
}

void conditional_dma_case(const ConditionalCase scenario) {
  // FM JSET short X:aa/bit0 plus absolute extension, JSNE/JSEQ short absolute.
  constexpr unsigned jset = 0x0a20a0; // Bit7 selects the two-word control-flow form.
  constexpr unsigned jsne = 0x0f2101;
  constexpr unsigned jseq = 0x0fa101;
  constexpr unsigned operand = 0x20;
  const auto destination = program + (scenario.bit_operand ? 2U : 1U);
  std::vector<unsigned> words = scenario.bit_operand
                                    ? std::vector<unsigned>{jset, destination}
                                    : std::vector<unsigned>{scenario.taken ? jsne : jseq};
  words.insert(words.end(), 3, 0);
  Fixture fixture(words, {.address = destination, .value = short_new});
  fixture.machine.bus().write(
      {.space = DspSpace::x, .address = operand, .value = scenario.taken ? 1U : 0U});
  fixture.read_words(words.size());
  fixture.fetch->issue();
  healthy(fixture.fetch->begin());

  conditional_dma_continuation(fixture, scenario, {.address = destination, .value = short_new});
}

void clocked_owner_guard() {
  OwnDspMachine machine(native::native_clocks);
  native::NativeProgram code;
  code.word(0);
  code.word(0);
  native::reset_native(machine, code);
  auto progress = machine.begin_fetch();
  const auto before = machine.cpu().registers();
  const auto count = machine.cpu().progress();
  const auto ticks = machine.clock().elapsed_ticks();
  constexpr auto timing = OwnDspAutomaticTiming::one_effect_per_cycle;
  static_cast<void>(rejects([&] { const OwnDspFetch competing(machine); }));
  static_cast<void>(rejects([&] { const OwnDspTimeline competing(machine, timing, 0); }));
  require(machine.cpu().registers() == before && machine.cpu().progress() == count &&
              machine.clock().elapsed_ticks() == ticks,
          "A pending native fetch rejects replacement by another frontend or timeline");
  finish(machine, progress);
  require(machine.cpu().progress().instructions == count.instructions + 1 &&
              machine.clock().elapsed_ticks() == ticks,
          "Rejected ownership attempts preserve genuine untimed native completion");
  OwnDspTimeline timeline(machine, timing, 0);
  static_cast<void>(rejects([&] { static_cast<void>(machine.begin_fetch()); }));
  unsigned cycles = 0;
  for (; cycles < bound && machine.cpu().progress().instructions < count.instructions + 2;
       ++cycles) {
    const auto deadline = timeline.next_deadline();
    require(
        !timeline.advance_one(deadline, OwnDspBoundary::before, banks::inputs({.index = cycles})),
        "A before boundary cannot consume its simultaneous reference cycle");
    require(
        timeline.advance_one(deadline, OwnDspBoundary::through, banks::inputs({.index = cycles}))
            .has_value(),
        "A through boundary consumes exactly one explicitly scheduled reference cycle");
  }
  constexpr std::uint64_t reference_ticks = 45;
  require(machine.cpu().progress().instructions == count.instructions + 2 &&
              machine.cpu().registers().pc == native::boot + 2 &&
              machine.clock().elapsed_ticks() == ticks + (cycles * reference_ticks) &&
              machine.clock().native_cycle_units() == cycles,
          "Timeline ownership preserves independent native retirement and reference-clock counts");
}

void call_and_return() {
  const std::array words{short_call, 0U, 0U, 0U, return_opcode, 0U};
  Fixture fixture(words, {.address = program + 1, .value = short_new});
  auto &machine = fixture.machine;
  fixture.read_words(words.size());
  fixture.overwrite({.address = program + 1, .value = short_new});
  auto expected = machine.cpu().registers();
  execute(machine, *fixture.fetch);
  expected.pc = program + 4;
  expected.sp = expected.sc = 1;
  expected.stack[1] = (std::uint64_t{program + 1} << word_bits) | expected.sr;
  require(machine.cpu().registers() == expected && fixture.fetch->view().used == 0,
          "Actual JSR pushes the independent full return frame and redirects the queue");
  fixture.read_words(1);
  fixture.fetch->issue();
  execute(machine, *fixture.fetch);
  expected.pc = program + 1;
  expected.sp = expected.sc = 0;
  require(machine.cpu().registers() == expected && fixture.fetch->view().used == 0 &&
              !fixture.fetch->view().pending_address &&
              fixture.fetch->view().next_address == program + 1,
          "Actual RTS pops its frame and discards both younger F2 and outstanding F1");
  fixture.read_words(1);
  execute(machine, *fixture.fetch);
  ++expected.pc;
  expected.r[0] = new_r0;
  require(machine.cpu().registers() == expected,
          "Return-target refetch observes the actual DMA overwrite");
}
}

int main() {
  try {
    for (const bool extension_word : {false, true}) {
      for (const bool before_capture : {false, true}) {
        overlapped_dma(extension_word, before_capture);
      }
    }
    branch_redirects();
    capacity_and_protocol();
    unknown_read_and_failure();
    program_data_is_not_prefetch();
    repeat_control();
    wait_control();
    reset_control();
    conditional_flow();
    for (const bool taken : {false, true}) {
      for (const bool bit_operand : {false, true}) {
        conditional_dma_case({.taken = taken, .bit_operand = bit_operand});
      }
    }
    clocked_owner_guard();
    call_and_return();
    std::cout << "Native queued program fetch, DMA, control and unknown-input checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
