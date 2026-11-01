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
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>

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
constexpr unsigned first_r0 = 0x12;
constexpr unsigned second_r0 = 0x34;
constexpr unsigned first = 0x123456;
constexpr unsigned second = 0x654321;
// FM MOVE immediate register fields and absolute-extension JMP.
constexpr unsigned short_first = 0x301200;
constexpr unsigned short_second = 0x303400;
constexpr unsigned long_r0 = 0x60f400;
constexpr unsigned long_jump = 0x0af080;
constexpr unsigned bound = 64;

void healthy(const dsp56300::InstructionProgress &progress) {
  if (progress.fault.kind != dsp56300::FaultKind::none) {
    throw Fault("Native latch instruction fault kind=" +
                std::to_string(static_cast<unsigned>(progress.fault.kind)) +
                " pc=" + hex(progress.fault.pc, 6) + " address=" + hex(progress.fault.address, 6));
  }
}

void finish(OwnDspMachine &machine, dsp56300::InstructionProgress progress) {
  healthy(progress);
  for (unsigned effect = 0; effect < bound && progress.pending && !progress.retired; ++effect) {
    progress = machine.advance(progress.pending->ticket);
    healthy(progress);
  }
  require(progress.retired, "The real native latch instruction retires within its effect bound");
}

void read_words(OwnDspFetch &fetch, const unsigned count = 1) {
  for (unsigned i = 0; i < count; ++i) {
    fetch.issue();
    fetch.capture();
  }
}

void execute(OwnDspMachine &machine, OwnDspFetch &fetch) {
  const auto before = machine.cpu().progress().instructions;
  const auto ticks = machine.clock().elapsed_ticks();
  finish(machine, fetch.begin());
  fetch.synchronize();
  require(machine.cpu().progress().instructions == before + 1 &&
              machine.clock().elapsed_ticks() == ticks,
          "Actual F2 admission retires once without inventing reference clock periods");
}

struct Replacement {
  unsigned value;
  unsigned destination = program;
};

struct Fixture {
  Fixture(const std::span<const std::uint32_t> words, const Replacement replacement) {
    native::NativeProgram code;
    code.word(long_r0);
    code.word(initial_r0);
    for (const auto store :
         {DspStore{.space = DspSpace::x, .address = native::source0, .value = source},
          DspStore{.space = DspSpace::x,
                   .address = native::destination0,
                   .value = replacement.destination},
          DspStore{.space = DspSpace::x, .address = native::counter0},
          DspStore{.space = DspSpace::x, .address = native::control0, .value = program_copy}}) {
      code.write(store);
    }
    code.word(long_jump);
    code.word(program);
    native::reset_native(machine, code);
    for (unsigned i = 0; i < words.size(); ++i) {
      machine.bus().write({.address = program + i, .value = words[i]});
    }
    machine.bus().write({.address = source, .value = replacement.value});
    static_cast<void>(native::dma(machine, banks::inputs({.index = 0}), banks::use()));
    for (unsigned i = 0; i < 6; ++i) { native::step_native(machine); }
    require(machine.cpu().registers().pc == program && machine.cpu().registers().r[0] == initial_r0,
            "Literal native setup configures actual P-to-P DMA and jumps into known RAM");
    fetch = std::make_unique<OwnDspFetch>(machine);
  }

  void source_read(const unsigned fetch_address = program) {
    const std::array core{DspMemoryAccess{.target = {.address = fetch_address}}};
    const auto actual =
        native::dma(machine, banks::inputs({.index = 1, .requests = 1}), banks::use(core));
    require(actual.phase && actual.phase->kind == DspDmaPhaseKind::source_read && !actual.word &&
                !actual.core_stall,
            "Actual DMA source and declared program-fetch bank have independent read resources");
  }

  void overwrite(const Replacement replacement) {
    const auto before = machine.cpu().registers();
    const auto progress = machine.cpu().progress();
    const auto actual =
        native::dma(machine, banks::inputs({.index = 2, .requests = 1}), banks::use());
    require(actual.word && actual.word->value == replacement.value && !actual.core_stall &&
                machine.bus().read({.address = replacement.destination}) == replacement.value &&
                machine.cpu().registers() == before && machine.cpu().progress() == progress,
            "Real DMA overwrites P-RAM without executing the held native instruction");
  }

  void restore(const OwnDspFetchCheckpoint &saved) {
    const auto old = std::move(fetch);
    fetch = std::make_unique<OwnDspFetch>(machine, saved);
    require(rejects([&] { static_cast<void>(old->view()); }).contains("Stale"),
            "Coordinated restore invalidates the previous frontend capability");
  }

  OwnDspMachine machine{native::native_clocks};
  std::unique_ptr<OwnDspFetch> fetch;
};

void opcode_and_restore() {
  const std::array words{short_first, long_jump, program};
  for (const bool latch : {false, true}) {
    Fixture fixture(words, {.value = short_second});
    auto &machine = fixture.machine;
    fixture.fetch->issue();
    const auto before = machine.cpu().registers();
    const auto progress = machine.cpu().progress();
    fixture.source_read();
    if (latch) {
      fixture.fetch->capture();
      const auto captured = fixture.fetch->view();
      require(captured.used == 1 &&
                  captured.words.front() ==
                      DspProgramWord{.address = program, .value = short_first} &&
                  machine.cpu().registers() == before && machine.cpu().progress() == progress,
              "Actual F2 holds its independently encoded opcode before decode or retirement");
      static_cast<void>(rejects([&] { fixture.fetch->capture(); }));
      require(fixture.fetch->view() == captured,
              "Duplicate F2 capture cannot replace its held word");
    }
    const auto saved = fixture.fetch->checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      fixture.restore(saved);
      require(fixture.fetch->view().used == (latch ? 1U : 0U),
              "Joint restore retains whether the real F2 read already happened");
      fixture.overwrite({.value = short_second});
      if (!latch) { fixture.fetch->capture(); }
      execute(machine, *fixture.fetch);
      auto expected = before;
      expected.r[0] = latch ? first_r0 : second_r0;
      expected.pc = program + 1;
      require(
          machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions == progress.instructions + 1 &&
              machine.cpu().progress().estimated_cycles == progress.estimated_cycles + 1,
          "Independent full registers and one base unit distinguish actual opcode capture time");
      read_words(*fixture.fetch, 2);
      execute(machine, *fixture.fetch);
      read_words(*fixture.fetch);
      execute(machine, *fixture.fetch);
      require(machine.cpu().registers().r[0] == second_r0 &&
                  machine.cpu().registers().pc == program + 1,
              "A later genuine refetch observes the actual DMA-written opcode");
    }
  }
}

void opcode_length_and_zero() {
  const std::array long_words{long_r0, first, 0U};
  const std::array nop_words{0U, 0U};
  for (const bool zero_opcode : {false, true}) {
    const auto words = zero_opcode ? std::span<const std::uint32_t>(nop_words)
                                   : std::span<const std::uint32_t>(long_words);
    const auto replacement = zero_opcode ? short_second : 0U;
    Fixture fixture(words, {.value = replacement});
    auto &machine = fixture.machine;
    const auto before = machine.cpu().progress();
    read_words(*fixture.fetch);
    require(fixture.fetch->view().words.front() ==
                DspProgramWord{.address = program, .value = words.front()},
            "Zero is an actual captured opcode, never absence");
    fixture.source_read();
    fixture.overwrite({.value = replacement});
    if (!zero_opcode) { read_words(*fixture.fetch); }
    execute(machine, *fixture.fetch);
    const auto units = zero_opcode ? 1U : 2U;
    require(machine.cpu().registers().r[0] == (zero_opcode ? initial_r0 : first) &&
                machine.cpu().registers().pc == program + units &&
                machine.cpu().progress().estimated_cycles == before.estimated_cycles + units,
            "Real decode and independent base cost retain the captured zero or long instruction");
  }
}

void extension_sampling() {
  const std::array words{long_r0, first, 0U};
  for (const bool latch : {false, true}) {
    Fixture fixture(words, {.value = second, .destination = program + 1});
    auto &machine = fixture.machine;
    read_words(*fixture.fetch);
    const auto before = machine.cpu().registers();
    const auto progress = machine.cpu().progress();
    static_cast<void>(rejects([&] { static_cast<void>(fixture.fetch->begin()); }));
    require(machine.cpu().registers() == before && machine.cpu().progress() == progress,
            "Decoder knowledge of length cannot fabricate a consumed extension");
    fixture.fetch->issue();
    fixture.source_read(program + 1);
    if (latch) { fixture.fetch->capture(); }
    const auto saved = fixture.fetch->checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      fixture.restore(saved);
      fixture.overwrite({.value = second, .destination = program + 1});
      if (!latch) { fixture.fetch->capture(); }
      execute(machine, *fixture.fetch);
      auto expected = before;
      expected.r[0] = latch ? first : second;
      expected.pc = program + 2;
      require(machine.cpu().registers() == expected &&
                  machine.cpu().progress().estimated_cycles == progress.estimated_cycles + 2,
              "Extension data uses its own real F2 time, never a stale decode lookahead value");
      read_words(*fixture.fetch);
      execute(machine, *fixture.fetch);
      require(machine.cpu().registers().pc == program + 3 && fixture.fetch->view().used == 0,
              "Retirement releases both words before the following genuinely captured NOP");
    }
  }
}

void send(DspShi &shi, const unsigned value) {
  shi.observe_spi({.selected = true, .mosi = false});
  constexpr unsigned bits = 24;
  for (unsigned bit = 0; bit < bits; ++bit) {
    const bool level = ((value >> (bits - 1 - bit)) & 1U) != 0;
    shi.observe_spi({.selected = true, .clock = true, .mosi = level});
    shi.observe_spi({.selected = true, .clock = false, .mosi = level});
  }
  shi.observe_spi({.mosi = false});
}

void nonprogram_and_ticket_guards() {
  OwnDspMachine machine(native::native_clocks);
  OwnDspMachine foreign(native::native_clocks);
  native::NativeProgram code;
  constexpr unsigned read_hrx_x0 = 0x044494;
  constexpr unsigned shi_control = 0xffff91;
  constexpr unsigned shi_configuration = 0xa9;
  constexpr unsigned fifo = 0xffff94;
  code.word(read_hrx_x0);
  code.word(0);
  native::reset_native(machine, code);
  native::reset_native(foreign, code);
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = shi_control, .value = shi_configuration});
  send(machine.bus().shi(), first);
  send(machine.bus().shi(), second);
  OwnDspFetch fetch(machine);
  OwnDspFetch other(foreign);
  read_words(fetch);
  read_words(other);
  const auto actual = fetch.begin();
  const auto wrong = other.begin();
  if (!actual.pending || !wrong.pending) { throw Fault("Expected two actual owned FIFO effects"); }
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto view = fetch.view();
  static_cast<void>(rejects([&] { static_cast<void>(machine.advance(wrong.pending->ticket)); }));
  static_cast<void>(rejects([&] { fetch.capture(); }));
  static_cast<void>(rejects([&] { const OwnDspFetch empty(machine, OwnDspFetchCheckpoint{}); }));
  require(actual.pending->effect.kind == dsp56300::EffectKind::read &&
              actual.pending->effect.address == fifo && machine.cpu().registers() == before &&
              machine.cpu().progress() == progress && fetch.view() == view &&
              machine.bus().read({.space = DspSpace::x, .address = fifo}) == first,
          "Foreign/empty capabilities and a nonprogram capture consume no actual FIFO word");
  finish(machine, machine.advance(actual.pending->ticket));
  fetch.synchronize();
  require(machine.cpu().registers().x == second,
          "The legitimate native data read still consumes the remaining real FIFO word");
  static_cast<void>(rejects([&] { static_cast<void>(machine.advance(actual.pending->ticket)); }));
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  static_cast<void>(rejects([&] { static_cast<void>(fetch.view()); }));
  static_cast<void>(rejects([&] { static_cast<void>(machine.advance(actual.pending->ticket)); }));
  require(machine.cpu().progress() == dsp56300::Progress{},
          "Real reset discards the old frontend and native effect capabilities");
}

unsigned prepare_unknown(OwnDspMachine &machine, const bool extension) {
  native::NativeProgram code;
  code.word(long_jump);
  code.word(program);
  native::reset_native(machine, code);
  const auto address = program + (extension ? 1U : 0U);
  constexpr unsigned guard_words = 16;
  for (unsigned i = 0; i < guard_words; ++i) {
    if (program + i != address) {
      machine.bus().write({.address = program + i, .value = extension && i == 0 ? long_r0 : 0U});
    }
  }
  native::step_native(machine);
  return address;
}

void held_unknown_snapshot(const bool extension) {
  OwnDspMachine machine(native::native_clocks);
  const auto address = prepare_unknown(machine, extension);
  const auto chip = machine.checkpoint();
  auto fetch = std::make_unique<OwnDspFetch>(machine);
  read_words(*fetch, extension ? 2U : 1U);
  const auto captured = fetch->view();
  require(!captured.words.at(extension ? 1U : 0U).data,
          "The actual F2 sample explicitly retains an unknown word, never a zero NOP");
  const auto saved = fetch->checkpoint();
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  for (unsigned replay = 0; replay < 2; ++replay) {
    auto stale = std::move(fetch);
    fetch = std::make_unique<OwnDspFetch>(machine, saved);
    machine.bus().write({.address = address, .value = extension ? first : 0U});
    const auto error = rejects([&] { static_cast<void>(fetch->begin()); });
    require(error.contains("P:" + hex(address, 6)) && fetch->view() == captured &&
                machine.cpu().registers() == before && machine.cpu().progress() == progress,
            "Later known RAM and trusted replay cannot replace an already captured unknown word");
  }
  fetch.reset();
  machine.restore(chip);
  machine.bus().write({.address = address, .value = extension ? first : 0U});
  fetch = std::make_unique<OwnDspFetch>(machine);
  read_words(*fetch, extension ? 2U : 1U);
  execute(machine, *fetch);
  require(machine.cpu().registers().pc == program + (extension ? 2U : 1U) &&
              (!extension || machine.cpu().registers().r[0] == first),
          "Restoring before F2 and recapturing real known memory permits the independent native "
          "result");
}

void unknown_program_and_recovery(const bool extension) {
  OwnDspMachine machine(native::native_clocks);
  const auto address = prepare_unknown(machine, extension);
  auto timeline =
      std::make_unique<OwnDspTimeline>(machine, OwnDspAutomaticTiming::one_effect_per_cycle, 0);
  std::optional<OwnDspTimelineCheckpoint> saved;
  const auto before = machine.cpu().progress();
  const auto advance = [&]() {
    const auto last = machine.bus().last_dma_cycle();
    return timeline->advance_one(timeline->next_deadline(),
                                 OwnDspBoundary::through,
                                 banks::inputs({.index = last ? *last + 1 : 0}));
  };
  bool failed = false;
  for (unsigned cycle = 0; cycle < bound && !failed; ++cycle) {
    try {
      const auto result = advance();
      if (result && result->issued_address == address) { saved = timeline->checkpoint(); }
    } catch (const Fault &error) {
      const std::string text(error.what());
      require(text.contains("P:" + hex(address, 6)) && text.contains("pc=0x000100") &&
                  text.contains("omr=0x000305") && timeline->faulted() &&
                  machine.cpu().progress() == before &&
                  rejects([&] { static_cast<void>(timeline->checkpoint()); }).contains("healthy"),
              "Unknown actual opcode/extension remains a contextual quarantined timeline failure");
      failed = true;
    }
  }
  if (!failed || !saved) { throw Fault("Unknown instruction must fail after its saved F1 issue"); }
  const auto stopped = machine.cpu().registers();
  const auto stopped_progress = machine.cpu().progress();
  const auto ticks = machine.clock().elapsed_ticks();
  static_cast<void>(rejects([&] { static_cast<void>(advance()); }));
  require(machine.cpu().registers() == stopped && machine.cpu().progress() == stopped_progress &&
              machine.clock().elapsed_ticks() == ticks,
          "A quarantined timeline cannot resume native work or consume another period");
  timeline = std::make_unique<OwnDspTimeline>(machine, saved.value());
  machine.bus().write({.address = address, .value = extension ? first : 0U});
  for (unsigned cycle = 0;
       cycle < bound && machine.cpu().progress().instructions == before.instructions;
       ++cycle) {
    static_cast<void>(advance());
  }
  require(machine.cpu().progress().instructions == before.instructions + 1 &&
              machine.cpu().registers().pc == program + (extension ? 2U : 1U) &&
              (!extension || machine.cpu().registers().r[0] == first),
          "Trusted pre-F2 restore and real supplied memory permit only the original native path");
}

void program_data_and_clock_guards() {
  OwnDspMachine machine(native::native_clocks);
  native::NativeProgram code;
  constexpr unsigned absolute_program_load = 0x07f084;
  code.word(absolute_program_load);
  code.word(program);
  code.word(0);
  native::reset_native(machine, code);
  machine.bus().write({.address = program, .value = first});
  OwnDspFetch fetch(machine);
  read_words(fetch, 2);
  const auto actual = fetch.begin();
  if (!actual.pending) { throw Fault("Expected an actual P-space data read"); }
  const auto view = fetch.view();
  static_cast<void>(rejects([&] { fetch.capture(); }));
  require(actual.pending->effect.kind == dsp56300::EffectKind::read &&
              actual.pending->effect.space == dsp56300::Space::program &&
              actual.pending->effect.address == program && fetch.view() == view,
          "A native P-data operand cannot be relabeled as an F2 opcode/extension capture");
  machine.bus().write({.address = program, .value = second});
  finish(machine, machine.advance(actual.pending->ticket));
  fetch.synchronize();
  require(machine.cpu().registers().x == second,
          "The native data effect reads actual current memory after the rejected program capture");
  OwnDspMachine clocked(native::native_clocks);
  native::NativeProgram nop;
  nop.word(0);
  native::reset_native(clocked, nop);
  OwnDspFetch frontend(clocked);
  const auto before = clocked.cpu().progress();
  const auto ticks = clocked.clock().elapsed_ticks();
  static_cast<void>(rejects([&] { static_cast<void>(clocked.begin_fetch()); }));
  read_words(frontend);
  require(clocked.cpu().progress() == before && clocked.clock().elapsed_ticks() == ticks,
          "Competing native ownership rejects; untimed F1/F2 does not replace a clock period");
  constexpr unsigned reference_period = 45;
  require(clocked.advance_reference_cycle() == reference_period &&
              clocked.clock().elapsed_ticks() == ticks + reference_period &&
              clocked.cpu().progress() == before,
          "Only an explicit reference-cycle call advances the independently declared clock");
  execute(clocked, frontend);
  require(clocked.cpu().registers().pc == native::boot + 1 &&
              clocked.cpu().progress().estimated_cycles == 1,
          "Rejected ownership and separate clock progress preserve the actual captured NOP");
}
}

int main() {
  try {
    opcode_and_restore();
    opcode_length_and_zero();
    extension_sampling();
    nonprogram_and_ticket_guards();
    held_unknown_snapshot(false);
    held_unknown_snapshot(true);
    unknown_program_and_recovery(false);
    unknown_program_and_recovery(true);
    program_data_and_clock_guards();
    std::cout
        << "Actual program fetch latches, native decoding, DMA and coordinated restore passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
