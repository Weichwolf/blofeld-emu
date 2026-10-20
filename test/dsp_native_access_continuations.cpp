// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp56kEmu/memory.h"
#include "dsp56kEmu/types.h"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"

#include <string>

namespace blofeld::test::native_access {
namespace {
void repeat_and_preview() {
  constexpr unsigned absolute_x0 = 0x449000; // FM13-118: W=1, X:aa=$10, X0, MOVE.
  const DspRunContract timing{
      .clock = {.extal_hz = 12000000,
                .timebase_hz = 540000000,
                .switch_timing = DspClockSwitch::reference_instruction_boundary},
      .timing = DspRunTiming::native_internal_estimates};
  DspMachine machine(timing);
  DspProgram code;
  code.emit("rep #3");
  const auto body = code.pc();
  code.emit("move x:<$10,x0");
  code.emit("nop");
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().write({.space = DspSpace::x, .address = 0x10, .value = first});
  machine.cpu().step();
  Recorder capture;
  const auto before = machine.cpu().debug_state();
  const auto plan = machine.plan_step();
  require(machine.cpu().debug_state() == before && plan.before == before && capture.rows.empty(),
          "Pure native timing preview has no execution or retained access observer");
  capture.step(machine);
  require(capture.rows.size() == 4 && machine.cpu().state().pc == body &&
              machine.cpu().state().lc == 2 && machine.cpu().state().x == first,
          "The first REP iteration fetches its legal single-word body and reads actual data");
  pair(capture.rows,
       {.kind = Kind::opcode, .target = {.address = body}, .value = absolute_x0, .pc = body});
  pair(capture.rows,
       {.kind = Kind::read,
        .target = {.space = DspSpace::x, .address = 0x10},
        .value = first,
        .offset = 2,
        .ordinal = 1,
        .pc = body});
  const auto loaded = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(loaded);
    for (unsigned remaining = 1; remaining <= 2; ++remaining) {
      capture.step(machine);
      require(capture.rows.size() == 2 && capture.rows.front().kind == Kind::read &&
                  machine.cpu().state().x == first,
              "Latched REP opcode creates no invented fetch while actual data is read each time");
    }
    require(machine.cpu().state().pc == body + 1 && machine.cpu().state().lc == 0 &&
                machine.cpu().debug_state().native_instructions == 4,
            "Both restored REP continuations retain the independent three-iteration result");
    capture.step(machine);
    require(capture.rows.size() == 2, "The following NOP starts a fresh native access ordinal");
    pair(capture.rows,
         {.kind = Kind::opcode, .target = {.address = body + 1}, .value = 0, .pc = body + 1});
  }
}

struct VectorInput {
  unsigned opcode;
  unsigned operand;
  unsigned instructions;
};

void failed_vector_observer(DspMachine &machine) {
  constexpr unsigned second_vector_word = 0x55;
  Recorder capture;
  const DspNativeAccessObserver reject{.observe = [&](const DspNativeAccess &event) {
    capture.rows.push_back(event);
    if (event.kind == Kind::vector && event.target.address == second_vector_word &&
        event.stage == Stage::after) {
      throw Fault("Own failure after fetched vector pair");
    }
  }};
  const auto error = fault([&] { machine.cpu().step(reject); });
  require(error.contains("after vector at P:0x000055 ordinal=1") && capture.rows.size() == 4 &&
              capture.rows.back().value.has_value() &&
              machine.cpu().debug_state().native_instructions == 5 &&
              machine.bus().interrupt_request(0).has_value(),
          "Failure after the second real vector read cannot acknowledge or execute its words");
  static_cast<void>(fault([&] { machine.cpu().step(); }));
}

void interrupt_fetch(const VectorInput input) {
  constexpr unsigned ordinary_status = 0xc00000;
  constexpr unsigned vector = 0x54;
  constexpr unsigned priority = 0xfffffe;
  constexpr unsigned counter = 0xffff8c;
  constexpr unsigned compare = 0xffff8d;
  constexpr unsigned control = 0xffff8f;
  for (const bool complete : {false, true}) {
    DspMachine machine;
    DspProgram code;
    code.immediate("sr", ordinary_status);
    code.emit("movep #>$100,x:<<$fffffe");
    code.emit("movep #>1,x:<<$ffff8e");
    code.emit("movep #>1,x:<<$ffff8d");
    code.emit("movep #>7,x:<<$ffff8f");
    const auto foreground = code.pc();
    code.emit("nop");
    code.install(machine.bus());
    machine.cpu().reset_spi_boot();
    machine.bus().write({.address = vector, .value = input.opcode});
    if (complete) { machine.bus().write({.address = vector + 1, .value = input.operand}); }
    execute(machine, 5);
    require(machine.bus().read({.space = DspSpace::x, .address = priority}) == 0x100 &&
                machine.bus().read({.space = DspSpace::x, .address = counter}) == 0 &&
                machine.bus().read({.space = DspSpace::x, .address = compare}) == 1,
            "Native setup supplies a real timer source and low interrupt priority");
    machine.bus().timers().advance_clk2(1);
    require(machine.bus().read({.space = DspSpace::x, .address = counter}) == 1 &&
                machine.bus().interrupt_request(0).has_value(),
            "One explicit CLK/2 edge applies the write-only load and reaches its real compare");
    const auto pending = machine.checkpoint();
    Recorder capture;
    if (!complete) {
      const auto error = fault([&] { capture.step(machine); });
      require(error.contains("P:0x000055") && capture.rows.size() == 3 &&
                  capture.rows.back().kind == Kind::vector &&
                  capture.rows.back().stage == Stage::before &&
                  machine.bus().interrupt_request(0).has_value(),
              "A missing second vector word cannot acknowledge the pending real source");
      machine.restore(pending);
      machine.bus().write({.address = vector + 1, .value = input.operand});
    }
    const auto ready = machine.checkpoint();
    failed_vector_observer(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      machine.restore(ready);
      capture.step(machine);
      require(capture.rows.size() == 4 && machine.cpu().state().pc == foreground &&
                  machine.cpu().state().sp == 0 &&
                  machine.cpu().debug_state().native_instructions == input.instructions &&
                  machine.cpu().state().x == input.operand && !machine.bus().interrupt_request(0) &&
                  machine.bus().timers().read(control, 0x200000U) == 0,
              "Exactly two fetched vector words acknowledge once and retain foreground; events=" +
                  std::to_string(capture.rows.size()));
      pair(capture.rows,
           {.kind = Kind::vector,
            .target = {.address = vector},
            .value = input.opcode,
            .pc = foreground});
      pair(capture.rows,
           {.kind = Kind::vector,
            .target = {.address = vector + 1},
            .value = input.operand,
            .offset = 2,
            .ordinal = 1,
            .pc = foreground});
    }
  }
}

void send(DspShi &shi, const unsigned value) {
  shi.observe_spi({.selected = true, .mosi = false});
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    const bool level = ((value >> (word_bits - 1 - bit)) & 1U) != 0;
    shi.observe_spi({.selected = true, .clock = true, .mosi = level});
    shi.observe_spi({.selected = true, .clock = false, .mosi = level});
  }
  shi.observe_spi({.mosi = false});
}

void destructive_fifo() {
  constexpr unsigned fifo = 0xffff94;
  DspMachine machine;
  DspProgram code;
  code.emit("movep #>$a9,x:<<$ffff91");
  code.emit("btst #0,x:<<$ffff94");
  code.emit("movep x:<<$ffff94,x0");
  code.emit("btst #0,x:<<$ffff94");
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  machine.bus().shi().observe_spi({.mosi = false});
  machine.cpu().step();
  constexpr unsigned first_word = 0x101;
  constexpr unsigned second_word = 0x202;
  send(machine.bus().shi(), first_word);
  send(machine.bus().shi(), second_word);
  Recorder capture;
  capture.step(machine);
  require(capture.rows.size() == 4 && (machine.cpu().state().sr & 1U) != 0,
          "A fallback native bit test consumes the first real FIFO word once");
  pair(capture.rows,
       {.kind = Kind::read,
        .target = {.space = DspSpace::x, .address = fifo},
        .value = first_word,
        .offset = 2,
        .ordinal = 1,
        .pc = boot + 2});
  capture.step(machine);
  require(capture.rows.size() == 4 && machine.cpu().state().x == second_word,
          "A later native read receives the second word, not a duplicate or skipped FIFO entry");
  pair(capture.rows,
       {.kind = Kind::read,
        .target = {.space = DspSpace::x, .address = fifo},
        .value = second_word,
        .offset = 2,
        .ordinal = 1,
        .pc = boot + 3});
  const auto error = fault([&] { capture.step(machine); });
  require(error.contains("empty DSP SHI FIFO") && capture.rows.size() == 3 &&
              capture.rows.back().kind == Kind::read && !capture.rows.back().value,
          "The third access proves exactly two destructive reads occurred");
}
}

void continuation_checks() {
  const dsp56k::DefaultMemoryValidator validator;
  dsp56k::Memory memory(validator, 0x1000);
  require(memory.set(dsp56k::MemArea_P, 7, first) && memory.getExtension(7) == first,
          "The optional native extension hook retains the real P-array fallback");
  repeat_and_preview();
  interrupt_fetch({.opcode = 0, .operand = 0, .instructions = 7});
  interrupt_fetch({.opcode = immediate_x0, .operand = first, .instructions = 6});
  destructive_fifo();
}
}
