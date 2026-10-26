// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_peripheral_read.hpp"
#include "dsp/dsp_peripheral_read_stage.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <variant>

namespace blofeld::test::loop_port {
namespace {
constexpr unsigned setup = 0x100;
constexpr unsigned body = setup + 2;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned htx = 0xffff93;
constexpr unsigned hrx = 0xffff94;
constexpr unsigned configuration = 0xa9;
constexpr unsigned first_word = 0x123456;
constexpr unsigned second_word = 0x654321;
constexpr unsigned not_empty = 0x20000;
constexpr unsigned old_la = 0x321;
constexpr unsigned old_lc = 7;
constexpr unsigned loop_flag = 0x8000;
constexpr unsigned immediate_do_two = 0x060280;
// NXP asm56300 pinned encoding: jset #17,x:<<$ffff91,target.
constexpr unsigned status_branch = 0x0191b1;
constexpr unsigned first_target = 0x200;
constexpr unsigned second_target = 0x300;
constexpr unsigned guards = 4;
constexpr auto execution = DspReferenceExecution::access_and_retirement_boundaries;

void spi_word(DspBus &bus, const unsigned value) {
  for (unsigned remaining = word_bits; remaining != 0; --remaining) {
    const bool bit = ((value >> (remaining - 1)) & 1U) != 0;
    bus.shi().observe_spi({.selected = true, .clock = true, .mosi = bit});
    bus.shi().observe_spi({.selected = true, .mosi = bit});
  }
}

void receive(DspBus &bus) {
  bus.shi().observe_spi({.mosi = false});
  bus.write({.space = DspSpace::x, .address = hcsr, .value = configuration});
  bus.write({.space = DspSpace::x, .address = htx, .value = first_word});
  bus.write({.space = DspSpace::x, .address = htx, .value = second_word});
  bus.shi().observe_spi({.selected = true, .mosi = false});
  spi_word(bus, first_word);
  spi_word(bus, second_word);
}

void read(DspProgramFetch &fetch, DspBus &bus) {
  fetch.issue();
  fetch.capture(bus);
}

void component_open(DspProgramFetch &fetch, DspBus &bus, const unsigned size) {
  bus.write({.address = setup, .value = immediate_do_two});
  bus.write({.address = setup + 1, .value = body + size - 1});
  fetch.start(setup, reset_omr);
  read(fetch, bus);
  read(fetch, bus);
  const auto pair = fetch.instruction(fetch.select(setup), 2);
  if (!pair.extension) { throw Fault("Own component DO lost its actual extension"); }
  // These are independent component control fixtures, not injected CPU results.
  const DspLoopOpening opening{.word = pair.opcode.word,
                               .extension = pair.extension->word,
                               .before = {.pc = setup,
                                          .omr = reset_omr,
                                          .la = old_la,
                                          .lc = old_lc,
                                          .status = masked_status},
                               .after = {.pc = body,
                                         .omr = reset_omr,
                                         .la = body + size - 1,
                                         .lc = 2,
                                         .sp = 2,
                                         .sc = 2,
                                         .status = masked_status | loop_flag}};
  static_cast<void>(fetch.consume(setup));
  static_cast<void>(fetch.consume(setup + 1));
  fetch.open_loop(opening, pair.opcode, *pair.extension);
  require(fetch.confirm_opening(opening.after, false), "Own component opening must confirm");
}

void two_live_responses() {
  DspBus bus;
  DspProgram reader;
  reader.emit("movep x:<<$ffff94,x0");
  require(reader.pc() == boot + 1, "Own HRX register instruction must occupy one word");
  reader.install(bus);
  const auto opcode_value = bus.read({.address = boot});
  bus.write({.address = body, .value = opcode_value});
  receive(bus);
  DspProgramFetch fetch;
  component_open(fetch, bus, 1);
  read(fetch, bus);
  const auto first = fetch.instruction(fetch.select(body), 1);
  read(fetch, bus);
  const auto younger = fetch.queued(1);
  if (!younger) { throw Fault("Missing real younger HRX opcode occurrence"); }
  const auto second = fetch.instruction(*younger, 1);
  auto earlier = DspPeripheralRead::capture(bus, fetch, {.address = body}, first.opcode.occurrence);
  auto later = DspPeripheralRead::capture(
      bus, fetch, {.address = body, .epoch = 1}, second.opcode.occurrence);
  require((bus.read({.space = DspSpace::x, .address = hcsr}) & not_empty) == 0,
          "Two live actual same-PC captures must pop exactly two real SPI receive words");
  const DspNativeAccess opcode{.kind = DspNativeAccessKind::opcode,
                               .stage = DspNativeAccessStage::after,
                               .target = {.address = body},
                               .value = opcode_value,
                               .pc = body,
                               .omr = reset_omr};
  const DspNativeAccess data{.kind = DspNativeAccessKind::read,
                             .target = {.space = DspSpace::x, .address = hrx},
                             .ordinal = 1,
                             .pc = body,
                             .omr = reset_omr};
  static_cast<void>(fetch.consume(body));
  earlier.fetched().check_consumed(fetch.consumed_pair(), reset_omr);
  static_cast<void>(
      checkpoint::rejects([&] { later.fetched().check_consumed(fetch.consumed_pair(), reset_omr); },
                          "An equal-PC future HRX sample cannot answer the older native word"));
  require(earlier.consume(opcode, std::nullopt, data, 2) == first_word,
          "Oldest live response must preserve the independent first SPI word");
  require(fetch.confirm_loop({.pc = body,
                              .omr = reset_omr,
                              .la = body,
                              .lc = 1,
                              .sp = 2,
                              .sc = 2,
                              .status = masked_status | loop_flag},
                             true),
          "Own single-word first end must confirm before the next actual admission");
  static_cast<void>(fetch.consume(body));
  later.fetched().check_consumed(fetch.consumed_pair(), reset_omr);
  require(later.consume(opcode, std::nullopt, data, 3) == second_word,
          "Younger live response must preserve the independent second SPI word");
  static_cast<void>(checkpoint::rejects(
      [&] { static_cast<void>(earlier.consume(opcode, std::nullopt, data, 3)); },
      "A consumed real response cannot be replayed to fabricate another FIFO read"));
}

void two_word_occurrences() {
  DspBus bus;
  receive(bus);
  bus.write({.address = body, .value = status_branch});
  bus.write({.address = body + 1, .value = first_target});
  DspProgramFetch fetch;
  component_open(fetch, bus, 2);
  read(fetch, bus);
  read(fetch, bus);
  const auto first = fetch.instruction(fetch.select(body), 2);
  bus.write({.address = body + 1, .value = second_target});
  read(fetch, bus);
  read(fetch, bus);
  const auto second_opcode = fetch.queued(2);
  if (!second_opcode) { throw Fault("Own second loop occurrence was not actually fetched"); }
  const auto second = fetch.instruction(*second_opcode, 2);
  static_cast<void>(
      checkpoint::rejects([&] { static_cast<void>(fetch.instruction(fetch.select(body), 2)); },
                          "An ambiguous address must not select the first iteration"));
  auto first_read =
      DspPeripheralRead::capture(bus, fetch, {.address = body}, first.opcode.occurrence);
  auto second_read = DspPeripheralRead::capture(
      bus, fetch, {.address = body, .epoch = 1}, second.opcode.occurrence);
  bus.write({.address = body + 1, .value = old_la});
  require(first_read.preview_branch(1).next_pc == first_target &&
              second_read.preview_branch(2).next_pc == second_target,
          "Each real status response must retain its own independently changed F2 extension");
  first_read.fetched().check_front(fetch);
  static_cast<void>(checkpoint::rejects([&] { second_read.fetched().check_front(fetch); },
                                        "Younger equal-PC words cannot own the current native E1"));
  static_cast<void>(fetch.consume(body));
  static_cast<void>(fetch.consume(body + 1));
  first_read.fetched().check_consumed(fetch.consumed_pair(), reset_omr);
  static_cast<void>(checkpoint::rejects(
      [&] { second_read.fetched().check_consumed(fetch.consumed_pair(), reset_omr); },
      "Identical opcode/PC does not authorize a different captured response"));
  require(fetch.confirm_loop({.pc = body,
                              .omr = reset_omr,
                              .la = body + 1,
                              .lc = 1,
                              .sp = 2,
                              .sc = 2,
                              .status = masked_status | loop_flag},
                             true),
          "Own first component loop end must confirm before later word consumption");
  second_read.fetched().check_front(fetch);
  static_cast<void>(fetch.consume(body));
  static_cast<void>(fetch.consume(body + 1));
  second_read.fetched().check_consumed(fetch.consumed_pair(), reset_omr);
  require(bus.read({.space = DspSpace::x, .address = hrx}) == first_word,
          "Two real status captures cannot silently drain the unrelated receive FIFO");
}

DspReferencePending pending(const DspMachine &machine) {
  const auto value = machine.reference_pending();
  if (!value) { throw Fault("Own loop lost its actual native boundary"); }
  return *value;
}

void fill(DspMachine &machine, const std::size_t count) {
  while (machine.reference_fetch().used < count) {
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::read));
  }
}

void epoch(DspMachine &machine, const std::uint64_t selected) {
  const auto previous = machine.bus().last_dma_cycle();
  for (auto value = previous ? *previous + 1 : 0; value <= selected; ++value) {
    require(!machine.advance_dma({.cycle = value,
                                  .known_requests = 15,
                                  .timing = DspDmaTiming::reference_sampled_cycles,
                                  .grant = DspDmaGrant::available}),
            "Own explicit phase must not execute a DMA transfer");
  }
}

void finish_read(DspMachine &machine,
                 const DspPeripheralReadTicket &ticket,
                 const unsigned next,
                 const unsigned value) {
  const auto before = machine.cpu().debug_state();
  auto progress = machine.advance_reference_access(pending(machine).ticket);
  require(progress.pending && !progress.retired, "Actual native opcode must await its HRX data");
  progress = machine.advance_reference_peripheral_read(ticket, pending(machine).ticket);
  require(progress.pending && machine.cpu().debug_state() == before,
          "Actual cached HRX response must remain a held native result until retirement");
  if (!progress.pending) { throw Fault("Actual port read lost its held native result"); }
  const auto *retire = std::get_if<DspInstructionRetire>(&progress.pending->effect);
  require(retire != nullptr && retire->next_pc == next,
          "Real native loop control must choose the independent next PC");
  progress = machine.advance_reference_access(pending(machine).ticket);
  require(progress.retired && machine.cpu().state().pc == next &&
              (machine.cpu().state().x & word_mask) == value,
          "Real native retirement must publish exactly its captured FIFO word and loop PC");
}

void native_fifo_loop() {
  DspProgram program;
  program.immediate("sr", masked_status);
  program.immediate("la", old_la);
  program.immediate("lc", old_lc);
  for (unsigned index = 0; index < guards; ++index) { program.word(0); }
  const auto first = program.pc() + 2;
  program.word(immediate_do_two);
  program.word(first);
  program.emit("movep x:<<$ffff94,x0");
  for (unsigned index = 0; index < 16; ++index) { program.word(0); }
  DspMachine machine;
  program.install(machine.bus());
  checkpoint::run(machine, 3 + guards);
  const auto before = machine.cpu().debug_state();
  receive(machine.bus());
  static_cast<void>(machine.begin_reference_instruction(execution));
  static_cast<void>(machine.start_reference_fetch(pending(machine).ticket));
  fill(machine, 2);
  static_cast<void>(machine.advance_reference_access(pending(machine).ticket));
  static_cast<void>(machine.advance_reference_access(pending(machine).ticket));
  fill(machine, dsp_program_fetch_capacity);
  require(machine.advance_reference_access(pending(machine).ticket).retired,
          "Actual native DO setup must retire before the first real reader");
  static_cast<void>(machine.begin_reference_instruction(execution));
  epoch(machine, 0);
  const auto captured = machine.capture_reference_peripheral_read({.address = first});
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    const auto current = machine.rebind_reference_peripheral_read(captured);
    static_cast<void>(checkpoint::rejects(
        [&] {
          static_cast<void>(
              machine.advance_reference_peripheral_read(captured, pending(machine).ticket));
        },
        "A retained real response still requires its current restored owner"));
    epoch(machine, 2);
    finish_read(machine, current, first, first_word);
    static_cast<void>(machine.begin_reference_instruction(execution));
    epoch(machine, 3);
    const auto last = machine.capture_reference_peripheral_read({.address = first, .epoch = 3});
    epoch(machine, 5);
    finish_read(machine, last, first + 1, second_word);
    auto expected = before;
    expected.cpu.pc = first + 1;
    expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | second_word;
    expected.stack_high.at(1) = old_la;
    expected.stack_low.at(1) = old_lc;
    expected.stack_high.at(2) = first;
    expected.stack_low.at(2) = before.cpu.sr;
    expected.native_instructions += 3;
    const auto actual = machine.cpu().debug_state();
    require(
        actual.cpu == expected.cpu && actual.stack_high == expected.stack_high &&
            actual.stack_low == expected.stack_low && actual.sc == expected.sc &&
            actual.ep == expected.ep && actual.sz == expected.sz &&
            actual.native_instructions == expected.native_instructions &&
            (machine.bus().read({.space = DspSpace::x, .address = hcsr}) & not_empty) == 0,
        "Actual repeated native HRX reads must preserve independent CPU/frame/FIFO expectations");
  }
}
}
}

int main() {
  try {
    blofeld::test::loop_port::two_live_responses();
    blofeld::test::loop_port::two_word_occurrences();
    blofeld::test::loop_port::native_fifo_loop();
    std::cout << "Actual repeated-PC port pairs and native FIFO-loop restore checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
