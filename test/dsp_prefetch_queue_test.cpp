// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_shi.hpp"
#include "dma_fixture.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
namespace access = blofeld::test::reference;
namespace dma = blofeld::test::dma_test;
using native_access::fault;
constexpr unsigned program = 0x100;
constexpr unsigned source = 0x800;
constexpr unsigned program_copy = 0x88024a;
constexpr unsigned initial_r0 = 0x7e;
constexpr unsigned old_r0 = 0x12;
constexpr unsigned new_r0 = 0x34;
constexpr unsigned changed_extension = 0x654321;

struct Replacement {
  unsigned address;
  unsigned value;
};

DspReferencePending pending(const DspMachine &machine) {
  const auto result = machine.reference_pending();
  if (!result) { throw Fault("Expected a native program-fetch admission boundary"); }
  return *result;
}

void prepare(DspMachine &machine,
             const std::span<const std::uint32_t> words,
             const Replacement replacement) {
  DspProgram setup;
  setup.immediate("r0", initial_r0);
  for (const auto store : {DspStore{.address = dma::source0, .value = source},
                           DspStore{.address = dma::destination0, .value = replacement.address},
                           DspStore{.address = dma::counter0},
                           DspStore{.address = dma::control0, .value = program_copy}}) {
    setup.emit("movep #>" + number(store.value) + ",x:<<" + number(store.address));
  }
  setup.word(access::long_jump);
  setup.word(program);
  setup.install(machine.bus());
  checkpoint::ram(machine.bus(), program, words);
  machine.bus().write({.address = source, .value = replacement.value});
  static_cast<void>(machine.advance_dma(banks::inputs({.index = 0}), banks::use()));
  checkpoint::run(machine, 6);
  require(machine.cpu().state().pc == program && machine.cpu().state().r[0] == initial_r0,
          "Native MOVEP instructions configure real P-to-P DMA before entering RAM");
}

void fetch_word(DspMachine &machine) {
  const auto before = machine.reference_fetch();
  const auto cpu = machine.cpu().debug_state();
  const auto issued = machine.advance_reference_fetch(DspProgramFetchStage::address);
  require(issued.used == before.used && issued.pending_address == before.next_address,
          "F1 generates the next sequential address without a memory sample");
  const auto sampled = machine.advance_reference_fetch(DspProgramFetchStage::read);
  require(sampled.used == before.used + 1 && !sampled.pending_address &&
              sampled.words.at(before.used).address == before.next_address &&
              machine.cpu().debug_state() == cpu,
          "F2 reads one strict program word without advancing native execution");
}

void overwrite(DspMachine &machine, const Replacement replacement) {
  const auto before = machine.cpu().debug_state();
  const auto sampled =
      machine.advance_dma(banks::inputs({.index = 1, .requests = 1}), banks::use());
  const auto written =
      machine.advance_dma(banks::inputs({.index = 2, .requests = 1}), banks::use());
  require(sampled.phase && sampled.phase->kind == DspDmaPhaseKind::source_read && written.word &&
              written.word->value == replacement.value &&
              machine.bus().read({.address = replacement.address}) == replacement.value &&
              machine.cpu().debug_state() == before,
          "Actual DMA reads its configured source and overwrites RAM while native execution waits");
}

std::vector<std::uint32_t> data_sequence() {
  auto words = checkpoint::assemble("movep x:<<$ffff94,x0");
  const auto immediate = checkpoint::assemble("move #<$12,r0");
  const auto extended = checkpoint::assemble("move #>$123456,r1");
  require(words.size() == 1 && immediate.size() == 1 && extended.size() == 2,
          "Independent one/one/two-word instruction lengths");
  words.insert(words.end(), immediate.begin(), immediate.end());
  words.insert(words.end(), extended.begin(), extended.end());
  words.push_back(0);
  return words;
}

void overlapped_dma(const bool extension_word, const bool before_capture) {
  const auto words = data_sequence();
  const auto changed_opcode = checkpoint::assemble("move #<$34,r0").front();
  const Replacement replacement{.address = program + (extension_word ? 3U : 1U),
                                .value = extension_word ? changed_extension : changed_opcode};
  DspMachine machine;
  prepare(machine, words, replacement);
  machine.bus().shi().observe_spi({.mosi = false});
  constexpr unsigned shi_control = 0xffff91;
  constexpr unsigned shi_configuration = 0xa9;
  machine.bus().write({.space = DspSpace::x, .address = shi_control, .value = shi_configuration});
  access::send(machine.bus().shi(), access::first);
  access::send(machine.bus().shi(), access::second);
  auto progress = machine.begin_reference_instruction(access::boundaries);
  static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
  fetch_word(machine);
  progress = machine.advance_reference_access(access::ticket(progress));
  require(access::request(progress).kind == DspNativeAccessKind::read &&
              access::request(progress).target.address == access::fifo,
          "Native execution waits at its real data read while frontend fetches later instructions");
  while (machine.reference_fetch().next_address < replacement.address) { fetch_word(machine); }
  if (before_capture) {
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
  } else {
    fetch_word(machine);
  }
  const auto saved_fetch = machine.reference_fetch();
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    require(machine.reference_fetch() == saved_fetch,
            "Coordinated restore retains ready words and the separate F1 address");
    overwrite(machine, replacement);
    if (before_capture) {
      static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::read));
    }
    while (machine.reference_fetch().next_address < program + words.size()) { fetch_word(machine); }
    progress = machine.advance_reference_access(access::ticket(machine));
    require(progress.retired && (machine.cpu().state().x & word_mask) == access::first &&
                machine.bus().read({.space = DspSpace::x, .address = access::fifo}) ==
                    access::second,
            "Only the actual data admission consumes the first FIFO word, exactly once");
    const auto before = machine.cpu().debug_state();
    access::execute(machine);
    access::execute(machine);
    const auto after = machine.cpu().debug_state();
    require(after.cpu.r[0] == (!extension_word && before_capture ? new_r0 : old_r0) &&
                after.cpu.r[1] ==
                    (extension_word && before_capture ? changed_extension : access::first) &&
                after.cpu.pc == program + 4 && after.cpu.sr == before.cpu.sr &&
                after.native_instructions == before.native_instructions + 2 &&
                after.native_cycles == before.native_cycles + 3 &&
                machine.reference_fetch().used == 1,
            "Queued native opcode/extension uses actual F2 sampling time and literal ISA results");
    access::execute(machine);
    require(machine.cpu().state().pc == program + words.size() &&
                machine.reference_fetch().used == 0,
            "A queued zero opcode survives across instruction boundaries and retires as NOP");
  }
}

void branch_redirects() {
  for (const bool fallthrough_target : {false, true}) {
    DspMachine machine;
    const auto immediate = checkpoint::assemble("move #<$34,r0").front();
    const auto destination = program + (fallthrough_target ? 2U : 4U);
    const std::array words{access::long_jump, destination, 0U, 0U, 0U, 0U};
    prepare(machine, words, {.address = destination, .value = immediate});
    auto progress = machine.begin_reference_instruction(access::boundaries);
    static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
    for (unsigned index = 0; index < 5; ++index) { fetch_word(machine); }
    const auto issued = machine.advance_reference_fetch(DspProgramFetchStage::address);
    require(issued.pending_address == program + 5, "A younger F1 address is still in flight");
    const auto saved = machine.checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      machine.restore(saved);
      overwrite(machine, {.address = destination, .value = immediate});
      progress = access::finish(machine, machine.advance_reference_access(access::ticket(machine)));
      const auto fetch = machine.reference_fetch();
      require(progress.retired && fetch.active && fetch.used == 0 && !fetch.pending_address &&
                  fetch.next_address == destination && fetch.consume_address == destination &&
                  machine.cpu().state().pc == destination,
              "Actual JMP redirects discard both younger sampled words and the issued F1 address");
      progress = machine.begin_reference_instruction(access::boundaries);
      fetch_word(machine);
      progress =
          access::finish(machine, machine.advance_reference_access(access::ticket(progress)));
      require(
          progress.retired && machine.cpu().state().r[0] == new_r0,
          "A fresh branch-target read observes DMA's replacement, including a fallthrough target");
    }
  }
}

void capacity_and_protocol() {
  DspMachine machine;
  std::array<std::uint32_t, dsp_program_fetch_capacity + 1> words{};
  prepare(machine, words, {.address = program, .value = 0});
  auto progress = machine.begin_reference_instruction(access::boundaries);
  const auto ticket = access::ticket(progress);
  static_cast<void>(fault([&] { static_cast<void>(machine.start_reference_fetch({})); }));
  static_cast<void>(machine.start_reference_fetch(ticket));
  static_cast<void>(fault(
      [&] { static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::read)); }));
  require(fault([&] {
            static_cast<void>(machine.advance_reference_access(ticket));
          }).contains("awaits actual queued read"),
          "Queue starvation cannot silently fall back to a fresh native memory read");
  const auto issued = machine.advance_reference_fetch(DspProgramFetchStage::address);
  static_cast<void>(fault(
      [&] { static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address)); }));
  require(machine.reference_fetch() == issued, "Duplicate F1 issue cannot skip a program address");
  static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::read));
  for (std::size_t index = 1; index < dsp_program_fetch_capacity; ++index) { fetch_word(machine); }
  const auto full = machine.reference_fetch();
  static_cast<void>(fault(
      [&] { static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address)); }));
  static_cast<void>(fault([&] { static_cast<void>(machine.latch_reference_program(ticket)); }));
  require(machine.reference_fetch() == full,
          "Capacity and competing latch errors preserve all actually sampled words");
  progress = machine.advance_reference_access(ticket);
  require(progress.retired && machine.reference_fetch().used == dsp_program_fetch_capacity - 1,
          "Admission releases exactly one queued NOP, restoring frontend capacity");
  static_cast<void>(fault([&] { machine.cpu().step(); }));
  static_cast<void>(
      fault([&] { static_cast<void>(machine.begin_reference_interrupt(access::boundaries)); }));
  require(machine.reference_fetch().used == dsp_program_fetch_capacity - 1,
          "Atomic and unqualified vector paths cannot bypass the owned frontend");
  static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
  require(machine.reference_fetch().pending_address.has_value(),
          "A newly issued F1 address is still outstanding at hardware reset");
  machine.cpu().reset_spi_boot();
  require(!machine.reference_fetch().active && machine.reference_fetch().used == 0 &&
              !machine.reference_fetch().pending_address,
          "Real hardware reset aborts issued addresses and all sampled frontend state");
  static_cast<void>(fault([&] { static_cast<void>(machine.start_reference_fetch(ticket)); }));
}

void unknown_read_and_recovery() {
  DspMachine machine;
  const std::array words{0U};
  prepare(machine, words, {.address = program, .value = 0});
  const auto progress = machine.begin_reference_instruction(access::boundaries);
  static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
  fetch_word(machine);
  static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
  const auto saved = machine.checkpoint();
  const auto before = machine.cpu().debug_state();
  const auto error = fault(
      [&] { static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::read)); });
  require(error.contains("Unknown DSP memory read at P:0x000101") &&
              error.contains("pc=0x000100") && error.contains("omr=0x000305") &&
              machine.cpu().debug_state() == before,
          "Actual speculative F2 errors retain operation, address and native context without "
          "retirement");
  static_cast<void>(fault([&] { static_cast<void>(machine.reference_fetch()); }));
  static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
  machine.restore(saved);
  machine.bus().write({.address = program + 1, .value = 0});
  const auto recovered = machine.advance_reference_fetch(DspProgramFetchStage::read);
  require(recovered.used == 2 && recovered.words[1] == DspProgramWord{.address = program + 1},
          "Trusted restore recovers the original F1 address and already captured NOP");
  static_cast<void>(
      access::finish(machine, machine.advance_reference_access(access::ticket(machine))));
  access::execute(machine);
}

void program_data_is_not_prefetch() {
  // FM MOVEM absolute P:ea,X0; independently checked with NXP asm56300 6.3.28.
  constexpr unsigned program_load = 0x07f084;
  const auto replacement = checkpoint::assemble("move #<$34,r0").front();
  const std::array words{program_load, program + 2, 0U};
  DspMachine machine;
  prepare(machine, words, {.address = program + 2, .value = replacement});
  auto progress = machine.begin_reference_instruction(access::boundaries);
  static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
  for (unsigned index = 0; index < words.size(); ++index) { fetch_word(machine); }
  overwrite(machine, {.address = program + 2, .value = replacement});
  progress = machine.advance_reference_access(access::ticket(progress));
  require(access::request(progress).kind == DspNativeAccessKind::extension,
          "The native instruction consumes its real queued address extension");
  progress = machine.advance_reference_access(access::ticket(progress));
  require(access::request(progress).kind == DspNativeAccessKind::read &&
              access::request(progress).target.space == DspSpace::p &&
              !pending(machine).latched_program_word,
          "A P-space operand is an actual data read, independent of the queued instruction word");
  static_cast<void>(
      fault([&] { static_cast<void>(machine.latch_reference_program(access::ticket(progress))); }));
  progress = machine.advance_reference_access(access::ticket(progress));
  require(
      progress.retired && (machine.cpu().state().x & word_mask) == replacement &&
          machine.reference_fetch().used == 1,
      "The real P-data read sees DMA's replacement while preserving the previously fetched NOP");
  access::execute(machine);
  require(machine.cpu().state().r[0] == initial_r0 && machine.cpu().state().pc == program + 3,
          "The same address's older actual instruction sample executes as NOP");
}

void control_frontiers() {
  for (const char *instruction : {"rep #3", "wait", "reset"}) {
    DspMachine machine;
    const auto control = checkpoint::assemble(instruction);
    prepare(machine, control, {.address = program, .value = 0});
    const auto progress = machine.begin_reference_instruction(access::boundaries);
    static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
    fetch_word(machine);
    const auto before = machine.cpu().debug_state();
    const auto fetch = machine.reference_fetch();
    const auto error = fault(
        [&] { static_cast<void>(machine.advance_reference_access(access::ticket(progress))); });
    require(error.contains("control opcode at P:0x000100") && error.contains("pc=0x000100") &&
                error.contains("omr=0x000305") && machine.cpu().debug_state() == before &&
                machine.reference_fetch() == fetch,
            "Unqualified loop/wait/reset pipeline behavior rejects before native effects");
  }
}

void conditional_flow() {
  for (const bool taken : {false, true}) {
    const auto branch = checkpoint::assemble(taken ? "jne $104" : "jeq $104");
    require(branch.size() == 1, "Independent Jcc short absolute instruction length");
    const std::array words{branch.front(), 0U, 0U, 0U, 0U};
    DspMachine machine;
    prepare(machine, words, {.address = program, .value = 0});
    const auto progress = machine.begin_reference_instruction(access::boundaries);
    static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
    for (unsigned index = 0; index < words.size(); ++index) { fetch_word(machine); }
    require((machine.cpu().state().sr & zero) == 0, "Literal Z=0 selects JNE and rejects JEQ");
    const auto saved = machine.checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      machine.restore(saved);
      const auto retired = machine.advance_reference_access(access::ticket(machine));
      const auto fetch = machine.reference_fetch();
      require(retired.retired && machine.cpu().state().pc == program + (taken ? 4U : 1U) &&
                  fetch.used == (taken ? 0U : words.size() - 1) &&
                  fetch.next_address == program + (taken ? 4U : words.size()),
              "Taken conditional flow redirects; actual fallthrough retains its younger samples");
    }
  }
}

struct ConditionalCase {
  bool taken;
  bool bit_operand;
};

void conditional_dma_replay(DspMachine &machine,
                            const ConditionalCase scenario,
                            const Replacement replacement) {
  const auto before = machine.cpu().debug_state();
  overwrite(machine, replacement);
  auto progress = machine.advance_reference_access(access::ticket(machine));
  const auto cpu = machine.cpu().state();
  const auto fetch = machine.reference_fetch();
  require(progress.retired && cpu.pc == replacement.address && cpu.sr == before.cpu.sr &&
              cpu.sp == (!scenario.bit_operand && scenario.taken ? 1U : 0U) &&
              (scenario.bit_operand || !scenario.taken || cpu.stack_pc == replacement.address) &&
              fetch.used == (scenario.taken ? 0U : 3U) &&
              fetch.pending_address.has_value() != scenario.taken,
          "Actual memory condition or conditional call preserves or redirects despite identical "
          "next PC");
  if (scenario.bit_operand) {
    require(access::completed(progress).value == (scenario.taken ? 1U : 0U),
            "The branch uses the actual independently seeded RAM operand");
  }
  progress = machine.begin_reference_instruction(access::boundaries);
  if (scenario.taken) { fetch_word(machine); }
  progress = machine.advance_reference_access(access::ticket(progress));
  require(progress.retired &&
              machine.cpu().state().r[0] == (scenario.taken ? new_r0 : initial_r0) &&
              machine.reference_fetch().used == (scenario.taken ? 0U : 2U),
          "The following ordinary opcode sees either genuine DMA refetch or its older queued NOP");
}

void conditional_dma_case(const ConditionalCase scenario) {
  constexpr unsigned operand = 0x20;
  const char *text = "jset #0,x:$20,$102";
  if (!scenario.bit_operand) { text = scenario.taken ? "jsne $101" : "jseq $101"; }
  const auto branch = checkpoint::assemble(text);
  require(branch.size() == (scenario.bit_operand ? 2U : 1U),
          "Independent JSET memory and short conditional JSR instruction lengths");
  const auto replacement = checkpoint::assemble("move #<$34,r0").front();
  const auto destination = program + static_cast<unsigned>(branch.size());
  auto words = branch;
  words.insert(words.end(), 3, 0);
  DspMachine machine;
  prepare(machine, words, {.address = destination, .value = replacement});
  machine.bus().write(
      {.space = DspSpace::x, .address = operand, .value = scenario.taken ? 1U : 0U});
  auto progress = machine.begin_reference_instruction(access::boundaries);
  static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
  for (std::size_t index = 0; index < words.size(); ++index) { fetch_word(machine); }
  static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
  if (scenario.bit_operand) {
    progress = machine.advance_reference_access(access::ticket(progress));
    progress = machine.advance_reference_access(access::ticket(progress));
    require(access::request(progress).kind == DspNativeAccessKind::read &&
                access::request(progress).target.space == DspSpace::x &&
                access::request(progress).target.address == operand,
            "The actual RAM bit-test operand suspends the native branch decision");
  }
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    conditional_dma_replay(machine, scenario, {.address = destination, .value = replacement});
  }
}

void conditional_dma() {
  for (const bool taken : {false, true}) {
    for (const bool bit_operand : {false, true}) {
      conditional_dma_case({.taken = taken, .bit_operand = bit_operand});
    }
  }
}

void clocked_owner_guard() {
  constexpr unsigned extal = 12000000;
  DspMachine machine(
      DspRunContract{.clock = {.extal_hz = extal,
                               .timebase_hz = extal,
                               .switch_timing = DspClockSwitch::reference_instruction_boundary},
                     .timing = DspRunTiming::native_internal_estimates});
  DspProgram code;
  code.emit("nop");
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  const auto before = machine.cpu().debug_state();
  static_cast<void>(fault([&] { static_cast<void>(machine.start_reference_fetch({})); }));
  static_cast<void>(fault(
      [&] { static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address)); }));
  require(machine.cpu().debug_state() == before && !machine.timing_faulted() &&
              !machine.reference_fetch().active,
          "Explicit F1/F2 operations cannot replace an existing completion-clock contract");
  machine.cpu().step();
  require(machine.run_clock().native_cycle_units() == 1 && machine.cpu().state().pc == boot + 1,
          "The rejected frontend preserves genuine ordinary clocked execution");
}

void call_and_return() {
  const auto call = checkpoint::assemble("jsr $104");
  const auto return_instruction = checkpoint::assemble("rts");
  require(call.size() == 1 && return_instruction.size() == 1,
          "Independent short JSR and RTS instruction lengths");
  const auto replacement = checkpoint::assemble("move #<$34,r0").front();
  const std::array words{call.front(), 0U, 0U, 0U, return_instruction.front(), 0U};
  DspMachine machine;
  prepare(machine, words, {.address = program + 1, .value = replacement});
  auto progress = machine.begin_reference_instruction(access::boundaries);
  static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
  for (unsigned index = 0; index < words.size(); ++index) { fetch_word(machine); }
  overwrite(machine, {.address = program + 1, .value = replacement});
  progress = access::finish(machine, machine.advance_reference_access(access::ticket(progress)));
  require(progress.retired && machine.cpu().state().pc == program + 4 &&
              machine.cpu().state().sp == 1 && machine.cpu().state().stack_pc == program + 1 &&
              machine.reference_fetch().used == 0,
          "Native JSR pushes the literal return PC and redirects the frontend to its target");
  fetch_word(machine);
  static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
  access::execute(machine);
  require(machine.cpu().state().pc == program + 1 && machine.cpu().state().sp == 0 &&
              machine.reference_fetch().used == 0 && !machine.reference_fetch().pending_address &&
              machine.reference_fetch().next_address == program + 1,
          "Native RTS pops the actual stack and discards the younger issued address");
  progress = machine.begin_reference_instruction(access::boundaries);
  fetch_word(machine);
  static_cast<void>(
      access::finish(machine, machine.advance_reference_access(access::ticket(progress))));
  require(machine.cpu().state().r[0] == new_r0 && machine.cpu().state().pc == program + 2,
          "Return-target refetch observes the actual DMA overwrite of the earlier sequential NOP");
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
    unknown_read_and_recovery();
    program_data_is_not_prefetch();
    control_frontiers();
    conditional_flow();
    conditional_dma();
    clocked_owner_guard();
    call_and_return();
    std::cout << "DSP queued program fetch checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
