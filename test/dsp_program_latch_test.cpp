// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>

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
constexpr unsigned first_r0 = 0x12;
constexpr unsigned second_r0 = 0x34;

struct Replacement {
  unsigned value;
  unsigned destination = program;
};

DspReferencePending pending(const DspMachine &machine) {
  const auto result = machine.reference_pending();
  if (!result) { throw Fault("Expected a pending program admission"); }
  return *result;
}

void jump_to_program(DspProgram &code) {
  // FM JMP ea, absolute extension. The SDK assembler does not accept forced '>'.
  code.word(access::long_jump);
  code.word(program);
}

void prepare(DspMachine &machine,
             const std::span<const std::uint32_t> words,
             const Replacement replacement) {
  DspProgram setup;
  setup.immediate("r0", initial_r0);
  for (const auto store : {DspStore{.address = dma::source0, .value = source},
                           DspStore{.address = dma::destination0, .value = replacement.destination},
                           DspStore{.address = dma::counter0},
                           DspStore{.address = dma::control0, .value = program_copy}}) {
    setup.emit("movep #>" + number(store.value) + ",x:<<" + number(store.address));
  }
  jump_to_program(setup);
  setup.install(machine.bus());
  checkpoint::ram(machine.bus(), program, words);
  machine.bus().write({.address = source, .value = replacement.value});
  static_cast<void>(machine.advance_dma(banks::inputs({.index = 0}), banks::use()));
  checkpoint::run(machine, 6);
  require(machine.cpu().state().pc == program && machine.cpu().state().r[0] == initial_r0,
          "Actual native setup configures P-to-P DMA and jumps to the RAM instruction");
}

void capture_dma_source(DspMachine &machine, const unsigned fetch_address = program) {
  const std::array core{DspMemoryAccess{.target = {.address = fetch_address}}};
  const auto result =
      machine.advance_dma(banks::inputs({.index = 1, .requests = 1}), banks::use(core));
  require(result.phase && result.phase->kind == DspDmaPhaseKind::source_read && !result.word &&
              !result.core_stall,
          "The separately declared P-fetch bank and actual DMA source are independent");
}

void overwrite(DspMachine &machine, const unsigned address, const unsigned value) {
  const auto before = machine.cpu().debug_state();
  const auto result = machine.advance_dma(banks::inputs({.index = 2, .requests = 1}), banks::use());
  require(result.word && result.word->value == value && !result.core_stall &&
              machine.bus().read({.address = address}) == value &&
              machine.cpu().debug_state() == before,
          "Real DMA replaces P-RAM without advancing the suspended native instruction");
}

void opcode_and_restore() {
  const auto old = checkpoint::assemble("move #<$12,r0");
  const auto replacement = checkpoint::assemble("move #<$34,r0");
  require(old.size() == 1 && replacement.size() == 1,
          "Independent short-immediate instruction lengths");
  const std::array words{old.front(), access::long_jump, program};
  for (const bool latch : {false, true}) {
    DspMachine machine;
    prepare(machine, words, {.value = replacement.front()});
    auto progress = machine.begin_reference_instruction(access::boundaries);
    const auto before = machine.cpu().debug_state();
    capture_dma_source(machine);
    const auto original = access::ticket(progress);
    if (latch) {
      const auto captured = machine.latch_reference_program(original);
      require(captured.ticket == original && captured.latched_program_word == old.front() &&
                  machine.cpu().debug_state() == before,
              "The actual opcode word is latched without decoding or retirement");
      require(fault([&] {
                static_cast<void>(machine.latch_reference_program(original));
              }).contains("Unavailable program latch"),
              "A duplicate latch cannot replace the captured word or perform another read");
    }
    const auto saved = machine.checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      machine.restore(saved);
      static_cast<void>(
          fault([&] { static_cast<void>(machine.latch_reference_program(original)); }));
      require(pending(machine).latched_program_word.has_value() == latch,
              "Coordinated restore retains the presence of the actual program latch");
      overwrite(machine, program, replacement.front());
      progress = machine.advance_reference_access(access::ticket(machine));
      const auto after = machine.cpu().debug_state();
      require(
          progress.retired && after.cpu.r[0] == (latch ? first_r0 : second_r0) &&
              after.cpu.pc == program + 1 && after.cpu.sr == before.cpu.sr &&
              after.native_instructions == before.native_instructions + 1 &&
              after.native_cycles == before.native_cycles + 1,
          "Literal register value, unchanged flags and one retirement distinguish fetched time");
      machine.cpu().step();
      machine.cpu().step();
      require(machine.cpu().state().r[0] == second_r0 && machine.cpu().state().pc == program + 1,
              "A later genuine fetch sees the DMA-written opcode rather than a stale latch");
    }
  }
}

void opcode_length_and_zero() {
  const auto long_move = checkpoint::assemble("move #>$123456,r0");
  const auto short_move = checkpoint::assemble("move #<$34,r0");
  const std::array long_words{long_move.front(), long_move.back(), 0U};
  const std::array nop_words{0U, 0U};
  for (const bool zero_opcode : {false, true}) {
    DspMachine machine;
    const auto words = zero_opcode ? std::span<const std::uint32_t>(nop_words)
                                   : std::span<const std::uint32_t>(long_words);
    const auto replacement = zero_opcode ? short_move.front() : 0;
    prepare(machine, words, {.value = replacement});
    auto progress = machine.begin_reference_instruction(access::boundaries);
    const auto before = machine.cpu().debug_state();
    const auto captured = machine.latch_reference_program(access::ticket(progress));
    require(captured.latched_program_word == words.front(), "Zero is a captured word, not absence");
    capture_dma_source(machine);
    overwrite(machine, program, replacement);
    progress = access::finish(machine, machine.advance_reference_access(access::ticket(machine)));
    const auto units = zero_opcode ? 1U : 2U;
    require(
        progress.retired &&
            machine.cpu().state().r[0] == (zero_opcode ? initial_r0 : access::first) &&
            machine.cpu().state().pc == program + units &&
            machine.cpu().debug_state().native_cycles == before.native_cycles + units,
        "Actual decoding and nominal word cost use the fetched opcode, including zero and length");
  }
}

void extension_sampling() {
  const auto instruction = checkpoint::assemble("move #>$123456,r0");
  constexpr unsigned changed = 0x654321;
  const std::array words{instruction.front(), instruction.back(), 0U};
  for (const bool latch : {false, true}) {
    DspMachine machine;
    prepare(machine, words, {.value = changed, .destination = program + 1});
    auto progress = machine.begin_reference_instruction(access::boundaries);
    static_cast<void>(machine.latch_reference_program(access::ticket(progress)));
    progress = machine.advance_reference_access(access::ticket(progress));
    access::expect(progress,
                   {.kind = DspNativeAccessKind::extension,
                    .target = {.address = program + 1},
                    .ordinal = 1,
                    .pc = program});
    require(!pending(machine).latched_program_word,
            "Decoder lookahead does not fabricate a consumed extension latch");
    capture_dma_source(machine, program + 1);
    if (latch) {
      require(machine.latch_reference_program(access::ticket(progress)).latched_program_word ==
                  access::first,
              "The extension's actual read is independently latched after opcode decode");
    }
    const auto saved = machine.checkpoint();
    const auto before = machine.cpu().debug_state();
    for (unsigned replay = 0; replay < 2; ++replay) {
      machine.restore(saved);
      overwrite(machine, program + 1, changed);
      progress = machine.advance_reference_access(access::ticket(machine));
      const auto expected = latch ? access::first : changed;
      require(progress.retired && access::completed(progress).value == expected &&
                  machine.cpu().state().r[0] == expected &&
                  machine.cpu().state().pc == program + 2 &&
                  machine.cpu().debug_state().native_cycles == before.native_cycles + 2,
              "The extension uses its actual capture time, never stale decoder lookahead");
      machine.cpu().step();
      require(machine.cpu().state().pc == program + 3 && !machine.reference_pending(),
              "Retirement releases the consumed program latch before the following NOP");
    }
  }
}

void nonprogram_and_ticket_guards() {
  DspMachine machine;
  DspProgram code;
  code.emit("movep x:<<$ffff94,x0");
  code.emit("nop");
  code.install(machine.bus());
  machine.bus().shi().observe_spi({.mosi = false});
  constexpr unsigned shi_control = 0xffff91;
  constexpr unsigned shi_configuration = 0xa9;
  machine.bus().write({.space = DspSpace::x, .address = shi_control, .value = shi_configuration});
  access::send(machine.bus().shi(), access::first);
  access::send(machine.bus().shi(), access::second);
  auto progress = machine.begin_reference_instruction(access::boundaries);
  const auto original = access::ticket(progress);
  DspMachine foreign;
  code.install(foreign.bus());
  const auto other = access::ticket(foreign.begin_reference_instruction(access::boundaries));
  for (const auto &invalid : {DspReferenceTicket{}, other}) {
    static_cast<void>(fault([&] { static_cast<void>(machine.latch_reference_program(invalid)); }));
  }
  require(access::ticket(machine) == original && !pending(machine).latched_program_word,
          "Default/foreign capabilities cannot trigger a program read");
  progress = machine.advance_reference_access(original);
  const auto before = machine.cpu().debug_state();
  const auto error =
      fault([&] { static_cast<void>(machine.latch_reference_program(access::ticket(progress))); });
  require(error.contains("read at X:0xffff94") && error.contains("pc=0xff0000") &&
              machine.cpu().debug_state() == before &&
              machine.bus().read({.space = DspSpace::x, .address = access::fifo}) == access::first,
          "A forbidden FIFO latch rejects before actual I/O and preserves the first real word");
  progress = machine.advance_reference_access(access::ticket(progress));
  require(progress.retired && (machine.cpu().state().x & word_mask) == access::second,
          "Ordinary data admission still consumes the remaining real FIFO word");
  static_cast<void>(fault([&] { static_cast<void>(machine.latch_reference_program(original)); }));
  progress = machine.begin_reference_instruction(access::boundaries);
  const auto stale = access::ticket(progress);
  static_cast<void>(machine.latch_reference_program(stale));
  machine.cpu().reset_spi_boot();
  static_cast<void>(fault([&] { static_cast<void>(machine.latch_reference_program(stale)); }));
  require(!machine.reference_pending(), "Hardware reset invalidates and clears the captured word");
}

void unknown_program_and_recovery() {
  for (const bool extension_word : {false, true}) {
    DspMachine machine;
    DspProgram entry;
    jump_to_program(entry);
    entry.install(machine.bus());
    if (extension_word) {
      const auto words = checkpoint::assemble("move #>$123456,r0");
      machine.bus().write({.address = program, .value = words.front()});
    }
    machine.cpu().step();
    auto progress = machine.begin_reference_instruction(access::boundaries);
    if (extension_word) { progress = machine.advance_reference_access(access::ticket(progress)); }
    const auto saved = machine.checkpoint();
    const auto before = machine.cpu().debug_state();
    const auto address = program + (extension_word ? 1U : 0U);
    const auto error = fault(
        [&] { static_cast<void>(machine.latch_reference_program(access::ticket(progress))); });
    require(error.contains("Unknown DSP memory read at P:" + hex(address, 6)) &&
                error.contains("pc=0x000100") && error.contains("omr=0x000305") &&
                machine.cpu().debug_state().native_instructions == before.native_instructions,
            "Actual unknown opcode/extension capture faults with address and native context");
    static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
    machine.restore(saved);
    const auto value = extension_word ? access::first : 0;
    machine.bus().write({.address = address, .value = value});
    require(machine.latch_reference_program(access::ticket(machine)).latched_program_word == value,
            "Trusted restore recovers the pending read before actual valid memory is supplied");
    progress = access::finish(machine, machine.advance_reference_access(access::ticket(machine)));
    require(progress.retired && (!extension_word || machine.cpu().state().r[0] == access::first),
            "Recovery executes the independently expected original opcode/extension path");
  }
}

void program_data_and_clock_guards() {
  // FM MOVE(M) P:ea,X0: W=1, MMM=110, RRR=000, dddddd=000100.
  // Independently assembled by NXP6.3.28; the SDK assembler omits this form.
  constexpr unsigned absolute_program_load = 0x07f084;
  DspMachine machine;
  DspProgram code;
  code.word(absolute_program_load);
  code.word(program);
  code.install(machine.bus());
  machine.bus().write({.address = program, .value = access::first});
  auto progress = machine.begin_reference_instruction(access::boundaries);
  progress = machine.advance_reference_access(access::ticket(progress));
  progress = machine.advance_reference_access(access::ticket(progress));
  access::expect(progress,
                 {.kind = DspNativeAccessKind::read, .target = {.address = program}, .ordinal = 2});
  require(fault([&] {
            static_cast<void>(machine.latch_reference_program(access::ticket(progress)));
          }).contains("read at P:0x000100"),
          "A P-space data operand cannot be relabeled as a program fetch");
  progress = machine.advance_reference_access(access::ticket(progress));
  require(progress.retired && (machine.cpu().state().x & word_mask) == access::first,
          "The rejected latch leaves the ordinary actual P-data read usable");
  constexpr unsigned extal = 12000000;
  DspMachine clocked(
      DspRunContract{.clock = {.extal_hz = extal,
                               .timebase_hz = extal,
                               .switch_timing = DspClockSwitch::reference_instruction_boundary},
                     .timing = DspRunTiming::native_internal_estimates});
  DspProgram nop;
  nop.emit("nop");
  nop.install(clocked.bus());
  clocked.bus().pll().observe_pinit(false);
  clocked.cpu().reset_spi_boot();
  const auto before = clocked.cpu().debug_state();
  require(fault([&] {
            static_cast<void>(clocked.latch_reference_program({}));
          }).contains("unclocked reference owner") &&
              clocked.cpu().debug_state() == before && !clocked.timing_faulted(),
          "The program latch cannot silently replace a clocked completion contract");
  clocked.cpu().step();
  require(clocked.run_clock().native_cycle_units() == 1,
          "Rejected latch admission preserves actual ordinary clocked execution");
}
}

int main() {
  try {
    opcode_and_restore();
    opcode_length_and_zero();
    extension_sampling();
    nonprogram_and_ticket_guards();
    unknown_program_and_recovery();
    program_data_and_clock_guards();
    std::cout
        << "Actual program fetch latches, native decoding, DMA and coordinated restore passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
