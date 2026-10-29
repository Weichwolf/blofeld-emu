// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 540000000;
constexpr unsigned base = 0x100;
constexpr unsigned target = base + 3;
constexpr unsigned short_branch = 0x0e2103; // Own JNE $103, FM13-80.
constexpr unsigned omr_branch = 0x0afa03;
constexpr unsigned omr = 0x305;
constexpr unsigned address_count = 0x1000;
constexpr unsigned condition_count = 16;
constexpr unsigned condition_shift = 12;
constexpr unsigned short_prefix = 0x0e0000;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

void fields() {
  for (unsigned condition = 0; condition < condition_count; ++condition) {
    for (unsigned address = 0; address < address_count; ++address) {
      const unsigned opcode = short_prefix | (condition << condition_shift) | address;
      const auto form = decode_branch_form(opcode);
      require(form && form->absolute_short == address && form->words() == 1 &&
                  !form->displacement && !form->address_register && !form->absolute_extension &&
                  !form->relative_extension && !form->subroutine_call && !form->subroutine_return &&
                  branch_condition_code(opcode) == condition,
              "All65536 own fields preserve exactly the independent absolute12/CCCC encoding");
    }
  }
  require(!decode_branch_form(0), "NOP cannot become a short conditional branch");
}

DspProgramWord queued_word(const DspProgramFetch &fetch, const unsigned index) {
  const auto actual = fetch.queued(index);
  if (!actual) { throw Fault("Own width fixture lost a genuine F2 word"); }
  return actual->word;
}

void width_refuses(const unsigned variant) {
  const bool two = variant >= 3;
  DspBus bus;
  bus.write({.address = base, .value = two ? omr_branch : short_branch});
  bus.write({.address = base + 1, .value = target});
  bus.write({.address = base + 2, .value = 0});
  DspProgramFetch fetch;
  fetch.start(base, omr);
  for (unsigned index = 0; index < 3; ++index) {
    fetch.issue();
    fetch.capture(bus);
  }
  const auto actual = fetch.queued(0);
  if (!actual) { throw Fault("Own width fixture lost its actual branch occurrence"); }
  const DspBranchDecision valid{.word = actual->word,
                                .extension =
                                    two ? std::optional(queued_word(fetch, 1)) : std::nullopt,
                                .omr = omr,
                                .next_pc = target,
                                .taken = true};
  auto invalid = valid;
  auto identity = actual->occurrence;
  switch (variant) {
    case 0: invalid.extension = queued_word(fetch, 1); break;
    case 1: invalid.word.value ^= 1U; break;
    case 2: ++identity; break;
    case 3: invalid.extension.reset(); break;
    case 4: invalid.extension = queued_word(fetch, 2); break;
    default: throw Fault("Unknown directed width refusal");
  }
  const auto before = fetch.view();
  bool refused = false;
  try {
    fetch.predict_branch(invalid, identity);
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(message.contains("P:") && message.contains("pc=") && message.contains("omr=") &&
                fetch.view() == before,
            "Invalid width/value/identity fails before changing the genuine fetch queue");
    refused = true;
  }
  require(refused, "Invalid single/two-word branch ownership unexpectedly succeeded");
  fetch.predict_branch(valid, actual->occurrence);
  require(fetch.view().used == (two ? 2U : 1U) && fetch.view().next_address == target &&
              !fetch.view().pending_address,
          "A refused forecast must leave no hidden owner that could block the valid actual "
          "instruction");
}

void one_word_available() {
  DspBus bus;
  bus.write({.address = base, .value = short_branch});
  DspProgramFetch fetch;
  fetch.start(base, omr);
  fetch.issue();
  fetch.capture(bus);
  const auto actual = fetch.queued(0);
  if (!actual) { throw Fault("Actual one-word fixture lost F2"); }
  fetch.predict_branch({.word = actual->word, .omr = omr, .next_pc = target, .taken = true},
                       actual->occurrence);
  require(fetch.view().used == 1 && fetch.view().next_address == target,
          "A real short branch needs no invented or readable following extension word");
}

void active_loop_refuses(const unsigned destination, const bool taken) {
  constexpr unsigned boot_address = 0xff0000;
  constexpr unsigned sr_immediate = 0x05f439;
  constexpr unsigned status = 0xc00300;
  constexpr unsigned jump = 0x0af080;
  constexpr unsigned loop_last = base + 15; // FM13-56: DO expr-1 -> LA.
  constexpr unsigned do_two = 0x060280;
  constexpr unsigned loop_flag = 0x8000;
  constexpr unsigned instruction = base + 2;
  DspMachine machine;
  const std::array setup{sr_immediate, status, jump, base};
  machine.bus().install_rom({.start = boot_address, .words = setup});
  const auto loop = blofeld::test::checkpoint::assemble("do #2,$110");
  require(loop.size() == 2 && loop.front() == do_two && loop.back() == loop_last,
          "Own DO count2/expr$110 must encode literal LA=$10f");
  blofeld::test::checkpoint::ram(machine.bus(), base, loop);
  for (unsigned i = 2; i < 32; ++i) { machine.bus().write({.address = base + i}); }
  const auto condition = taken ? 2U : 10U; // Independent seed Z=0: NE/EQ.
  machine.bus().write({.address = instruction,
                       .value = short_prefix | (condition << condition_shift) | destination});
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.cpu().step();
  machine.cpu().step();
  const auto before = machine.cpu().debug_state();
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run({.extal_hz = extal_hz,
                   .timebase_hz = timebase_hz,
                   .timing = DspFetchTiming::reference_cycle_order,
                   .execution = DspReferenceExecution::staged_register_boundaries,
                   .pipeline = DspFetchPipeline::automatic_memory},
                  machine);
  try {
    for (unsigned period = 0; period < 32; ++period) {
      static_cast<void>(
          run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available}));
    }
  } catch (const Fault &error) {
    const std::string message = error.what();
    const auto actual = machine.cpu().debug_state();
    require(
        message.contains("Unqualified short Jcc/active DO read P:") && message.contains("pc=") &&
            message.contains("omr=") && run.faulted() && actual.cpu.pc == instruction &&
            actual.cpu.la == loop_last && actual.cpu.lc == 2 && (actual.cpu.sr & loop_flag) != 0 &&
            actual.native_instructions == before.native_instructions + 1,
        "Unqualified jump/DO boundary must refuse after actual DO, before Jcc effects: " + message +
            " la=" + hex(actual.cpu.la, 6) + " lc=" + hex(actual.cpu.lc, 6) + " sr=" +
            hex(actual.cpu.sr, 6) + " instructions=" + std::to_string(actual.native_instructions) +
            " before=" + std::to_string(before.native_instructions));
    return;
  }
  throw Fault("Unqualified short Jcc/DO target was silently accepted");
}

}

int main() {
  try {
    fields();
    for (unsigned variant = 0; variant < 5; ++variant) { width_refuses(variant); }
    one_word_available();
    for (const unsigned destination : {base + 15, base + 14, base + 128}) {
      for (const bool taken : {false, true}) { active_loop_refuses(destination, taken); }
    }
    std::cout << "65536 own absolute fields,five ownership refusals,six actual DO guards,single-F2 "
                 "forecast "
                 "passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
