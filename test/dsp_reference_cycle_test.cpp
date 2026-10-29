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
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>

namespace {
using namespace blofeld;
using namespace blofeld::test;
namespace access = blofeld::test::reference;
namespace dma = blofeld::test::dma_test;
using native_access::fault;
constexpr unsigned program = 0x100;
constexpr unsigned initial_r0 = 0x7e;
constexpr unsigned copied_r0 = 0x34;
constexpr unsigned program_source = 0x800;
constexpr unsigned program_copy = 0x88024a;
constexpr auto cycle_order = DspReferenceBusOrder::dma_then_fetch_then_native;

void prepare(DspMachine &machine,
             const std::span<const std::uint32_t> words,
             const banks::Transfer transfer = {}) {
  DspProgram setup;
  setup.immediate("r0", initial_r0);
  for (const auto store :
       {DspStore{.space = DspSpace::x, .address = dma::source0, .value = transfer.source},
        DspStore{.space = DspSpace::x, .address = dma::destination0, .value = transfer.destination},
        DspStore{.space = DspSpace::x, .address = dma::counter0},
        DspStore{.space = DspSpace::x, .address = dma::control0, .value = transfer.control}}) {
    setup.emit("movep #>" + number(store.value) + ",x:<<" + number(store.address));
  }
  setup.word(access::long_jump);
  setup.word(program);
  setup.install(machine.bus());
  checkpoint::ram(machine.bus(), program, words);
  machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = access::first});
  machine.bus().write({.space = DspSpace::y, .address = dma::destination, .value = access::second});
  static_cast<void>(machine.advance_dma(banks::inputs({.index = 0}), banks::use()));
  checkpoint::run(machine, 6);
  require(machine.cpu().state().pc == program && machine.cpu().state().r[0] == initial_r0,
          "Actual native MOVEP setup configures DMA and jumps to the own RAM program");
}

DspReferenceCycleInputs cycle(const banks::Cycle input,
                              const std::optional<DspReferenceTicket> &ticket = std::nullopt,
                              const std::optional<DspProgramFetchStage> fetch = std::nullopt) {
  return {.dma = banks::inputs(input), .fetch = fetch, .access = ticket, .order = cycle_order};
}

DspReferenceProgress until_data(DspMachine &machine, const DspNativeAccessKind kind) {
  auto progress = machine.begin_reference_instruction(access::boundaries);
  for (unsigned budget = 0; budget < 8 && !progress.retired; ++budget) {
    if (access::request(progress).kind == kind) { return progress; }
    progress = machine.advance_reference_access(access::ticket(progress));
  }
  throw Fault("Own native instruction never reached the expected actual data effect");
}

const DspReferenceProgress &completed(const DspReferenceCycleResult &result) {
  if (!result.progress) { throw Fault("Expected actual native bus-cycle progress"); }
  return *result.progress;
}

void one_access(const DspReferenceCycleResult &result, const DspMemoryAccess expected) {
  const auto actual = result.core_accesses.front();
  require(result.core_access_count == 1 && actual.target.space == expected.target.space &&
              actual.target.address == expected.target.address &&
              actual.operation == expected.operation,
          "Core occupancy is derived from the actual native/F2 address and direction");
}

void held_destination_load() {
  DspMachine machine;
  prepare(machine, checkpoint::assemble("move y:>$900,x0"));
  const auto progress = until_data(machine, DspNativeAccessKind::read);
  const auto before = machine.cpu().debug_state();
  const auto captured = machine.advance_reference_bus_cycle(cycle({.index = 1, .requests = 1}));
  require(captured.dma.phase && captured.dma.phase->kind == DspDmaPhaseKind::source_read &&
              captured.core_access_count == 0 && !captured.progress,
          "A real DMA source phase captures its word before the pending native load is selected");
  const auto held = machine.advance_reference_bus_cycle(
      cycle({.index = 2, .requests = 1, .grant = DspDmaGrant::wait}, access::ticket(progress)));
  one_access(held, {.target = {.space = DspSpace::y, .address = dma::destination}});
  require(
      held.dma.core_stall && !held.dma.word && !held.progress &&
          machine.cpu().debug_state() == before &&
          access::ticket(machine) == access::ticket(progress),
      "A held DMA destination actually holds native execution, not just an advisory stall flag");
  const auto saved = machine.checkpoint();
  const auto stale = access::ticket(progress);
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    static_cast<void>(fault([&] {
      static_cast<void>(
          machine.advance_reference_bus_cycle(cycle({.index = 3, .requests = 1}, stale)));
    }));
    machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = access::third});
    const auto written = machine.advance_reference_bus_cycle(
        cycle({.index = 3, .requests = 1}, access::ticket(machine)));
    require(written.dma.word && written.dma.word->value == access::first &&
                written.dma.core_stall && !written.progress &&
                machine.cpu().debug_state() == before,
            "DMA commits the already captured word while the conflicting native load remains held");
    const auto loaded =
        machine.advance_reference_bus_cycle(cycle({.index = 4}, access::ticket(machine)));
    require(!loaded.dma.core_stall && completed(loaded).retired &&
                access::completed(completed(loaded)).value == access::first &&
                (machine.cpu().state().x & word_mask) == access::first &&
                machine.cpu().state().pc == program + 2 &&
                machine.cpu().debug_state().native_instructions == before.native_instructions + 1,
            "The next actual native admission reads DMA's held value and retires exactly once");
  }
}

void native_store_wins_source() {
  DspMachine machine;
  auto words = checkpoint::assemble("move #>$234567,x0");
  const auto store = checkpoint::assemble("move x0,x:>$660");
  words.insert(words.end(), store.begin(), store.end());
  prepare(machine, words);
  machine.cpu().step();
  const auto progress = until_data(machine, DspNativeAccessKind::write);
  const auto before = machine.cpu().debug_state();
  const auto stored = machine.advance_reference_bus_cycle(
      cycle({.index = 1, .requests = 1}, access::ticket(progress)));
  one_access(stored,
             {.target = {.space = DspSpace::x, .address = dma::ring_start},
              .operation = DspMemoryOperation::write});
  require(stored.dma.phase && stored.dma.phase->kind == DspDmaPhaseKind::source_read &&
              !stored.dma.word && !stored.dma.core_stall && completed(stored).retired &&
              machine.bus().read({.space = DspSpace::x, .address = dma::ring_start}) ==
                  access::second,
          "Actual pending core store delays a fresh DMA source and commits its real native value");
  static_cast<void>(machine.advance_reference_bus_cycle(cycle({.index = 2, .requests = 1})));
  machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = access::third});
  const auto copied = machine.advance_reference_bus_cycle(cycle({.index = 3, .requests = 1}));
  require(copied.dma.word && copied.dma.word->value == access::second &&
              machine.bus().read({.space = DspSpace::y, .address = dma::destination}) ==
                  access::second &&
              machine.cpu().debug_state().native_instructions == before.native_instructions + 1,
          "The delayed source captures the new core value and retains it after a later source "
          "mutation");
}

void fetch_wait_and_admit() {
  DspMachine machine;
  const std::array words{0U, 0U};
  const auto opcode = checkpoint::assemble("move #<$34,r0").front();
  prepare(
      machine, words, {.source = program_source, .destination = program, .control = program_copy});
  machine.bus().write({.address = program_source, .value = opcode});
  const auto progress = machine.begin_reference_instruction(access::boundaries);
  static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
  const auto first = machine.advance_reference_bus_cycle(
      cycle({.index = 1, .requests = 1}, std::nullopt, DspProgramFetchStage::address));
  require(first.core_access_count == 0 && first.fetch && first.fetch->pending_address == program,
          "F1 uses its actual sequential address but performs no fictitious memory read");
  const auto held = machine.advance_reference_bus_cycle(
      cycle({.index = 2, .requests = 1, .grant = DspDmaGrant::wait},
            std::nullopt,
            DspProgramFetchStage::read));
  one_access(held, {.target = {.address = program}});
  require(held.dma.core_stall && !held.fetch && machine.reference_fetch().used == 0,
          "A real F2 memory sample is held by the conflicting DMA destination");
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    const auto before = machine.cpu().debug_state();
    const auto copied = machine.advance_reference_bus_cycle(
        cycle({.index = 3, .requests = 1}, std::nullopt, DspProgramFetchStage::read));
    require(copied.dma.word && copied.dma.word->value == opcode && copied.dma.core_stall &&
                !copied.fetch && machine.reference_fetch().used == 0,
            "The actual destination write wins and does not also sample the conflicting P word");
    const auto fetched = machine.advance_reference_bus_cycle(
        cycle({.index = 4}, std::nullopt, DspProgramFetchStage::read));
    require(fetched.fetch && fetched.fetch->words.front().data == opcode &&
                machine.cpu().debug_state() == before,
            "A later actual F2 read sees DMA's replacement without native retirement");
    const auto retired = machine.advance_reference_bus_cycle(
        cycle({.index = 5}, access::ticket(machine), DspProgramFetchStage::address));
    require(retired.core_access_count == 0 && completed(retired).retired &&
                machine.cpu().state().r[0] == copied_r0 && machine.cpu().state().pc == program + 1,
            "Native admission of an actually queued word uses no new P-bus footprint");
  }
}

void simultaneous_fetch_and_redirect() {
  DspMachine machine;
  const std::array words{access::long_jump, program + 2, 0U};
  prepare(machine, words);
  const auto progress = machine.begin_reference_instruction(access::boundaries);
  static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
  for (unsigned index = 0; index < 2; ++index) {
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::read));
  }
  const auto decoded = machine.advance_reference_bus_cycle(
      cycle({.index = 1}, access::ticket(progress), DspProgramFetchStage::address));
  require(!completed(decoded).retired &&
              access::request(completed(decoded)).kind == DspNativeAccessKind::extension,
          "Actual opcode admission can coexist with a real younger F1 address");
  const auto redirected = machine.advance_reference_bus_cycle(
      cycle({.index = 2}, access::ticket(machine), DspProgramFetchStage::read));
  one_access(redirected, {.target = {.address = program + 2}});
  require(
      completed(redirected).retired && redirected.fetch && redirected.fetch->used == 0 &&
          !redirected.fetch->pending_address && redirected.fetch->next_address == program + 2,
      "The native same-cursor JMP discards the actual younger F2 sample in final returned state");
}

void dual_port_preflight() {
  DspMachine machine;
  constexpr unsigned program_load = 0x07f084; // Independently checked FM MOVEM P:ea,X0.
  constexpr unsigned operand = 0x500;
  const std::array words{program_load, operand, 0U};
  prepare(machine, words);
  machine.bus().write({.address = operand, .value = access::first});
  auto progress = machine.begin_reference_instruction(access::boundaries);
  static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
  for (unsigned index = 0; index < 2; ++index) {
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::read));
    progress = machine.advance_reference_access(access::ticket(progress));
  }
  static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
  const auto before = machine.cpu().debug_state();
  const auto fetch = machine.reference_fetch();
  const auto error = fault([&] {
    static_cast<void>(machine.advance_reference_bus_cycle(
        cycle({.index = 1}, access::ticket(progress), DspProgramFetchStage::read)));
  });
  require(error.contains("dual core bus-cycle use") && error.contains("pc=0x000100") &&
              error.contains("omr=0x000305") && machine.cpu().debug_state() == before &&
              machine.reference_fetch() == fetch &&
              access::ticket(machine) == access::ticket(progress),
          "Unqualified simultaneous P reads reject before DMA sampling or actual core effects");
  static_cast<void>(machine.advance_reference_bus_cycle(
      cycle({.index = 1}, std::nullopt, DspProgramFetchStage::read)));
  const auto loaded =
      machine.advance_reference_bus_cycle(cycle({.index = 2}, access::ticket(machine)));
  require(
      completed(loaded).retired && (machine.cpu().state().x & word_mask) == access::first,
      "Rejected preflight preserves the same DMA index and legitimate separate P-data admission");
}

void failed_required_read_keeps_dma_prefix() {
  DspMachine machine;
  const auto words = checkpoint::assemble("move y:>$20,x0");
  require(words.size() == 2, "Own absolute Y-data load has its independently expected extension");
  prepare(machine, words);
  auto progress = machine.begin_reference_instruction(access::boundaries);
  static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
  for (unsigned index = 0; index < 2; ++index) {
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::read));
  }
  progress = machine.advance_reference_access(access::ticket(progress));
  progress = machine.advance_reference_access(access::ticket(progress));
  require(access::request(progress).kind == DspNativeAccessKind::read &&
              access::request(progress).target.space == DspSpace::y &&
              access::request(progress).target.address == 0x20,
          "Actual opcode and extension reach the genuine required Y-data read");
  static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
  const auto saved = machine.checkpoint();
  const auto before = machine.cpu().debug_state();
  const auto error = fault([&] {
    static_cast<void>(machine.advance_reference_bus_cycle(
        cycle({.index = 1, .requests = 1}, access::ticket(progress), DspProgramFetchStage::read)));
  });
  require(
      error.contains("cycle=1") && error.contains("Unknown DSP memory read at Y:0x000020") &&
          error.contains("pc=0x000100") && error.contains("omr=0x000305") &&
          machine.cpu().debug_state() == before &&
          machine.bus().access_peripheral_bit({.space = DspSpace::x,
                                               .address = dma::status,
                                               .bit = 8,
                                               .operation = DspBitOperation::test}) == true &&
          machine.bus().read({.space = DspSpace::y, .address = dma::destination}) == access::second,
      "Required unknown data still fails after DMA source/F2 without committing native results");
  static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
  static_cast<void>(fault([&] { static_cast<void>(machine.reference_pending()); }));
  machine.restore(saved);
  machine.bus().write({.space = DspSpace::y, .address = 0x20, .value = access::third});
  machine.bus().write({.address = program + 2});
  const auto changed_opcode = checkpoint::assemble("move #<$34,r0").front();
  machine.bus().write({.address = program, .value = changed_opcode});
  const auto recovered = machine.advance_reference_bus_cycle(
      cycle({.index = 1, .requests = 1}, access::ticket(machine), DspProgramFetchStage::read));
  require(recovered.dma.phase && recovered.dma.phase->kind == DspDmaPhaseKind::source_read &&
              completed(recovered).retired && machine.cpu().state().pc == program + 2 &&
              machine.cpu().state().r[0] == initial_r0 &&
              (machine.cpu().state().x & word_mask) == access::third &&
              recovered.core_access_count == 2,
          "Trusted restore preserves the captured instruction and samples initialized data with "
          "actual F2");
}

void protocol_preflight() {
  DspMachine machine;
  const std::array words{0U};
  prepare(machine, words);
  const auto progress = machine.begin_reference_instruction(access::boundaries);
  const auto before = machine.cpu().debug_state();
  auto invalid = cycle({.index = 1, .requests = 1}, access::ticket(progress));
  invalid.order = DspReferenceBusOrder::unqualified;
  static_cast<void>(
      fault([&] { static_cast<void>(machine.advance_reference_bus_cycle(invalid)); }));
  invalid.order = cycle_order;
  invalid.access = DspReferenceTicket{};
  static_cast<void>(
      fault([&] { static_cast<void>(machine.advance_reference_bus_cycle(invalid)); }));
  invalid.access = access::ticket(progress);
  invalid.fetch = DspProgramFetchStage::read;
  static_cast<void>(
      fault([&] { static_cast<void>(machine.advance_reference_bus_cycle(invalid)); }));
  require(
      machine.cpu().debug_state() == before &&
          access::ticket(machine) == access::ticket(progress) &&
          machine.bus().access_peripheral_bit({.space = DspSpace::x,
                                               .address = dma::status,
                                               .bit = 8,
                                               .operation = DspBitOperation::test}) == false,
      "Order, capability and fetch-stage errors reject before request sampling or native effects");
  const auto valid = machine.advance_reference_bus_cycle(
      cycle({.index = 1, .requests = 1}, access::ticket(machine)));
  one_access(valid, {.target = {.address = program}});
  require(
      valid.dma.phase && valid.dma.phase->kind == DspDmaPhaseKind::source_read &&
          completed(valid).retired,
      "The unchanged actual opcode and DMA index remain admissible after pure preflight refusal");
}

void controller_bit_with_fetch() {
  DspMachine machine;
  auto words = checkpoint::assemble("jset #8,x:<<$fffff4,$102");
  require(words.size() == 2, "Independent peripheral bit-branch instruction length");
  words.push_back(0);
  prepare(machine, words);
  auto progress = machine.begin_reference_instruction(access::boundaries);
  static_cast<void>(machine.start_reference_fetch(access::ticket(progress)));
  for (unsigned index = 0; index < 2; ++index) {
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::read));
    progress = machine.advance_reference_access(access::ticket(progress));
  }
  require(access::request(progress).kind == DspNativeAccessKind::bit &&
              access::request(progress).target.address == dma::status,
          "The original native bit-test path suspends at its actual DMA-controller read");
  static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
  const auto before = machine.cpu().debug_state();
  const auto result = machine.advance_reference_bus_cycle(
      cycle({.index = 1, .requests = 1}, access::ticket(progress), DspProgramFetchStage::read));
  require(result.core_access_count == 2 && result.core_accesses[0].target.address == program + 2 &&
              result.core_accesses[1].target.space == DspSpace::x &&
              result.core_accesses[1].target.address == dma::status && result.dma.phase &&
              result.dma.phase->kind == DspDmaPhaseKind::source_read && !result.dma.core_stall &&
              completed(result).retired && access::completed(completed(result)).value == 1 &&
              machine.cpu().state().pc == program + 2 &&
              machine.cpu().state().sr == before.cpu.sr && result.fetch &&
              result.fetch->used == 0 && !result.fetch->pending_address,
          "Declared DMA-first order exposes real active bit8 to the native branch alongside an "
          "independent F2 read");
}

void peripheral_preflight() {
  DspMachine machine;
  prepare(machine, checkpoint::assemble("movep x:<<$ffff94,x0"));
  machine.bus().shi().observe_spi({.mosi = false});
  constexpr unsigned shi_control = 0xffff91;
  constexpr unsigned shi_configuration = 0xa9;
  machine.bus().write({.space = DspSpace::x, .address = shi_control, .value = shi_configuration});
  access::send(machine.bus().shi(), access::first);
  const auto progress = until_data(machine, DspNativeAccessKind::read);
  const auto error = fault([&] {
    static_cast<void>(machine.advance_reference_bus_cycle(
        cycle({.index = 1, .requests = 1}, access::ticket(progress))));
  });
  require(
      error.contains("pc=0x000100") && error.contains("omr=0x000305") &&
          error.contains("0xffff94") && access::ticket(machine) == access::ticket(progress) &&
          machine.bus().read({.space = DspSpace::x, .address = access::fifo}) == access::first,
      "Unqualified simultaneous peripheral occupancy refuses before consuming actual FIFO data");
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
  static_cast<void>(
      fault([&] { static_cast<void>(machine.advance_reference_bus_cycle(cycle({.index = 1}))); }));
  require(machine.cpu().debug_state() == before && !machine.timing_faulted(),
          "Composed semantic bus cycles cannot bypass an existing instruction-clock contract");
  machine.cpu().step();
  require(machine.run_clock().native_cycle_units() == 1,
          "Rejected composed-cycle admission preserves the genuine existing clocked path");
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
    peripheral_preflight();
    clocked_owner_guard();
    std::cout << "Actual native/F2 DMA bus-cycle admission checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
