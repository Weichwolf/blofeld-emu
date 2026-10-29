// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_spi_prefix_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <variant>

namespace blofeld::test::program_movep {
namespace {
constexpr unsigned control = 0xffff91;
constexpr unsigned hrx = 0xffff94;
constexpr unsigned not_empty = 0x20000;
constexpr unsigned sentinel = 0x654321;
constexpr unsigned following = 0xa13579;
constexpr unsigned offset = 3;
constexpr unsigned prefix_instructions = 3;
constexpr unsigned first_destination = 0x300;
constexpr unsigned long_r = 0x60f400;
constexpr unsigned long_n = 0x70f400;
constexpr unsigned long_m = 0x05f420;
constexpr unsigned clock_control = 0xffff90;
constexpr unsigned fifo_24 = 0x29;

struct Input {
  unsigned reg;
  unsigned mode;
  unsigned payload;
};

unsigned destination(const Input input) {
  return first_destination + (input.reg * 16U);
}

unsigned instruction(const Input input) {
  // Independent DSP56300FM13-136 low-I/O P-space MOVEP fields. R0 forms also
  // agree with the unchanged vendor assembler in the separate local reference.
  constexpr unsigned low_program_form = 0x008000;
  constexpr unsigned hrx_offset = 0x14;
  return low_program_form | (input.mode << 11) | (input.reg << 8) | hrx_offset;
}

void prepare(DspMachine &machine, const Input input) {
  DspProgram code;
  code.word(long_r | (input.reg << 16));
  code.word(destination(input));
  code.word(long_n | (input.reg << 16));
  code.word(offset);
  code.word(long_m | input.reg);
  code.word(word_mask);
  code.word(instruction(input));
  code.word(0);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  for (unsigned index = 0; index < prefix_instructions; ++index) { machine.cpu().step(); }
  machine.bus().write({.address = destination(input), .value = sentinel});
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().shi().write({.address = clock_control, .value = 0});
  machine.bus().shi().write({.address = control, .value = fifo_24});
  for (const auto value : {input.payload, following}) {
    spi_prefix::word(machine.bus(), value);
    // CPHA=0 requires SS to return inactive between the two real wire words.
    machine.bus().shi().observe_spi({.mosi = false});
  }
  require((machine.bus().shi().read(control) & not_empty) != 0,
          "Only actual serial edges may seed the declared two-word receive FIFO");
}

void cpu(const DspDebugState &actual, DspDebugState expected, const Input input) {
  ++expected.cpu.pc;
  ++expected.native_instructions;
  if (input.mode == 3) {
    ++expected.cpu.r.at(input.reg);
  } else if (input.mode == 1) {
    expected.cpu.r.at(input.reg) += offset;
  } else if (input.mode == 0) {
    expected.cpu.r.at(input.reg) -= offset;
  } else if (input.mode == 2) {
    --expected.cpu.r.at(input.reg);
  }
  require(actual.cpu == expected.cpu && actual.sc == expected.sc && actual.ep == expected.ep &&
              actual.sz == expected.sz && actual.stack_high == expected.stack_high &&
              actual.stack_low == expected.stack_low &&
              actual.native_instructions == expected.native_instructions,
          "Native MOVEP matches literal PC/R increment, preserved CCR/registers and stack");
}

void fifo(DspMachine &machine, const Input input) {
  require(machine.bus().inspect_program(destination(input)) == input.payload &&
              machine.bus().read({.space = DspSpace::x, .address = hrx}) == following &&
              (machine.bus().shi().read(control) & not_empty) == 0,
          "Exactly the real first SPI word is stored; the independently chosen second survives");
}

void access(const DspNativeAccess &actual,
            const Input input,
            const unsigned ordinal,
            const DspNativeAccessStage stage,
            const unsigned pc) {
  constexpr std::array kinds{
      DspNativeAccessKind::opcode, DspNativeAccessKind::read, DspNativeAccessKind::write};
  const std::array targets{DspAddress{.address = pc},
                           DspAddress{.space = DspSpace::x, .address = hrx},
                           DspAddress{.address = destination(input)}};
  const auto target = targets.at(ordinal);
  const auto value = ordinal == 0 ? instruction(input) : input.payload;
  require(actual.kind == kinds.at(ordinal) && actual.stage == stage &&
              actual.target.space == target.space && actual.target.address == target.address &&
              actual.ordinal == ordinal && actual.pc == pc && actual.omr == reset_omr &&
              !actual.bit,
          "Each native opcode/read/write keeps its literal space/address/order/PC/OMR");
  require(actual.value == (stage == DspNativeAccessStage::before && ordinal != 2
                               ? std::nullopt
                               : std::optional(value)),
          "A source read is unknown before access; writes and completed reads carry real data");
}

DspReferenceTicket ticket(const DspReferenceProgress &progress) {
  if (!progress.pending) { throw Fault("A real native continuation must retain its ticket"); }
  return progress.pending->ticket;
}

void pending(const DspReferenceProgress &progress,
             const Input input,
             const unsigned ordinal,
             const unsigned pc) {
  if (!progress.pending) { throw Fault("The next real data access must remain pending"); }
  const auto *effect = std::get_if<DspNativeAccess>(&progress.pending->effect);
  require(effect != nullptr, "Expected a native bus operation, not an invented CPU result");
  access(*effect, input, ordinal, DspNativeAccessStage::before, pc);
}

DspReferenceCycleInputs bus_cycle(const unsigned index) {
  return {.dma = banks::inputs({.index = index}),
          .order = DspReferenceBusOrder::dma_then_fetch_then_native};
}

unsigned after_pointer(const Input input) {
  if (input.mode == 3) { return destination(input) + 1; }
  if (input.mode == 1) { return destination(input) + offset; }
  if (input.mode == 0) { return destination(input) - offset; }
  if (input.mode == 2) { return destination(input) - 1; }
  return destination(input);
}

void native_staged(const Input input) {
  DspMachine machine;
  prepare(machine, input);
  const auto before = machine.cpu().debug_state();
  auto visible = before;
  auto progress =
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries);
  progress = machine.advance_reference_access(ticket(progress));
  if (input.mode != 4) {
    if (!progress.pending) { throw Fault("Expected the actual native address publication"); }
    const auto *update = std::get_if<DspAddressUpdate>(&progress.pending->effect);
    require(update != nullptr && update->index == input.reg &&
                update->before == destination(input) && update->value == after_pointer(input) &&
                update->space == DspSpace::x && update->address == hrx &&
                update->operation == DspMemoryOperation::read && update->pc == before.cpu.pc &&
                update->omr == reset_omr,
            "Explicit native address publication precedes its actual first peripheral read");
    progress = machine.advance_reference_access(ticket(progress));
    visible.cpu.r.at(input.reg) = after_pointer(input);
  }
  pending(progress, input, 1, before.cpu.pc);
  require(machine.cpu().debug_state() == visible &&
              machine.bus().inspect_program(destination(input)) == sentinel,
          "Published address fields do not consume the real receive FIFO or write P RAM");
  progress = machine.advance_reference_access(ticket(progress));
  pending(progress, input, 2, before.cpu.pc);
  progress = machine.advance_reference_access(ticket(progress));
  require(machine.cpu().debug_state() == visible,
          "Only native R publication is visible before actual complete retirement");
  progress = machine.advance_reference_access(ticket(progress));
  require(progress.retired, "The true staged MOVEP must retire");
  cpu(machine.cpu().debug_state(), before, input);
  fifo(machine, input);
}

DspInstructionPipeline capture(DspMachine &machine, const Input input) {
  const auto progress =
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries);
  static_cast<void>(machine.start_reference_fetch(ticket(progress)));
  static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
  const auto fetched = machine.advance_reference_fetch(DspProgramFetchStage::read);
  require(fetched.used == 1 && fetched.words.front().data == instruction(input),
          "Only a real F1/F2 sample supplies the captured memory component");
  DspInstructionPipeline component{.profile = DspFetchPipeline::automatic_memory, .view = {}};
  component.view.cycle.ag1 =
      fetched.words.front().require_known({.pc = fetched.consume_address, .omr = fetched.omr});
  // Invoke the existing component with actual fetched input. This does not run
  // automatic Decode or claim any hardware stage clock.
  component.capture_address(machine);
  const auto actual = component.view.cycle.ag1_address;
  require(actual && !actual->published && !actual->extension && !actual->secondary &&
              actual->data_access_count == 2 && actual->pointer_update == (input.mode != 4) &&
              actual->update.before == destination(input) &&
              actual->update.value == after_pointer(input) && actual->update.space == DspSpace::p &&
              actual->update.address == destination(input) &&
              actual->update.operation == DspMemoryOperation::write &&
              actual->data_accesses[0].target.space == DspSpace::x &&
              actual->data_accesses[0].target.address == hrx &&
              actual->data_accesses[0].operation == DspMemoryOperation::read &&
              actual->data_accesses[1].target.space == DspSpace::p &&
              actual->data_accesses[1].target.address == destination(input) &&
              actual->data_accesses[1].operation == DspMemoryOperation::write,
          "Native AGU capture carries literal P destination and separate real port-read/P-write "
          "footprint");
  return component;
}

DspAddressStageTicket address_ticket(const DspInstructionPipeline &component) {
  const auto captured = component.addresses[0];
  if (!captured) { throw Fault("Actual captured memory owner is absent"); }
  return *captured;
}

void memory_component(const Input input) {
  DspMachine machine;
  prepare(machine, input);
  const auto before = machine.cpu().debug_state();
  auto component = capture(machine, input);
  require(machine.cpu().debug_state() == before &&
              machine.bus().inspect_program(destination(input)) == sentinel,
          "Real AGU capture leaves CPU and RAM untouched");
  auto publication = bus_cycle(0);
  publication.address_update = address_ticket(component);
  const auto published = machine.advance_reference_bus_cycle(publication);
  auto visible = before;
  visible.cpu.r.at(input.reg) = after_pointer(input);
  require(published.address_update && published.address_update->published &&
              machine.cpu().debug_state() == visible,
          "Only genuine native calculated R fields publish at the selected component boundary");
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    component.rebind(machine);
    auto execute = bus_cycle(1);
    execute.memory = address_ticket(component);
    const auto cycle = machine.advance_reference_bus_cycle(execute);
    require(
        cycle.memory && cycle.memory->additional_data && !cycle.memory->move &&
            cycle.core_access_count == 2 && cycle.memory->data.kind == DspNativeAccessKind::read &&
            cycle.memory->data.target.space == DspSpace::x &&
            cycle.memory->data.target.address == hrx && cycle.memory->data.value == input.payload &&
            cycle.memory->additional_data->kind == DspNativeAccessKind::write &&
            cycle.memory->additional_data->target.space == DspSpace::p &&
            cycle.memory->additional_data->target.address == destination(input) &&
            cycle.memory->additional_data->value == input.payload &&
            machine.cpu().debug_state() == visible &&
            machine.bus().inspect_program(destination(input)) == input.payload,
        "The owner commits actual FIFO/P effects through the native interpreter before retirement");
    const auto pending_result = machine.reference_pending();
    if (!pending_result) { throw Fault("The actual native retirement owner is absent"); }
    auto retirement = bus_cycle(2);
    retirement.access = pending_result->ticket;
    const auto retired = machine.advance_reference_bus_cycle(retirement);
    require(retired.progress && retired.progress->retired && retired.core_access_count == 0,
            "Architectural retirement is not fabricated bus occupancy");
    cpu(machine.cpu().debug_state(), before, input);
    fifo(machine, input);
  }
}

void resource_conflict() {
  constexpr Input input{.reg = 0, .mode = 3, .payload = 0xabcdef};
  DspMachine machine;
  prepare(machine, input);
  auto component = capture(machine, input);
  auto publication = bus_cycle(0);
  publication.address_update = address_ticket(component);
  static_cast<void>(machine.advance_reference_bus_cycle(publication));
  const auto before = machine.cpu().debug_state();
  static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
  auto conflict = bus_cycle(1);
  conflict.memory = address_ticket(component);
  conflict.fetch = DspProgramFetchStage::read;
  try {
    static_cast<void>(machine.advance_reference_bus_cycle(conflict));
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(message.contains("Unqualified dual core bus-cycle use") &&
                message.contains("read P:") && message.contains("write P:") &&
                message.contains("pc=") && message.contains("omr=") &&
                machine.cpu().debug_state() == before &&
                machine.bus().inspect_program(destination(input)) == sentinel &&
                machine.bus().read({.space = DspSpace::x, .address = hrx}) == input.payload,
            "Unqualified concurrent F2/P-write fails before actual FIFO or P-RAM effects");
    return;
  }
  throw Fault("Unqualified simultaneous program fetch/write unexpectedly succeeded");
}

void active_dma_refuses() {
  constexpr Input input{.reg = 0, .mode = 3, .payload = 0xabcdef};
  DspMachine machine;
  prepare(machine, input);
  auto component = capture(machine, input);
  auto publication = bus_cycle(0);
  publication.address_update = address_ticket(component);
  static_cast<void>(machine.advance_reference_bus_cycle(publication));
  banks::configure(machine.bus());
  const auto before = machine.cpu().debug_state();
  const auto dma_before = machine.bus().last_dma_cycle();
  auto execute = bus_cycle(1);
  execute.memory = address_ticket(component);
  try {
    static_cast<void>(machine.advance_reference_bus_cycle(execute));
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(message.contains("Unqualified DSP peripheral memory resource read") &&
                message.contains("active DMA") && message.contains("pc=") &&
                message.contains("omr=") && machine.cpu().debug_state() == before &&
                machine.bus().last_dma_cycle() == dma_before &&
                machine.bus().inspect_program(destination(input)) == sentinel &&
                machine.bus().read({.space = DspSpace::x, .address = hrx}) == input.payload,
            "Unqualified active DMA fails before request sampling, FIFO consumption or P write");
    return;
  }
  throw Fault("Unqualified concurrent DMA/peripheral-to-P transfer unexpectedly succeeded");
}

}
}

int main() {
  try {
    using namespace blofeld::test::program_movep;
    for (unsigned reg = 0; reg < 8; ++reg) {
      for (const auto mode : {0U, 1U, 2U, 3U, 4U}) {
        for (const auto payload : {0U, 1U, 0x7fffffU, 0x800000U, 0xffffffU}) {
          const Input input{.reg = reg, .mode = mode, .payload = payload};
          native_staged(input);
          memory_component(input);
        }
      }
    }
    resource_conflict();
    active_dma_refuses();
    std::cout
        << "200 native staged and200 real captured peripheral/P component vectors plus F2/DMA "
           "conflict passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
