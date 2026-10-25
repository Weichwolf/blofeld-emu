// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_pipeline_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <algorithm>
#include <array>
#include <string>

namespace blofeld::test::peripheral_pipeline {
namespace {
void independent_encodings() {
  struct Reference {
    const char *assembly;
    unsigned opcode;
    unsigned extension;
    DspSpace space;
    unsigned address;
  };

  // Independent Freescale asm56300 6.3.28 encodings of our own instructions.
  constexpr std::array references{Reference{.assembly = "movep #>0,x:<<$fffffc",
                                            .opcode = 0x08f4bc,
                                            .extension = 0,
                                            .space = DspSpace::x,
                                            .address = 0xfffffc},
                                  Reference{.assembly = "movep #>$234567,x:<<$fffffc",
                                            .opcode = 0x08f4bc,
                                            .extension = 0x234567,
                                            .space = DspSpace::x,
                                            .address = 0xfffffc},
                                  Reference{.assembly = "movep #>$800001,x:<<$fffffc",
                                            .opcode = 0x08f4bc,
                                            .extension = 0x800001,
                                            .space = DspSpace::x,
                                            .address = 0xfffffc},
                                  Reference{.assembly = "movep #>$ffffff,x:<<$fffffc",
                                            .opcode = 0x08f4bc,
                                            .extension = 0xffffff,
                                            .space = DspSpace::x,
                                            .address = 0xfffffc},
                                  Reference{.assembly = "movep #>9,x:<<$ffff90",
                                            .opcode = 0x07f410,
                                            .extension = 9,
                                            .space = DspSpace::x,
                                            .address = 0xffff90},
                                  Reference{.assembly = "movep #>$100,y:<<$ffff9f",
                                            .opcode = 0x07b49f,
                                            .extension = 0x100,
                                            .space = DspSpace::y,
                                            .address = 0xffff9f},
                                  Reference{.assembly = "movep #>$a9,x:<<$ffff91",
                                            .opcode = 0x07f411,
                                            .extension = 0xa9,
                                            .space = DspSpace::x,
                                            .address = 0xffff91},
                                  Reference{.assembly = "movep #>$234567,x:<<$ffff93",
                                            .opcode = 0x07f413,
                                            .extension = 0x234567,
                                            .space = DspSpace::x,
                                            .address = 0xffff93},
                                  Reference{.assembly = "movep #>$234567,y:<<$fffffc",
                                            .opcode = 0x09f4bc,
                                            .extension = 0x234567,
                                            .space = DspSpace::y,
                                            .address = 0xfffffc}};
  for (const auto &reference : references) {
    const auto words = checkpoint::assemble(reference.assembly);
    require(std::ranges::equal(words, std::array{reference.opcode, reference.extension}),
            "Own immediate MOVEP must match the independent assembler's exact opcode/extension");
    const auto form = decode_memory_stage(reference.opcode);
    require(form && form->peripheral && form->peripheral->space == reference.space &&
                form->peripheral->address == reference.address,
            "Independent immediate opcode selects its exact X/Y peripheral address");
  }
}

void one_write(const unsigned value) {
  DspMachine machine;
  const std::array<std::string, 1> code{"movep #>" + number(value) + ",x:<<$fffffc"};
  prepare(machine, code);
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 3);
  const auto first = stages(run.advance_cycle(machine, fetch_clock::input()));
  require(first.ag1_address && first.ag1_extension &&
              first.ag1_address->kind == DspAddressStageKind::peripheral_immediate &&
              first.ag1_address->extension == first.ag1_extension &&
              first.ag1_extension->address == body + 1 && first.ag1_extension->value == value &&
              !first.ag1_address->pointer_update && !first.ag1_address->published &&
              machine.cpu().debug_state() == before && !machine.bus().once_latch(),
          "Actual F2 literal reaches AG1 without a fictitious R update or early peripheral write");
  const auto second = stages(run.advance_cycle(machine, fetch_clock::input()));
  require(
      second.ag2_address && second.ag2_address->published && !second.ag2_address->pointer_update &&
          machine.cpu().debug_state() == before && !machine.bus().once_latch(),
      "Peripheral address continuation advances through AG2 without changing architectural state");
  const auto e1 = run.advance_cycle(machine, fetch_clock::input());
  require(e1.bus.core_access_count == 2 && e1.bus.memory &&
              e1.bus.memory->data.kind == DspNativeAccessKind::write &&
              e1.bus.memory->data.target.space == DspSpace::x &&
              e1.bus.memory->data.target.address == once && e1.bus.memory->data.value == value &&
              !e1.bus.memory->move && machine.bus().once_latch() == value &&
              machine.cpu().debug_state() == before,
          "E1 commits one genuine native immediate I/O write with simultaneous actual P fetch");
  const auto e2 = stages(run.advance_cycle(machine, fetch_clock::input()));
  auto expected = before;
  expected.cpu.pc = body + 2;
  ++expected.native_instructions;
  expected.native_cycles += 2;
  require(
      e2.retirement_published && machine.cpu().debug_state() == expected &&
          machine.bus().once_latch() == value && run.completed_cycles() == 7 &&
          run.elapsed_ticks() == 7 * pipeline::period,
      "E2 publishes the real two-cycle native result with every R/N/M/CCR and stack field intact");
}

void low_x_and_y_ports() {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    DspMachine machine;
    const auto address = space == DspSpace::x ? clock_control : 0xffff9f;
    const unsigned value = space == DspSpace::x ? 9U : 0x100U;
    const std::string name = space == DspSpace::x ? "x" : "y";
    const std::array<std::string, 1> code{"movep #>" + number(value) + ',' + name + ":<<" +
                                          number(address)};
    prepare(machine, code);
    const auto before = machine.cpu().debug_state();
    DspFetchRun run(contract(), machine);
    clocks(run, machine, 7);
    auto expected = before;
    expected.cpu.pc = body + 2;
    ++expected.native_instructions;
    expected.native_cycles += 2;
    require(machine.bus().read({.space = space, .address = address}) == value &&
                machine.cpu().debug_state() == expected,
            "Low X/Y immediate MOVEP uses its actual peripheral owner and changes no CPU operand");
  }
}

void no_repeated_transmit() {
  DspMachine machine;
  const std::array<std::string, 2> code{"movep #>$a9,x:<<$ffff91", "movep #>$234567,x:<<$ffff93"};
  prepare(machine, code);
  machine.bus().shi().observe_spi({.mosi = false});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 8);
  constexpr unsigned transmit_empty = 0x8000;
  require((machine.bus().shi().read(control) & transmit_empty) != 0,
          "The actual HTX word has moved into the real SHI shifter after E1");
  const auto e2 = stages(run.advance_cycle(machine, fetch_clock::input()));
  auto expected = before;
  expected.cpu.pc = body + 4;
  expected.native_instructions += 2;
  expected.native_cycles += 4;
  require(e2.retirement_published && machine.cpu().debug_state() == expected &&
              (machine.bus().shi().read(control) & transmit_empty) != 0,
          "E2 never replays the committed HTX write into an additional pending serial word");
}

void unrelated_agu() {
  DspMachine machine;
  const std::array<std::string, 3> code{
      "move x1,r0", "movep #>$234567,x:<<$fffffc", "move x:(r0)+,x0"};
  prepare(machine, code);
  machine.bus().write({.space = DspSpace::x, .address = 1, .value = literal});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 11);
  auto expected = before;
  expected.cpu.pc = body + 4;
  expected.cpu.r[0] = 2;
  expected.cpu.x = pipeline::unit + literal;
  expected.native_instructions += 3;
  expected.native_cycles += 4;
  require(machine.cpu().debug_state() == expected && machine.bus().once_latch() == literal,
          "Literal peripheral address capture never binds stale R0 over actual adjacent AGU work");
}
}

void writes() {
  independent_encodings();
  for (const unsigned value : {0U, literal, 0x800001U, 0xffffffU}) { one_write(value); }
  low_x_and_y_ports();
  no_repeated_transmit();
  unrelated_agu();
}
}
