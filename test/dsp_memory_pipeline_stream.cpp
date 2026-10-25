// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pipeline.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "native_access_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::memory_pipeline {
namespace {
using native_access::fault;

void consecutive_loads() {
  DspMachine machine;
  std::array<std::string, 12> code{};
  code.fill("move x:(r0)+,x0");
  prepare(machine, code, {.pointer = data_base, .modifier = linear_modifier});
  for (unsigned index = 0; index < code.size(); ++index) {
    machine.bus().write(
        {.space = DspSpace::x, .address = data_base + index, .value = first + index});
  }
  DspFetchRun run(contract(), machine);
  const auto before = machine.cpu().debug_state();
  for (unsigned clock = 1; clock <= code.size() + 6; ++clock) {
    const auto result = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = stages(result);
    const auto updates = clock >= 5 ? std::min(clock - 4, 12U) : 0;
    const auto loads = clock >= 6 ? std::min(clock - 5, 12U) : 0;
    const auto retired = clock >= 7 ? clock - 6 : 0;
    auto expected = before;
    expected.cpu.r[0] = data_base + updates;
    if (loads != 0) { expected.cpu.x = pipeline::unit + first + loads - 1; }
    expected.cpu.pc += retired;
    expected.native_instructions += retired;
    expected.native_cycles += retired;
    require(machine.cpu().debug_state() == expected,
            "Consecutive native AG2/E1/E2 effects retain literal youngest R and load values at " +
                std::to_string(clock));
    if (phase.ag1_address) {
      const auto address = data_base + clock - 4;
      require(phase.ag1_address->update.address == address &&
                  phase.ag1_address->update.before == address &&
                  phase.ag1_address->update.value == address + 1,
              "AG1 samples the prior AG2 forwarding value before the older E1 can restore R");
    }
    if (phase.memory) {
      require(phase.memory->data.value == first + clock - 6 &&
                  phase.memory->data.target.address == data_base + clock - 6,
              "E1 uses its frozen EA even with two newer post-update results in flight");
    }
  }
}

struct Field {
  const char *name;
  unsigned store;
  std::uint64_t x;
  std::uint64_t y;
};

void one_data_field(const Field field, const DspSpace space, const bool store) {
  DspMachine machine;
  const std::string memory = space == DspSpace::x ? "x:(r0)" : "y:(r0)";
  const auto reg = std::string(field.name);
  auto instruction = std::string("move ");
  instruction.append(store ? reg : memory);
  instruction.push_back(',');
  instruction.append(store ? memory : reg);
  const std::array code{instruction};
  prepare(machine, code);
  machine.bus().write({.space = space, .address = 7, .value = second});
  auto expected = machine.cpu().state();
  if (!store) {
    expected.x = field.x;
    expected.y = field.y;
  }
  ++expected.pc;
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 7);
  require(machine.cpu().state() == expected &&
              machine.bus().read({.space = space, .address = 7}) == (store ? field.store : second),
          "All native X0/X1/Y0/Y1 load/store fields preserve independent sibling registers");
}

void data_register_fields() {
  constexpr std::array fields{
      Field{.name = "x0", .store = first, .x = pipeline::unit + second, .y = half},
      Field{.name = "x1", .store = 1, .x = (pipeline::unit * second) + first, .y = half},
      Field{.name = "y0", .store = half, .x = pipeline::unit + first, .y = second},
      Field{.name = "y1",
            .store = 0,
            .x = pipeline::unit + first,
            .y = (pipeline::unit * second) + half}};
  for (const auto field : fields) {
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      for (const bool store : {false, true}) { one_data_field(field, space, store); }
    }
  }
}

void load_store_and_alu() {
  const std::array<std::string, 4> code{
      "move x:(r0)+,x0", "move x0,y:(r0)+", "mpy y0,x0,a", "add x1,a"};
  for (unsigned checkpoint_clock = 3; checkpoint_clock <= 8; ++checkpoint_clock) {
    DspMachine machine;
    prepare(machine, code, {.pointer = data_base, .modifier = linear_modifier});
    machine.bus().write({.space = DspSpace::x, .address = data_base, .value = half});
    DspFetchRun run(contract(), machine);
    clocks(run, machine, checkpoint_clock);
    const auto saved = run.checkpoint(machine);
    const auto state = machine.cpu().debug_state();
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      require(machine.cpu().debug_state() == state && run.completed_cycles() == checkpoint_clock,
              "Joint snapshots retain pre/post AG1, AG2, E1 and E2 states with rebound private "
              "tickets");
      clocks(run, machine, 10 - checkpoint_clock);
      require(
          machine.bus().read({.space = DspSpace::y, .address = (data_base + 1)}) == half &&
              machine.cpu().state().r[0] == (data_base + 2) &&
              machine.cpu().state().x == pipeline::unit + half &&
              machine.cpu().state().a == product_plus_unit &&
              machine.cpu().state().pc == body + 4 &&
              machine.cpu().state().sr == (masked_status | unnormalized),
          "A store takes live E1 data after its earlier AG1, followed by actual MPY/ADD and CCR");
    }
  }
}

void captured_and_future_words() {
  DspMachine machine;
  const std::array<std::string, 8> code{
      "move x:(r0)+,x0", "move x0,y:(r0)+", "nop", "nop", "nop", "nop", "nop", "move x:(r0)+,x0"};
  prepare(machine, code, {.pointer = data_base, .modifier = linear_modifier});
  machine.bus().write({.space = DspSpace::x, .address = data_base, .value = second});
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 5);
  machine.bus().write({.address = body});
  machine.bus().write({.address = body + 1});
  machine.bus().write({.address = body + 7});
  clocks(run, machine, 9);
  require(machine.cpu().state().r[0] == (data_base + 2) &&
              machine.cpu().state().x == pipeline::unit + second &&
              machine.bus().read({.space = DspSpace::y, .address = (data_base + 1)}) == second &&
              machine.cpu().state().pc == body + 8,
          "Memory opcodes retain actual F2 words through AG1/E1, while future F2 sees replacement");
}

void failed_data_recovery() {
  DspMachine machine;
  const std::array<std::string, 3> code{"add x1,a", "move x:(r0)+,x0", "move x:(r0)+,x1"};
  prepare(machine, code, {.pointer = data_base, .modifier = linear_modifier});
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 6);
  const auto saved = run.checkpoint(machine);
  const auto error =
      fault([&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
  require(error.contains("read X:0x000100") && error.contains("pc=0x00011d") &&
              error.contains("omr=") && run.faulted() && run.completed_cycles() == 6 &&
              run.elapsed_ticks() == 6 * pipeline::period && machine.bus().last_dma_cycle() == 7 &&
              machine.cpu().state().a == pipeline::unit && machine.cpu().state().pc == body + 1 &&
              machine.cpu().state().r[0] == (data_base + 2),
          "An entered E1 data failure retains real older E2 and younger AG2 without elapsed-clock "
          "success");
  run.restore(saved, machine);
  machine.bus().write({.space = DspSpace::x, .address = data_base, .value = first});
  machine.bus().write({.space = DspSpace::x, .address = (data_base + 1), .value = second});
  clocks(run, machine, 3);
  require(
      machine.cpu().state().x == (pipeline::unit * second) + first &&
          machine.cpu().state().r[0] == (data_base + 2) && machine.cpu().state().pc == body + 3,
      "Trusted joint restore clears faulted native/address continuations and replays real reads");
}
}

void stream_checks() {
  consecutive_loads();
  data_register_fields();
  load_store_and_alu();
  captured_and_future_words();
  failed_data_recovery();
}
}
