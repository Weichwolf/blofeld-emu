// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using test::DspProgram;
using test::require;
constexpr unsigned sr_opcode = 0x05f439;
constexpr unsigned sr_short = 0x0500b9;
constexpr unsigned sr_carry = 0x301;
constexpr unsigned sr_zero = 0x304;
constexpr unsigned data_address = 0x100;
constexpr unsigned cookie = 0x123456;
constexpr unsigned short_eq = 0x05a400;
constexpr unsigned displacement = 16;
constexpr unsigned padding = 96;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
constexpr unsigned budget = 64;
constexpr unsigned byte_bits = 24;
constexpr std::uint64_t word_mask = 0xffffff;
enum class Follower : std::uint8_t { register_move, memory_read, conditional };

struct Input {
  unsigned status;
  unsigned before = sr_carry;
  bool long_form = true;
  Follower follower = Follower::register_move;
};

struct Setup {
  DspDebugState before;
  Input input;
  unsigned writer;
  unsigned follower;
  unsigned end;
};

struct Seen {
  bool e1 = false;
  bool writer = false;
  bool done = false;
  std::uint64_t writer_cycle = 0;
};

unsigned simple_cases = 0;
unsigned restored_suffixes = 0;
unsigned cloned_suffixes = 0;

DspFetchRunInputs contract() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory};
}

Setup prepare(DspMachine &machine, const Input input) {
  DspProgram code;
  code.word(sr_opcode);
  code.word(input.before);
  unsigned seeds = 1;
  if (input.follower == Follower::memory_read) {
    code.immediate("r0", data_address);
    ++seeds;
  }
  const unsigned writer = code.pc();
  code.word(input.long_form ? sr_opcode : sr_short | (input.status << 8U));
  if (input.long_form) { code.word(input.status); }
  const unsigned follower = code.pc();
  switch (input.follower) {
    case Follower::register_move: code.immediate("x1", cookie); break;
    case Follower::memory_read: code.emit("move x:(r0),x1"); break;
    case Follower::conditional: code.word(short_eq | displacement); break;
    default: throw Fault("Invalid own SR follower");
  }
  const unsigned end = input.follower == Follower::conditional && (input.status & test::zero) != 0
                           ? follower + displacement
                           : code.pc();
  for (unsigned index = 0; index < padding; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().write({.space = DspSpace::x, .address = data_address, .value = cookie});
  for (unsigned index = 0; index < seeds; ++index) { machine.cpu().step(); }
  require(machine.cpu().state().pc == writer && machine.cpu().state().sr == input.before,
          "Actual native seed is independent of the tested automatic write");
  const auto before = machine.cpu().debug_state();
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return {.before = before, .input = input, .writer = writer, .follower = follower, .end = end};
}

void endpoint(DspMachine &machine, const Setup &setup) {
  auto expected = setup.before.cpu;
  expected.pc = setup.end;
  expected.sr = setup.input.status;
  if (setup.input.follower != Follower::conditional) {
    expected.x = (expected.x & word_mask) | (static_cast<std::uint64_t>(cookie) << byte_bits);
  }
  require(machine.cpu().state() == expected,
          "Literal complete architectural PC/SR/register/stack endpoint matches native execution");
  require(machine.bus().read({.space = DspSpace::x, .address = data_address}) == cookie,
          "The own canary cell was not modified by control or read execution");
  require(machine.cpu().debug_state().native_instructions == setup.before.native_instructions + 2,
          "Exactly the SR write and its genuine follower retire");
}

void advance(DspFetchRun &run, DspMachine &machine, const Setup &setup, Seen &seen) {
  const auto cycle =
      run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
  if (!cycle.pipeline) { throw Fault("Missing automatic SR edge observation"); }
  const auto &phase = *cycle.pipeline;
  if (phase.execute1 && phase.execute1->address == setup.writer) {
    require(machine.cpu().state().sr == setup.input.before && !phase.transfer,
            "SR remains held at E1 without an ordinary writeback field");
    seen.e1 = true;
  }
  if (!phase.retirement_published) { return; }
  if (!phase.execute2) { throw Fault("Actual SR retirement lost its instruction identity"); }
  if (!seen.writer) {
    require(seen.e1 && phase.execute2->address == setup.writer &&
                machine.cpu().state().sr == setup.input.status,
            "The first real retirement publishes exactly the independently expected SR");
    seen.writer = true;
    seen.writer_cycle = cycle.completed_cycles;
    return;
  }
  require(phase.execute2->address == setup.follower && !seen.done,
          "The real follower is not skipped or replaced by another instruction");
  if (setup.input.follower != Follower::conditional) {
    const unsigned nominal = setup.input.long_form ? 2U : 1U;
    require(cycle.completed_cycles - seen.writer_cycle == nominal,
            "FM A1 one/two-word MOVEC cost creates no guessed extra stall");
  }
  endpoint(machine, setup);
  seen.done = true;
}

void finish(DspFetchRun &run, DspMachine &machine, const Setup &setup, Seen &seen) {
  for (unsigned index = 0; index < budget && !seen.done; ++index) {
    advance(run, machine, setup, seen);
  }
  require(seen.done, "Bounded native SR suffix reaches its independent endpoint");
}

void simple() {
  constexpr std::array cases{
      Input{.status = 0, .follower = Follower::memory_read},
      Input{.status = sr_zero, .follower = Follower::memory_read},
      Input{.status = 4, .long_form = false, .follower = Follower::memory_read},
      Input{.status = sr_zero, .follower = Follower::conditional},
      Input{.status = 0x300, .before = sr_zero, .follower = Follower::conditional},
      Input{.status = 4, .long_form = false, .follower = Follower::conditional},
      Input{.status = 0, .before = sr_zero, .long_form = false, .follower = Follower::conditional}};
  for (const auto input : cases) {
    DspMachine machine;
    const auto setup = prepare(machine, input);
    DspFetchRun run(contract(), machine);
    Seen seen;
    finish(run, machine, setup, seen);
    ++simple_cases;
  }
}

void restores() {
  constexpr std::array cases{
      Input{.status = 0},
      Input{.status = 4, .long_form = false, .follower = Follower::conditional},
      Input{.status = 0, .follower = Follower::memory_read}};
  for (const auto input : cases) {
    for (unsigned cut = 2; cut <= 7; ++cut) {
      DspMachine machine;
      const auto setup = prepare(machine, input);
      DspFetchRun run(contract(), machine);
      Seen prefix;
      while (run.completed_cycles() < cut) { advance(run, machine, setup, prefix); }
      require(!prefix.done, "Every declared checkpoint has a nonempty native suffix");
      const auto saved = run.checkpoint(machine);
      auto seen = prefix;
      finish(run, machine, setup, seen);
      const auto expected = machine.cpu().debug_state();
      const auto expected_ticks = run.elapsed_ticks();
      for (unsigned replay = 0; replay < 2; ++replay) {
        run.restore(saved, machine);
        seen = prefix;
        finish(run, machine, setup, seen);
        require(machine.cpu().debug_state() == expected && run.elapsed_ticks() == expected_ticks,
                "Complete native state and clock suffix survive trusted SR restore");
        ++restored_suffixes;
      }
      DspMachine target;
      DspFetchRun clone(saved, target);
      seen = prefix;
      finish(clone, target, setup, seen);
      require(target.cpu().debug_state() == expected && clone.elapsed_ticks() == expected_ticks,
              "A fresh native owner has the same independently checked SR suffix");
      ++cloned_suffixes;
    }
  }
}
}

int main() {
  try {
    simple();
    restores();
    std::cout << "PASS SR_EDGES simple=" << simple_cases << " restores=" << restored_suffixes
              << " fresh-owner-clones=" << cloned_suffixes << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
