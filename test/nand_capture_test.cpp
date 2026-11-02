// SPDX-License-Identifier: GPL-3.0-only
#include "board/nand_bus_decode.hpp"
#include "board/nand_peer.hpp"
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"
#include "nand_native_run.hpp"
#include "storage/nand.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
namespace native = test::dma_test;
constexpr unsigned csr = 0xffff8b, load = csr - 1, count = csr - 3;
constexpr unsigned flags = 0x300000, captured = 0x200000, mode = 0x60, enabled = 5;
constexpr unsigned invert = 0x100, di = 0x1000, seed = 7, compare = 0xffffff;
constexpr unsigned port_c = 0xffffbd, port_f = 0xfffff5, vector = 0x58;
constexpr unsigned initial_status = 0xc00000, priority = 0xfffffe, timer_priority = 0x100;
constexpr unsigned move_sr = 0x05f439, move_r2 = 0x62f400;
constexpr unsigned irq_move_r2 = 0x320700, foreground_r7 = 0x370900;
constexpr unsigned limit = 3000, nops = 256;
constexpr unsigned extal = 12000000, bus_control = 0xfffffb, bus_value = 0x01ffff;
constexpr unsigned directions = 0xf00, functions = 0xff, command_controls = 0x700;
constexpr unsigned output_functions = 0x300, output_directions = 0x4ff;
constexpr std::uint64_t inherited_tick = 1000000000;
enum class Latch : std::uint8_t { command, address };

void store(native::NativeProgram &program, const DspStore value) {
  if (value.space == DspSpace::x) {
    program.write(value);
    return;
  }
  // FM high-Y immediate MOVEP, with independent port register/extension words.
  constexpr unsigned high_y = 0x09f480;
  constexpr unsigned io_base = 0xffffc0;
  program.word(high_y | (value.address - io_base));
  program.word(value.value);
}

void latch(native::NativeProgram &program, const Latch kind, const unsigned value) {
  constexpr unsigned command = 0x700;
  constexpr unsigned address = 0xb00;
  constexpr unsigned we = 0x200;
  const auto controls = kind == Latch::command ? command : address;
  store(program, {.space = DspSpace::x, .address = port_c, .value = controls});
  store(program, {.space = DspSpace::y, .address = port_f, .value = value});
  store(program, {.space = DspSpace::x, .address = port_c, .value = controls & ~we});
  program.word(0);
  store(program, {.space = DspSpace::x, .address = port_c, .value = controls});
}

struct Program {
  native::NativeProgram code;
  unsigned arm = 0, stop = 0;
};

Program program(const bool falling) {
  Program result;
  auto &code = result.code;
  code.word(move_sr);
  code.word(initial_status);
  code.word(move_r2);
  code.word(0);
  code.write({.space = DspSpace::x, .address = priority, .value = timer_priority});
  code.write({.space = DspSpace::x, .address = load, .value = seed});
  code.write({.space = DspSpace::x, .address = csr - 2, .value = compare});
  code.write({.space = DspSpace::x, .address = bus_control, .value = bus_value});
  code.write({.space = DspSpace::x, .address = port_c + 1, .value = directions});
  code.write({.space = DspSpace::x, .address = port_c + 2, .value = functions});
  code.write({.space = DspSpace::x, .address = port_c, .value = command_controls});
  store(code, {.space = DspSpace::y, .address = port_f + 2, .value = output_functions});
  store(code, {.space = DspSpace::y, .address = port_f + 1, .value = output_directions});
  if (falling) {
    result.arm = code.pc();
    code.write({.space = DspSpace::x, .address = csr, .value = flags | mode | invert | enabled});
  }
  latch(code, Latch::command, 0);
  for (unsigned index = 0; index < 4; ++index) { latch(code, Latch::address, 0); }
  if (!falling) {
    result.arm = code.pc();
    code.write({.space = DspSpace::x, .address = csr, .value = flags | mode | enabled});
  }
  for (unsigned index = 0; index < nops; ++index) { code.word(0); }
  code.word(foreground_r7);
  result.stop = code.pc();
  code.word(0);
  return result;
}

struct Observation {
  unsigned cycles = 0;
  std::optional<std::uint64_t> armed_edges;
  std::optional<unsigned> expected_capture;
  bool busy = false, ready = false, interrupted = false;
};

struct Fixture {
  Fixture(const unsigned rate, const bool falling, const NandBusContract contract)
      : falling(falling),
        machine({.extal_hz = extal,
                 .timebase_hz = rate,
                 .switch_timing = DspClockSwitch::reference_cycle_boundary}),
        peer(machine, Nand{}, {}, contract),
        code(program(falling)) {
    code.code.install(machine.bus());
    machine.bus().write({.address = vector, .value = irq_move_r2});
    machine.bus().write({.address = vector + 1});
    runtime = std::make_unique<test::NandNativeRun>(
        machine,
        peer,
        NandPeerEvents{
            .identity = std::make_shared<const PeripheralEventIdentity>(),
            .next_ticks = [] { return std::optional{inherited_tick}; },
            .observe = [](const std::uint64_t) { throw Fault("Unexpected inherited event"); }});
  }

  void step() {
    const auto previous_edges = machine.clock().timer_clk2_edges();
    const bool was_armed = (machine.bus().timers().read(csr, 1) & 1U) != 0;
    const auto before = peer.detailed().base;
    static_cast<void>(runtime->tick());
    ++observed.cycles;
    const auto after = peer.detailed().base;
    auto &timers = machine.bus().timers();
    const bool inverted = falling && timers.read(csr, 1) != 0;
    require(after.ready.has_value() && timers.read(csr, di) == (after.ready == !inverted ? di : 0U),
            "Raw Timer1 DI follows the genuine NAND level and configured input inversion");
    if (!was_armed && (timers.read(csr, 1) & 1U) != 0) {
      require(!observed.armed_edges && timers.read(csr, flags) == 0,
              "Actual native capture enable cannot manufacture a Ready edge or capture");
      observed.armed_edges = previous_edges;
    }
    const bool busy_edge = before.ready == true && after.ready == false;
    const bool ready_edge = before.ready == false && after.ready == true;
    if (busy_edge) { observed.busy = true; }
    if (ready_edge) { observed.ready = true; }
    if ((falling && busy_edge) || (!falling && ready_edge)) {
      if (!observed.armed_edges) { throw Fault("Real R/B edge preceded native capture arming"); }
      // Busy is caused after this native period; autonomous Ready is dispatched
      // before it. CLK/2 first loads seed, then increments once per actual edge.
      const auto edges =
          (falling ? machine.clock().timer_clk2_edges() : previous_edges) - *observed.armed_edges;
      require(edges > 0 && edges < compare, "The independent capture count remains bounded");
      observed.expected_capture = seed + static_cast<unsigned>(edges - 1);
      require(timers.read(count) == *observed.expected_capture &&
                  timers.read(csr, flags) == captured,
              "Only the genuine selected NAND edge captures the independently counted CLK/2 value");
    }
    if (machine.cpu().registers().r[2] == seed) { observed.interrupted = true; }
    if (observed.expected_capture) {
      require(timers.read(count) == *observed.expected_capture,
              "Repeated same-level Ready observations cannot restart or replace the capture");
    }
  }

  void finish() {
    while ((machine.cpu().registers().pc != code.stop || machine.cpu().pending_instruction()) &&
           observed.cycles < limit) {
      step();
    }
    require(machine.cpu().registers().pc == code.stop && observed.busy && observed.ready &&
                observed.expected_capture && observed.interrupted &&
                machine.cpu().registers().r[7] == 9 && !machine.interrupt_pending() &&
                !machine.bus().interrupt_request(0) &&
                machine.bus().timers().read(csr, flags) == 0 &&
                peer.detailed().base.ready == true && !peer.faulted(),
            "Actual NAND completion, fast vector acknowledgement and foreground execution finish");
  }

  bool falling;
  OwnDspMachine machine;
  NandPeer peer;
  Program code;
  std::unique_ptr<test::NandNativeRun> runtime;
  Observation observed;
};

void native_case(const unsigned rate, const bool falling, const NandBusContract contract) {
  Fixture fixture(rate, falling, contract);
  while (fixture.machine.cpu().registers().pc <= fixture.code.arm &&
         fixture.observed.cycles < limit) {
    fixture.step();
  }
  require(fixture.observed.armed_edges.has_value(), "The actual capture instruction must complete");
  const auto peer = fixture.peer.checkpoint();
  const auto saved = fixture.runtime->checkpoint();
  const auto observed = fixture.observed;
  fixture.finish();
  const auto cpu = test::native_cpu(fixture.machine);
  const auto state = fixture.runtime->state();
  const auto status = fixture.peer.detailed();
  const auto captured_count = fixture.machine.bus().timers().read(count);
  fixture.peer.restore(peer);
  fixture.runtime->restore(saved);
  fixture.observed = observed;
  fixture.finish();
  require(test::native_cpu(fixture.machine) == cpu && fixture.runtime->state() == state &&
              fixture.peer.detailed() == status &&
              fixture.machine.bus().timers().read(count) == captured_count,
          "Joint restore repeats the actual capture, vector, full CPU state and NAND deadline");
}

void unknown_ready() {
  OwnDspMachine machine(native::native_clocks);
  const NandPeer peer(machine, Nand{});
  auto &timers = machine.bus().timers();
  timers.write({.address = load});
  timers.write({.address = csr, .value = mode | enabled});
  require(rejects([&] { timers.advance_clk2(1); }).contains("Unknown DSP timer capture input") &&
              !peer.detailed().base.ready && timers.read(csr, flags) == 0,
          "An attached but unpowered NAND cannot supply a fabricated synchronized Ready baseline");
}
}

int main() {
  try {
    constexpr std::array rates{540000000U, 1080000000U};
    unsigned cases = 0;
    for (const auto rate : rates) {
      for (const auto falling : {false, true}) {
        for (const auto contract :
             {NandBusContract::ordered_pins, NandBusContract::functional_transactions}) {
          native_case(rate, falling, contract);
          ++cases;
        }
      }
    }
    unknown_ready();
    std::cout << "NAND_CAPTURE native_cases=" << cases << " joint_replays=" << cases
              << " unknown_source=1 passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "NAND_CAPTURE: " << error.what() << '\n';
    return 1;
  }
}
