// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "nand_native_peer.hpp"
#include "nand_native_run.hpp"
#include "storage/nand.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr std::uint32_t boot = 0xff0000, timer0_control = 0xffff88;
constexpr std::uint32_t btst_timer0_tio = 0x01482c, carry = 1;
constexpr std::uint64_t power_tick = 108000, period = 90;
constexpr unsigned instruction_count = 16, sample_count = 8, cycle_limit = 128;
constexpr DspRunClockInputs clock_inputs{.extal_hz = 12000000,
                                         .timebase_hz = nand_timebase_hz,
                                         .switch_timing = DspClockSwitch::reference_cycle_boundary};

void require(const bool condition, const std::string &reason) {
  if (!condition) { throw Fault(reason); }
}

void reject(const auto &action, const std::string &reason) {
  try {
    action();
  } catch (const Fault &error) {
    require(std::string(error.what()).contains(reason),
            "Unexpected refusal: " + std::string(error.what()));
    return;
  }
  throw Fault("Expected native NAND runtime refusal: " + reason);
}

struct Fixture {
  Fixture() : peer(machine, Nand{}) {
    // FM low-X qq BTST; timer0 TIO is an actual external pin, not NAND's R/B.
    std::array<std::uint32_t, static_cast<std::size_t>(instruction_count) * 2> words{};
    for (unsigned i = 0; i < instruction_count; i += 2) { words[i] = btst_timer0_tio; }
    machine.bus().install_rom({.start = boot, .words = words});
    runtime = std::make_unique<NandNativeRun>(
        machine,
        peer,
        NandPeerEvents{.identity = std::make_shared<const PeripheralEventIdentity>(),
                       .next_ticks = [this]() -> std::optional<std::uint64_t> {
                         return power_tick + (cursor * period);
                       },
                       .observe =
                           [this](const std::uint64_t ticks) {
                             require(ticks == power_tick + (cursor * period),
                                     "Inherited pin event retains its exact deadline");
                             ++cursor;
                             machine.bus().timers().set_tio_drive(0, (cursor % 2) != 0);
                           }});
  }

  void step() {
    const auto pending = machine.cpu().pending_instruction();
    const auto cycle = runtime->tick();
    const bool admitted_bit =
        cycle.instruction_started && cycle.native_pc && *cycle.native_pc >= boot &&
        *cycle.native_pc < boot + instruction_count && (*cycle.native_pc - boot) % 2 == 0;
    const bool pending_bit = pending && pending->effect.kind == dsp56300::EffectKind::bit &&
                             pending->effect.address == timer0_control;
    if (admitted_bit || pending_bit) {
      // A captured BTST can admit and sample its real I/O bit in the same cycle.
      // The actual next native effect must still hold its uncommitted Carry.
      require(cycle.native && cycle.native->pending &&
                  cycle.native->pending->opcode == btst_timer0_tio &&
                  cycle.native->pending->effect.kind == dsp56300::EffectKind::unit_update,
              "Real TIO sample precedes its pending native scalar-unit publication");
      sampled = (cursor % 2) != 0;
      ++samples;
    }
    if (cycle.native && cycle.native->retired && sampled) {
      require(((machine.cpu().registers().sr & carry) != 0) == *sampled,
              "Native Carry retains the real TIO sample after equal-tick inherited input");
      sampled.reset();
    }
  }

  void finish(const unsigned instructions) {
    unsigned cycles = 0;
    while (machine.cpu().progress().instructions < instructions) {
      require(++cycles < cycle_limit, "Literal BTST/NOP program is bounded");
      step();
    }
    require(machine.cpu().registers().pc == boot + instructions && !sampled,
            "All requested native instructions actually retire");
  }

  OwnDspMachine machine{clock_inputs};
  NandPeer peer;
  std::unique_ptr<NandNativeRun> runtime;
  std::uint64_t cursor = 0;
  std::optional<bool> sampled;
  unsigned samples = 0;
};

void inherited_pin_and_boundary() {
  Fixture fixture;
  fixture.finish(2);
  fixture.finish(instruction_count);
  require(fixture.samples == sample_count,
          "All eight real TIO bit reads reached native retirement; actual=" +
              std::to_string(fixture.samples));
  const auto cpu = native_cpu(fixture.machine);
  const auto state = fixture.runtime->state();
  const auto calendar = fixture.runtime->peripheral_state();
  require(state.external_events == fixture.cursor &&
              state.next_external == power_tick + (fixture.cursor * period) &&
              calendar.peripheral_events == 1 && calendar.last_peripheral == power_tick &&
              !calendar.pending && fixture.peer.detailed().power_ready &&
              fixture.peer.detailed().base.ready == true,
          "Real input consumption and the sole NAND power event retain their literal deadlines");
  const auto dma = fixture.machine.bus().last_dma_cycle();
  ++fixture.cursor;
  reject([&] { static_cast<void>(fixture.runtime->tick()); }, "healthy clock/input boundary");
  require(native_cpu(fixture.machine) == cpu &&
              fixture.machine.clock().elapsed_ticks() == state.horizon &&
              fixture.machine.bus().last_dma_cycle() == dma,
          "An out-of-band input cursor change refuses before native CPU, clock or DMA work");
  std::cout << "PASS NAND_NATIVE_INPUT samples=8 power-events=1 cursor-refusal=1\n";
}
}

int main() {
  try {
    inherited_pin_and_boundary();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
