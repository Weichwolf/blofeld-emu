// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp56kEmu/assembler.h"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000, extal = 12000000, timebase = 1080000000;
constexpr unsigned pin = 0x200, padding = 16, maximum_periods = 96;
constexpr std::uint64_t reset_ticks = 180;
// FM13-20/24/36: own one-word X:$ffffbd short-I/O bit9 diagnostics.
constexpr unsigned clear_word = 0x013d09, set_word = 0x013d29, change_word = 0x017d09;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

struct Case {
  std::string_view name;
  unsigned word;
  bool old;
};

constexpr std::array cases{Case{.name = "clear-low", .word = clear_word, .old = false},
                           Case{.name = "clear-high", .word = clear_word, .old = true},
                           Case{.name = "set-low", .word = set_word, .old = false},
                           Case{.name = "set-high", .word = set_word, .old = true},
                           Case{.name = "change-low", .word = change_word, .old = false},
                           Case{.name = "change-high", .word = change_word, .old = true}};

class Program {
public:
  void emit(const char *source) {
    const auto result = assembler_.assemble(source);
    require(result.success(), "Own GPIO input assembly must assemble");
    for (unsigned i = 0; i < result.wordCount; ++i) { words_.push_back(result.word[i]); }
  }

  unsigned install(DspBus &bus, const unsigned word) {
    emit("movep #$000000,x:<<$ffffbe");
    emit("movep #$000200,x:<<$ffffbf");
    for (unsigned i = 0; i < padding; ++i) { emit("nop"); }
    const auto body = boot + static_cast<unsigned>(words_.size());
    words_.push_back(word);
    for (unsigned i = 0; i < padding; ++i) { emit("nop"); }
    bus.install_rom({.start = boot, .words = words_});
    return body;
  }

private:
  dsp56k::Assembler assembler_;
  std::vector<std::uint32_t> words_;
};

DspFetchRunInputs settings() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory,
          .resets = DspFetchResetTiming::reference_cycle_boundary};
}

DspFetchCycleInputs inputs() {
  return {.known_requests = 15, .grant = DspDmaGrant::available};
}

struct Cut {
  DspFetchCheckpoint saved;
  DspDebugState before;
  DspReferenceTicket ticket;
};

Cut until_read(DspMachine &machine, DspFetchRun &run, const unsigned body) {
  for (unsigned i = 0; i < maximum_periods; ++i) {
    const auto cycle = run.advance_cycle(machine, inputs());
    if (!cycle.bus.memory || !cycle.bus.memory->bit_read) { continue; }
    const auto pending = machine.reference_pending();
    require(pending.has_value() && machine.cpu().state().pc == body,
            "Actual native RMW read precedes its CPU/CCR retirement at the own bit opcode");
    if (!pending) { throw Fault("Missing actual pending bit ticket"); }
    return {.saved = run.checkpoint(machine),
            .before = machine.cpu().debug_state(),
            .ticket = pending->ticket};
  }
  throw Fault("Own GPIO input program did not reach its actual RMW read");
}

void refuses_ticket(DspMachine &machine, const DspReferenceTicket &ticket) {
  const auto before = machine.cpu().debug_state();
  const auto pending = machine.reference_pending();
  bool refused = false;
  try {
    static_cast<void>(machine.advance_reference_access(ticket));
  } catch (const Fault &fault) {
    refused = std::string_view(fault.what()).contains("Stale, foreign or unavailable");
  }
  const auto after = machine.reference_pending();
  require(refused && machine.cpu().debug_state() == before &&
              pending.has_value() == after.has_value() &&
              (!pending || (after && pending->ticket == after->ticket)),
          "Foreign/stale native tickets refuse before CPU or pending-phase mutation");
}

void finish(DspMachine &machine, DspFetchRun &run, const Cut &cut, const Case &test) {
  auto &port = machine.bus().gpio(DspPort::c);
  // A real peer input transition after E1 must not replace the captured old bit.
  port.set_drive({.mask = pin, .levels = test.old ? 0U : pin});
  const auto cycle = run.advance_cycle(machine, inputs());
  auto expected = cut.before;
  ++expected.cpu.pc;
  expected.cpu.sr = (expected.cpu.sr & ~1U) | static_cast<unsigned>(test.old);
  ++expected.native_instructions;
  expected.native_cycles += 2;
  require(cycle.bus.bit_write && cycle.bus.bit_write->value == static_cast<unsigned>(test.old) &&
              cycle.pipeline && cycle.pipeline->retirement_published &&
              machine.cpu().debug_state() == expected,
          "Real E2 write feeds the genuine E1 old bit to native Carry despite changed peer input");
  const auto pads = port.observe_pads(pin);
  const auto output = port.outputs();
  require(pads.known == pin && pads.levels == (test.old ? 0U : pin) && (output.mask & pin) == 0 &&
              (output.known & pin) == 0,
          "Native RMW does not drive,repair or invent the peer-controlled input pad");
}

void execute(const Case &test) {
  DspMachine machine;
  Program program;
  const auto body = program.install(machine.bus(), test.word);
  DspFetchRun run(settings(),
                  machine,
                  DspHardwareResetInputs{
                      .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Own input fixture supplies the actual two-period reset assertion");
  run.release_hardware_reset(machine);
  require(run.advance_hardware_reset_ticks(machine, reset_ticks) == reset_ticks,
          "Own input fixture completes actual two-period reset recovery");
  machine.bus().gpio(DspPort::c).set_drive({.mask = pin, .levels = test.old ? pin : 0U});
  const auto cut = until_read(machine, run, body);
  refuses_ticket(machine, DspReferenceTicket{});
  finish(machine, run, cut, test);
  refuses_ticket(machine, cut.ticket);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(cut.saved, machine);
    require(machine.cpu().debug_state() == cut.before,
            "Pending read survives trusted native restore");
    finish(machine, run, cut, test);
  }
  DspMachine target;
  DspFetchRun cloned(cut.saved, target);
  require(target.cpu().debug_state() == cut.before,
          "Distinct native owner retains the captured old bit cut");
  finish(target, cloned, cut, test);
  std::cout << "PASS native RMW peer-change " << test.name << " old-bit=" << test.old
            << " same-owner-restores=2 distinct-owner-clones=1"
               " foreign-and-stale-tickets-refused=1 no-peer-repair=1\n";
}
}

int main(const int argc, const char *const *argv) {
  try {
    require(argc == 2, "Choose an own clear/set/change low/high input case");
    const std::string_view requested(argv[1]);
    for (const auto &test : cases) {
      if (test.name == requested) {
        execute(test);
        return 0;
      }
    }
    throw Fault("Unknown own RMW peer-change case");
  } catch (const std::exception &error) {
    std::cerr << "FAIL native RMW peer-change: " << error.what() << '\n';
    return 1;
  }
}
