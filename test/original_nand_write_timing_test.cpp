// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "nand_gpio_fixture.hpp"
#include "nand_program_fixture.hpp"
#include "nand_read_timing_fixture.hpp"
#include "storage/nand.hpp"
#include "storage/nand_write_timing.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace blofeld::test::nand_input {
namespace {
constexpr unsigned timebase = 1080000000;
constexpr unsigned extal = 12000000;
constexpr unsigned buffer = 0xc000;
constexpr unsigned reload = 0xc0b0;
constexpr unsigned reader = 0x9c;
constexpr unsigned poll = 0xbb;
constexpr unsigned command_fall = 0xa8;
constexpr unsigned max_steps = 1000;
constexpr unsigned held_polls = 8;
constexpr unsigned memory_mode = 0x600389;

DspRunContract contract() {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = timebase,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = DspRunTiming::native_internal_estimates};
}

void caller(DspBus &bus, const unsigned row) {
  DspProgram program;
  setup_gpio(program);
  program.immediate("omr", memory_mode);
  program.immediate("r4", reload);
  program.immediate("r2", buffer);
  program.immediate("r5", row);
  program.emit("jsr " + number(reader));
  program.install(bus);
}

struct WireState {
  NandWriteTiming timing{timebase};
  std::vector<NandWriteLatch> latches;
  unsigned polls = 0;
};

class Wire {
public:
  Wire(DspMachine &machine, Nand &nand, std::ostream &trace, WireState state = {})
      : machine_(machine), nand_(nand), trace_(trace), state_(std::move(state)) {
    machine_.bus().gpio(DspPort::f).set_drive({.mask = nand_other_f});
    drive_ready();
  }

  void step() {
    const auto before = machine_.cpu().state();
    machine_.cpu().step();
    try {
      const auto control = machine_.bus().gpio(DspPort::c).observe_pads(nand_controls);
      if (control.known == nand_controls) {
        const auto outputs = machine_.bus().gpio(DspPort::f).outputs();
        const auto known = outputs.known & nand_data;
        const auto result = state_.timing.observe(
            machine_.run_clock().elapsed_ticks(),
            {.ce_n = false,
             .re_n = (control.levels & nand_re) != 0,
             .we_n = (control.levels & nand_we) != 0,
             .cle = (control.levels & nand_cle) != 0,
             .ale = (control.levels & nand_ale) != 0,
             .data = {.value = static_cast<std::uint8_t>(outputs.levels & known),
                      .known = static_cast<std::uint8_t>(known)}});
        if (result) { latch(*result); }
      }
      if (before.pc == poll) { ++state_.polls; }
      drive_ready();
      const auto after = machine_.cpu().state();
      trace_ << "pc=" << hex(before.pc, 6) << " next=" << hex(after.pc, 6)
             << " ticks=" << machine_.run_clock().elapsed_ticks()
             << " latches=" << state_.latches.size() << " polls=" << state_.polls
             << " r5=" << hex(after.r[5], 6) << '\n';
    } catch (const Fault &error) {
      throw Fault("NAND input reference port=X:" + hex(port_c_data, 6) +
                  " dsp-pc=" + hex(before.pc, 6) + " omr=" + hex(before.omr, 6) + " page=" +
                  hex(nand_.page(), 6) + " column=" + hex(nand_.column(), 3) + ": " + error.what());
    }
  }

  [[nodiscard]] const WireState &state() const { return state_; }

private:
  void drive_ready() { machine_.bus().timers().set_tio_drive(nand_timer_one, nand_.ready()); }

  void latch(const NandWriteLatch value) {
    switch (value.kind) {
      case NandWriteKind::command: nand_.command(value.value); break;
      case NandWriteKind::address: nand_.address(value.value); break;
      case NandWriteKind::data: throw Fault("Unqualified data write in command/address fixture");
      default: throw Fault("Invalid NAND write latch kind");
    }
    state_.latches.push_back(value);
    trace_ << "latch kind=" << static_cast<unsigned>(value.kind) << " value=" << hex(value.value, 2)
           << '\n';
  }

  DspMachine &machine_;
  Nand &nand_;
  std::ostream &trace_;
  WireState state_;
};

struct Joint {
  DspCheckpoint machine;
  Nand nand;
  WireState wire;
  DspDebugState debug;
  std::string prefix;
};

void finish(DspMachine &machine, Nand &nand, const Wire &wire, const unsigned row) {
  const std::vector<NandWriteLatch> expected{
      {.kind = NandWriteKind::command, .value = 0},
      {.kind = NandWriteKind::address, .value = 0},
      {.kind = NandWriteKind::address, .value = static_cast<std::uint8_t>(row & byte_mask)},
      {.kind = NandWriteKind::address, .value = static_cast<std::uint8_t>((row >> 8U) & byte_mask)},
      {.kind = NandWriteKind::address, .value = 0}};
  const auto state = machine.cpu().state();
  require(wire.state().latches == expected && wire.state().polls == held_polls &&
              nand.pending() == NandPending::read && nand.ready() == false && nand.page() == row &&
              nand.column() == 0 && state.pc == poll && state.r[5] == row + 1 &&
              state.r[0] == buffer && state.sp == 1,
          "Timed original writes must emit the exact independent command/address sequence and "
          "remain in the real ready poll without an invented completion");
}

void until_busy(Wire &wire) {
  for (unsigned count = 0; wire.state().polls < held_polls; ++count) {
    require(count < max_steps, "Timed original command/address path instruction bound exceeded");
    wire.step();
  }
}

void run(const Firmware &firmware, const unsigned row, std::ostream &out) {
  DspMachine machine(contract());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  nand_read_reference::install_stage(machine.bus(), firmware);
  caller(machine.bus(), row);
  Nand nand;
  nand.observe_power_ready();
  std::ostringstream trace;
  Wire wire(machine, nand, trace);
  std::optional<Joint> saved;
  for (unsigned count = 0; wire.state().polls < held_polls; ++count) {
    require(count < max_steps, "Original command/address path instruction bound exceeded");
    const auto before = machine.cpu().state();
    wire.step();
    if (before.pc == command_fall) {
      require(wire.state().latches.empty() && nand.ready() == true,
              "Falling command WE does not latch or begin an array operation");
      saved.emplace(Joint{.machine = machine.checkpoint(),
                          .nand = nand,
                          .wire = wire.state(),
                          .debug = machine.cpu().debug_state(),
                          .prefix = trace.str()});
    }
  }
  if (!saved) { throw Fault("Original path must reach its actual pending command pulse"); }
  finish(machine, nand, wire, row);
  DspMachine branch(contract());
  branch.restore(saved->machine);
  Nand peer = saved->nand;
  std::ostringstream repeated;
  Wire restored(branch, peer, repeated, saved->wire);
  require(branch.cpu().debug_state() == saved->debug && restored.state().latches.empty(),
          "Fresh joint restoration retains the unlatched low command pulse");
  until_busy(restored);
  finish(branch, peer, restored, row);
  require(trace.str().starts_with(saved->prefix) &&
              trace.str().substr(saved->prefix.size()) == repeated.str() &&
              branch.cpu().debug_state() == machine.cpu().debug_state(),
          "Every original latch, ready poll and native state/time row survives pending-WE replay");
  out << "row=" << hex(row, 6) << '\n'
      << trace.str() << "verified-latches=5 held-polls=" << held_polls
      << " restored-suffix-bytes=" << repeated.str().size() << '\n';
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-nand-write-timing firmware.zip private.trace");
    }
    const auto firmware = blofeld::Firmware::load(std::filesystem::path(arguments[1]));
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    for (const unsigned row : {0U, 0xffU, 0x100U, 0x7ffU}) {
      blofeld::test::nand_input::run(firmware, row, trace);
    }
    std::cout << "Original NAND writes: four timed command/address sequences and pending-WE "
                 "restorations; busy is held without fabricated completion.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
