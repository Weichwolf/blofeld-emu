// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "nand_gpio_fixture.hpp"
#include "nand_program_fixture.hpp"
#include "nand_read_timing_fixture.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_read_setup.hpp"
#include "storage/nand_read_timing.hpp"
#include "storage/nand_write_timing.hpp"

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

namespace blofeld::test::nand_setup {
namespace {
constexpr unsigned timebase = 1080000000;
constexpr unsigned extal = 12000000;
constexpr unsigned buffer = 0xc000;
constexpr unsigned reload = 0xc0b0;
constexpr unsigned reader = 0x9c;
constexpr unsigned poll = 0xbb;
constexpr unsigned reserved_function = 0x94;
constexpr unsigned immediate_words = 2;
constexpr unsigned held_polls = 8;
constexpr unsigned page_words = 176;
constexpr unsigned max_steps = 1000;
constexpr unsigned memory_mode = 0x600389;
constexpr unsigned first_byte = 0x96;

DspRunContract contract() {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = timebase,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = DspRunTiming::native_internal_estimates};
}

void caller(DspBus &bus, const Firmware &firmware, const unsigned row) {
  nand_read_reference::install_stage(bus, firmware);
  DspProgram program;
  setup_gpio(program);
  program.immediate("omr", memory_mode);
  program.immediate("r4", reload);
  program.immediate("r2", buffer);
  program.immediate("r5", row);
  program.emit("jsr " + number(reader));
  program.install(bus);
  // FM13-135 MOVEP high-I/O format; Table12-13 immediate EA=110100.
  // Its immediate extension is fetched before the rejected peripheral write.
  constexpr unsigned reference_mask = 0xfe0000;
  constexpr unsigned reference_prefix = 0x080000;
  constexpr unsigned write_peripheral = 0x8000;
  constexpr unsigned high_io_format = 0x4000;
  constexpr unsigned effective_address_format = 0x80;
  constexpr unsigned immediate_mode = 0x34;
  constexpr unsigned six_bits = 0x3f;
  const auto opcode = bus.inspect_program(reserved_function);
  require((opcode & reference_mask) == reference_prefix && (opcode & write_peripheral) != 0 &&
              (opcode & high_io_format) != 0 && (opcode & effective_address_format) != 0 &&
              ((opcode >> 16U) & 1U) == 1 && ((opcode >> 8U) & six_bits) == immediate_mode &&
              (opcode & six_bits) == (port_f_control & six_bits),
          "Independent MOVEP format identifies the two-word Y peripheral-write frontier");
}

struct WireState {
  NandWriteTiming input{timebase};
  NandReadSetup setup{timebase};
  NandReadTiming output{timebase};
  std::vector<NandWriteLatch> latches;
  unsigned polls = 0;
  unsigned reads = 0;
  unsigned samples = 0;
  unsigned driven_on_read = 0;
  bool uncertain_read_output_seen = false;
  bool initialized = false;
};

class Wire {
public:
  Wire(DspMachine &machine, Nand &nand, std::ostream &trace, WireState state = {})
      : machine_(machine), nand_(nand), trace_(trace), state_(std::move(state)) {
    if (!state_.initialized) {
      state_.output.observe(0, {.ce_n = true, .re_n = true});
      state_.initialized = true;
    }
    drive();
    ready();
  }

  void step() {
    const auto before = machine_.cpu().state();
    try {
      drive();
      machine_.cpu().step({.observe = [&](const DspNativeAccess &access) {
        if (access.kind == DspNativeAccessKind::read &&
            access.stage == DspNativeAccessStage::after && access.target.space == DspSpace::y &&
            access.target.address == port_f_data) {
          ++state_.samples;
        }
      }});
      observe();
      drive();
      ready();
      if (before.pc == poll) { ++state_.polls; }
      trace_ << "pc=" << hex(before.pc, 6) << " next=" << hex(machine_.cpu().state().pc, 6)
             << " ticks=" << machine_.run_clock().elapsed_ticks()
             << " latches=" << state_.latches.size() << " polls=" << state_.polls
             << " reads=" << state_.reads << " samples=" << state_.samples << '\n';
    } catch (const Fault &fault) {
      throw Fault("NAND coupled timing dsp-pc=" + hex(before.pc, 6) + " omr=" + hex(before.omr, 6) +
                  " page=" + hex(nand_.page(), 6) + " column=" + hex(nand_.column(), 3) + ": " +
                  fault.what());
    }
  }

  void observed_completion() {
    require(state_.polls == held_polls && nand_.pending() == NandPending::read &&
                nand_.ready() == false && state_.reads == 0,
            "External read-completion observation requires the actual held-busy path");
    nand_.observe_completion();
    observe();
    ready();
    trace_ << "external-read-completion ticks=" << machine_.run_clock().elapsed_ticks() << '\n';
  }

  [[nodiscard]] const WireState &state() const { return state_; }

private:
  void ready() { machine_.bus().timers().set_tio_drive(nand_timer_one, nand_.ready()); }

  void latch(const NandWriteLatch value) {
    switch (value.kind) {
      case NandWriteKind::command: nand_.command(value.value); break;
      case NandWriteKind::address: nand_.address(value.value); break;
      case NandWriteKind::data: throw Fault("Unqualified data write in original read fixture");
      default: throw Fault("Invalid NAND write latch kind");
    }
    state_.latches.push_back(value);
    trace_ << "latch kind=" << static_cast<unsigned>(value.kind) << " value=" << hex(value.value, 2)
           << '\n';
  }

  void observe() {
    const auto control = machine_.bus().gpio(DspPort::c).observe_pads(nand_controls);
    if (control.known != nand_controls) { return; }
    const auto ticks = machine_.run_clock().elapsed_ticks();
    const auto outputs = machine_.bus().gpio(DspPort::f).outputs();
    const auto known = outputs.known & nand_data;
    const bool re_n = (control.levels & nand_re) != 0;
    const bool we_n = (control.levels & nand_we) != 0;
    const bool cle = (control.levels & nand_cle) != 0;
    const bool ale = (control.levels & nand_ale) != 0;
    const auto written =
        state_.input.observe(ticks,
                             {.ce_n = false,
                              .re_n = re_n,
                              .we_n = we_n,
                              .cle = cle,
                              .ale = ale,
                              .data = {.value = static_cast<std::uint8_t>(outputs.levels & known),
                                       .known = static_cast<std::uint8_t>(known)}});
    if (written) { latch(*written); }
    const bool admitted = state_.setup.observe(ticks,
                                               {.ce_n = false,
                                                .re_n = re_n,
                                                .we_n = we_n,
                                                .ale = ale,
                                                .cle = cle,
                                                .ready = nand_.ready()});
    if (admitted) { state_.driven_on_read = outputs.mask & nand_data; }
    state_.output.observe(ticks, {.ce_n = false, .re_n = re_n}, [&] {
      require(admitted, "Protocol data cannot be consumed before read-setup admission");
      const auto value = nand_.read();
      ++state_.reads;
      return NandByte{.value = value, .known = Nand::byte_mask};
    });
  }

  void drive() {
    const auto output = state_.output.output(machine_.run_clock().elapsed_ticks());
    DspGpioDrive external{.mask = nand_other_f};
    if (!output.driven.has_value()) {
      external.unknown = nand_data;
      state_.uncertain_read_output_seen = state_.uncertain_read_output_seen || state_.reads != 0;
    } else if (*output.driven) {
      external.mask |= output.data.known;
      external.levels = output.data.value;
      external.unknown = nand_data & ~static_cast<unsigned>(output.data.known);
    }
    machine_.bus().gpio(DspPort::f).set_drive(external);
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

std::string until_fault(Wire &wire) {
  for (unsigned count = 0; count < max_steps; ++count) {
    try {
      wire.step();
    } catch (const Fault &fault) { return fault.what(); }
  }
  throw Fault("Original coupled NAND timing must reach its unsupported hardware frontier");
}

void verify(DspMachine &machine,
            Nand &nand,
            const Wire &wire,
            const unsigned row,
            const std::string &failure) {
  const std::vector<NandWriteLatch> expected{
      {.kind = NandWriteKind::command, .value = 0},
      {.kind = NandWriteKind::address, .value = 0},
      {.kind = NandWriteKind::address, .value = static_cast<std::uint8_t>(row & byte_mask)},
      {.kind = NandWriteKind::address, .value = static_cast<std::uint8_t>((row >> 8U) & byte_mask)},
      {.kind = NandWriteKind::address, .value = 0}};
  const auto &wire_state = wire.state();
  const auto state = machine.cpu().state();
  require(failure.contains("Reserved DSP Port F pin function selected") &&
              failure.contains("Y:" + hex(port_f_control, 6)) &&
              failure.contains("dsp-pc=" + hex(reserved_function, 6)) &&
              failure.contains("omr=" + hex(memory_mode, 6)),
          "The real original reserved function write must remain fatal: " + failure);
  require(wire_state.latches == expected && wire_state.polls == held_polls + 1 &&
              wire_state.reads == 1 && wire_state.samples == 0 &&
              wire_state.driven_on_read == nand_data && wire_state.uncertain_read_output_seen &&
              nand.page() == row && nand.column() == 1 && nand.ready() == true &&
              state.pc == reserved_function + immediate_words && state.r[5] == row + 1 &&
              state.r[0] == buffer && state.r[4] == reload && state.sp == 4 &&
              state.lc == page_words,
          "Coupled read frontier pc=" + hex(state.pc, 6) + " sp=" + std::to_string(state.sp) +
              " lc=" + std::to_string(state.lc) + " r0=" + hex(state.r[0], 6) +
              " r4=" + hex(state.r[4], 6) + " r5=" + hex(state.r[5], 6) + " polls=" +
              std::to_string(wire_state.polls) + " reads=" + std::to_string(wire_state.reads) +
              " samples=" + std::to_string(wire_state.samples) +
              " driven=" + hex(wire_state.driven_on_read, 2) +
              " uncertain=" + (wire_state.uncertain_read_output_seen ? "true" : "false") +
              " page=" + hex(nand.page(), 6) + " column=" + std::to_string(nand.column()) +
              " ready=" + (nand.ready() == true ? "true" : "false"));
  require(machine.timing_faulted() &&
              machine.bus().gpio(DspPort::f).read(DspGpioRegister::control) == nand_other_f &&
              machine.bus().gpio(DspPort::f).read(DspGpioRegister::direction) == output_f,
          "The consumed MOVEP extension must not commit its unsupported register value or "
          "leave a resumable native/clock boundary");
  std::string guard;
  try {
    static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = buffer}));
  } catch (const Fault &fault) { guard = fault.what(); }
  require(guard.contains("Unknown DSP memory read at X:" + hex(buffer, 6)),
          "No host or native payload store may manufacture a buffer word before data sampling");
  const auto stopped = machine.cpu().debug_state();
  const auto ticks = machine.run_clock().elapsed_ticks();
  std::string restart;
  try {
    machine.cpu().step();
  } catch (const Fault &fault) { restart = fault.what(); }
  require(restart.contains("requires reset after a rejected reference step") &&
              machine.cpu().debug_state() == stopped &&
              machine.run_clock().elapsed_ticks() == ticks,
          "The failed native access cannot execute again or advance its clock without recovery");
}

void run(const Firmware &firmware, const unsigned row, std::ostream &out) {
  DspMachine machine(contract());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  caller(machine.bus(), firmware, row);
  Nand nand;
  nand.observe_cell({.page = row}, {.value = first_byte, .known = Nand::byte_mask});
  nand.observe_power_ready();
  std::ostringstream trace;
  Wire wire(machine, nand, trace);
  for (unsigned count = 0; wire.state().polls < held_polls; ++count) {
    require(count < max_steps, "Original held-busy instruction budget exceeded");
    wire.step();
  }
  require(machine.cpu().state().pc == poll && nand.ready() == false && nand.column() == 0,
          "Original polls really wait before the separate external completion observation");
  wire.observed_completion();
  const Joint saved{.machine = machine.checkpoint(),
                    .nand = nand,
                    .wire = wire.state(),
                    .debug = machine.cpu().debug_state(),
                    .prefix = trace.str()};
  const auto failure = until_fault(wire);
  verify(machine, nand, wire, row, failure);
  DspMachine branch(contract());
  branch.restore(saved.machine);
  Nand peer = saved.nand;
  std::ostringstream repeated;
  Wire restored(branch, peer, repeated, saved.wire);
  require(branch.cpu().debug_state() == saved.debug && restored.state().reads == 0,
          "Fresh joint Ready-edge restoration retains the unconsumed first protocol byte");
  const auto repeated_failure = until_fault(restored);
  verify(branch, peer, restored, row, repeated_failure);
  require(failure == repeated_failure &&
              branch.cpu().debug_state() == machine.cpu().debug_state() &&
              trace.str().starts_with(saved.prefix) &&
              trace.str().substr(saved.prefix.size()) == repeated.str(),
          "The complete post-Ready native timing suffix and unsupported access replay exactly");
  out << "row=" << hex(row, 6) << '\n'
      << trace.str()
      << "verified first-read=1 cpu-samples=0 board-qualified=false post-ready-suffix-bytes="
      << repeated.str().size() << " failure=" << failure << '\n';
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-nand-read-setup firmware.zip private.trace");
    }
    const auto firmware = blofeld::Firmware::load(std::filesystem::path(arguments[1]));
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    for (const unsigned row : {0U, 0xffU, 0x100U, 0x7ffU}) {
      blofeld::test::nand_setup::run(firmware, row, trace);
    }
    std::cout << "Original NAND input/setup/output timing: actual command/address/ready/read "
                 "transition and reserved Port F failure retained, with exact joint replay.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
