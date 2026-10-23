// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/nand.hpp"
#include "dsp_norm_fixture.hpp"
#include "nand_program_fixture.hpp"
#include "nand_read_timing_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ostream>
#include <span>
#include <string>

namespace blofeld::test::nand_timing {
namespace {
using nand_read_reference::install_stage;
using nand_read_reference::Link;
constexpr unsigned timebase_hz = 1080000000;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned fast_pll = 0x02e112;
constexpr unsigned pll_register = 0xfffffd;
constexpr unsigned original_reader = 0x93;
constexpr unsigned original_sample = 0x98;
constexpr unsigned destination = 0x100;
constexpr unsigned row = 0x123;
constexpr unsigned absent = 37;
constexpr unsigned maximum_steps = 10000;
enum class Case : std::uint8_t { page, missing, early };

std::array<std::uint8_t, Nand::page_bytes> bytes() {
  std::array<std::uint8_t, Nand::page_bytes> result{};
  constexpr unsigned multiplier = 29;
  for (unsigned index = 0; index < result.size(); ++index) {
    result[index] = static_cast<std::uint8_t>((index * multiplier + (index >> 8U)) & byte_mask);
  }
  return result;
}

void prepare(Nand &nand, const Case which) {
  const auto source = bytes();
  if (which == Case::missing) {
    nand.observe_bytes(row, 0, std::span(source).first(absent));
    nand.observe_bytes(row, absent + 1, std::span(source).subspan(absent + 1));
  } else {
    nand.observe_bytes(row, 0, source);
  }
  // Declared ready page/register preconditions, not native preceding NAND setup.
  nand.observe_power_ready();
  nand.command(0);
  for (const unsigned value : {0U, row & byte_mask, row >> 8U, 0U}) {
    nand.address(static_cast<std::uint8_t>(value));
  }
  nand.observe_completion();
}

struct Caller {
  unsigned end = 0;
  unsigned sample_pc = original_sample;
};

Caller caller(DspBus &bus, const Case which) {
  DspProgram program;
  if (which == Case::early) { movep(program, fast_pll, pll_register, 'x'); }
  setup_gpio(program);
  // Deliberately supplied released-data entry condition; this does not execute
  // the original refill's earlier, still-unqualified turnaround/configuration.
  movep(program, input_f, port_f_direction, 'y');
  movep(program, all_inputs_f, port_f_control, 'y');
  program.immediate("r0", destination);
  if (which == Case::early) {
    program.emit("bclr #8,x:<<" + number(port_c_data));
    const auto sample = program.pc();
    program.emit("movep y:<<" + number(port_f_data) + ",a");
    program.install(bus);
    return {.end = program.pc(), .sample_pc = sample};
  }
  for (unsigned index = 0; index < Nand::page_bytes; ++index) {
    program.emit("jsr " + number(original_reader));
    program.emit("move a1,x:(r0)+");
  }
  program.install(bus);
  return {.end = program.pc()};
}

void run(const Firmware &firmware, const Case which, std::ostream &trace) {
  DspMachine machine(
      DspRunContract{.clock = {.extal_hz = extal_hz,
                               .timebase_hz = timebase_hz,
                               .switch_timing = DspClockSwitch::reference_instruction_boundary},
                     .timing = DspRunTiming::native_internal_estimates});
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  install_stage(machine.bus(), firmware);
  const auto code = caller(machine.bus(), which);
  Nand nand;
  prepare(nand, which);
  Link link(machine, nand, trace);
  std::string failure;
  trace << "case=" << static_cast<unsigned>(which) << '\n';
  for (unsigned steps = 0; machine.cpu().state().pc != code.end && failure.empty(); ++steps) {
    require(steps < maximum_steps, "Original timed NAND reader exceeded its instruction budget");
    try {
      link.step();
    } catch (const Fault &error) { failure = error.what(); }
  }
  unsigned copied = 0;
  if (which == Case::page) { copied = Nand::page_bytes; }
  if (which == Case::missing) { copied = absent; }
  const auto source = bytes();
  for (unsigned index = 0; index < copied; ++index) {
    require(machine.bus().read({.space = DspSpace::x, .address = destination + index}) ==
                source[index],
            "Every original byte read/RTS and native caller store matches the independent source");
  }
  require(machine.cpu().state().r[0] == destination + copied,
          "Only genuinely completed calls advance the native destination");
  if (which == Case::page) {
    require(failure.empty() && machine.cpu().state().sp == 0 && link.reads() == Nand::page_bytes &&
                nand.column() == Nand::page_bytes && !nand.ready().has_value(),
            "All528 original calls return normally; post-page readiness is not guessed");
  } else if (which == Case::missing) {
    require(failure.contains("NAND read") && failure.contains("column=" + hex(absent, 3)) &&
                link.reads() == absent && nand.column() == absent,
            "Missing original protocol data fails without advancing the NAND column: " + failure);
  } else {
    require(failure.contains("Unknown DSP GPIO data read at Y:" + hex(port_f_data, 6)) &&
                failure.contains("pc=" + hex(code.sample_pc, 6)) && link.reads() == 1,
            "Early native sampling must fail with actual address and PC: " + failure);
  }
  trace << "result copied=" << copied << " reads=" << link.reads() << " failure=" << failure
        << '\n';
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-nand-read-timing firmware.zip private.trace");
    }
    const auto firmware = blofeld::Firmware::load(std::filesystem::path(arguments[1]));
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    for (const auto which : {blofeld::test::nand_timing::Case::page,
                             blofeld::test::nand_timing::Case::missing,
                             blofeld::test::nand_timing::Case::early}) {
      blofeld::test::nand_timing::run(firmware, which, trace);
    }
    std::cout << "Original NAND byte reader:528 clocked calls, missing-cell and early-sample "
                 "failures. No complete boot or physical timing proof.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
