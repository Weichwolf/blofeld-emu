// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/nand.hpp"
#include "blofeld/nand_array.hpp"
#include "blofeld/nand_read_timing.hpp"
#include "dsp_boot_reference.hpp"
#include "dsp_norm_fixture.hpp"
#include "nand_gpio_fixture.hpp"
#include "nand_program_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <ostream>
#include <span>
#include <string>

namespace blofeld::test::nand_timing {
namespace {
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

void install_stage(DspBus &bus, const Firmware &firmware) {
  constexpr std::size_t frame_offset = 0x7875;
  constexpr unsigned stage_base = 0x6a;
  constexpr std::size_t count = 94;
  const auto frame = std::span(firmware.mcu).subspan(frame_offset, (count + 2) * 3);
  require(dsp_boot_word(std::span<const std::uint8_t, 3>(frame.first<3>())) == count &&
              dsp_boot_word(std::span<const std::uint8_t, 3>(frame.subspan<3, 3>())) == stage_base,
          "Pinned original MCU supplies the complete unchanged first-stage object");
  for (std::size_t index = 0; index < count; ++index) {
    bus.write({.address = stage_base + static_cast<unsigned>(index),
               .value = dsp_boot_word(
                   std::span<const std::uint8_t, 3>(frame.subspan((index + 2) * 3, 3)))});
  }
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

class Link {
public:
  Link(DspMachine &machine, Nand &nand, std::ostream &trace)
      : machine_(machine), nand_(nand), trace_(trace), timing_(timebase_hz) {
    timing_.observe(0, {.ce_n = true, .re_n = true});
    drive(0);
  }

  void step() {
    const auto before = machine_.cpu().state();
    drive(machine_.run_clock().elapsed_ticks());
    machine_.cpu().step();
    const auto ticks = machine_.run_clock().elapsed_ticks();
    try {
      const auto control = machine_.bus().gpio(DspPort::c).observe_pads(nand_controls);
      if (control.known == nand_controls) {
        timing_.observe(ticks, {.ce_n = false, .re_n = (control.levels & nand_re) != 0}, [&] {
          const auto value = nand_.read();
          ++reads_;
          return NandByte{.value = value, .known = Nand::byte_mask};
        });
      }
      drive(ticks);
      trace_ << "pc=" << hex(before.pc, 6) << " ticks=" << ticks << " column=" << nand_.column()
             << " reads=" << reads_ << '\n';
    } catch (const Fault &error) {
      throw Fault("NAND reference read-edge at X:" + hex(port_c_data, 6) + " dsp-pc=" +
                  hex(before.pc, 6) + " omr=" + hex(before.omr, 6) + ": " + error.what());
    }
  }

  [[nodiscard]] unsigned reads() const { return reads_; }

private:
  void drive(const std::uint64_t ticks) {
    const auto output = timing_.output(ticks);
    DspGpioDrive drive{.mask = nand_other_f};
    if (!output.driven.has_value()) {
      drive.unknown = nand_data;
    } else if (*output.driven) {
      drive.mask |= output.data.known;
      drive.levels = output.data.value;
      drive.unknown = nand_data & ~static_cast<unsigned>(output.data.known);
    }
    machine_.bus().gpio(DspPort::f).set_drive(drive);
  }

  DspMachine &machine_;
  Nand &nand_;
  std::ostream &trace_;
  NandReadTiming timing_;
  unsigned reads_ = 0;
};

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
