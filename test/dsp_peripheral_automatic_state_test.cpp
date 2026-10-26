// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace blofeld::test::peripheral_automatic_state {
namespace {
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned spi_configuration = 0xa9;
constexpr unsigned htx = 0xffff93;
constexpr unsigned hrx = 0xffff94;
constexpr unsigned first = 0x123456;
constexpr unsigned second = 0x654321;
constexpr unsigned not_empty = 0x20000;
constexpr unsigned port_c_data = 0xffffbd;
constexpr unsigned body = memory_pipeline::body;
constexpr unsigned unknown_target = 0x800;
using memory_pipeline::contract;
using memory_pipeline::prepare;
using pipeline::clocks;

void spi_word(DspBus &bus, const std::uint32_t value) {
  for (unsigned remaining = word_bits; remaining != 0; --remaining) {
    const bool bit = ((value >> (remaining - 1)) & 1U) != 0;
    bus.shi().observe_spi({.selected = true, .clock = true, .mosi = bit});
    bus.shi().observe_spi({.selected = true, .mosi = bit});
  }
}

void receive(DspBus &bus) {
  bus.shi().observe_spi({.mosi = false});
  bus.write({.space = DspSpace::x, .address = hcsr, .value = spi_configuration});
  bus.write({.space = DspSpace::x, .address = htx, .value = first});
  bus.write({.space = DspSpace::x, .address = htx, .value = second});
  bus.shi().observe_spi({.selected = true, .mosi = false});
  spi_word(bus, first);
  spi_word(bus, second);
}

void fifo_snapshots() {
  const std::array<std::string, 2> code{"movep x:<<$ffff94,x0", "movep x:<<$ffff94,y0"};
  for (unsigned clock = 0; clock <= 7; ++clock) {
    DspMachine machine;
    prepare(machine, code);
    auto expected = machine.cpu().debug_state();
    receive(machine.bus());
    DspFetchRun run(contract(), machine);
    clocks(run, machine, clock);
    const auto saved = run.checkpoint(machine);
    const auto captured = machine.cpu().debug_state();
    const auto status = machine.bus().read({.space = DspSpace::x, .address = hcsr});
    require(((status & not_empty) != 0) == (clock < 5),
            "FIFO empties at the second actual AG1, before either E2 publication");
    expected.cpu.pc = body + 2;
    expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | first;
    expected.cpu.y = (expected.cpu.y & ~std::uint64_t{word_mask}) | second;
    expected.native_instructions += 2;
    expected.native_cycles += 2;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      require(machine.cpu().debug_state() == captured &&
                  machine.bus().read({.space = DspSpace::x, .address = hcsr}) == status,
              "Joint FIFO restore lost actual capture/consumption state");
      clocks(run, machine, 8 - clock);
      require(machine.cpu().debug_state() == expected && run.completed_cycles() == 8 &&
                  (machine.bus().read({.space = DspSpace::x, .address = hcsr}) & not_empty) == 0,
              "Automatic native replay duplicated FIFO pops, writeback or retirement");
      static_cast<void>(checkpoint::rejects(
          [&] { static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = hrx})); },
          "Two actual automatic reads did not leave the receive FIFO empty"));
    }
  }
}

void branch_snapshots(const bool taken) {
  const std::array<std::string, 3> code{std::string(taken ? "jset" : "jclr") + " #0,x:<<$ffff90," +
                                            number(body + 4),
                                        "movep #>$654321,x:<<$fffffc",
                                        "move #>$654321,x0"};
  for (unsigned clock = 3; clock <= 10; ++clock) {
    DspMachine machine;
    prepare(machine, code);
    auto expected = machine.cpu().debug_state();
    DspFetchRun run(contract(), machine);
    clocks(run, machine, clock);
    const auto saved = run.checkpoint(machine);
    const auto captured = machine.cpu().debug_state();
    const auto latch = machine.bus().once_latch();
    expected.cpu.pc = body + 6;
    expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | second;
    expected.native_instructions += taken ? 2U : 3U;
    expected.native_cycles += taken ? 6U : 8U;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      require(machine.cpu().debug_state() == captured && machine.bus().once_latch() == latch,
              "Peripheral branch restore lost actual sample, preview or target ownership");
      clocks(run, machine, (taken ? 11U : 14U) - clock);
      require(machine.cpu().debug_state() == expected &&
                  machine.bus().once_latch() == (taken ? std::nullopt : std::optional(second)),
              "Restored peripheral branch changed native state or executed discarded I/O");
    }
  }
}

void self_branch() {
  const std::array<std::string, 1> code{"jset #0,x:<<$ffff90," + number(body)};
  DspMachine machine;
  prepare(machine, code);
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 7);
  for (unsigned iteration = 0; iteration < 4; ++iteration) {
    if (iteration != 0) { clocks(run, machine, 4); }
    ++expected.native_instructions;
    expected.native_cycles += 4;
    require(
        machine.cpu().debug_state() == expected && run.completed_cycles() == 7 + 4 * iteration,
        "Repeated peripheral self branch must capture fresh F2/data and retire every four clocks");
  }
}

void unused_target() {
  DspMachine machine;
  const std::array<std::string, 2> code{"jset #1,x:<<$ffff90," + number(unknown_target), "nop"};
  prepare(machine, code);
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  clocks(run, machine, 12);
  expected.cpu.pc = body + 3;
  expected.native_instructions += 2;
  expected.native_cycles += 5;
  require(machine.cpu().debug_state() == expected && run.completed_cycles() == 12,
          "A false peripheral branch must never fetch its unused unknown target");
}

void selective_bit(const bool level, const bool set) {
  DspMachine machine;
  const std::array<std::string, 2> code{
      std::string(set ? "jset" : "jclr") + " #0,x:<<$ffffbd," + number(body + 2), "nop"};
  prepare(machine, code);
  auto expected = machine.cpu().debug_state();
  machine.bus().gpio(DspPort::c).write({.reg = DspGpioRegister::control, .value = 1, .mask = 1});
  machine.bus().gpio(DspPort::c).set_drive({.mask = 1, .levels = level ? 1U : 0U});
  const bool taken = level == set;
  DspFetchRun run(contract(), machine);
  clocks(run, machine, taken ? 11 : 12);
  expected.cpu.pc = body + 3;
  expected.native_instructions += 2;
  expected.native_cycles += 5;
  require(machine.cpu().debug_state() == expected,
          "Selective automatic branch invented unobserved GPIO bits or lost equal-target timing");
  static_cast<void>(checkpoint::rejects(
      [&] {
        static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = port_c_data}));
      },
      "A selective automatic branch invented whole-word GPIO observations"));
}

void state() {
  fifo_snapshots();
  branch_snapshots(false);
  branch_snapshots(true);
  self_branch();
  unused_target();
  for (const bool level : {false, true}) {
    for (const bool set : {false, true}) { selective_bit(level, set); }
  }
}
}
}

int main(const int argc, const char *const argv[]) {
  try {
    blofeld::test::require(argc == 2, "Select automatic peripheral state");
    if (std::string_view(argv[1]) == "state") {
      blofeld::test::peripheral_automatic_state::state();
    } else {
      throw blofeld::Fault("Unknown automatic peripheral state group");
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
