// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <cstdint>
#include <ostream>

namespace blofeld::test::spi_prefix {
inline constexpr unsigned control = 0xffff91;
inline constexpr unsigned not_empty = 0x20000;
inline constexpr unsigned length_poll = 0xff001a;
inline constexpr unsigned address_poll = 0xff001d;
inline constexpr unsigned empty_poll_clocks = 43;
inline constexpr unsigned replayed_poll_clocks = 51;
inline constexpr unsigned length_received_clocks = 53;

inline DspDebugState waiting(DspDebugState expected, const unsigned branches) {
  expected.cpu.pc = length_poll;
  expected.native_instructions += 1 + branches;
  expected.native_cycles += 1 + 4 * branches;
  return expected;
}

inline void word(DspBus &bus, const std::uint32_t value) {
  bus.shi().observe_spi({.selected = true, .mosi = false});
  for (unsigned remaining = word_bits; remaining != 0; --remaining) {
    const bool bit = ((value >> (remaining - 1)) & 1U) != 0;
    bus.shi().observe_spi({.selected = true, .clock = true, .mosi = bit});
    bus.shi().observe_spi({.selected = true, .mosi = bit});
  }
}

inline void record(std::ostream &output, const DspFetchRun &run, const DspMachine &machine) {
  const auto cpu = machine.cpu().debug_state();
  output << run.completed_cycles() << " pc=" << hex(cpu.cpu.pc, 6)
         << " instructions=" << cpu.native_instructions << " native-cycles=" << cpu.native_cycles
         << " a=" << hex(cpu.cpu.a, 14) << '\n';
}

inline void continue_prefix(DspFetchRun &run,
                            DspMachine &machine,
                            const DspDebugState &prefix,
                            std::ostream &output) {
  pipeline::clocks(run, machine, 13);
  const auto empty = waiting(prefix, 3);
  require(machine.cpu().debug_state() == empty && run.completed_cycles() == empty_poll_clocks &&
              (machine.bus().shi().read(control) & (not_empty | 1U)) == 1,
          "The document prefix enables real SHI and retires three actual empty HRNE polls");
  record(output, run, machine);
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    pipeline::clocks(run, machine, 8);
    require(machine.cpu().debug_state() == waiting(prefix, 5) &&
                run.completed_cycles() == replayed_poll_clocks &&
                (machine.bus().shi().read(control) & not_empty) == 0,
            "Empty input keeps the original HRNE loop running with fresh four-clock branches");
  }
  run.restore(saved, machine);
  word(machine.bus(), 1);
  pipeline::clocks(run, machine, 10);
  auto expected = empty;
  expected.cpu.pc = address_poll;
  expected.cpu.a = 1;
  expected.native_instructions += 3;
  expected.native_cycles += 9;
  require(machine.cpu().debug_state() == expected &&
              run.completed_cycles() == length_received_clocks &&
              (machine.bus().shi().read(control) & not_empty) == 0,
          "Real SPI length exits HRNE, reaches native A0 and enters the genuine address poll");
  record(output, run, machine);
}
}
