// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/firmware.hpp"
#include "dsp56kEmu/disasm.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp_boot_reference.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <ostream>
#include <span>
#include <string>

namespace {
constexpr unsigned bootstrap_base = 0xff0000;
constexpr unsigned frame_offset = 0x7875;
constexpr std::size_t stage_words = 94;
constexpr unsigned stage_base = 0x6a;
constexpr unsigned clear_pc = 0x72;
constexpr unsigned clear_count = 0x20000;
constexpr unsigned mapped_pairs = 0x2000;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 540000000;
constexpr unsigned initial_pctl = 0x04401d;
constexpr unsigned stage_pctl = 0x02e112;
constexpr unsigned original_omr = 0x600389;
constexpr unsigned step_limit = 20000;
constexpr unsigned word_bits = 24;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

blofeld::DspRunContract contract() {
  return {.clock = {.extal_hz = extal_hz,
                    .timebase_hz = timebase_hz,
                    .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
          .timing = blofeld::DspRunTiming::native_internal_estimates};
}

class Runner {
  struct Row {
    std::string name;
    std::uint32_t opcode = 0;
    std::uint32_t extension = 0;
    unsigned length = 0;
    unsigned count = 0;
    std::uint64_t units = 0;
    std::uint32_t pctl = 0;
  };

public:
  struct Progress {
    std::map<unsigned, Row> rows;
    unsigned steps = 0;
  };

  explicit Runner(blofeld::DspMachine &machine) : machine_(machine), disassembler_(opcodes_) {}

  [[nodiscard]] Progress checkpoint() const { return {.rows = rows_, .steps = steps_}; }

  void restore(const Progress &progress) {
    rows_ = progress.rows;
    steps_ = progress.steps;
  }

  void step() {
    require(steps_ < step_limit, "Original DSP clocked prefix exceeded its instruction bound");
    const auto before = machine_.cpu().state();
    const auto first = machine_.bus().read({.address = before.pc});
    const auto second = machine_.bus().peek_program(before.pc + 1);
    const auto pctl = machine_.bus().pll().read();
    const auto cycles = machine_.run_clock().native_cycle_units();
    machine_.cpu().step();
    const auto used = machine_.run_clock().native_cycle_units() - cycles;
    auto &row = rows_[before.pc];
    if (row.count == 0) {
      dsp56k::Disassembler::Line line;
      row.length = disassembler_.disassemble(line, first, second, before.sr, before.omr, before.pc);
      row.name = line.opName;
      require(row.name.find_first_of("\n\r\t") == std::string::npos,
              "Native diagnostic mnemonic must be one printable JSON string");
      row.opcode = first;
      row.extension = second;
      row.units = used;
      row.pctl = pctl;
    }
    require(row.opcode == first && row.units == used && row.pctl == pctl,
            "Each unmodified original PC retains its opcode, cycle estimate and clock source");
    ++row.count;
    ++steps_;
  }

  void send(const std::uint32_t value) {
    auto &shi = machine_.bus().shi();
    shi.observe_spi({.selected = true, .mosi = false});
    step();
    for (unsigned bit = 0; bit < word_bits; ++bit) {
      const bool high = ((value >> (word_bits - 1 - bit)) & 1U) != 0;
      shi.observe_spi({.selected = true, .clock = true, .mosi = high});
      step();
      shi.observe_spi({.selected = true, .mosi = high});
      step();
    }
    shi.observe_spi({.mosi = false});
    step();
  }

  void finish_clear(const unsigned end) {
    while (machine_.cpu().state().pc != clear_pc || machine_.cpu().state().r[4] != end) { step(); }
  }

  void write(const std::filesystem::path &path) const {
    std::ofstream file(path);
    file.exceptions(std::ios::failbit | std::ios::badbit);
    for (const auto &[pc, row] : rows_) {
      file << "{\"pc\":" << pc << ",\"opcode\":" << row.opcode << ",\"extension\":" << row.extension
           << ",\"length\":" << row.length << ",\"name\":" << std::quoted(row.name)
           << ",\"count\":" << row.count << ",\"units\":" << row.units << ",\"pctl\":" << row.pctl
           << "}\n";
    }
  }

private:
  blofeld::DspMachine &machine_;
  dsp56k::Opcodes opcodes_;
  dsp56k::Disassembler disassembler_;
  std::map<unsigned, Row> rows_;
  unsigned steps_ = 0;
};

void record(const std::string &section, const blofeld::DspMachine &machine, std::ostream &trace) {
  const auto state = machine.cpu().state();
  const auto &clock = machine.run_clock();
  trace << section << " cycles=" << clock.native_cycle_units() << " ticks=" << clock.elapsed_ticks()
        << " clk2=" << clock.timer_clk2_edges() << " pc=" << state.pc << " r4=" << state.r[4]
        << " lc=" << state.lc << " omr=" << state.omr << '\n';
}

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path rom;
  std::filesystem::path trace;
};

void run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  const auto rom = blofeld::test::read_spi_boot_prefix(inputs.rom);
  blofeld::DspMachine machine(contract());
  machine.bus().install_rom({.start = bootstrap_base, .words = rom});
  // Explicit reference EXTAL/PINIT inputs, not measured Blofeld reset or clock data.
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().shi().observe_spi({.mosi = false});
  require(machine.bus().pll().read() == initial_pctl, "Declared low PINIT selects bypass at reset");
  std::ofstream trace(inputs.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  Runner runner(machine);
  while ((machine.bus().shi().read(hcsr) & 1U) == 0) { runner.step(); }
  const auto frame =
      std::span<const std::uint8_t>(firmware.mcu).subspan(frame_offset, (stage_words + 2) * 3);
  for (unsigned offset = 0; offset < frame.size(); offset += 3) {
    runner.send(
        blofeld::test::dsp_boot_word(std::span<const std::uint8_t, 3>(frame.subspan(offset, 3))));
  }
  while (machine.cpu().state().pc != stage_base) { runner.step(); }
  for (std::size_t index = 0; index < stage_words; ++index) {
    const auto expected = blofeld::test::dsp_boot_word(
        std::span<const std::uint8_t, 3>(frame.subspan((index + 2) * 3, 3)));
    require(machine.bus().read({.address = stage_base + static_cast<unsigned>(index)}) == expected,
            "Executed ROM loads every unmodified stage word from its wire input");
  }
  constexpr unsigned partial_pairs = 4096;
  runner.finish_clear(partial_pairs);
  record("partial", machine, trace);
  const auto partial = machine.checkpoint();
  const auto partial_runner = runner.checkpoint();
  const auto continue_to_frontier = [&] {
    runner.finish_clear(mapped_pairs);
    const auto state = machine.cpu().state();
    require(
        state.pc == clear_pc && state.r[4] == mapped_pairs &&
            state.lc == clear_count - mapped_pairs && state.omr == original_omr &&
            machine.bus().pll().read() == stage_pctl && !machine.timing_faulted(),
        "Actual original clear count, memory mode and PLL command reach the same strict frontier");
    for (const auto space : {blofeld::DspSpace::x, blofeld::DspSpace::y}) {
      for (unsigned address = 0; address < mapped_pairs; ++address) {
        require(
            machine.bus().read({.space = space, .address = address}) == 0,
            "Native stores clear all mapped pairs without a broad RAM or dropped-write shortcut");
      }
    }
  };
  continue_to_frontier();
  const auto expected_cpu = machine.cpu().debug_state();
  const auto expected_cycles = machine.run_clock().native_cycle_units();
  const auto expected_ticks = machine.run_clock().elapsed_ticks();
  record("frontier", machine, trace);
  runner.write(inputs.trace.string() + ".census.jsonl");
  machine.restore(partial);
  runner.restore(partial_runner);
  continue_to_frontier();
  require(machine.cpu().debug_state() == expected_cpu &&
              machine.run_clock().native_cycle_units() == expected_cycles &&
              machine.run_clock().elapsed_ticks() == expected_ticks,
          "Coordinated native REP/PLL/CLK2/memory-map replay preserves state and reference time");
  record("replay", machine, trace);
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 4) {
      throw blofeld::Fault("Usage: dsp-clock-test firmware.zip private-rom.bin private.trace");
    }
    run({.firmware = arguments[1], .rom = arguments[2], .trace = arguments[3]});
    std::cout << "Original DSP ROM/stage, declared cycle/PLL reference and mid-REP replay passed; "
                 "reserved-write/physical timing/full board boot remain unqualified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
