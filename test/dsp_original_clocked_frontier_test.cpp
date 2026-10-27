// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_boot_reference.hpp"
#include "dsp_document_boot_reference.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_spi_prefix_fixture.hpp"
#include "firmware/firmware.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ostream>
#include <span>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned frame_offset = 0x7875;
constexpr unsigned stage_words = 94;
constexpr unsigned frame_words = stage_words + 2;
constexpr unsigned stage = 0x6a;
constexpr unsigned entry_jump = 0xff0035;
constexpr unsigned maximum_cycles = 40000;
constexpr unsigned word_bytes = 3;

std::uint32_t frame_word(const std::span<const std::uint8_t> frame, const unsigned index) {
  return dsp_boot_word(std::span<const std::uint8_t, word_bytes>(
      frame.subspan(static_cast<std::size_t>(index) * word_bytes, word_bytes)));
}

void verify_stage(const DspMachine &machine, const std::span<const std::uint8_t> frame) {
  require(frame_word(frame, 0) == stage_words && frame_word(frame, 1) == stage,
          "Original frame owns the literal length and target address");
  for (unsigned index = 0; index < stage_words; ++index) {
    require(machine.bus().inspect_program(stage + index) == frame_word(frame, index + 2),
            "Every original first-stage word must arrive through actual native SPI/P stores");
  }
}

struct Files {
  std::filesystem::path firmware;
  std::filesystem::path rom;
  std::filesystem::path trace;
};

void run(const Files &files) {
  const auto firmware = Firmware::load(files.firmware);
  const auto frame = std::span(firmware.mcu)
                         .subspan(frame_offset, static_cast<std::size_t>(frame_words) * word_bytes);
  const auto rom = read_document_boot_rom(files.rom);
  DspMachine machine;
  machine.bus().install_rom({.start = boot, .words = rom});
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().shi().observe_spi({.mosi = false});
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  auto contract = pipeline::contract();
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun owner(contract, machine);
  std::ofstream trace(files.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  trace << "Clocked document ROM + original94word frame;explicit between-period SPI stimulus;"
           "no MCU scheduling/physical phase claim\n";
  unsigned sent = 0;
  bool entered = false;
  bool loaded = false;
  try {
    while (owner.completed_cycles() < maximum_cycles) {
      const auto cycle = owner.advance_cycle(machine, fetch_clock::input());
      const auto &phase = pipeline::stages(cycle);
      trace << cycle.completed_cycles << " pc=" << hex(machine.cpu().state().pc, 6);
      if (phase.decode) { trace << " decode=" << hex(phase.decode->address, 6); }
      if (phase.execute2 && phase.retirement_published) {
        trace << " retired=" << hex(phase.execute2->address, 6);
      }
      trace << '\n';
      if (phase.ag1 && phase.ag1->address == entry_jump) {
        require(sent == frame_words, "All original wire words precede actual JMP resolution");
        verify_stage(machine, frame);
        loaded = true;
        trace << "ORIGINAL_STAGE_LOADED all94Pwords_exact=1\n";
      }
      if (phase.execute2 && phase.retirement_published && phase.execute2->address == entry_jump) {
        require(sent == frame_words && machine.cpu().state().pc == stage,
                "The genuine loader JMP must enter the unchanged received original stage");
        verify_stage(machine, frame);
        entered = true;
        trace << "ORIGINAL_STAGE_ENTERED all94Pwords_exact=1\n";
      }
      if (cycle.completed_cycles >= spi_prefix::empty_poll_clocks && sent < frame_words &&
          (machine.bus().shi().read(spi_prefix::control) & spi_prefix::not_empty) == 0) {
        spi_prefix::word(machine.bus(), frame_word(frame, sent));
        ++sent;
      }
    }
  } catch (const Fault &error) {
    trace << "FRONTIER sent=" << sent << " entered=" << entered << " loaded=" << loaded
          << " cycles=" << owner.completed_cycles() << " pc=" << hex(machine.cpu().state().pc, 6)
          << " error=" << error.what() << '\n';
    std::cerr << "Original clocked first-stage frontier:sent=" << sent << " entered=" << entered
              << " loaded=" << loaded << " cycles=" << owner.completed_cycles()
              << " pc=" << hex(machine.cpu().state().pc, 6) << " " << error.what() << '\n';
    throw;
  }
  throw Fault("Original first-stage probe exceeded its explicit cycle bound");
}
}

int main(const int argc, const char *const *const argv) {
  try {
    require(argc == 4, "Expected original firmware ZIP, document main ROM and trace");
    run({.firmware = argv[1], .rom = argv[2], .trace = argv[3]});
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
