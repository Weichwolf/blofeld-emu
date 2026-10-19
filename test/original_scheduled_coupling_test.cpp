// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "coupled_boot_fixture.hpp"
#include "dsp_boot_reference.hpp"
#include "original_owned_clock_fixture.hpp"
#include "scheduled_coupling_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <optional>
#include <ostream>
#include <span>
#include <sstream>
#include <string>

namespace blofeld::test::scheduled {
namespace {
struct Progress {
  owned_clock::Census census;
  owned_clock::FlashPhases phases;
  owned_clock::Interrupts interrupts;
  std::uint64_t ready_tick = 0;
};

void step(McuMachine &machine, Progress &progress) {
  const auto before = machine.cpu().state();
  const auto opcode = machine.bus().peek(before.pc);
  progress.census.before(machine);
  const auto result = machine.cpu().step();
  require(result.instruction_executed || result.interrupt_vector.has_value(),
          "Original scheduled startup must not silently stall outside its native execution");
  progress.census.retire(before, result);
  progress.interrupts.observe(before, result, machine.cpu(), opcode);
}

void cold_start(McuMachine &machine, Progress &progress, std::ostream &trace) {
  constexpr unsigned initial_e = 0xfb;
  constexpr unsigned byte_mask = 0xff;
  constexpr unsigned receive_pad = 4;
  constexpr unsigned ready_inputs = 0x3f;
  machine.bus().set_pin_drive(McuPort::e, {.mask = initial_e, .levels = 3});
  machine.bus().set_pin_drive(McuPort::q, {.mask = byte_mask});
  machine.bus().set_pin_drive(McuPort::r, {.mask = receive_pad, .levels = receive_pad});
  owned_clock::Inputs inputs;
  machine.set_cycle_observer(
      [&](const auto cycle) { progress.phases.observe(cycle, machine.run_clock().bus_cycles()); });
  machine.bus().set_observer([&](const auto &event) { record_clock_write(machine, event, trace); });
  try {
    constexpr unsigned limit = 2500000;
    unsigned steps = 0;
    while (machine.cpu().state().pc != coupled::ready_poll) {
      require(++steps <= limit, "Native clocked MCU startup exceeded the ready-poll bound");
      inputs.observe(machine);
      step(machine, progress);
    }
  } catch (...) {
    machine.set_cycle_observer({});
    machine.bus().set_observer({});
    throw;
  }
  machine.set_cycle_observer({});
  machine.bus().set_observer({});
  machine.bus().set_pin_drive(McuPort::u, {.mask = ready_inputs});
  machine.bus().set_pin_drive(McuPort::b, {.mask = coupled::other_b_inputs});
}

struct Replay {
  McuCheckpoint mcu;
  DspCheckpoint dsp;
  LinkCheckpoint link;
  Progress progress;
  std::size_t offset = 0;
};

void write_trace(const std::filesystem::path &path, const std::string &text) {
  std::ofstream file(path);
  file.exceptions(std::ios::failbit | std::ios::badbit);
  file << text;
}

void transmit(McuMachine &mcu,
              DspMachine &dsp,
              ClockedLink &link,
              Progress &progress,
              const std::uint64_t epoch,
              std::ostringstream &trace,
              std::optional<Replay> *replay) {
  link.attach();
  try {
    constexpr unsigned limit = 500000;
    constexpr unsigned ready_inputs = 0x3f;
    constexpr unsigned middle_frame = 48;
    unsigned steps = 0;
    while (mcu.cpu().state().pc != coupled::sender_end) {
      require(++steps <= limit, "Original scheduled sender exceeded its native instruction bound");
      if (mcu.cpu().state().pc == coupled::ready_poll &&
          common_ticks(mcu) >= epoch + ready_delay_ticks) {
        if (progress.ready_tick == 0) {
          progress.ready_tick = common_ticks(mcu);
          trace << "ready end=" << progress.ready_tick << '\n';
        }
        mcu.bus().set_pin_drive(McuPort::u, {.mask = ready_inputs, .levels = 1});
      }
      step(mcu, progress);
      link.after_mcu_step();
      if (replay != nullptr && !replay->has_value() && link.frames() >= middle_frame &&
          link.checkpoint().previous.selected) {
        *replay = Replay{.mcu = mcu.checkpoint(),
                         .dsp = dsp.checkpoint(),
                         .link = link.checkpoint(),
                         .progress = progress,
                         .offset = trace.str().size()};
      }
    }
  } catch (...) {
    link.detach();
    throw;
  }
  link.detach();
}

void verify_loaded(const McuMachine &mcu,
                   DspMachine &dsp,
                   const ClockedLink &link,
                   const std::span<const std::uint8_t> image) {
  require(link.writes() == coupled::expected_port_writes &&
              link.frames() == coupled::stage_words + 3,
          "Scheduled original MCU must retain every one of 7275 stores and 97 wire frames");
  for (unsigned index = 0; index < coupled::stage_words; ++index) {
    const auto offset = coupled::frame_offset + ((static_cast<std::size_t>(index) + 2) * 3);
    const auto expected = dsp_boot_word(std::span<const std::uint8_t, 3>(image.subspan(offset, 3)));
    require(dsp.bus().read({.address = coupled::stage_base + index}) == expected,
            "Executed DSP ROM must load every original word from timed native MCU stores");
  }
  const auto state = dsp.cpu().state();
  constexpr unsigned original_omr = 0x600389;
  constexpr unsigned first_stage_pctl = 0x02e112;
  require(state.pc == coupled::clear_pc && state.r[4] > 0 && state.r[4] < coupled::long_ram_end &&
              state.omr == original_omr && state.lc == coupled::clear_count - state.r[4] &&
              dsp.bus().pll().read() == first_stage_pctl,
          "Both real CPUs reach sender return and the unmodified partial mapped clear together");
  for (unsigned address = 0; address < state.r[4]; ++address) {
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      require(dsp.bus().read({.space = space, .address = address}) == 0,
              "Every actual mapped clear store has its independent zero result");
    }
  }
  auto probe = dsp.bus(); // Diagnostic FIFO inspection must not alter live native state.
  constexpr unsigned receive_not_empty = 0x020000;
  constexpr unsigned receive_overrun = 0x100000;
  require(
      (probe.shi().read(coupled::hcsr) & (receive_not_empty | receive_overrun)) ==
          receive_not_empty,
      "Timed receiver consumes its stage without overrun and retains exactly the trailing word");
  const auto trailing_offset =
      coupled::frame_offset + ((static_cast<std::size_t>(coupled::stage_words) + 2) * 3);
  require(probe.shi().read(coupled::hrx) == dsp_boot_word(std::span<const std::uint8_t, 3>(
                                                image.subspan(trailing_offset, 3))) &&
              (probe.shi().read(coupled::hcsr) & receive_not_empty) == 0,
          "A copied FIFO reveals exactly the original trailing MCU word");
  require(!mcu.timing_faulted() && !dsp.timing_faulted(), "Both native time owners remain healthy");
  const auto observed = link.checkpoint();
  require(observed.ordered_pairs == 1 && !observed.injected,
          "Positive runs order the one source ambiguity and inject no malformed input");
}

void record(const std::string &name,
            const McuMachine &mcu,
            const DspMachine &dsp,
            const ClockedLink &link,
            const Progress &progress,
            const std::uint64_t epoch,
            std::ostream &trace) {
  trace << "case name=" << name << " epoch=" << epoch << " ready=" << progress.ready_tick
        << " retired=" << progress.census.retired << " entries=" << progress.interrupts.entries
        << " cycles=" << mcu.run_clock().bus_cycles()
        << " ticks=" << mcu.run_clock().elapsed_ticks()
        << " OSC=" << mcu.run_clock().oscillator_cycles() << " extra=" << progress.phases.low
        << " common=" << common_ticks(mcu) << " dsp-steps=" << link.steps()
        << " dsp-units=" << dsp.run_clock().native_cycle_units()
        << " dsp-ticks=" << dsp.run_clock().elapsed_ticks() << " pairs=" << dsp.cpu().state().r[4]
        << " ties=" << link.ties() << '\n';
}

unsigned experiment(McuMachine &mcu,
                    const std::span<const std::uint8_t> image,
                    const std::span<const std::uint32_t> rom,
                    const Progress &cold,
                    const Experiment input,
                    const std::filesystem::path &path,
                    const std::string &name,
                    std::ostream &summary) {
  DspMachine dsp(
      DspRunContract{.clock = {.extal_hz = oscillator_hz,
                               .timebase_hz = common_hz,
                               .switch_timing = DspClockSwitch::reference_instruction_boundary},
                     .timing = DspRunTiming::native_internal_estimates});
  dsp.bus().install_rom({.start = coupled::bootstrap_base, .words = rom});
  dsp.bus().pll().observe_pinit(false);
  dsp.cpu().reset_spi_boot();
  Progress progress = cold;
  const auto epoch = common_ticks(mcu) + input.phase_ticks;
  std::ostringstream trace;
  ClockedLink link(mcu, dsp, progress.phases, trace, input, epoch);
  link.start();
  std::optional<Replay> frozen;
  transmit(mcu, dsp, link, progress, epoch, trace, &frozen);
  verify_loaded(mcu, dsp, link, image);
  if (!frozen) { throw Fault("Scheduled source/receiver did not capture its selected mid-frame"); }
  const auto expected_mcu = mcu.cpu().state();
  const auto expected_dsp = dsp.cpu().debug_state();
  const auto expected_mcu_ticks = mcu.run_clock().elapsed_ticks();
  const auto expected_dsp_ticks = dsp.run_clock().elapsed_ticks();
  const auto original = trace.str();
  record(name, mcu, dsp, link, progress, epoch, summary);
  progress.census.write(path.string() + "." + name + ".mcu.jsonl");
  progress.phases.write(path.string() + "." + name + ".phases.json");
  link.write_census(path.string() + "." + name + ".dsp.jsonl");
  write_trace(path.string() + "." + name + ".trace", original);
  mcu.restore(frozen->mcu);
  dsp.restore(frozen->dsp);
  progress = frozen->progress;
  trace.str({});
  trace.clear();
  link.restore(frozen->link);
  transmit(mcu, dsp, link, progress, epoch, trace, nullptr);
  verify_loaded(mcu, dsp, link, image);
  require(trace.str() == original.substr(frozen->offset) &&
              owned_clock::same_cpu(mcu.cpu().state(), expected_mcu) &&
              dsp.cpu().debug_state() == expected_dsp &&
              mcu.run_clock().elapsed_ticks() == expected_mcu_ticks &&
              dsp.run_clock().elapsed_ticks() == expected_dsp_ticks,
          "Joint native CPU/bus/pin/deadline checkpoint replays the exact timestamped suffix");
  return link.ties();
}

void rejection(McuMachine &mcu,
               const std::span<const std::uint8_t> image,
               const std::span<const std::uint32_t> rom,
               const Progress &cold,
               const Experiment input,
               const std::filesystem::path &path,
               const std::string &expected,
               std::ostream &trace) {
  try {
    experiment(mcu, image, rom, cold, input, path, "invalid", trace);
  } catch (const Fault &fault) {
    const std::string message = fault.what();
    require(message.contains(expected) && message.contains("mcu-pc=") &&
                message.contains("dsp-pc=") && message.contains("0x0001"),
            "Scheduled failure must retain both PCs, reason and actual write address: " + message);
    trace << "rejected " << message << '\n';
    return;
  }
  throw Fault("An unresolved scheduled input unexpectedly succeeded: " + expected);
}

struct Paths {
  std::filesystem::path firmware;
  std::filesystem::path rom;
  std::filesystem::path trace;
};

void run(const Paths &paths) {
  const auto firmware = Firmware::load(paths.firmware);
  const auto rom = read_spi_boot_prefix(paths.rom);
  const auto image = McuFlashImage::from_known(firmware.mcu);
  McuMachine mcu(
      image,
      BLOFELD_DEFAULT_SLA,
      McuRunContract{.clock = {.oscillator_hz = oscillator_hz,
                               .initial_oscillator_phase_ticks = 0,
                               .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                     .memory_timing = McuRunMemory::internal_flash_reads});
  Progress cold;
  std::ofstream trace(paths.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  cold_start(mcu, cold, trace);
  const auto root = mcu.checkpoint();
  std::optional<unsigned> tied_phase;
  for (const unsigned phase : {0U, 9U}) {
    for (const auto order : {TieOrder::dsp_before_mcu, TieOrder::mcu_before_dsp}) {
      mcu.restore(root);
      const std::string name = "phase" + std::to_string(phase) +
                               (order == TieOrder::dsp_before_mcu ? "-dsp-first" : "-mcu-first");
      const auto ties = experiment(
          mcu,
          firmware.mcu,
          rom,
          cold,
          {.pads = coupled::PadOrder::clock_before_select, .tie = order, .phase_ticks = phase},
          paths.trace,
          name,
          trace);
      if (ties != 0) { tied_phase = phase; }
      owned_clock::check_nvm(mcu, image);
    }
  }
  if (!tied_phase) { throw Fault("Original scheduled experiments did not exercise exact ties"); }
  mcu.restore(root);
  rejection(mcu,
            firmware.mcu,
            rom,
            cold,
            {.tie = TieOrder::dsp_before_mcu},
            paths.trace,
            "Ambiguous simultaneous",
            trace);
  mcu.restore(root);
  rejection(mcu,
            firmware.mcu,
            rom,
            cold,
            {.pads = coupled::PadOrder::select_before_clock, .tie = TieOrder::dsp_before_mcu},
            paths.trace,
            "configured idle SCK",
            trace);
  mcu.restore(root);
  rejection(mcu,
            firmware.mcu,
            rom,
            cold,
            {.pads = coupled::PadOrder::clock_before_select,
             .tie = TieOrder::unqualified,
             .phase_ticks = *tied_phase},
            paths.trace,
            "Unqualified simultaneous DSP completion",
            trace);
  mcu.restore(root);
  rejection(mcu,
            firmware.mcu,
            rom,
            cold,
            {.pads = coupled::PadOrder::clock_before_select,
             .tie = TieOrder::dsp_before_mcu,
             .break_mosi = true},
            paths.trace,
            "Unknown DSP SHI MOSI",
            trace);
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 4) {
      throw blofeld::Fault(
          "Usage: scheduled-coupling-test firmware.zip private-rom.bin private.trace");
    }
    blofeld::test::scheduled::run(
        {.firmware = arguments[1], .rom = arguments[2], .trace = arguments[3]});
    std::cout
        << "Original MCU/DSP common nominal time, native wire transfer and joint replay passed; "
           "physical timing/board boot remain unqualified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
