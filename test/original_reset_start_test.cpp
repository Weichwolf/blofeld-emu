// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "coupled_boot_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp56kEmu/disasm.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp_boot_reference.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "original_owned_clock_fixture.hpp"
#include "scheduled_coupling_fixture.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <span>
#include <sstream>
#include <string>

namespace blofeld::test::scheduled {
void step(McuMachine &machine, Progress &progress) {
  const auto before = machine.cpu().state();
  const auto opcode = machine.bus().peek(before.pc);
  progress.census.before(machine);
  const auto result = machine.cpu().step();
  require(result.instruction_executed || result.interrupt_vector.has_value(), "Native progress");
  progress.census.retire(before, result);
  progress.interrupts.observe(before, result, machine.cpu(), opcode);
}
}

namespace {
using namespace blofeld;
using namespace blofeld::test;
namespace source = blofeld::test::scheduled;
using source::require;
constexpr unsigned u_direction = 0x27a, u_data = 0x278, release_pc = 0xb98a;
constexpr unsigned other_u = 0x3e, spi_pins = coupled::serial_pins;
constexpr unsigned all_modes = 15, spi_mode = 5;
constexpr unsigned limit = 2500000, release_boot_inputs = 0x820e;
constexpr unsigned initial_e = 0xfb, byte_mask = 0xff, receive_pad = 4;
constexpr unsigned reset_status = 0xc00300, stage_mode = 0x600389, rx_not_empty = 0x020000;
constexpr unsigned rx_overrun = 0x100000, ram_begin = 0x1200, ram_size = 5120;
constexpr unsigned recovery_periods = 2, word_bytes = 3;
constexpr std::uint64_t common = source::common_hz;

struct Lab {
  Lab(McuMachine &native_mcu, source::Progress &native_progress, std::ostream &output)
      : mcu(native_mcu), progress(native_progress), trace(output) {}

  void inputs() {
    const auto pc = mcu.cpu().state().pc;
    // Retain existing explicit PHY/loopback/TIO observations, never its ready input.
    if (pc != coupled::ready_poll) { observed.observe(mcu); }
    if (pc == release_boot_inputs) {
      mcu.bus().set_pin_drive(McuPort::b, {.mask = coupled::other_b_inputs});
    }
  }

  [[nodiscard]] GpioPadSample net(const std::uint32_t mask) const {
    const auto drive = mcu.bus().gpio_outputs(McuPort::u);
    const auto bias = mcu.bus().gpio_biases(McuPort::u);
    return resolve_gpio_net(
        mask,
        {.active = drive.mask & mask,
         .known = drive.known & mask,
         .levels = drive.levels & mask,
         .reduced = drive.reduced_drive & mask,
         .pull_up = bias.up & mask,
         .pull_down = bias.down & mask},
        {.pull_up = 1U & mask, .pull_down = unused_pulldowns ? (other_u & mask) : 0});
  }

  void bind_u() {
    mcu.bus().attach_gpio_pad_source(McuPort::u, [this](const auto mask) { return net(mask); });
  }

  [[nodiscard]] std::uint64_t assert_tick() const {
    if (!asserted) { throw Fault("Missing actual native U0 assertion boundary"); }
    return *asserted;
  }

  McuMachine &mcu;
  source::Progress &progress;
  std::ostream &trace;
  owned_clock::Inputs observed;
  ExecutionContext context;
  std::optional<std::uint64_t> asserted;
  bool unused_pulldowns = false;
};

void prefix(Lab &lab) {
  auto &mcu = lab.mcu;
  mcu.bus().set_pin_drive(McuPort::e, {.mask = initial_e, .levels = 3});
  mcu.bus().set_pin_drive(McuPort::q, {.mask = byte_mask});
  mcu.bus().set_pin_drive(McuPort::r, {.mask = receive_pad, .levels = receive_pad});
  mcu.set_cycle_observer([&](const auto cycle) {
    lab.context = cycle.context;
    lab.progress.phases.observe(cycle, mcu.run_clock().bus_cycles());
  });
  mcu.bus().set_observer([&](const auto &event) {
    source::record_clock_write(mcu, event, lab.trace);
    if (!event.write || (event.address != u_data && event.address != u_direction)) { return; }
    const auto output = mcu.bus().gpio_outputs(McuPort::u);
    lab.trace << "U-PREFIX address=" << hex(event.address) << " value=" << hex(event.value, 2)
              << " pc=" << hex(lab.context.pc) << " active=" << static_cast<unsigned>(output.mask)
              << " known=" << static_cast<unsigned>(output.known)
              << " levels=" << static_cast<unsigned>(output.levels) << '\n';
    if (!lab.asserted && (output.mask & output.known & 1U) != 0 && (output.levels & 1U) == 0) {
      lab.asserted = source::common_ticks(mcu);
    }
  });
  for (unsigned count = 0; count < limit && !lab.asserted; ++count) {
    lab.inputs();
    source::step(mcu, lab.progress);
  }
  mcu.set_cycle_observer({});
  mcu.bus().set_observer({});
  if (!lab.asserted) { throw Fault("No actual native U0 low output assertion"); }
  lab.trace << "ASSERT tick=" << lab.assert_tick() << " completed=" << source::common_ticks(mcu)
            << " pc=" << hex(mcu.cpu().state().pc) << " retired=" << lab.progress.census.retired
            << '\n';
}

struct LinkCounts {
  bool released = false;
  std::uint64_t release_tick = 0;
  std::uint64_t dsp_steps = 0;
  unsigned writes = 0;
  unsigned frames = 0;
  unsigned ordered_pairs = 0;
};

class Link {
public:
  Link(Lab &lab, DspMachine &dsp)
      : lab_(lab),
        dsp_(dsp),
        scheduler_({.dsp = {.epoch_ticks = lab.assert_tick(),
                            .observation = DspStepObservation::reference_completion_boundary,
                            .resets = DspResetObservation::reference_extal_hold},
                    .shared_write_order = McuDspTieOrder::dsp_before_mcu,
                    .maximum_dsp_steps_per_cycle = 32},
                   lab.mcu,
                   dsp) {}

  [[nodiscard]] SpiInputs sample() const {
    const auto pins = lab_.mcu.bus().observe_pads(McuPort::b, spi_pins);
    if (pins.known != spi_pins) {
      throw Fault("Unknown actual SPI outputs known=" + hex(pins.known, 2) +
                  " levels=" + hex(pins.levels, 2));
    }
    return {.selected = (pins.levels & coupled::select_n) == 0,
            .clock = (pins.levels & coupled::clock_pin) != 0,
            .mosi = (pins.levels & coupled::mosi_pin) != 0};
  }

  void before(const DspScheduledStep &event) {
    ++counts_.dsp_steps;
    const auto state = event.plan.before.cpu;
    const auto opcode = dsp_.bus().read({.address = state.pc});
    auto &row = census[state.pc];
    if (row.count == 0) {
      const dsp56k::Opcodes opcodes;
      dsp56k::Disassembler decoder(opcodes);
      dsp56k::Disassembler::Line decoded;
      row.opcode = opcode;
      row.extension = dsp_.bus().peek_program(state.pc + 1);
      row.length =
          decoder.disassemble(decoded, opcode, row.extension, state.sr, state.omr, state.pc);
      row.name = decoded.opName;
      row.units = event.plan.native_cycle_units;
      row.pctl = dsp_.bus().pll().read();
    }
    require(row.opcode == opcode && row.units == event.plan.native_cycle_units &&
                row.pctl == dsp_.bus().pll().read(),
            "Each native source PC retains its opcode, nominal cost and clock route");
    ++row.count;
    lab_.trace << "DSP end=" << event.completion_ticks << " pc=" << hex(state.pc, 6)
               << " start=" << event.plan.start_ticks << " units=" << event.plan.native_cycle_units
               << " ticks=" << event.plan.duration_ticks << '\n';
  }

  void attach() {
    lab_.mcu.set_cycle_observer([this](const auto cycle) {
      lab_.context = cycle.context;
      lab_.progress.phases.observe(cycle, lab_.mcu.run_clock().bus_cycles());
      const bool shared = cycle.address && (cycle.kind == 'w' || cycle.kind == 'W') &&
                          (*cycle.address == coupled::port_b || *cycle.address == u_data ||
                           *cycle.address == u_direction);
      scheduler_.begin_cycle(cycle, shared, [this](const auto &event) { before(event); });
    });
    lab_.mcu.bus().set_observer([this](const auto &event) {
      if (!event.write) { return; }
      source::record_clock_write(lab_.mcu, event, lab_.trace);
      if (event.address != coupled::port_b && event.address != u_data &&
          event.address != u_direction) {
        return;
      }
      scheduler_.observe_write(
          event, [&] { observe(event); }, [this](const auto &native) { before(native); });
    });
  }

  void observe(const BusEvent &event) {
    if (event.address != coupled::port_b) {
      const auto level = lab_.net(1);
      require_known_gpio_pads(level, 1);
      const bool low = (level.levels & 1U) == 0;
      lab_.trace << "U " << scheduler_.end_ticks() << ' ' << hex(lab_.context.pc) << ' '
                 << hex(event.address) << ' ' << hex(event.value, 2) << " low=" << low << '\n';
      if (!low && !counts_.released) {
        require(dsp_.run_clock().native_cycle_units() == 0,
                "No DSP instruction during the actual low hold");
        previous_ = sample();
        dsp_.bus().shi().observe_spi(*previous_);
        scheduler_.release_dsp_reset();
        counts_.released = true;
        counts_.release_tick = scheduler_.end_ticks();
      } else if (low && counts_.released) {
        throw Fault("Unexpected native reassertion");
      }
      return;
    }
    if (!counts_.released) { return; }
    const auto next = sample();
    if (!previous_) { throw Fault("Released SPI link lost its actual sampled inputs"); }
    if (previous_->selected != next.selected && previous_->clock != next.clock) {
      auto intermediate = next;
      intermediate.selected = previous_->selected;
      dsp_.bus().shi().observe_spi(intermediate);
      ++counts_.ordered_pairs;
    }
    dsp_.bus().shi().observe_spi(next);
    ++counts_.writes;
    if (previous_->selected && !next.selected) { ++counts_.frames; }
    previous_ = next;
    lab_.trace << "B " << scheduler_.end_ticks() << ' ' << hex(lab_.context.pc) << ' '
               << hex(event.value, 2) << " frames=" << counts_.frames << '\n';
  }

  void step() {
    lab_.inputs();
    source::step(lab_.mcu, lab_.progress);
    scheduler_.end_instruction();
  }

  McuDspSchedule &schedule() { return scheduler_; }

  [[nodiscard]] std::optional<SpiInputs> previous() const { return previous_; }

  void write_census(const std::filesystem::path &path) const {
    std::ofstream file(path);
    file.exceptions(std::ios::failbit | std::ios::badbit);
    for (const auto &[pc, row] : census) {
      file << "{\"pc\":" << pc << ",\"opcode\":" << row.opcode << ",\"extension\":" << row.extension
           << ",\"length\":" << row.length << ",\"name\":" << std::quoted(row.name)
           << ",\"count\":" << row.count << ",\"units\":" << row.units << ",\"pctl\":" << row.pctl
           << "}\n";
    }
  }

  void restore_wires(const std::optional<SpiInputs> pins) {
    previous_ = pins;
    counts_.released = false;
    counts_.release_tick = 0;
    counts_.writes = 0;
    counts_.frames = 0;
    counts_.ordered_pairs = 0;
    counts_.dsp_steps = 0;
    census.clear();
    attach();
  }

  [[nodiscard]] const LinkCounts &counts() const { return counts_; }

private:
  std::map<unsigned, source::DspRow> census;
  LinkCounts counts_;
  Lab &lab_;
  DspMachine &dsp_;
  McuDspSchedule scheduler_;
  std::optional<SpiInputs> previous_;
};

void write_text(const std::filesystem::path &path, const std::string &text) {
  std::ofstream file(path);
  file.exceptions(std::ios::failbit | std::ios::badbit);
  file << text;
}

std::array<std::uint8_t, ram_size> ram(const McuMachine &mcu) {
  std::array<std::uint8_t, ram_size> result{};
  for (unsigned n = 0; n < ram_size; ++n) {
    result[n] = mcu.bus().peek(static_cast<std::uint16_t>(ram_begin + n));
  }
  return result;
}

void verify_payload(DspMachine &dsp, const Firmware &firmware, const Link &link) {
  require(
      link.counts().frames == coupled::stage_words + 3 &&
          link.counts().writes == coupled::expected_port_writes && link.counts().ordered_pairs == 1,
      "Original native stores retain97 frames,7275 writes and the declared pad-order transition");
  const std::span<const std::uint8_t> image(firmware.mcu);
  for (unsigned n = 0; n < coupled::stage_words; ++n) {
    const auto offset = coupled::frame_offset + ((static_cast<std::size_t>(n) + 2) * word_bytes);
    require(dsp.bus().read({.address = coupled::stage_base + n}) ==
                dsp_boot_word(
                    std::span<const std::uint8_t, word_bytes>(image.subspan(offset, word_bytes))),
            "Actual ROM execution receives every original stage word from native SPI stores");
  }
  for (unsigned n = 0; n < coupled::long_ram_end; ++n) {
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      require(dsp.bus().read({.space = space, .address = n}) == 0,
              "Every preceding mapped clear pair is zero");
    }
  }
  auto copy = dsp.bus();
  const auto trailing =
      coupled::frame_offset + ((static_cast<std::size_t>(coupled::stage_words) + 2) * word_bytes);
  require((copy.shi().read(coupled::hcsr) & (rx_not_empty | rx_overrun)) == rx_not_empty &&
              copy.shi().read(coupled::hrx) ==
                  dsp_boot_word(std::span<const std::uint8_t, word_bytes>(
                      image.subspan(trailing, word_bytes))) &&
              (copy.shi().read(coupled::hcsr) & rx_not_empty) == 0,
          "Only the genuine trailing UI word remains in the copied native FIFO, without overrun");
}

void record(const unsigned replay,
            Lab &lab,
            DspMachine &dsp,
            const Link &link,
            std::ostream &summary,
            const std::string &error) {
  const auto state = dsp.cpu().state();
  const auto &clock = lab.mcu.run_clock();
  summary << "STOP replay=" << replay << " unused_pulldowns=" << lab.unused_pulldowns
          << " release=" << link.counts().release_tick << " retired=" << lab.progress.census.retired
          << " cycles=" << clock.bus_cycles() << " ticks=" << clock.elapsed_ticks()
          << " OSC=" << clock.oscillator_cycles() << " entries=" << lab.progress.interrupts.entries
          << " common=" << source::common_ticks(lab.mcu) << " mcu=" << hex(lab.mcu.cpu().state().pc)
          << " dsp=" << hex(state.pc, 6) << " r4=" << state.r[4] << " lc=" << state.lc
          << " omr=" << hex(state.omr, 6) << " units=" << dsp.run_clock().native_cycle_units()
          << " dsp_ticks=" << dsp.run_clock().elapsed_ticks()
          << " reset_ticks=" << dsp.run_clock().hardware_reset_ticks()
          << " dsp_attempts=" << link.counts().dsp_steps << " frames=" << link.counts().frames
          << " writes=" << link.counts().writes << " pairs=" << link.counts().ordered_pairs
          << " nvm=1 fault=" << error << '\n';
}

void exercise(Lab &lab,
              DspMachine &dsp,
              Link &link,
              const Firmware &firmware,
              std::ostringstream &trace,
              const std::string &path,
              std::ostream &summary) {
  auto &mcu = lab.mcu;
  auto &progress = lab.progress;
  const auto cm = mcu.checkpoint();
  const auto cd = dsp.checkpoint();
  const auto cs = link.schedule().checkpoint();
  const auto cp = progress;
  const auto ci = lab.observed;
  const auto wires = link.previous();
  std::optional<CpuState> expected_mcu;
  std::optional<DspDebugState> expected_dsp;
  std::optional<std::array<std::uint8_t, ram_size>> expected_ram;
  std::string expected_error;
  std::string expected_trace;
  for (const unsigned replay : {0U, 1U, 2U}) {
    mcu.restore(cm);
    dsp.restore(cd);
    progress = cp;
    lab.observed = ci;
    lab.unused_pulldowns = replay != 0;
    lab.bind_u();
    link.schedule().restore(cs);
    link.restore_wires(wires);
    trace.str({});
    trace.clear();
    std::string error;
    try {
      for (unsigned count = 0; count < limit; ++count) { link.step(); }
      throw Fault("Original source exceeded its bounded frontier run");
    } catch (const Fault &fault) { error = fault.what(); }
    owned_clock::check_nvm(mcu, McuFlashImage::from_known(firmware.mcu));
    require(mcu.timing_faulted() && link.schedule().faulted(),
            "Actual source failure quarantines the shared owner");
    if (replay == 0) {
      require(error.contains("Unknown MCU GPIO input at 0x0278 mask=0x3e") &&
                  mcu.cpu().state().pc == coupled::ready_poll &&
                  dsp.run_clock().native_cycle_units() == 0,
              "Only undeclared unused U pins fail; reset pull-up cannot invent their levels");
    } else {
      require(error.contains("Unmapped DSP memory write at Y:0x002000") &&
                  error.contains("mcu-pc=0xb9ad") && dsp.timing_faulted() &&
                  dsp.cpu().state().omr == stage_mode,
              "Original source retains its strict reserved-write frontier");
      verify_payload(dsp, firmware, link);
      if (expected_mcu && expected_dsp && expected_ram) {
        require(owned_clock::same_cpu(mcu.cpu().state(), *expected_mcu) &&
                    dsp.cpu().debug_state() == *expected_dsp && ram(mcu) == *expected_ram &&
                    error == expected_error && trace.str() == expected_trace,
                "Held-reset replay retains every event, full observed CPU/stack state and all MCU "
                "SRAM");
      } else {
        expected_mcu = mcu.cpu().state();
        expected_dsp = dsp.cpu().debug_state();
        expected_ram = ram(mcu);
        expected_error = error;
        expected_trace = trace.str();
      }
    }
    record(replay, lab, dsp, link, summary, error);
    const auto output = path + "." + std::to_string(replay);
    write_text(output + ".trace", trace.str());
    progress.census.write(output + ".mcu.jsonl");
    progress.phases.write(output + ".phases.json");
    link.write_census(output + ".dsp.jsonl");
  }
  summary << "REPLAY native=1 ram=1 events=1\n";
}

struct Paths {
  std::filesystem::path firmware;
  std::filesystem::path rom;
  std::filesystem::path trace;
};

void run(const Paths &paths) {
  const auto &firmware_path = paths.firmware;
  const auto &rom_path = paths.rom;
  const auto trace_path = paths.trace.string();
  const auto firmware = Firmware::load(firmware_path);
  const auto rom = read_spi_boot_prefix(rom_path);
  const auto original = McuFlashImage::from_known(firmware.mcu);
  McuMachine mcu(
      original,
      BLOFELD_DEFAULT_SLA,
      McuRunContract{.clock = {.oscillator_hz = source::oscillator_hz,
                               .initial_oscillator_phase_ticks = 0,
                               .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                     .memory_timing = McuRunMemory::internal_flash_reads});
  source::Progress progress;
  std::ostringstream trace;
  std::ofstream summary(trace_path);
  summary.exceptions(std::ios::failbit | std::ios::badbit);
  Lab lab(mcu, progress, trace);
  prefix(lab);
  // Earlier DSP power-on execution is outside this conditional source fixture.
  DspMachine dsp(
      DspRunContract{.clock = {.extal_hz = source::oscillator_hz,
                               .timebase_hz = common,
                               .switch_timing = DspClockSwitch::reference_instruction_boundary},
                     .timing = DspRunTiming::native_internal_estimates,
                     .resets = DspResetTiming::reference_extal_hold});
  dsp.bus().install_rom({.start = coupled::bootstrap_base, .words = rom});
  dsp.begin_hardware_reset({.mode_known = all_modes,
                            .mode_levels = spi_mode,
                            .pinit = false,
                            .recovery_cycles = recovery_periods});
  const auto tail = source::common_ticks(mcu) - lab.assert_tick();
  if (tail != 0) {
    require(dsp.advance_hardware_reset_ticks(tail) == tail, "Exact native trailing slot time");
  }
  Link link(lab, dsp);
  lab.bind_u();
  link.attach();
  for (unsigned count = 0; count < limit && mcu.cpu().state().pc != release_pc; ++count) {
    link.step();
  }
  require(mcu.cpu().state().pc == release_pc && !link.counts().released &&
              dsp.hardware_reset_active() && dsp.run_clock().native_cycle_units() == 0 &&
              dsp.cpu().state().sr == reset_status,
          "Unchanged MCU delays execute with DSP held");
  const auto reset = dsp.run_clock().hardware_reset_state();
  if (!reset) { throw Fault("Expected the actual held native reset phase"); }
  summary << "HELD_CUT retired=" << progress.census.retired
          << " cycles=" << mcu.run_clock().bus_cycles()
          << " ticks=" << mcu.run_clock().elapsed_ticks() << " common=" << source::common_ticks(mcu)
          << " assert=" << lab.assert_tick() << " tail=" << tail
          << " held=" << dsp.run_clock().elapsed_ticks() << " phase=" << reset->phase_ticks << '\n';
  write_text(std::string(trace_path) + ".prefix.trace", trace.str());
  progress.census.write(std::string(trace_path) + ".held.mcu.jsonl");
  progress.phases.write(std::string(trace_path) + ".held.phases.json");
  exercise(lab, dsp, link, firmware, trace, trace_path, summary);
}

}

int main(int count, char **args) {
  try {
    if (count != 4) { throw blofeld::Fault("firmware.zip private-rom private.trace"); }
    run({.firmware = args[1], .rom = args[2], .trace = args[3]});
    std::cout << "Original reset-net source, native SPI, strict faults and held replay passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
