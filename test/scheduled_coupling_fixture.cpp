// SPDX-License-Identifier: GPL-3.0-only
#include "scheduled_coupling_fixture.hpp"

#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "coupled_boot_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp56kEmu/disasm.h"
#include "dsp56kEmu/opcodes.h"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "original_owned_clock_fixture.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <limits>
#include <ostream>
#include <string>

namespace blofeld::test::scheduled {
void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

std::uint64_t common_ticks(const McuMachine &machine) {
  const auto &clock = machine.run_clock();
  require(common_hz % clock.timebase_hz() == 0, "MCU/common timebase ratio must be exact");
  const auto factor = common_hz / clock.timebase_hz();
  require(clock.elapsed_ticks() <= std::numeric_limits<std::uint64_t>::max() / factor,
          "MCU/common clock conversion must not wrap");
  return clock.elapsed_ticks() * factor;
}

void record_clock_write(const McuMachine &machine, const BusEvent &event, std::ostream &trace) {
  constexpr unsigned register_end = 0x70;
  if (event.write && event.address < register_end) {
    trace << "cycle=" << machine.run_clock().bus_cycles() << " write=" << hex(event.address) << ':'
          << hex(event.value, 2) << '\n';
  }
}

ClockedLink::ClockedLink(McuMachine &mcu,
                         DspMachine &dsp,
                         owned_clock::FlashPhases &phases,
                         std::ostream &trace,
                         const Experiment experiment,
                         const std::uint64_t release_tick)
    : mcu_(mcu),
      dsp_(dsp),
      phases_(phases),
      trace_(trace),
      experiment_(experiment),
      scheduler_({.dsp = {.epoch_ticks = release_tick,
                          .observation = DspStepObservation::reference_completion_boundary},
                  .shared_write_order = experiment.tie,
                  .maximum_dsp_steps_per_cycle = 16},
                 mcu,
                 dsp) {
  require(dsp.run_clock().timebase_hz() == common_hz, "DSP must use the declared common timebase");
  require(experiment.tie == TieOrder::unqualified || experiment.tie == TieOrder::dsp_before_mcu ||
              experiment.tie == TieOrder::mcu_before_dsp,
          "Unknown shared-cycle tie experiment");
}

SpiInputs ClockedLink::sample() const {
  const auto pins = mcu_.bus().observe_pads(McuPort::b, coupled::serial_pins);
  require(pins.known == coupled::serial_pins,
          "Unknown scheduled MCU pad observation at " + hex(coupled::port_b) +
              " known=" + hex(pins.known, 2) + " mask=" + hex(coupled::serial_pins, 2) +
              " mcu-pc=" + hex(mcu_.cpu().state().pc) + " dsp-pc=" + hex(dsp_.cpu().state().pc, 6));
  return {.selected = (pins.levels & coupled::select_n) == 0,
          .clock = (pins.levels & coupled::clock_pin) != 0,
          .mosi = (pins.levels & coupled::mosi_pin) != 0};
}

void ClockedLink::start() {
  previous_ = sample();
  require(!previous_.selected && dsp_.cpu().state().pc == coupled::bootstrap_base &&
              dsp_.run_clock().native_cycle_units() == 0,
          "Scheduled release uses original reset PC and actual deasserted MCU pads");
  dsp_.bus().shi().observe_spi(previous_);
}

void ClockedLink::attach() {
  mcu_.set_cycle_observer([this](const auto cycle) { on_cycle(cycle); });
  mcu_.bus().set_observer([this](const auto &event) { on_bus_event(event); });
}

void ClockedLink::detach() {
  mcu_.set_cycle_observer({});
  mcu_.bus().set_observer({});
}

void ClockedLink::after_mcu_step() {
  scheduler_.end_instruction();
  const auto pins = sample();
  require(pins.selected == previous_.selected && pins.clock == previous_.clock &&
              pins.mosi == previous_.mosi,
          "Unobserved MCU serial-pad change during the completed instruction");
}

void ClockedLink::on_dsp_step(const DspScheduledStep &step) {
  const auto &before = step.plan.before.cpu;
  const auto opcode = dsp_.bus().read({.address = before.pc});
  auto &row = census_[before.pc];
  if (row.count == 0) {
    const dsp56k::Opcodes opcodes;
    dsp56k::Disassembler decoder(opcodes);
    dsp56k::Disassembler::Line line;
    row.extension = dsp_.bus().peek_program(before.pc + 1);
    row.length = decoder.disassemble(line, opcode, row.extension, before.sr, before.omr, before.pc);
    row.name = line.opName;
    row.opcode = opcode;
    row.units = step.plan.native_cycle_units;
    row.pctl = dsp_.bus().pll().read();
  }
  require(row.opcode == opcode && row.units == step.plan.native_cycle_units &&
              row.pctl == dsp_.bus().pll().read(),
          "Original scheduled DSP PC retains its native opcode, nominal cost and clock source");
  ++row.count;
  trace_ << "DSP end=" << step.completion_ticks << " pc=" << hex(before.pc, 6)
         << " start=" << step.plan.start_ticks << " units=" << step.plan.native_cycle_units
         << " ticks=" << step.plan.duration_ticks << " r4=" << before.r[4] << '\n';
}

void ClockedLink::on_cycle(const Hc12BusCycle cycle) {
  const auto pins = sample();
  require(pins.selected == previous_.selected && pins.clock == previous_.clock &&
              pins.mosi == previous_.mosi,
          "Unobserved MCU serial-pad change before the next clock boundary");
  phases_.observe(cycle, mcu_.run_clock().bus_cycles());
  context_ = cycle.context;
  const bool write = cycle.kind == 'w' || cycle.kind == 'W';
  const bool shared = write && cycle.address && *cycle.address <= coupled::port_b &&
                      coupled::port_b - *cycle.address < cycle.size;
  scheduler_.begin_cycle(cycle, shared, [this](const auto &step) { on_dsp_step(step); });
}

void ClockedLink::deliver(SpiInputs pins) {
  const bool capture = previous_.selected && pins.selected && previous_.clock && !pins.clock;
  if (experiment_.break_mosi && !injected_ && capture) {
    pins.mosi.reset();
    injected_ = true;
  }
  dsp_.bus().shi().observe_spi(pins);
}

void ClockedLink::transition(const SpiInputs pins) {
  if (previous_.selected != pins.selected && previous_.clock != pins.clock) {
    ++ordered_pairs_;
    switch (experiment_.pads) {
      case coupled::PadOrder::unresolved: break;
      case coupled::PadOrder::clock_before_select: {
        auto intermediate = pins;
        intermediate.selected = previous_.selected;
        deliver(intermediate);
        break;
      }
      case coupled::PadOrder::select_before_clock: {
        auto intermediate = previous_;
        intermediate.selected = pins.selected;
        intermediate.mosi = pins.mosi;
        deliver(intermediate);
        break;
      }
      default: throw Fault("Unknown scheduled pad-order experiment");
    }
  }
  deliver(pins);
}

void ClockedLink::on_bus_event(const BusEvent &event) {
  if (!event.write) { return; }
  record_clock_write(mcu_, event, trace_);
  const auto pins = sample();
  if (event.address != coupled::port_b) {
    require(pins.selected == previous_.selected && pins.clock == previous_.clock &&
                pins.mosi == previous_.mosi,
            "Unsupported MCU serial-pad control change at write=" + hex(event.address));
    return;
  }
  scheduler_.observe_write(
      event,
      [&] {
        try {
          transition(pins);
        } catch (const Fault &fault) {
          throw Fault("Scheduled MCU write at " + hex(event.address) +
                      " mcu-pc=" + hex(context_.pc) + " dsp-pc=" + hex(dsp_.cpu().state().pc, 6) +
                      ": " + fault.what());
        }
        ++writes_;
        if (previous_.selected && !pins.selected) { ++frames_; }
        previous_ = pins;
        trace_ << "MCU-WIRE end=" << scheduler_.end_ticks() << " pc=" << hex(context_.pc)
               << " value=" << hex(event.value, 2) << " frame=" << frames_ << '\n';
      },
      [this](const auto &step) { on_dsp_step(step); });
}

LinkCheckpoint ClockedLink::checkpoint() const {
  return {.schedule = scheduler_.checkpoint(),
          .previous = previous_,
          .census = census_,
          .end_tick = scheduler_.end_ticks(),
          .writes = writes_,
          .frames = frames_,
          .ordered_pairs = ordered_pairs_,
          .injected = injected_};
}

void ClockedLink::restore(const LinkCheckpoint &checkpoint) {
  const auto pins = sample();
  require(common_ticks(mcu_) == checkpoint.end_tick &&
              pins.selected == checkpoint.previous.selected &&
              pins.clock == checkpoint.previous.clock && pins.mosi == checkpoint.previous.mosi,
          "Scheduled link restore must match restored native time and source pads");
  scheduler_.restore(checkpoint.schedule);
  previous_ = checkpoint.previous;
  census_ = checkpoint.census;
  writes_ = checkpoint.writes;
  frames_ = checkpoint.frames;
  ordered_pairs_ = checkpoint.ordered_pairs;
  injected_ = checkpoint.injected;
}

void ClockedLink::write_census(const std::filesystem::path &path) const {
  std::ofstream file(path);
  file.exceptions(std::ios::failbit | std::ios::badbit);
  for (const auto &[pc, row] : census_) {
    file << "{\"pc\":" << pc << ",\"opcode\":" << row.opcode << ",\"extension\":" << row.extension
         << ",\"length\":" << row.length << ",\"name\":" << std::quoted(row.name)
         << ",\"count\":" << row.count << ",\"units\":" << row.units << ",\"pctl\":" << row.pctl
         << "}\n";
  }
}
}
