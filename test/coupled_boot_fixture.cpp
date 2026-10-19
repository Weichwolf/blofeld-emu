// SPDX-License-Identifier: GPL-3.0-only
#include "coupled_boot_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"

#include <ostream>
#include <string>

namespace blofeld::test::coupled {
void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

Link::Link(
    McuMachine &mcu, DspBus &bus, DspCore &dsp, std::ostream &trace, const Experiment experiment)
    : mcu_(mcu), bus_(bus), dsp_(dsp), trace_(trace), experiment_(experiment) {}

SpiInputs Link::sample() const {
  const auto pads = mcu_.bus().observe_pads(McuPort::b, serial_pins);
  require(pads.known == serial_pins, "Coupled experiment requires known native MCU serial pads");
  return {.selected = (pads.levels & select_n) == 0,
          .clock = (pads.levels & clock_pin) != 0,
          .mosi = (pads.levels & mosi_pin) != 0};
}

void Link::start() {
  previous_ = sample();
  require(!previous_.selected, "Original MCU initialization must leave SS deasserted");
  bus_.shi().observe_spi(previous_);
  // Explicit laboratory DSP reset release at this native MCU checkpoint.
  require(dsp_.state().pc == bootstrap_base, "DSP starts at its unmodified mode-five reset entry");
  while ((bus_.shi().read(hcsr) & 1U) == 0) {
    constexpr unsigned setup_limit = 100;
    require(dsp_steps_ < setup_limit, "Documented DSP SPI initialization must enable SHI");
    step_dsp();
  }
  require(bus_.once_latch() == 0, "Native documented ROM performs its own OnCE initialization");
}

void Link::step_dsp() {
  constexpr unsigned instruction_limit = 1000000;
  require(dsp_steps_ < instruction_limit, "Coupled DSP instruction budget exhausted");
  const auto before = dsp_.state();
  trace_ << "DSP " << dsp_steps_ << " pc=" << hex(before.pc, 6) << " sr=" << hex(before.sr, 6)
         << " omr=" << hex(before.omr, 6) << " lc=" << hex(before.lc, 6)
         << " r4=" << hex(before.r[4], 6) << '\n';
  dsp_.step();
  ++dsp_steps_;
}

void Link::deliver(SpiInputs pins) {
  const bool falling_capture =
      previous_.selected && pins.selected && previous_.clock && !pins.clock;
  if (experiment_.break_mosi && !injected_ && falling_capture) {
    pins.mosi.reset();
    injected_ = true;
  }
  bus_.shi().observe_spi(pins);
}

void Link::ordered_transition(const SpiInputs pins) {
  ++ordered_pairs_;
  trace_ << "unordered-source-pair " << ordered_pairs_ << '\n';
  switch (experiment_.order) {
    case PadOrder::unresolved: deliver(pins); return;
    case PadOrder::clock_before_select: {
      auto intermediate = pins;
      intermediate.selected = previous_.selected;
      trace_ << "controlled-clock-before-select\n";
      deliver(intermediate);
      break;
    }
    case PadOrder::select_before_clock: {
      auto intermediate = previous_;
      intermediate.selected = pins.selected;
      intermediate.mosi = pins.mosi;
      trace_ << "controlled-select-before-clock\n";
      deliver(intermediate);
      break;
    }
    default: throw Fault("Unknown controlled pad-order experiment");
  }
  deliver(pins);
}

void Link::observe(const BusEvent &event) {
  if (!event.write || event.address != port_b) { return; }
  ++writes_;
  const auto current = sample();
  if (!current.mosi) { throw Fault("Unknown MCU MOSI observation before coupling"); }
  trace_ << "MCU-PORTB " << writes_ << " pc=" << hex(mcu_.cpu().state().pc)
         << " value=" << hex(event.value, 2) << " ss=" << current.selected
         << " sck=" << current.clock << " mosi=" << current.mosi.value() << '\n';
  try {
    if (previous_.selected != current.selected && previous_.clock != current.clock) {
      ordered_transition(current);
    } else {
      deliver(current);
    }
  } catch (const Fault &fault) {
    throw Fault("Coupled PORTB observation mcu-pc=" + hex(mcu_.cpu().state().pc) +
                " dsp-pc=" + hex(dsp_.state().pc, 6) + ": " + fault.what());
  }
  if (previous_.selected && !current.selected) { ++completed_frames_; }
  previous_ = current;
}

void Link::after_mcu_instruction() {
  for (unsigned count = 0; count < experiment_.dsp_steps_per_mcu_instruction; ++count) {
    step_dsp();
  }
}

void Link::verify_transmission() const {
  constexpr unsigned expected_frames = stage_words + 3;
  require(writes_ == expected_port_writes && completed_frames_ == expected_frames &&
              !previous_.selected,
          "Original MCU must supply exactly 7275 stores and 97 completed SPI frames");
  require(ordered_pairs_ == 1 && !injected_,
          "Positive experiment explicitly orders the one source ambiguity and injects no fault");
}
}
