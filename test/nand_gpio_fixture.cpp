// SPDX-License-Identifier: GPL-3.0-only
#include "nand_gpio_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_timers.hpp"
#include "blofeld/nand_read.hpp"

#include <cstdint>
#include <ostream>
#include <string>

namespace blofeld::test {
NandLink::NandLink(DspBus &bus,
                   NandRead &nand,
                   std::ostream &trace,
                   const NandLinkConditions conditions)
    : bus_(bus), nand_(nand), trace_(trace), conditions_(conditions) {
  // Separate controlled observations for non-NAND F8/F9; no board pulls are inferred.
  bus_.gpio(DspPort::f).set_drive({.mask = nand_other_f});
  bus_.timers().set_tio_drive(nand_timer_one, nand_.ready());
}

void NandLink::latch_write(const unsigned controls) {
  const auto outputs = bus_.gpio(DspPort::f).outputs();
  if ((outputs.mask & outputs.known & nand_data) != nand_data) {
    throw Fault("NAND write latch requires eight known driven F data pins");
  }
  const auto value = static_cast<std::uint8_t>(outputs.levels & nand_data);
  if ((controls & (nand_cle | nand_ale)) == nand_cle) {
    nand_.command(value);
    ++commands_;
    trace_ << "command=" << hex(value, 2) << '\n';
  } else if ((controls & (nand_cle | nand_ale)) == nand_ale) {
    nand_.address(value);
    ++addresses_;
    trace_ << "address=" << hex(value, 2) << '\n';
  } else {
    throw Fault("Unqualified NAND write CLE/ALE combination");
  }
}

void NandLink::observe_edges() {
  const auto controls = bus_.gpio(DspPort::c).observe_pads(nand_controls);
  if (controls.known != nand_controls) {
    if (previous_) { throw Fault("Unknown NAND control/select pins after route establishment"); }
    return;
  }
  if (!conditions_.selected.has_value()) { throw Fault("Unknown NAND chip-select observation"); }
  if (!*conditions_.selected) { throw Fault("NAND device deselected in held-select experiment"); }
  if (previous_) {
    const auto changed = *previous_ ^ controls.levels;
    constexpr unsigned strobes = nand_re | nand_we;
    if ((changed & strobes) == strobes) { throw Fault("Unordered NAND RE/WE transitions"); }
    if ((changed & controls.levels & nand_we) != 0) { latch_write(controls.levels); }
    if ((changed & ~controls.levels & nand_re) != 0) {
      const auto outputs = bus_.gpio(DspPort::f).outputs();
      if ((outputs.mask & nand_data) != 0) {
        throw Fault("Unqualified NAND read turnaround: DSP data pins still driven, mask=" +
                    hex(outputs.mask & nand_data, 2));
      }
      const auto value = nand_.read();
      bus_.gpio(DspPort::f).set_drive({.mask = nand_other_f | nand_data, .levels = value});
      ++reads_;
      trace_ << "read=" << hex(value, 2) << '\n';
    }
    if ((changed & controls.levels & nand_re) != 0) {
      bus_.gpio(DspPort::f).set_drive({.mask = nand_other_f});
    }
  }
  previous_ = controls.levels;
}

void NandLink::after_step(const DspState &before) {
  try {
    observe_edges();
    if (before.pc == conditions_.poll_pc && nand_.ready() == false) {
      ++polls_;
      constexpr unsigned completion_poll = 3;
      if (polls_ == completion_poll) {
        nand_.observe_completion();
        trace_ << "observed-read-completion\n";
      }
    }
    bus_.timers().set_tio_drive(nand_timer_one, nand_.ready());
  } catch (const Fault &fault) {
    throw Fault(std::string(fault.what()) + " dsp-pc=" + hex(before.pc, 6) +
                " omr=" + hex(before.omr, 6) + " NAND-page=" + hex(nand_.page(), 6) +
                " column=" + hex(nand_.column(), 3));
  }
}
}
