// SPDX-License-Identifier: GPL-3.0-only
#include "nand_gpio_fixture.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_timers.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_store.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

namespace blofeld::test {
NandLink::NandLink(DspBus &bus, Nand &nand, std::ostream &trace, NandLinkConditions conditions)
    : NandLink(bus, Device{&nand}, trace, std::move(conditions)) {}

NandLink::NandLink(DspBus &bus, NandStore &nand, std::ostream &trace, NandLinkConditions conditions)
    : NandLink(bus, Device{&nand}, trace, std::move(conditions)) {}

NandLink::NandLink(DspBus &bus, Device device, std::ostream &trace, NandLinkConditions conditions)
    : bus_(bus), device_(device), trace_(trace), conditions_(std::move(conditions)) {
  // Separate controlled observations for non-NAND F8/F9; no board pulls are inferred.
  bus_.gpio(DspPort::f).set_drive({.mask = nand_other_f});
  bus_.timers().set_tio_drive(nand_timer_one, with([](auto &nand) { return nand.ready(); }));
}

void NandLink::latch_write(const unsigned controls) {
  const auto outputs = bus_.gpio(DspPort::f).outputs();
  if ((outputs.mask & outputs.known & nand_data) != nand_data) {
    throw Fault("NAND write latch requires eight known driven F data pins");
  }
  const auto value = static_cast<std::uint8_t>(outputs.levels & nand_data);
  if ((controls & (nand_cle | nand_ale)) == nand_cle) {
    with([value](auto &nand) { nand.command(value); });
    ++commands_;
    trace_ << "command=" << hex(value, 2) << '\n';
  } else if ((controls & (nand_cle | nand_ale)) == nand_ale) {
    with([value](auto &nand) { nand.address(value); });
    ++addresses_;
    trace_ << "address=" << hex(value, 2) << '\n';
  } else if ((controls & (nand_cle | nand_ale)) == 0) {
    with([value](auto &nand) { nand.write(value); });
    ++writes_;
    trace_ << "data=" << hex(value, 2) << '\n';
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
      reading_ = true;
      if (with([](auto &nand) { return nand.status_mode(); })) {
        drive_status();
      } else {
        const auto value = with([](auto &nand) { return nand.read(); });
        bus_.gpio(DspPort::f).set_drive({.mask = nand_other_f | nand_data, .levels = value});
        trace_ << "read=" << hex(value, 2) << '\n';
      }
      ++reads_;
    }
    if ((changed & controls.levels & nand_re) != 0) {
      bus_.gpio(DspPort::f).set_drive({.mask = nand_other_f});
      reading_ = false;
    }
  }
  previous_ = controls.levels;
}

void NandLink::drive_status() {
  if ((bus_.gpio(DspPort::f).outputs().mask & nand_data) != 0) {
    throw Fault("Unqualified NAND status turnaround while data pins are driven");
  }
  const auto sample = with([](auto &nand) { return nand.status(); });
  if (sample.known == 0) { throw Fault("Unknown NAND status output"); }
  bus_.gpio(DspPort::f).set_drive({.mask = nand_other_f | sample.known, .levels = sample.value});
  if (!previous_status_ || *previous_status_ != sample) {
    trace_ << "status=" << hex(sample.value, 2) << " known=" << hex(sample.known, 2) << '\n';
    previous_status_ = sample;
  }
}

void NandLink::complete_operation() {
  const auto pending = with([](auto &nand) { return nand.pending(); });
  if (pending == NandPending::program || pending == NandPending::erase) {
    if (completed_writes_ >= conditions_.outcomes.size()) {
      throw Fault("Unobserved NAND program/erase outcome");
    }
    const auto outcome = conditions_.outcomes.at(completed_writes_);
    with([outcome](auto &nand) { nand.observe_completion(outcome); });
    ++completed_writes_;
    trace_ << "observed-write-completion=" << (outcome == NandOutcome::pass ? "pass" : "fail")
           << '\n';
  } else {
    with([](auto &nand) { nand.observe_completion(); });
    trace_ << "observed-read-completion\n";
  }
}

void NandLink::after_step(const DspState &before) {
  std::optional<unsigned> page;
  std::optional<unsigned> column;
  try {
    page = with([](auto &nand) { return nand.page(); });
    column = with([](auto &nand) { return nand.column(); });
    observe_edges();
    const bool poll =
        std::ranges::find(conditions_.poll_pcs, before.pc) != conditions_.poll_pcs.end();
    if (poll && with([](auto &nand) { return nand.ready(); }) == false) {
      ++polls_;
      ++operation_polls_;
      constexpr unsigned completion_poll = 3;
      if (operation_polls_ == completion_poll) {
        complete_operation();
        operation_polls_ = 0;
      }
    }
    if (reading_ && with([](auto &nand) { return nand.status_mode(); })) { drive_status(); }
    bus_.timers().set_tio_drive(nand_timer_one, with([](auto &nand) { return nand.ready(); }));
  } catch (const Fault &fault) {
    throw Fault(std::string(fault.what()) + " dsp-pc=" + hex(before.pc, 6) +
                " omr=" + hex(before.omr, 6) + " NAND-page=" + (page ? hex(*page, 6) : "unknown") +
                " column=" + (column ? hex(*column, 3) : "unknown"));
  }
}
}
