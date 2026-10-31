// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_read_setup.hpp"
#include "storage/nand_read_timing.hpp"
#include "storage/nand_write_timing.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned ghz = 1000000000;
constexpr unsigned clearance = 100;
constexpr unsigned ce_release = 20;
constexpr unsigned data_mask = 0xff;
constexpr NandByte sample{.value = 0xa5, .known = data_mask};
unsigned assertions = 0;

void require(const bool value, const std::string_view why) {
  ++assertions;
  if (!value) { throw Fault(std::string(why)); }
}

void rejects(const auto &action, const std::string_view expected) {
  try {
    action();
  } catch (const Fault &error) {
    require(std::string_view(error.what()).contains(expected), error.what());
    return;
  }
  throw Fault("Missing expected refusal: " + std::string(expected));
}

NandWritePins write_pins(const bool ce = true) {
  return {.ce_n = ce, .re_n = true, .we_n = true, .cle = false, .ale = false};
}

NandReadSetupPins setup_pins(const bool ce = true) {
  return {.ce_n = ce, .re_n = true, .we_n = true, .ale = false, .cle = false, .ready = true};
}

void ignored_pins() {
  for (const bool first_re : {false, true}) {
    for (const bool first_we : {false, true}) {
      NandWriteTiming write(ghz);
      NandReadSetup setup(ghz);
      NandReadTiming read(ghz);
      unsigned sources = 0;
      auto wp = write_pins();
      auto sp = setup_pins();
      for (unsigned step = 0; step < 64; ++step) {
        const bool re = (step % 2 != 0) != first_re;
        const bool we = (step % 2 != 0) != first_we;
        wp.re_n = re;
        wp.we_n = we;
        wp.cle = step % 3 != 0;
        wp.ale = step % 3 != 0;
        sp.re_n = re;
        sp.we_n = we;
        sp.cle = wp.cle;
        sp.ale = wp.ale;
        require(!write.observe(step, wp), "Standby never produces an input latch");
        require(!setup.observe(step, sp), "Standby never admits a read");
        read.observe(step, {.ce_n = true, .re_n = re}, [&] {
          ++sources;
          return sample;
        });
        require(sources == 0, "Ignored clocks never consume protocol bytes");
        require(read.output(step) ==
                    (step < ce_release ? NandReadOutput{} : NandReadOutput{.driven = false}),
                "Initial unknown drive expires only at the independent CE-release bound");
      }
    }
  }
}

void selected_write_after_standby() {
  constexpr unsigned ignored_rise = 101;
  constexpr unsigned selection = 300;
  constexpr unsigned data_setup = 400;
  constexpr unsigned selected_fall = 500;
  constexpr unsigned early_rise = 520;
  constexpr unsigned valid_rise = 521;
  NandWriteTiming prefix(ghz);
  auto pins = write_pins();
  require(!prefix.observe(0, pins), "Known initial standby");
  pins.we_n = false;
  pins.re_n = false;
  pins.cle = true;
  pins.ale = true;
  require(!prefix.observe(clearance, pins), "Both low strobes remain ignored while deselected");
  const auto before_rise = prefix;
  for (unsigned replay = 0; replay < 3; ++replay) {
    auto model = before_rise;
    auto value = pins;
    value.we_n = true;
    value.re_n = true;
    value.cle = false;
    value.ale = false;
    require(!model.observe(ignored_rise, value), "One-tick ignored low pulse has no latch");
    value.ce_n = false;
    require(!model.observe(selection, value), "Clean selection changes no input latch");
    value.cle = true;
    value.data = {.value = data_mask, .known = data_mask};
    require(!model.observe(data_setup, value), "Literal command/data setup");
    value.we_n = false;
    require(!model.observe(selected_fall, value), "A genuine selected pulse begins");
    const auto pending = model;
    value.we_n = true;
    rejects([&] { static_cast<void>(model.observe(early_rise, value)); }, "tWP");
    require(model == pending, "Premature selected latch leaves every timing field unchanged");
    require(model.observe(valid_rise, value) ==
                NandWriteLatch{.kind = NandWriteKind::command, .value = data_mask},
            "Exactly one genuine FF command latches at the independent21ns boundary");
  }
}

void selected_setup_after_standby() {
  constexpr unsigned ignored_rise = 101;
  constexpr unsigned selection = 120;
  constexpr unsigned early_read = 160;
  constexpr unsigned valid_read = 161;
  NandReadSetup setup(ghz);
  auto pins = setup_pins();
  require(!setup.observe(0, pins), "Initial ready standby");
  pins.we_n = false;
  pins.re_n = false;
  pins.cle = true;
  pins.ale = true;
  require(!setup.observe(clearance, pins), "Deselected simultaneous pin changes have no read");
  pins.we_n = true;
  pins.re_n = true;
  pins.cle = false;
  pins.ale = false;
  require(!setup.observe(ignored_rise, pins),
          "Actual setup anchors follow ignored physical pin changes");
  pins.ce_n = false;
  require(!setup.observe(selection, pins), "Clean selection has no falling RE");
  const auto selected = setup;
  pins.re_n = false;
  rejects([&] { static_cast<void>(setup.observe(early_read, pins)); }, "tWHR");
  require(setup == selected, "Invalid selected setup retains the complete preceding state");
  require(setup.observe(valid_read, pins),
          "Exactly60ns of actual WE-high setup permits the selected read");
}

void selected_read_after_standby() {
  constexpr unsigned selection = 20;
  constexpr unsigned early_fall = 41;
  constexpr unsigned valid_fall = 42;
  constexpr unsigned early_access = 71;
  constexpr unsigned valid_access = 72;
  NandReadTiming prefix(ghz);
  unsigned calls = 0;
  const auto source = [&] {
    ++calls;
    return sample;
  };
  prefix.observe(0, {.ce_n = true, .re_n = true}, source);
  prefix.observe(10, {.ce_n = true, .re_n = false}, source);
  prefix.observe(11, {.ce_n = true, .re_n = true}, source);
  prefix.observe(selection, {.ce_n = false, .re_n = true}, source);
  const auto selected = prefix;
  for (unsigned replay = 0; replay < 3; ++replay) {
    auto read = selected;
    calls = 0;
    rejects([&] { read.observe(early_fall, {.ce_n = false, .re_n = false}, source); }, "tRC");
    require(calls == 0 && read == selected,
            "Prior bounded history still guards the first active cycle");
    read.observe(valid_fall, {.ce_n = false, .re_n = false}, source);
    require(calls == 1 && read.output(early_access) == NandReadOutput{} &&
                read.output(valid_access) == NandReadOutput{.driven = true, .data = sample},
            "Ignored pulses neither move the active cycle anchor nor fabricate data before tREA");
  }
}

void retained_release_and_pulse() {
  constexpr unsigned deselection = 131;
  constexpr unsigned inactive_rise = 132;
  constexpr unsigned ignored_fall = 133;
  constexpr unsigned ignored_rise = 134;
  constexpr unsigned last_held = 145;
  constexpr unsigned first_unknown = 146;
  constexpr unsigned last_unknown = 150;
  constexpr unsigned first_floating = 151;
  constexpr unsigned early_deselection = 105;
  constexpr unsigned early_rise = 120;
  constexpr unsigned valid_rise = 121;
  constexpr unsigned next_ignored_fall = 122;
  constexpr unsigned next_ignored_rise = 123;
  NandReadTiming read(ghz);
  unsigned calls = 0;
  const auto source = [&] {
    ++calls;
    return sample;
  };
  read.observe(0, {.ce_n = false, .re_n = true});
  read.observe(clearance, {.ce_n = false, .re_n = false}, source);
  read.observe(deselection, {.ce_n = true, .re_n = false}, source);
  read.observe(inactive_rise, {.ce_n = true, .re_n = true}, source);
  read.observe(ignored_fall, {.ce_n = true, .re_n = false}, source);
  read.observe(ignored_rise, {.ce_n = true, .re_n = true}, source);
  require(calls == 1 && read.output(last_held) == NandReadOutput{.driven = true, .data = sample} &&
              read.output(first_unknown) == NandReadOutput{} &&
              read.output(last_unknown) == NandReadOutput{} &&
              read.output(first_floating) == NandReadOutput{.driven = false},
          "Ignored activity preserves the actual CE-release hold/Hi-Z bounds");
  NandReadTiming early(ghz);
  early.observe(0, {.ce_n = false, .re_n = true});
  early.observe(clearance, {.ce_n = false, .re_n = false}, source);
  early.observe(early_deselection, {.ce_n = true, .re_n = false}, source);
  const auto pending = early;
  rejects([&] { early.observe(early_rise, {.ce_n = true, .re_n = true}, source); }, "tRP");
  require(early == pending, "Deselection cannot hide a too-short already selected RE pulse");
  early.observe(valid_rise, {.ce_n = true, .re_n = true}, source);
  early.observe(next_ignored_fall, {.ce_n = true, .re_n = false}, source);
  early.observe(next_ignored_rise, {.ce_n = true, .re_n = true}, source);
  require(calls == 2, "After the selected pulse closes, later unselected pulses stay ignored");
}

void selected_write_history() {
  constexpr unsigned first_latch = 121;
  constexpr unsigned deselection = 126;
  constexpr unsigned ignored_rise = 127;
  constexpr unsigned selection = 130;
  constexpr unsigned early_fall = 141;
  constexpr unsigned valid_fall = 142;
  constexpr unsigned second_latch = 163;
  for (const bool ignored_we : {false, true}) {
    NandWriteTiming model(ghz);
    auto pins = write_pins(false);
    pins.cle = true;
    pins.data = sample;
    require(!model.observe(0, pins), "Selected known command setup");
    pins.we_n = false;
    require(!model.observe(clearance, pins), "Actual selected falling WE");
    pins.we_n = true;
    require(model.observe(first_latch, pins) ==
                NandWriteLatch{.kind = NandWriteKind::command, .value = sample.value},
            "First selected latch at21ns");
    pins.ce_n = true;
    require(!model.observe(deselection, pins), "Deselect only after the exact5ns CE hold");
    if (ignored_we) {
      pins.we_n = false;
    } else {
      pins.re_n = false;
    }
    require(!model.observe(deselection, pins), "Ordered ignored fall after known deselection");
    pins.we_n = true;
    pins.re_n = true;
    require(!model.observe(ignored_rise, pins), "Ignored one-tick pulse");
    pins.ce_n = false;
    require(!model.observe(selection, pins), "Clean reselect");
    const auto selected = model;
    pins.we_n = false;
    rejects([&] { static_cast<void>(model.observe(early_fall, pins)); },
            ignored_we ? "tWH" : "tWC");
    require(model == selected, "Standby cannot erase selected cycle or actual high setup");
    require(!model.observe(valid_fall, pins),
            "Ignored WE fall cannot become a selected-cycle anchor");
    pins.we_n = true;
    require(model.observe(second_latch, pins) ==
                NandWriteLatch{.kind = NandWriteKind::command, .value = sample.value},
            "The second selected latch retains independent setup,pulse and cycle boundaries");
  }
}

void deselected_pending_pulses() {
  constexpr unsigned deselection = 101;
  constexpr unsigned ignored_rise = 102;
  constexpr std::array<std::array<unsigned, 2>, 3> rows{
      {{1000000000, 21}, {1080000000, 23}, {216000000, 5}}};
  for (const auto &row : rows) {
    NandWriteTiming write(row[0]);
    auto pins = write_pins(false);
    pins.cle = true;
    pins.data = sample;
    require(!write.observe(0, pins), "Known selected command seed");
    pins.we_n = false;
    require(!write.observe(clearance, pins), "Selected WE pulse begins");
    pins.ce_n = true;
    require(!write.observe(deselection, pins), "CE release does not fabricate a latch");
    const auto low = write;
    pins.we_n = true;
    rejects([&] { static_cast<void>(write.observe(clearance + row[1] - 1, pins)); }, "tWP");
    require(write == low, "A selected WE pulse retains its minimum after deselection");
    require(!write.observe(clearance + row[1], pins), "The deselected rising WE creates no latch");
    pins.we_n = false;
    require(!write.observe(deselection + row[1], pins), "A wholly ignored low pulse begins");
    pins.we_n = true;
    require(!write.observe(ignored_rise + row[1], pins),
            "A subsequent wholly ignored short pulse is legal");
    NandReadTiming read(row[0]);
    unsigned calls = 0;
    const auto source = [&] {
      ++calls;
      return sample;
    };
    read.observe(0, {.ce_n = false, .re_n = true});
    read.observe(clearance, {.ce_n = false, .re_n = false}, source);
    read.observe(deselection, {.ce_n = true, .re_n = false}, source);
    const auto active = read;
    rejects([&] { read.observe(clearance + row[1] - 1, {.ce_n = true, .re_n = true}, source); },
            "tRP");
    require(read == active && calls == 1, "Selected RE width survives CE release without replay");
    read.observe(clearance + row[1], {.ce_n = true, .re_n = true}, source);
    read.observe(deselection + row[1], {.ce_n = true, .re_n = false}, source);
    read.observe(ignored_rise + row[1], {.ce_n = true, .re_n = true}, source);
    require(calls == 1, "No byte follows the completed selected pulse while CE remains high");
  }
}

void selected_guards() {
  NandWriteTiming write(ghz);
  auto wp = write_pins(false);
  require(!write.observe(0, wp), "Initial selected write timing");
  const auto saved_write = write;
  wp.re_n = false;
  wp.we_n = false;
  rejects([&] { static_cast<void>(write.observe(clearance, wp)); }, "unordered");
  require(write == saved_write, "Selected simultaneous edges stay rejected transactionally");
  NandReadSetup setup(ghz);
  auto sp = setup_pins(false);
  require(!setup.observe(0, sp), "Initial selected read setup");
  const auto saved_setup = setup;
  sp.re_n = false;
  sp.we_n = false;
  rejects([&] { static_cast<void>(setup.observe(clearance, sp)); }, "unordered");
  require(setup == saved_setup, "Standby permission cannot authorize selected read/write overlap");
  NandReadTiming read(ghz);
  read.observe(0, {.ce_n = false, .re_n = true});
  const auto saved_read = read;
  unsigned calls = 0;
  rejects(
      [&] {
        read.observe(clearance, {.ce_n = true, .re_n = false}, [&] {
          ++calls;
          return sample;
        });
      },
      "unordered");
  require(read == saved_read && calls == 0, "Unknown CE/RE ordering cannot consume a source");
  NandWriteTiming standby(ghz);
  wp = write_pins();
  require(!standby.observe(0, wp), "Initial standby");
  const auto known = standby;
  wp.re_n.reset();
  rejects([&] { static_cast<void>(standby.observe(clearance, wp)); }, "unknown control");
  require(standby == known, "A known CE cannot invent an unknown control level");
}

void retained_high_setup() {
  constexpr unsigned ignored_rise = 101;
  constexpr unsigned selection = 110;
  constexpr unsigned early_fall = 115;
  constexpr unsigned valid_fall = 116;
  constexpr unsigned early_access = 145;
  constexpr unsigned valid_access = 146;
  NandReadTiming model(ghz);
  unsigned calls = 0;
  const auto source = [&] {
    ++calls;
    return sample;
  };
  model.observe(0, {.ce_n = true, .re_n = true});
  model.observe(clearance, {.ce_n = true, .re_n = false});
  model.observe(ignored_rise, {.ce_n = true, .re_n = true});
  model.observe(selection, {.ce_n = false, .re_n = true});
  const auto before = model;
  rejects([&] { model.observe(early_fall, {.ce_n = false, .re_n = false}, source); }, "tREH");
  require(model == before && calls == 0, "Ignored activity preserves the actual input-high setup");
  model.observe(valid_fall, {.ce_n = false, .re_n = false}, source);
  require(calls == 1 && model.output(early_access) == NandReadOutput{} &&
              model.output(valid_access) == NandReadOutput{.driven = true, .data = sample},
          "Exact15ns high interval and independent CE/RE access maxima qualify fresh data");
}
}

int main() {
  try {
    ignored_pins();
    selected_write_after_standby();
    selected_setup_after_standby();
    selected_read_after_standby();
    retained_release_and_pulse();
    retained_high_setup();
    selected_write_history();
    deselected_pending_pulses();
    selected_guards();
    std::cout
        << "Standby ignored pins,selected boundaries,release,copies and source counts;assertions="
        << assertions << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
