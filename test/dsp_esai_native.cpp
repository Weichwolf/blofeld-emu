// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <string>

namespace blofeld::test::esai {
namespace {
constexpr unsigned primary_base = 0xffffa0;
constexpr unsigned mux = 0x858; // PC/PE11=TX0, PC/PE6=TX5, PC/PE3=SCKT, PC/PE4=FST.
constexpr unsigned channel5 = 0x89abcd;
constexpr unsigned next5 = 0xfedcba;
constexpr unsigned two_lanes = 0x21;
constexpr unsigned before_capture = 7;
constexpr unsigned port_c_control = 0xffffbf;
constexpr unsigned port_c_direction = 0xffffbe;

constexpr DspRunClockInputs native_clocks{.extal_hz = 12000000,
                                          .timebase_hz = 540000000,
                                          .switch_timing =
                                              DspClockSwitch::reference_cycle_boundary};
constexpr std::uint64_t reset_hold = 90, reset_recovery = 90;
constexpr unsigned boot = 0xff0000, effect_limit = 16, diagnostic_words = 19;

std::array<std::uint32_t, diagnostic_words> diagnostic(const bool secondary) {
  // FM MOVEP immediate low-X/low-Y groups and split-qq register-read forms.
  // Primary X:ffffa0 and secondary Y:ffff80; x0/x1 selector fields4/5.
  constexpr unsigned primary_write = 0x07f420;
  constexpr unsigned secondary_write = 0x07b480;
  constexpr unsigned primary_status = 0x0444d3;
  constexpr unsigned secondary_status = 0x044433;
  constexpr unsigned x1_selector = 0x100;
  constexpr unsigned port_control = 31;
  constexpr unsigned port_direction = 30;
  const auto write = secondary ? secondary_write : primary_write;
  const auto read = secondary ? secondary_status : primary_status;
  return {write + clock,
          0,
          write,
          other,
          write + 5,
          channel5,
          write + control,
          normal24 | two_lanes,
          write + port_control,
          mux,
          write + port_direction,
          mux,
          read,
          write,
          input,
          write + 5,
          next5,
          read | x1_selector,
          0};
}

void reset_native(OwnDspMachine &machine) {
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  require(machine.advance_hardware_reset_ticks(reset_hold) == reset_hold,
          "Actual mode5 reset retains its complete EXTAL hold");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(reset_recovery) == reset_recovery,
          "Actual native recovery completes before the first instruction");
}

void step_native(OwnDspMachine &machine) {
  auto progress = machine.begin_fetch();
  for (unsigned effects = 0; effects < effect_limit && progress.pending && !progress.retired &&
                             progress.fault.kind == dsp56300::FaultKind::none;
       ++effects) {
    progress = machine.advance(progress.pending->ticket);
  }
  if (progress.fault.kind != dsp56300::FaultKind::none) {
    std::string cause = "native fault";
    const auto failure = machine.io().failure();
    if (failure && failure->cause) {
      try {
        std::rethrow_exception(failure->cause);
      } catch (const std::exception &error) { cause = error.what(); }
    }
    constexpr std::array spaces{"P:", "X:", "Y:"};
    constexpr std::array operations{"read", "write", "bit", "reset"};
    const std::string operation =
        failure ? operations.at(static_cast<unsigned>(failure->operation)) : "native";
    throw Fault("Independent DSP " + operation + " " +
                std::string(spaces.at(static_cast<unsigned>(progress.fault.space))) +
                hex(progress.fault.address, 6) + " pc=" + hex(progress.fault.pc, 6) +
                " omr=" + hex(machine.cpu().registers().omr, 6) + ": " + cause);
  }
  require(progress.retired, "Authored MOVEP completes its actual bounded native effects");
  require(machine.clock().elapsed_ticks() == reset_hold + reset_recovery,
          "Untimed instruction tests cannot manufacture board-clock periods");
}

DspEsaiOutputs edge(DspBus &bus, const DspEsaiInstance which, const bool sync) {
  bus.observe_esai_transmit(which, {.clock = false, .frame_sync = sync});
  bus.observe_esai_transmit(which, {.clock = true, .frame_sync = sync});
  return bus.esai_transmit_outputs(which);
}

std::array<std::string, 6> sample_bus(DspBus &bus,
                                      const DspEsaiInstance which,
                                      const unsigned begin,
                                      const unsigned end,
                                      const bool frame) {
  std::array<std::string, 6> data;
  for (unsigned i = begin; i < end; ++i) {
    const auto output = edge(bus, which, frame && i == 0);
    for (unsigned lane = 0; lane < data.size(); ++lane) {
      data.at(lane).push_back(sample(output, lane));
    }
  }
  return data;
}

void native_snapshot(const bool secondary) {
  const auto which = secondary ? DspEsaiInstance::secondary : DspEsaiInstance::primary;
  const auto program = diagnostic(secondary);
  OwnDspMachine machine(native_clocks);
  machine.bus().install_rom({.start = boot, .words = program});
  reset_native(machine);
  for (unsigned i = 0; i < 6; ++i) { step_native(machine); }
  constexpr unsigned setup_units = 12;
  require(machine.cpu().progress().estimated_cycles == setup_units &&
              machine.cpu().progress().instructions == 6,
          "Six native immediate MOVEP retain their independent12 base-cost units");
  machine.bus().observe_esai_transmit(which, {.clock = true, .frame_sync = false});
  const auto prefix = sample_bus(machine.bus(), which, 0, before_capture, true);
  require(machine.cpu().progress().estimated_cycles == setup_units,
          "External bit clocks do not invent CPU cycles");
  step_native(machine);
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    for (unsigned step = 0; step < 3; ++step) { step_native(machine); }
    const auto tail = sample_bus(machine.bus(), which, before_capture, word_bits, false);
    require(prefix[0] + tail[0] == "000100100011010001010110" &&
                prefix[5] + tail[5] == "100010011010101111001101",
            "Actual native TX writes preserve both partial24-bit shifts across restore");
    const auto next = sample_bus(machine.bus(), which, 0, word_bits, true);
    require(next[0] == "101001011100001110011111" && next[5] == "111111101101110010111010",
            "Mid-word native writes affect next-frame buffers, not the current shift");
    constexpr std::uint64_t expected_x = (std::uint64_t{0x2000} << word_bits) | empty_even_frame;
    constexpr unsigned completed_units = 18;
    require(machine.cpu().registers().x == expected_x &&
                machine.cpu().progress().estimated_cycles == completed_units &&
                machine.cpu().progress().instructions == 10,
            "Native MOVEP reads observe exact independent all-channel TDE transitions and counts");
    require(next[1] == std::string(word_bits, 'z'), "Unselected pads never become audio outputs");
  }
}

void mux_and_contention() {
  DspBus bus;
  bus.write({.space = DspSpace::x, .address = primary_base, .value = other});
  bus.write({.space = DspSpace::x, .address = primary_base + control, .value = normal24 | 1U});
  constexpr unsigned only_data = 0x800;
  bus.write({.space = DspSpace::x, .address = port_c_control, .value = only_data});
  bus.write({.space = DspSpace::x, .address = port_c_direction, .value = only_data});
  static_cast<void>(rejects([&] {
    bus.observe_esai_transmit(DspEsaiInstance::primary, {.clock = true, .frame_sync = false});
  }));
  bus.write({.space = DspSpace::x, .address = port_c_control, .value = mux});
  bus.write({.space = DspSpace::x, .address = port_c_direction, .value = mux});
  bus.observe_esai_transmit(DspEsaiInstance::primary, {.clock = true, .frame_sync = false});
  bus.gpio(DspPort::c).set_drive({.mask = only_data, .levels = only_data});
  const auto error = rejects([&] { static_cast<void>(edge(bus, DspEsaiInstance::primary, true)); });
  require(error.contains("contention") && error.contains("X:0xffffbd"),
          "Actual peripheral mux and opposed external drive reject a required output");
  bus.gpio(DspPort::c).set_drive({});
  bus.observe_esai_transmit(DspEsaiInstance::primary, {.clock = true, .frame_sync = true});
  require(bus.esai_transmit_outputs(DspEsaiInstance::primary).known == 1,
          "Rejected launch did not consume the pending first bit");
}

void native_missing_clock_fault(const bool secondary) {
  const auto program = diagnostic(secondary);
  OwnDspMachine machine(native_clocks);
  machine.bus().install_rom({.start = boot, .words = program});
  reset_native(machine);
  for (unsigned i = 0; i < 6; ++i) { step_native(machine); }
  const auto saved = machine.checkpoint();
  const auto error = rejects([&] { step_native(machine); });
  require(error.contains(secondary ? "Y:0xffff93" : "X:0xffffb3") && error.contains("0xff000c") &&
              error.contains("clock/frame"),
          "Native required status read fails with actual PC/address when clocks are unobserved");
  const auto failed_cpu = machine.cpu().registers();
  const auto failed_progress = machine.cpu().progress();
  static_cast<void>(rejects([&] { step_native(machine); }));
  require(machine.io().failure() && machine.cpu().registers() == failed_cpu &&
              machine.cpu().progress() == failed_progress,
          "Latched native I/O failure cannot consume another instruction");
  machine.restore(saved);
  require(!machine.io().failure(), "Trusted restore clears only the captured I/O failure state");
  machine.bus().observe_esai_transmit(secondary ? DspEsaiInstance::secondary
                                                : DspEsaiInstance::primary,
                                      {.clock = true, .frame_sync = false});
  step_native(machine);
  require(machine.cpu().registers().x == 0,
          "Trusted restore and explicit frozen-clock baseline permit only the real reset status");
}
}

void native_checks() {
  native_snapshot(false);
  native_snapshot(true);
  mux_and_contention();
  native_missing_clock_fault(false);
  native_missing_clock_fault(true);
}
}
