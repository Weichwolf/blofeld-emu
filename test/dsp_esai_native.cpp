// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::esai {
namespace {
constexpr unsigned primary_base = 0xffffa0;
constexpr unsigned secondary_base = 0xffff80;
constexpr unsigned mux = 0x858; // PC/PE11=TX0, PC/PE6=TX5, PC/PE3=SCKT, PC/PE4=FST.
constexpr unsigned channel5 = 0x89abcd;
constexpr unsigned next5 = 0xfedcba;
constexpr unsigned two_lanes = 0x21;
constexpr unsigned before_capture = 7;
constexpr unsigned port_c_control = 0xffffbf;
constexpr unsigned port_c_direction = 0xffffbe;

void put(DspProgram &program,
         const std::string &space,
         const unsigned address,
         const unsigned value) {
  program.emit("movep #>" + number(value) + ',' + space + ":<<" + number(address));
}

void get(DspProgram &program, const std::string &space, const unsigned address, const char *reg) {
  program.emit("movep " + space + ":<<" + number(address) + ',' + reg);
}

DspProgram diagnostic(const bool secondary) {
  const std::string space = secondary ? "y" : "x";
  const auto base = secondary ? secondary_base : primary_base;
  constexpr unsigned port_control = 31;
  constexpr unsigned port_direction = 30;
  DspProgram result;
  put(result, space, base + clock, 0);
  put(result, space, base, other);
  put(result, space, base + 5, channel5);
  put(result, space, base + control, normal24 | two_lanes);
  put(result, space, base + port_control, mux);
  put(result, space, base + port_direction, mux);
  get(result, space, base + status, "x0");
  put(result, space, base, input);
  put(result, space, base + 5, next5);
  get(result, space, base + status, "x1");
  result.emit("nop");
  return result;
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
  DspMachine machine;
  program.install(machine.bus());
  for (unsigned i = 0; i < 6; ++i) { machine.cpu().step(); }
  constexpr unsigned setup_units = 12;
  require(machine.cpu().debug_state().native_cycles == setup_units,
          "Six native immediate MOVEP cost12 units");
  machine.bus().observe_esai_transmit(which, {.clock = true, .frame_sync = false});
  const auto prefix = sample_bus(machine.bus(), which, 0, before_capture, true);
  require(machine.cpu().debug_state().native_cycles == setup_units,
          "External bit clocks do not invent CPU cycles");
  machine.cpu().step();
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    for (unsigned step = 0; step < 3; ++step) { machine.cpu().step(); }
    const auto tail = sample_bus(machine.bus(), which, before_capture, word_bits, false);
    require(prefix[0] + tail[0] == "000100100011010001010110" &&
                prefix[5] + tail[5] == "100010011010101111001101",
            "Actual native TX writes preserve both partial24-bit shifts across restore");
    const auto next = sample_bus(machine.bus(), which, 0, word_bits, true);
    require(next[0] == "101001011100001110011111" && next[5] == "111111101101110010111010",
            "Mid-word native writes affect next-frame buffers, not the current shift");
    constexpr std::uint64_t expected_x = (std::uint64_t{0x2000} << word_bits) | empty_even_frame;
    constexpr unsigned completed_units = 18;
    require(machine.cpu().state().x == expected_x &&
                machine.cpu().debug_state().native_cycles == completed_units,
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

void native_missing_clock_fault() {
  const auto program = diagnostic(false);
  DspMachine machine;
  program.install(machine.bus());
  for (unsigned i = 0; i < 6; ++i) { machine.cpu().step(); }
  const auto saved = machine.checkpoint();
  const auto error = rejects([&] { machine.cpu().step(); });
  require(error.contains("X:0xffffb3") && error.contains("0xff000c") &&
              error.contains("clock/frame"),
          "Native required status read fails with actual PC/address when clocks are unobserved");
  machine.restore(saved);
  machine.bus().observe_esai_transmit(DspEsaiInstance::primary,
                                      {.clock = true, .frame_sync = false});
  machine.cpu().step();
  require(machine.cpu().state().x == 0,
          "Trusted restore and explicit frozen-clock baseline permit only the real reset status");
}
}

void native_checks() {
  native_snapshot(false);
  native_snapshot(true);
  mux_and_contention();
  native_missing_clock_fault();
}
}
