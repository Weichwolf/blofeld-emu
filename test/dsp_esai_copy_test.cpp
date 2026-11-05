// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_gpio.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {
constexpr unsigned sample_word = 0x123456;
constexpr unsigned data_pad = 0x800;
constexpr unsigned mux_pads = 0x818;
constexpr unsigned control = 21;
constexpr unsigned clock_control = 22;
constexpr unsigned normal24 = 0xf800;
constexpr unsigned word32 = 0x7d00;
constexpr unsigned generated_clock = 0x600301;
constexpr unsigned word_bits = 24;
unsigned assertions = 0;

void require(const bool condition, const std::string &message) {
  ++assertions;
  if (!condition) { throw blofeld::Fault(message); }
}

std::string refuses(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected an ESAI observation failure");
}

void test_standalone_observation_atomicity(const blofeld::DspEsaiInstance instance) {
  blofeld::DspEsai serial(instance);
  constexpr auto timing = blofeld::DspEsaiClockTiming::reference_reset_edges;
  serial.observe({.high_clock = false, .timing = timing});
  serial.write(clock_control, generated_clock);
  serial.write(control, word32);
  serial.set_active(true);
  serial.write(0, sample_word);
  serial.write(control, word32 | 1U);
  static_cast<void>(refuses([&] { serial.observe({.high_clock = true}); }));
  serial.observe({.high_clock = true, .timing = timing});
  serial.observe({.high_clock = false, .timing = timing});
  require(serial.outputs().driven == 0 && serial.frames_started() == 0,
          "Rejected standalone observation must not consume a selected divider edge");
  serial.observe({.high_clock = true, .timing = timing});
  serial.observe({.high_clock = false, .timing = timing});
  const auto output = serial.outputs();
  require(output.driven == 1 && output.known == 1 && output.levels == 0 &&
              serial.frames_started() == 1 && serial.slots_started() == 1,
          "Two accepted selected edges launch exactly the literal first zero bit");
}

void test_bus_qualification_atomicity(const blofeld::DspEsaiInstance instance) {
  const bool primary = instance == blofeld::DspEsaiInstance::primary;
  const auto space = primary ? blofeld::DspSpace::x : blofeld::DspSpace::y;
  constexpr unsigned primary_base = 0xffffa0;
  constexpr unsigned secondary_base = 0xffff80;
  const auto base = primary ? primary_base : secondary_base;
  const auto port = primary ? blofeld::DspPort::c : blofeld::DspPort::e;
  blofeld::DspBus bus;
  bus.write({.space = space, .address = base + clock_control, .value = 0});
  bus.write({.space = space, .address = base, .value = sample_word});
  bus.write({.space = space, .address = base + control, .value = normal24 | 1U});
  auto &gpio = bus.gpio(port);
  gpio.write({.reg = blofeld::DspGpioRegister::control, .value = mux_pads});
  gpio.write({.reg = blofeld::DspGpioRegister::direction, .value = mux_pads});
  bus.observe_esai_transmit(instance, {.clock = true, .frame_sync = false});
  gpio.set_drive({.mask = data_pad, .levels = data_pad});
  bus.observe_esai_transmit(instance, {.clock = false, .frame_sync = true});
  const auto failure =
      refuses([&] { bus.observe_esai_transmit(instance, {.clock = true, .frame_sync = true}); });
  require(failure.contains("contention") && failure.contains(primary ? "X:0xffffbd" : "Y:0xffff9d"),
          "Real opposed GPIO drive rejects publication with the correct port context");
  gpio.set_drive({});
  require(bus.esai_transmit_outputs(instance).driven == 0,
          "After failed output qualification the original serial state remains uncommitted");
  std::string actual;
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    if (bit != 0) { bus.observe_esai_transmit(instance, {.clock = false, .frame_sync = false}); }
    bus.observe_esai_transmit(instance, {.clock = true, .frame_sync = bit == 0});
    const auto output = bus.esai_transmit_outputs(instance);
    require(output.driven == 1 && output.known == 1,
            "Retried real bus launch preserves one selected known data lane");
    actual.push_back((output.levels & 1U) != 0 ? '1' : '0');
  }
  require(actual == "000100100011010001010110",
          "Failure followed by retry emits the complete independently specified24-bit payload");
}
}

int main() {
  try {
    for (const auto instance :
         {blofeld::DspEsaiInstance::primary, blofeld::DspEsaiInstance::secondary}) {
      test_standalone_observation_atomicity(instance);
      test_bus_qualification_atomicity(instance);
    }
    std::cout << "ESAI transactional copies: " << assertions << " assertions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
