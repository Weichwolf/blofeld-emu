// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_esai_registers.hpp"
#include "esai_clock_fixture.hpp"
#include "esai_fixture.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::esai_clock;
using blofeld::test::esai::rejects;
using blofeld::test::esai::require;
constexpr auto reference = DspEsaiConfigurationContract::cold_before_bit_clock_reference;
constexpr unsigned clock_register = 22;
constexpr unsigned control_register = 21;
constexpr unsigned common_register = 20;
constexpr unsigned status_register = 19;
constexpr unsigned synchronized = 0x40;

void partial_divider(const DspEsaiInstance instance, const unsigned options) {
  const bool inverted_high = (options & 1U) != 0;
  const bool inverted_bit = (options & 2U) != 0;
  const auto clock =
      base_clock | (inverted_high ? high_polarity : 0U) | (inverted_bit ? clock_polarity : 0U);
  DspEsai serial(instance);
  serial.set_configuration_contract(reference);
  level(serial, inverted_high);
  serial.set_active(true);
  serial.write(clock_register, clock);
  level(serial, inverted_high);
  level(serial, !inverted_high); // First selected HCKT edge, half of the literal /2 divisor.
  level(serial, !inverted_high);
  serial.write(control_register, word32);
  serial.write(common_register, synchronized);
  require(serial.clock_outputs() == DspEsaiClockOutputs{.driven = 3, .known = 3},
          "Known baseline and one HCKT edge do not fabricate a bit-clock edge");
  auto copy = serial;
  static_cast<void>(rejects([&] { serial.write(clock_register, clock ^ clock_polarity); }));
  require(serial.read(clock_register) == clock,
          "Reprogramming a partially elapsed clock divider remains unqualified and atomic");
  for (auto *target : {&serial, &copy}) {
    level(*target, inverted_high);
    level(*target, !inverted_high);
    const unsigned expected = inverted_bit ? 1U : 3U;
    require(target->clock_outputs() ==
                DspEsaiClockOutputs{.driven = 3, .known = 3, .levels = expected},
            "TCR/common programming and copying preserve the already counted half-divider");
    require(target->read(status_register) == 0 && target->frames_started() == 0 &&
                target->slots_started() == 0 && target->outputs().driven == 0,
            "Disabled transmitters produce no data-empty status, frames or serial payload");
    static_cast<void>(
        rejects([&] { target->write(control_register, word32 | blofeld::esai::shift_lsb); }));
    static_cast<void>(rejects([&] { target->write(common_register, 0); }));
    static_cast<void>(rejects([&] { target->write(clock_register, clock ^ clock_polarity); }));
    target->set_active(false);
    static_cast<void>(rejects([&] { target->set_configuration_contract(reference); }));
  }
}

void literal_frame(const DspEsaiInstance instance) {
  DspEsai serial(instance);
  serial.set_configuration_contract(reference);
  level(serial, false);
  serial.set_active(true);
  serial.write(clock_register, base_clock | clock_polarity);
  serial.write(control_register, word32);
  serial.write(0, blofeld::test::esai::input);
  serial.write(control_register, word32 | 1U);
  static_cast<void>(rejects([&] { serial.write(common_register, synchronized); }));
  std::string actual;
  bool previous = false;
  constexpr unsigned payload_bits = 64;
  constexpr unsigned input_half_edges = 540;
  for (unsigned edge = 0; edge < input_half_edges && actual.size() < payload_bits; ++edge) {
    level(serial, (edge & 1U) != 0);
    const bool now = (serial.clock_outputs().levels & 1U) != 0;
    if (now != previous && now) {
      const auto output = serial.outputs();
      if (output.driven != 0) {
        require(output.driven == 1 && output.known == 1, "Only primed TX0 drives known data");
        actual.push_back(blofeld::test::esai::sample(output, 0));
      }
    }
    previous = now;
  }
  constexpr unsigned empty_underrun = 0x3c000;
  require(
      actual == "1010010111000011100111111111111110100101110000111001111111111111" &&
          serial.frames_started() == 1 && serial.slots_started() == 2 &&
          serial.read(status_register) == empty_underrun,
      "Opposite-edge receiver sees one literal two-slot frame with real retained-word underrun");
}

void external_and_reset(const DspEsaiInstance instance) {
  DspEsai serial(instance);
  serial.set_configuration_contract(reference);
  serial.observe({.clock = false, .frame_sync = false});
  serial.set_active(true);
  serial.write(control_register, blofeld::test::esai::normal24);
  serial.observe({.clock = false, .frame_sync = false});
  serial.write(common_register, synchronized);
  serial.observe({.clock = true, .frame_sync = false});
  static_cast<void>(rejects([&] { serial.write(common_register, 0); }));
  serial.write(control_register, blofeld::test::esai::normal24 | personal_reset);
  serial.write(common_register, 0);
  require(serial.read(common_register) == 0,
          "Actual personal reset still allows documented programming after an external bit edge");
  serial.write(control_register, blofeld::test::esai::normal24);
  static_cast<void>(rejects([&] { serial.write(common_register, synchronized); }));
  serial.reset();
  serial.set_active(true);
  serial.write(control_register, blofeld::test::esai::normal24);
  serial.write(common_register, synchronized);
  require(
      serial.read(common_register) == synchronized,
      "Hardware/software reset retains the selected contract and starts a fresh pre-bit window");
}

void missing_inputs(const DspEsaiInstance instance) {
  DspEsai serial(instance);
  serial.set_configuration_contract(reference);
  serial.set_active(true);
  serial.write(clock_register, base_clock);
  serial.write(control_register, word32);
  serial.write(control_register, word32 | 1U);
  static_cast<void>(rejects([&] { static_cast<void>(serial.outputs()); }));
  static_cast<void>(rejects([&] { static_cast<void>(serial.clock_outputs()); }));
  static_cast<void>(rejects([&] { serial.observe({.high_clock = false}); }));
  static_cast<void>(rejects([&] { serial.observe({.timing = timing}); }));
  require(serial.frames_started() == 0 && serial.slots_started() == 0,
          "The initialization hypothesis never supplies a missing source or edge phase");
}
}

int main() {
  try {
    for (const auto instance : {DspEsaiInstance::primary, DspEsaiInstance::secondary}) {
      for (unsigned options = 0; options < 4; ++options) { partial_divider(instance, options); }
      literal_frame(instance);
      external_and_reset(instance);
      missing_inputs(instance);
    }
    std::cout << "ESAI explicit pre-bit initialization and preserved clock phase passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
