// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_esai.hpp"
#include "esai_clock_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>

namespace blofeld::test::esai_clock {
namespace {
using esai::rejects;
using esai::require;

struct Divider {
  unsigned modulus;
  bool bypass;
  unsigned high;
  unsigned half_period;
};

void divider_ratios() {
  // Literal expectations from the fixed /2, /1-or-8, TPM+1 and TFP+1 chain.
  constexpr std::array cases{
      Divider{.modulus = 0, .bypass = false, .high = 0, .half_period = 8},
      Divider{.modulus = 1, .bypass = true, .high = 0, .half_period = 2},
      Divider{.modulus = 255, .bypass = true, .high = 0, .half_period = 256},
      Divider{.modulus = 1, .bypass = true, .high = 15, .half_period = 32},
      Divider{.modulus = 255, .bypass = false, .high = 15, .half_period = 32768},
      Divider{.modulus = 2, .bypass = false, .high = 3, .half_period = 96}};
  for (const auto spec : cases) {
    DspEsai serial(DspEsaiInstance::primary);
    constexpr unsigned high_shift = 14;
    const auto control = output_directions | two_slots | spec.modulus |
                         (spec.bypass ? bypass : 0U) | (spec.high << high_shift);
    configure(serial, control);
    for (unsigned input = 1; input <= 4 * spec.half_period; ++input) {
      level(serial, true);
      level(serial, true); // Repeated observations carry no additional edge.
      level(serial, false);
      const auto pins = serial.clock_outputs();
      require(pins.driven == 3 && pins.known == 3 &&
                  (pins.levels & 1U) == (input / spec.half_period) % 2,
              "Clock toggle phase follows independent literal divider ratios without a CPU");
      require(serial.outputs().driven == 0 && serial.read(esai::status) == 0,
              "Clocks run with TE=0 without inventing transmit-empty requests or samples");
    }
  }
}

void serial_words(const unsigned options) {
  const bool bit_sync = (options & 1U) != 0;
  const bool inverted_clock = (options & 2U) != 0;
  const bool inverted_frame = (options & 4U) != 0;
  const bool inverted_high = (options & 8U) != 0;
  const auto clock = base_clock | (inverted_clock ? clock_polarity : 0U) |
                     (inverted_frame ? frame_polarity : 0U) | (inverted_high ? high_polarity : 0U);
  const auto format = bit_sync ? bit24 : word32;
  DspEsai serial(DspEsaiInstance::secondary);
  configure(serial, clock, format);
  serial.write(0, esai::input);
  serial.write(esai::control, format | 1U);
  std::string actual;
  unsigned launches = 0;
  bool last_clock = false;
  const unsigned width = bit_sync ? 24U : 32U;
  const unsigned frame_bits = width * 2;
  const unsigned bits = frame_bits * 2;
  const unsigned limit = (bits + 2) * 8;
  for (unsigned edge = 1; edge < limit && actual.size() < bits; ++edge) {
    level(serial, edge % 2 != 0);
    const auto pins = serial.clock_outputs();
    const bool now = (pins.levels & 1U) != 0;
    if (now == last_clock) { continue; }
    last_clock = now;
    if (now == inverted_clock) { continue; }
    const bool expected_sync =
        bit_sync ? launches % frame_bits == 0 : launches % frame_bits < width;
    require(((pins.levels & 2U) != 0) == (expected_sync != inverted_frame),
            "Generated FST has the independent slot width, leading bit and selected polarity");
    ++launches;
    const auto output = serial.outputs();
    if (output.driven == 0) {
      require(bit_sync && launches == 1, "Bit-length FS precedes the first data bit by one bit");
      continue;
    }
    require(output.driven == 1 && output.known == 1, "Primed data remains fully known");
    actual.push_back(esai::sample(output, 0));
    if (actual.size() == 7) { serial.write(0, esai::other); }
  }
  require(
      actual == word(true, !bit_sync) + word(false, !bit_sync) + word(false, !bit_sync) +
                    word(false, !bit_sync),
      "Two generated frames retain literal payload/padding, including actual retained underruns");
  constexpr unsigned all_empty_underrun = 0x3c000;
  require(serial.frames_started() == 2 && serial.slots_started() == 4 &&
              serial.read(esai::status) == all_empty_underrun,
          "Frame wrap retains actual data and sets independently expected underrun/even/odd flags");
  serial.write(0, esai::input);
  require(serial.read(esai::status) == 0,
          "Read then refill clears the real underrun and all requests");
  level(serial, false); // Complete the last high half-cycle before counting fresh whole cycles.
  for (unsigned i = 0; i < 4; ++i) { cycle(serial); }
  require(esai::sample(serial.outputs(), 0) == '1' &&
              serial.read(esai::status) == esai::empty_even_frame,
          "The third complete frame consumes the new word at exactly the next boundary");
}

void late_enable_and_reset(const bool inverted) {
  DspEsai serial(DspEsaiInstance::primary);
  configure(serial, base_clock | (inverted ? frame_polarity : 0U));
  // Two HCKT cycles per SCKT toggle; four per data bit.
  constexpr unsigned before_enable = 10;
  for (unsigned i = 0; i < before_enable; ++i) { cycle(serial); }
  serial.write(0, esai::input);
  serial.write(esai::control, word32 | 1U);
  require(serial.read(esai::status) == 0x2000 && serial.outputs().driven == 0,
          "Enabling in an existing first slot observes TFS but waits for the next frame");
  constexpr unsigned next_frame_cycle = 258;
  for (unsigned i = before_enable + 1; i < next_frame_cycle; ++i) {
    cycle(serial);
    require(serial.outputs().driven == 0, "Late enable never starts a truncated first word");
  }
  cycle(serial);
  require(esai::sample(serial.outputs(), 0) == '1' &&
              serial.read(esai::status) == esai::empty_even_frame,
          "The next generated frame starts with the literal MSB and real empty flags");
  serial.write(esai::control, word32 | personal_reset);
  require(serial.clock_outputs() == DspEsaiClockOutputs{.driven = 3, .known = 3} &&
              serial.outputs().driven == 0 && serial.read(esai::status) == 0,
          "Personal reset drives zero clocks, high-Z data and reset status");
  for (unsigned i = 0; i < 5; ++i) { cycle(serial); }
  serial.write(esai::control, word32);
  serial.write(0, esai::other);
  serial.write(esai::control, word32 | 1U);
  cycle(serial);
  require(serial.outputs().driven == 0, "Reset-held input edges do not advance the new divider");
  cycle(serial);
  require(esai::sample(serial.outputs(), 0) == '0',
          "Reprime and two fresh input cycles start exactly the new first MSB");
}

void strict_and_cached_inputs() {
  DspEsai serial(DspEsaiInstance::primary);
  static_cast<void>(rejects([&] { serial.write(esai::clock, output_directions | bypass); }));
  configure(serial);
  serial.write(0, esai::input);
  serial.write(esai::control, word32 | 1U);
  static_cast<void>(rejects([&] { serial.observe({.timing = timing}); }));
  static_cast<void>(rejects([&] { serial.observe({.high_clock = true}); }));
  static_cast<void>(
      rejects([&] { serial.observe({.clock = false, .high_clock = true, .timing = timing}); }));
  static_cast<void>(rejects(
      [&] { serial.observe({.frame_sync = false, .high_clock = true, .timing = timing}); }));
  cycle(serial);
  require(serial.outputs().driven == 0, "Rejected observations did not consume a high-clock edge");
  cycle(serial);
  require(serial.outputs().driven == 1, "Two accepted edges start exactly the first data bit");
  static_cast<void>(rejects([&] { serial.write(esai::clock, base_clock | clock_polarity); }));
  constexpr unsigned fosc_output = 0x800000;
  serial.set_active(false);
  static_cast<void>(rejects([&] { serial.write(esai::clock, base_clock | fosc_output); }));

  DspEsai unframed(DspEsaiInstance::primary);
  level(unframed, false);
  unframed.write(esai::clock, output_directions | bypass | 1U);
  unframed.set_active(true);
  static_cast<void>(rejects([&] { static_cast<void>(unframed.clock_outputs()); }));
  static_cast<void>(rejects([&] { level(unframed, true); }));

  DspEsai external(DspEsaiInstance::secondary);
  external.observe({.clock = true, .frame_sync = false});
  external.write(esai::control, esai::normal24);
  external.set_active(true);
  external.write(0, esai::other);
  external.write(esai::control, esai::normal24 | 1U);
  require(external.read(esai::status) == 0,
          "An observed inactive external baseline survives muxing and transmitter enable");
  require(esai::slot(external)[0] == "000100100011010001010110",
          "The preserved baseline invents no earlier frame or data edge");
}

void generated_frame_from_sckt() {
  DspEsai serial(DspEsaiInstance::primary);
  constexpr unsigned frame_direction = 0x400000;
  serial.observe({.clock = false, .timing = timing});
  serial.write(esai::clock, frame_direction | two_slots);
  serial.write(esai::control, word32);
  serial.set_active(true);
  serial.write(0, esai::other);
  serial.write(esai::control, word32 | 1U);
  std::string bits;
  constexpr unsigned width = 32;
  for (unsigned i = 0; i < width * 2; ++i) {
    serial.observe({.clock = true, .timing = timing});
    bits.push_back(esai::sample(serial.outputs(), 0));
    require(serial.clock_outputs().driven == 2 &&
                serial.clock_outputs().levels == (i < width ? 2U : 0U),
            "External SCKT generates FST alone with a complete first-slot pulse");
    serial.observe({.clock = false, .timing = timing});
    if (i + 1 == width) { serial.write(0, esai::input); }
  }
  require(bits == word(false, true) + word(true, true),
          "Generated word frame sync remains independent of the bit-clock source");
}
}
}

int main() {
  using namespace blofeld::test::esai_clock;
  try {
    divider_ratios();
    constexpr unsigned combinations = 16;
    for (unsigned options = 0; options < combinations; ++options) { serial_words(options); }
    late_enable_and_reset(false);
    late_enable_and_reset(true);
    strict_and_cached_inputs();
    generated_frame_from_sckt();
    native_clock();
    std::cout
        << "ESAI high-clock dividers, generated frames, native access and phase replay passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
