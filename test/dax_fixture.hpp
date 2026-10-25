// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_dax.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace blofeld::test::dax_test {
using esai::rejects;
using esai::require;
inline constexpr unsigned control = 0xffffd0;
inline constexpr unsigned non_audio = 0xffffd1;
inline constexpr unsigned audio_a = 0xffffd2;
inline constexpr unsigned audio_b = 0xffffd3;
inline constexpr unsigned status = 0xffffd4;
inline constexpr unsigned port_data = 0xffffd5;
inline constexpr unsigned port_direction = 0xffffd6;
inline constexpr unsigned port_control = 0xffffd7;
inline constexpr unsigned empty = 1;
inline constexpr unsigned underrun = 2;
inline constexpr unsigned last_block = 4;
inline constexpr unsigned restart = 0x20;
inline constexpr unsigned frame_units = 128;
inline constexpr unsigned subframe_units = 64;
inline constexpr unsigned sample_mask = 0xffffff;
inline constexpr unsigned first_sample = 0x800001;
inline constexpr unsigned other_sample = 0x123456;
inline constexpr auto timing = DspDaxTiming::reference_rising_edges;
inline constexpr std::string_view zero_z =
    "0001011100110011001100110011001100110011001100110011001100110011";
inline constexpr std::string_view ones_y =
    "0001101101010101010101010101010101010101010101010101010101010101";
inline constexpr std::string_view lsb_x =
    "0001110101001100110011001100110011001100110011001100110011001101";
inline constexpr std::string_view msb_z =
    "0001011100110011001100110011001100110011001100110011001011001101";

inline DspDaxRates rates(const unsigned selection = 0) {
  constexpr std::uint64_t master = 49152000;
  constexpr std::uint64_t core = 54000000;
  constexpr std::array<std::uint64_t, 4> input{0, 12288000, 18432000, 24576000};
  return {.core_hz = selection == 0 ? master : core, .aci_hz = input.at(selection)};
}

inline DspDaxInputs inputs(const bool level, const unsigned selection = 0) {
  return {.core_clock = selection == 0 ? level : false,
          .aci = selection != 0 ? level : false,
          .rates = rates(selection),
          .timing = timing};
}

inline void prime(DspDax &serial, const unsigned selection = 0) {
  serial.observe(inputs(false, selection));
  serial.write(control, selection << 3U);
  serial.write(non_audio, 0);
  serial.write(audio_a, 0);
  serial.write(audio_a, 0);
  serial.set_active(true);
}

struct Capture {
  std::size_t count;
  unsigned selection = 0;
};

inline std::string collect(DspDax &serial, const Capture capture) {
  const auto count = capture.count;
  const auto selection = capture.selection;
  std::string stream;
  serial.observe(inputs(false, selection));
  const auto initial = serial.half_bits();
  constexpr unsigned maximum_cycles_per_unit = 8;
  for (std::size_t i = 0; i < count * maximum_cycles_per_unit && stream.size() < count; ++i) {
    serial.observe(inputs(true, selection));
    if (serial.half_bits() != initial + stream.size()) {
      require(serial.output().driven, "A selected known clock drives the logical ADO signal");
      stream.push_back(serial.output().level ? '1' : '0');
    }
    serial.observe(inputs(false, selection));
  }
  require(stream.size() == count, "The literal divider bound produces exactly the expected units");
  return stream;
}

inline std::string collect(DspDax &serial, const std::size_t count) {
  return collect(serial, Capture{.count = count});
}

struct Decoded {
  unsigned audio;
  unsigned flags;
  bool operator==(const Decoded &) const = default;
};

inline Decoded decode(const std::string_view stream, const char preamble) {
  constexpr std::array prefixes{
      std::string_view{"00011101"}, std::string_view{"00011011"}, std::string_view{"00010111"}};
  unsigned index = 0;
  if (preamble == 'Z') { index = 2; }
  if (preamble == 'Y') { index = 1; }
  require(stream.size() == subframe_units && stream.substr(0, 8) == prefixes.at(index),
          "The receiver checks literal manual X/Y/Z preambles");
  unsigned bits = 0;
  unsigned ones = 0;
  bool previous = stream[7] == '1';
  constexpr unsigned data_bits = 28;
  for (unsigned bit = 0; bit < data_bits; ++bit) {
    const bool first = stream[8 + (bit * 2)] == '1';
    const bool second = stream[9 + (bit * 2)] == '1';
    require(first != previous, "Every data cell begins with its independent BMC boundary");
    if (first != second) {
      bits |= 1U << bit;
      ++ones;
    }
    previous = second;
  }
  require(ones % 2 == 0, "The complete transmitted payload has even parity");
  constexpr unsigned flag_shift = 24;
  return {.audio = bits & sample_mask, .flags = (bits >> flag_shift) & 7U};
}

void native_checks();
void dma_checks();
}
