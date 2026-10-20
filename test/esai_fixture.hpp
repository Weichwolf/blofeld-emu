// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_esai.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <source_location>
#include <string>
#include <string_view>

namespace blofeld::test::esai {
inline constexpr unsigned status = 19;
inline constexpr unsigned common = 20;
inline constexpr unsigned control = 21;
inline constexpr unsigned clock = 22;
inline constexpr unsigned mask_a = 25;
inline constexpr unsigned mask_b = 26;
inline constexpr unsigned normal24 = 0xf800;
inline constexpr unsigned input = 0xa5c39f;
inline constexpr unsigned other = 0x123456;
inline constexpr unsigned empty_even_frame = 0x1a000;
inline constexpr unsigned underrun_flag = 0x4000;
inline constexpr unsigned lane_mask = 0x3f;
inline constexpr unsigned word_bits = 24;

inline void require(const bool value,
                    const std::string_view message,
                    const std::source_location location = std::source_location::current()) {
  if (!value) { throw Fault(std::string(message) + " at line " + std::to_string(location.line())); }
}

inline std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const std::exception &error) { return error.what(); }
  throw Fault("Expected strict ESAI failure");
}

inline DspEsaiOutputs
bit(DspEsai &serial, const bool frame, const bool polarity = false, const bool inverted = false) {
  serial.observe({.clock = polarity, .frame_sync = frame != inverted});
  serial.observe({.clock = !polarity, .frame_sync = frame != inverted});
  return serial.outputs();
}

inline char sample(const DspEsaiOutputs output, const unsigned lane) {
  const auto mask = 1U << lane;
  if ((output.driven & mask) == 0) { return 'z'; }
  if ((output.known & mask) == 0) { return '?'; }
  return (output.levels & mask) != 0 ? '1' : '0';
}

inline std::array<std::string, 6> slot(DspEsai &serial,
                                       const unsigned width = word_bits,
                                       const bool frame = true,
                                       const bool polarity = false,
                                       const bool inverted = false) {
  std::array<std::string, 6> result;
  for (unsigned i = 0; i < width; ++i) {
    const auto output = bit(serial, frame && i == 0, polarity, inverted);
    for (unsigned lane = 0; lane < result.size(); ++lane) {
      result.at(lane).push_back(sample(output, lane));
    }
  }
  return result;
}

inline void baseline(DspEsai &serial, const bool polarity = false, const bool inverted = false) {
  serial.set_active(true);
  serial.observe({.clock = !polarity, .frame_sync = inverted});
}

void native_checks();
}
