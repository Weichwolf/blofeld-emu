// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_esai.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld::test::esai_irq {
using blofeld::test::esai::rejects;
using blofeld::test::esai::require;
constexpr unsigned priority_register = 0xfffffe, core_priority_register = 0xffffff;
constexpr unsigned primary_address = 0xffffa0, secondary_address = 0xffff80;
constexpr unsigned primary_vector = 0x38, secondary_vector = 0x78;
constexpr unsigned primary_even = 0x3a, primary_error = 0x3c, secondary_even = 0x7a,
                   secondary_error = 0x7c;
constexpr unsigned all_core_priorities = 0xfff003, all_priorities = 0xf0f;
constexpr unsigned both_high_priorities = 0xc03, mask_offset = 25, enable_shift = 20;
constexpr unsigned last_enable = 0x800000;
constexpr unsigned format = 0xf800, network = 0x100, personal_reset = 0x80000;
constexpr unsigned all_enables = 0x700000, data_enable = 0x400000;
constexpr unsigned even_enable = 0x200000, error_enable = 0x100000;
constexpr unsigned status_offset = 19, control_offset = 21, clock_offset = 22;
constexpr unsigned receive_control_offset = 23, port_control_offset = 31,
                   port_direction_offset = 30;
constexpr unsigned status_underrun = 0x4000, word_bits = 24;
constexpr unsigned two_slots = 0x200, mux = 0x858, two_lanes = 0x21;
constexpr unsigned first_data = 0xa5c39f, second_data = 0x89abcd;
constexpr unsigned next_first = 0x123456, next_second = 0xfedcba;

inline unsigned enables(const unsigned code) {
  return code << enable_shift;
}

inline unsigned base(const bool second) {
  return second ? secondary_address : primary_address;
}

inline unsigned vector_base(const bool second) {
  return second ? secondary_vector : primary_vector;
}

inline auto instance(const bool second) {
  return second ? DspEsaiInstance::secondary : DspEsaiInstance::primary;
}

struct Flags {
  bool even;
  bool error;
};

inline std::optional<std::uint32_t>
expected(const bool second, const unsigned mode, const Flags flags) {
  if (flags.error && (mode & 1U) != 0) { return vector_base(second) + 4; }
  if (flags.even && (mode & 2U) != 0) { return vector_base(second) + 2; }
  return (mode & 4U) != 0 ? std::optional<std::uint32_t>(vector_base(second)) : std::nullopt;
}

inline void fill(DspEsai &serial, const unsigned lanes) {
  for (unsigned lane = 0; lane < 6; ++lane) {
    if ((lanes & (1U << lane)) != 0) { serial.write(lane, first_data + lane); }
  }
}

inline void setup(DspEsai &serial, const unsigned mode, const unsigned lanes) {
  serial.write(clock_offset, two_slots);
  fill(serial, lanes);
  serial.write(control_offset, format | network | lanes | enables(mode));
  serial.set_active(true);
  serial.observe({.clock = true, .frame_sync = false});
}

inline void put(DspBus &bus, const bool second, const unsigned offset, const unsigned value) {
  bus.write({.space = second ? DspSpace::y : DspSpace::x,
             .address = base(second) + offset,
             .value = value});
}

inline void setup(DspBus &bus, const bool second, const unsigned mode, const bool known = true) {
  put(bus, second, 0, first_data);
  put(bus, second, 5, second_data);
  put(bus, second, control_offset, format | two_lanes | enables(mode));
  put(bus, second, port_control_offset, mux);
  put(bus, second, port_direction_offset, mux);
  if (known) { bus.observe_esai_transmit(instance(second), {.clock = true, .frame_sync = false}); }
}

inline DspEsaiOutputs bit(DspBus &bus, const bool second, const bool sync) {
  bus.observe_esai_transmit(instance(second), {.clock = false, .frame_sync = sync});
  bus.observe_esai_transmit(instance(second), {.clock = true, .frame_sync = sync});
  return bus.esai_transmit_outputs(instance(second));
}

inline std::array<std::uint32_t, 2> word(DspBus &bus, const bool second) {
  std::array<std::uint32_t, 2> result{};
  for (unsigned index = 0; index < word_bits; ++index) {
    const auto output = bit(bus, second, index == 0);
    require(output.driven == two_lanes && output.known == two_lanes,
            "Actual ESAI pins retain both declared transmitter lanes");
    result[0] = (result[0] << 1U) | (output.levels & 1U);
    result[1] = (result[1] << 1U) | ((output.levels >> 5U) & 1U);
  }
  return result;
}

void native_checks();
} // namespace blofeld::test::esai_irq
