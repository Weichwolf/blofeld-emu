// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/hcs08_spi.hpp"
#include "hcs08_fixture.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld::test::hcs08_spi_test {
namespace spi = hcs08_spi;
constexpr auto contract = Hcs08SpiContract::candidate_gt8a_reference_master_edges;
constexpr unsigned control = 0x74, quiet_control = 0x54, baud = 0x12;
constexpr unsigned hcsr = 0xffff91, htx = 0xffff93, hrx = 0xffff94;
constexpr unsigned dsp_config = 9, reply = 0x123456, event = 0x903c64;
constexpr unsigned edges = 16, samples = 8, byte_mask = 255;
constexpr std::array<std::uint8_t, 3> event_bytes{0x90, 0x3c, 0x64};
constexpr std::array<std::uint8_t, 3> reply_bytes{0x12, 0x34, 0x56};
using hcs08::rejected;
using hcs08::require;

inline void configure(Hcs08Spi &device, const std::uint8_t value = control) {
  device.write({.address = spi::baud, .value = baud});
  device.write({.address = spi::control1, .value = value});
  device.write({.address = spi::control2, .value = 0});
}

inline void queue(Hcs08Spi &device, const std::uint8_t value) {
  require((device.read(spi::status) & spi::transmit_empty) != 0,
          "Actual SPI status read arms a genuinely empty transmitter");
  device.write({.address = spi::data, .value = value});
}

inline void select(DspShi &peer) {
  peer.observe_spi({.selected = false, .clock = false});
  peer.write({.address = hcsr, .value = dsp_config});
  peer.write({.address = htx, .value = reply});
  peer.observe_spi({.selected = true, .clock = false});
}

inline void coupled_edge(Hcs08Spi &device, DspShi &peer) {
  const auto input = peer.outputs().miso;
  device.edge(input.driven ? input.level : std::nullopt);
  const auto output = device.pins();
  require(output.driven, "Enabled HCS08 master owns actual clock/data output drivers");
  peer.observe_spi({.selected = true, .clock = output.clock, .mosi = output.mosi});
}
} // namespace blofeld::test::hcs08_spi_test
