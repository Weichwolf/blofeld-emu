// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/mcu_bus.hpp"

#include <array>
#include <cstdint>
#include <string_view>
#include <vector>

namespace blofeld::test::endpoint_reset {
// Independent literals: S12USB20D6E2FV1 section 3.1.1 and register reset tables.
// Payload, endpoint descriptions and executable HC12 instructions are own fixtures.
inline constexpr unsigned control = 0x300;
inline constexpr unsigned status = 0x302;
inline constexpr unsigned mask = 0x304;
inline constexpr unsigned timestamp = 0x30a;
inline constexpr unsigned configuration_status = 0x30c;
inline constexpr unsigned selector = 0x30e;
inline constexpr unsigned buffer_selector = 0x30f;
inline constexpr unsigned config_data = 0x310;
inline constexpr unsigned endpoint = 0x314;
inline constexpr unsigned setup_buffer = 0x338;
inline constexpr unsigned selected_buffers = 0x32;
inline constexpr unsigned selected_registers = 0x132;
inline constexpr unsigned enabled_eprst_high = 0xc2;
inline constexpr unsigned local = 0x380;
inline constexpr unsigned own_entry = 0x8000;
inline constexpr unsigned store_pc = own_entry + 3;
inline constexpr unsigned after_store = own_entry + 6;
inline constexpr unsigned own_stack = 0x1ff0;
inline constexpr unsigned byte_mask = 0xff;
inline constexpr unsigned word_mask = 0xffff;
inline constexpr unsigned eprst_high = 0x42;
inline constexpr unsigned mce_high = 0x40;
inline constexpr unsigned men_high = 0xc0;
inline constexpr unsigned phy_edges = 15;
inline constexpr unsigned last_frame = 42;
inline constexpr unsigned pending_mask = 0x3807;
inline constexpr unsigned ready_configuration = 0x8100;
inline constexpr unsigned reset_status = 0xe444;
inline constexpr unsigned software_ready = 0x5004;
inline constexpr unsigned default_endpoint = 0x02000000;
inline constexpr unsigned bulk_in = 0x020000d1;
inline constexpr unsigned bulk_out = 0x020000c2;
inline constexpr std::array<std::uint8_t, 8> request{0x80, 6, 0, 1, 0, 0, 18, 0};
inline constexpr std::array<std::uint8_t, 3> payload{0x19, 0x37, 0xa5};
inline constexpr std::array<unsigned, 6> controls{0x314, 0x316, 0x318, 0x31a, 0x31c, 0x320};

void require(bool condition, std::string_view message);

void reject(const auto &operation, const std::string_view message) {
  bool failed = false;
  try {
    operation();
  } catch (const Fault &fault) {
    require(std::string_view(fault.what()).contains(message),
            "Reset failure has the expected reason");
    failed = true;
  }
  require(failed, "An unqualified reset/transfer state cannot report successful data");
}

unsigned word(McuBus &bus, unsigned address);
void put(McuBus &bus, MemoryWrite write);

struct Description {
  unsigned bank;
  unsigned value;
};

void configure(McuBus &bus, Description description);
void prepare(McuBus &bus);
void populate(McuBus &bus);
void reset_command(McuBus &bus);
void enable(McuBus &bus);
void activate_bulk(McuBus &bus);
void complete_staging(McuBus &bus);
std::vector<unsigned> inspect(McuBus &bus);
void verify_reset(McuBus &bus);
McuFlashImage own_image();
}
