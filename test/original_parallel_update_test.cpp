// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "parallel_update_fixture.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

namespace {
constexpr unsigned original_first_bytes = 761;
constexpr unsigned original_main_bytes = 632;
constexpr unsigned byte_mask = 255;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path trace;
};

int run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  blofeld::McuBus bus(firmware.mcu);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  std::ofstream trace(inputs.trace);
  require(trace.is_open(), "Cannot create private original parallel-update trace");
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  blofeld::test::boot(bus, cpu, trace);
  blofeld::test::transfer(bus,
                          cpu,
                          {.payload = blofeld::test::original_helper(firmware.update_blocks.at(0),
                                                                     original_first_bytes),
                           .dispatch = true},
                          trace);
  blofeld::test::check_first_callback(bus, cpu, trace);
  blofeld::test::transfer(bus,
                          cpu,
                          {.payload = blofeld::test::original_helper(firmware.update_blocks.at(4),
                                                                     original_main_bytes)},
                          trace);
  for (const auto size : {0U, 1U, 2U, 3U, 4U, 5U, 8U, 31U, 255U, 256U, 511U, 512U}) {
    std::vector<std::uint8_t> bytes(size);
    constexpr unsigned pattern_stride = 73;
    for (unsigned index = 0; index < size; ++index) {
      bytes[index] = static_cast<std::uint8_t>((index * pattern_stride) & byte_mask);
    }
    blofeld::test::transfer(bus, cpu, {.payload = bytes}, trace);
  }
  const std::vector<std::uint8_t> interrupted(64, byte_mask);
  for (unsigned cut = 0; cut < 12; ++cut) {
    blofeld::test::transfer(bus, cpu, {.payload = interrupted, .loss_at = cut}, trace);
  }
  const auto image = bus.power_off_flash_image();
  require(std::ranges::equal(image.bytes, firmware.mcu) && image.known.all() && image.fresh.none(),
          "All original normal-copy cases preserve every MCU flash byte and its initial history");
  std::cout << "Original parallel loader: both original helpers, 12 synthetic lengths and 12 "
               "withdrawn-input positions checked; original caller/table/first callback verified. "
               "Conditional pin/entry fixture, no full DSP/MIDI update or calibrated board boot.\n";
  return 0;
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-parallel-update-test firmware.zip private.trace");
    }
    return run({.firmware = arguments[1], .trace = arguments[2]});
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
