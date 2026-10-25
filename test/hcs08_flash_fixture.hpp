// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <cstdint>
#include <string>

namespace blofeld::test::hcs08_nvm {
namespace hf = hcs08_flash;
constexpr auto contract = Hcs08FlashContract::candidate_gt8a;
constexpr auto transfer = Hcs08FlashMilestone::transfer_buffer;
constexpr auto complete = Hcs08FlashMilestone::complete_command;
constexpr auto cpu = Hcs08FlashAgent::cpu, bdc = Hcs08FlashAgent::background;
constexpr unsigned hz = 6000000, divisor = 30, flags = 0x68, ram_pc = 0xa0;
constexpr unsigned example_data = 0x5a, stored_options = 0xfe;
constexpr unsigned seed_data = 0xa5, example = 0xe123, second = example + 1;

inline void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

inline std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected strict HCS08 FLASH rejection");
}

inline Hcs08FlashImage initial() {
  Hcs08FlashImage image;
  image.data.fill(seed_data);
  image.known.set();
  image.data[hf::nv_options - hf::first] = stored_options;
  image.data[hf::nv_protection - hf::first] = hf::erased;
  return image;
}

inline void setup(Hcs08Flash &flash) {
  flash.context({.pc = ram_pc, .ccr = flags});
  flash.observe_bus_clock(hz);
  flash.write({.address = hf::divider, .value = divisor});
}

struct Command {
  std::uint16_t address = example;
  std::uint8_t value = example_data;
  std::uint8_t opcode = hf::erase_page;
};

inline void launch(Hcs08Flash &flash, const Command command) {
  flash.stage({.address = command.address, .value = command.value});
  flash.write({.address = hf::command, .value = command.opcode});
  flash.write({.address = hf::status, .value = hf::empty});
}

inline void execute(Hcs08Flash &flash, const Command command) {
  launch(flash, command);
  flash.advance(transfer);
  flash.advance(complete);
}
} // namespace blofeld::test::hcs08_nvm
