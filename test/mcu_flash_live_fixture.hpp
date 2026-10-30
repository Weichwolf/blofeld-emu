// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_flash.hpp"
#include "mcu/mcu_flash_timing_state.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace live {
constexpr unsigned entry = 0x8000, ram = 0x1800, stack = 0x1ff0, target = 0xa000;
constexpr unsigned divider_register = 0x100, status_register = 0x105, command_register = 0x106;
constexpr unsigned control_register = 0x103, cop_control = 0x3c;
constexpr unsigned program = 0x20, sector = 0x40, mass = 0x41, verify = 5;
constexpr unsigned launch_flag = 0x80, complete_flag = 0x40, idle = 0xc0;
constexpr unsigned byte_mask = 0xff, word_data = 0x1234, marker = 0x1700, signature = 0x55;
constexpr unsigned handler = 0x8700, vector = 0xffb8, protection = 0xff0d, security = 0xff0f;
constexpr unsigned oscillator_hz = 12000000, reset_edges = 195, instruction_limit = 2000000;
constexpr unsigned default_divider = 0x48, fractional_divider = 0x3c;
constexpr unsigned program_ticks = 3490, fractional_ticks = 2995, erase_ticks = 1440000,
                   mass_ticks = 7200000, verify_ticks = 110, observation_count = 100;
using Code = std::vector<std::uint8_t>;

inline void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

inline std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected explicit rejection");
}

inline void byte(Code &code, const unsigned value) {
  require(value <= byte_mask, "Own assembler byte must fit");
  code.push_back(static_cast<std::uint8_t>(value));
}

inline void word(Code &code, const unsigned value) {
  byte(code, value >> 8U);
  byte(code, value & byte_mask);
}

struct Literal {
  unsigned address;
  unsigned value;
};

inline void store(Code &code, const Literal data) {
  constexpr unsigned ldaa = 0x86;
  constexpr unsigned staa = 0x7a;
  byte(code, ldaa);
  byte(code, data.value);
  byte(code, staa);
  word(code, data.address);
}

struct Scenario {
  unsigned command = program;
  unsigned divider = default_divider;
  unsigned duration = program_ticks;
  bool wait = false;
};

inline Code helper(const Scenario scenario) {
  constexpr unsigned ldd = 0xcc;
  constexpr unsigned std_extended = 0x7c;
  constexpr unsigned brclr = 0x1f;
  constexpr unsigned backward_five = 0xfb;
  constexpr unsigned andcc = 0x10;
  constexpr unsigned clear_i = 0xef;
  constexpr unsigned wai = 0x3e;
  constexpr unsigned bra = 0x20;
  constexpr unsigned backward_two = 0xfe;
  Code code;
  store(code, {.address = cop_control, .value = 0});
  store(code, {.address = divider_register, .value = scenario.divider});
  if (scenario.wait) { store(code, {.address = control_register, .value = complete_flag}); }
  byte(code, ldd);
  word(code, word_data);
  byte(code, std_extended);
  word(code, target);
  store(code, {.address = command_register, .value = scenario.command});
  store(code, {.address = status_register, .value = launch_flag});
  if (scenario.wait) {
    byte(code, andcc);
    byte(code, clear_i);
    byte(code, wai);
  } else {
    byte(code, brclr);
    word(code, status_register);
    byte(code, complete_flag);
    byte(code, backward_five);
  }
  store(code, {.address = marker, .value = signature});
  byte(code, bra);
  byte(code, backward_two);
  return code;
}

inline void put(blofeld::McuFlashImage &image, const Literal data) {
  const auto offset = data.address - entry;
  image.bytes.at(offset) = static_cast<std::uint8_t>(data.value);
  image.fresh.reset(offset / 2);
}

inline blofeld::McuFlashImage image(const Code &algorithm, const Scenario scenario) {
  constexpr unsigned lds = 0xcf;
  constexpr unsigned jump = 6;
  constexpr unsigned rti = 0x0b;
  constexpr unsigned unsecured = 0xfe;
  blofeld::McuFlashImage result;
  result.bytes.fill(byte_mask);
  result.known.set();
  result.fresh.set();
  Code boot;
  byte(boot, lds);
  word(boot, stack);
  for (unsigned i = 0; i < algorithm.size(); ++i) {
    store(boot, {.address = ram + i, .value = algorithm[i]});
  }
  byte(boot, jump);
  word(boot, ram);
  for (unsigned i = 0; i < boot.size(); ++i) {
    put(result, {.address = entry + i, .value = boot[i]});
  }
  Code isr;
  store(isr, {.address = control_register, .value = 0});
  store(isr, {.address = marker + 1, .value = signature});
  byte(isr, rti);
  for (unsigned i = 0; i < isr.size(); ++i) {
    put(result, {.address = handler + i, .value = isr[i]});
  }
  for (const unsigned address : {vector, 0xfffeU}) {
    const auto destination = address == vector ? handler : entry;
    put(result, {.address = address, .value = destination >> 8U});
    put(result, {.address = address + 1, .value = destination & byte_mask});
  }
  put(result, {.address = protection, .value = byte_mask});
  put(result, {.address = security, .value = unsecured});
  if (scenario.command == sector) { put(result, {.address = target, .value = 0}); }
  result.validate();
  return result;
}

inline blofeld::McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .initial_oscillator_phase_ticks = 0,
                    .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = blofeld::McuRunMemory::internal_flash_reads,
          .flash_timing = {.contract = blofeld::FlashTimingContract::uf32_formula_candidate,
                           .transfer_bus_cycles = 1,
                           .verify_unit = blofeld::FlashVerifyUnit::words}};
}

inline void check_array(blofeld::McuMachine &machine, const Scenario scenario) {
  const auto after = machine.bus().power_off_flash_image();
  require(after.known.all(), "Completed nominal operations preserve known cells");
  if (scenario.command == program) {
    require(machine.bus().read(target) == word_data >> 8U &&
                machine.bus().read(target + 1) == (word_data & byte_mask) &&
                !after.fresh.test((target - entry) / 2),
            "Actual native command programs the expected big-endian word and history");
  } else if (scenario.command == sector) {
    constexpr unsigned sector_bytes = 512;
    for (unsigned i = 0; i < sector_bytes; ++i) {
      require(after.bytes.at(target - entry + i) == byte_mask &&
                  after.fresh.test((target - entry + i) / 2),
              "Native erase restores every cell and word history in its sector");
    }
    require(after.bytes[0] != byte_mask, "Sector completion preserves the boot program");
  } else if (scenario.command == mass) {
    for (const auto value : after.bytes) {
      require(value == byte_mask, "Mass completion erases every actual array byte");
    }
    require(after.fresh.all(), "Mass completion restores every word's erase history");
  } else {
    require(machine.bus().read(status_register) == idle,
            "Verify reports the independently nonblank own boot ROM without BLANK");
  }
}

}
