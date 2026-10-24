// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_clock_startup_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/gpio_pads.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_flash.hpp"
#include "blofeld/hcs08_gpio.hpp"
#include "blofeld/hcs08_icg.hpp"
#include "blofeld/hcs08_system_options.hpp"
#include "hcs08_fixture.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::test::hcs08_clock_startup {
using hcs08::require;

namespace {
constexpr unsigned canary = 0xa5, record_size = 514, configured_ccr = 0x6c;
constexpr unsigned fixed_frame = 0xc1, calibration_low = 44;

Hcs08FlashImage image(const std::span<const std::uint8_t> records, const std::uint8_t trim) {
  require(records.size() == records_bytes, "Pinned seven-page stream has its exact extent");
  Hcs08FlashImage result;
  for (unsigned page = 0; page < pages.size(); ++page) {
    const auto first = page * record_size;
    const auto address = (static_cast<unsigned>(records[first]) << 8U) | records[first + 1];
    require(address == pages[page], "Pinned page addresses retain original mapping");
    for (unsigned n = 0; n < hf::page_bytes; ++n) {
      const auto offset = address + n - hf::first;
      result.data[offset] = records[first + 2 + n];
      result.known.set(offset);
    }
  }
  // Declared per-unit calibration data only; all executable/reference bytes stay
  // unchanged. The original HC12 updater preserves this existing FLASH byte.
  result.data[factory_trim - hf::first] = trim;
  return result;
}

bool icg_register(const unsigned address) {
  return address >= hi::control1 && address <= hi::trim;
}

bool flash_register(const unsigned address) {
  return address >= hf::divider && address <= hf::command;
}
} // namespace

ApplicationMemory::ApplicationMemory(const std::span<const std::uint8_t> records, const Input input)
    : flash(image(records, input.trim), Hcs08FlashContract::candidate_gt8a) {
  bytes.fill(canary);
  for (unsigned address = ram_first; address < ram_end; ++address) { writable.set(address); }
  if (input.seed != 0) {
    icg.write({.address = hi::trim, .value = prior_live_trim});
    constexpr unsigned old_control1 = 0x38;
    constexpr unsigned old_control2 = 0x42;
    icg.write({.address = hi::control1, .value = old_control1});
    icg.write({.address = hi::control2, .value = old_control2});
  }
  bind(input.observation);
}

void ApplicationMemory::bind(const int observation) {
  if (observation >= 0) {
    gpio.attach_pad_source(Hcs08GpioPort::e, [observation](const unsigned mask) {
      return GpioPadSample{.known = mask, .levels = static_cast<unsigned>(observation) & mask};
    });
  }
}

void ApplicationMemory::warm_reset() {
  require(!resetting, "Peripheral reset cannot occur within a CPU vector transaction");
  flash.reset();
  gpio.reset();
  icg.reset(Hcs08IcgReset::warm);
  options.reset();
}

void ApplicationMemory::begin_instruction(const ExecutionContext value) {
  require(!resetting, "No instruction can execute inside an incomplete reset-vector transaction");
  hcs08::Memory::begin_instruction(value);
  flash.context(value);
  icg.context(value);
}

void ApplicationMemory::begin_reset(const ExecutionContext value) {
  begin_instruction(value);
  resetting = true;
}

void ApplicationMemory::end_reset() {
  require(resetting, "Reset-vector phase has a matching begin boundary");
  resetting = false;
}

std::uint8_t ApplicationMemory::read(const std::uint16_t address) {
  if (address >= hf::first) {
    const auto value =
        resetting ? flash.read_reset_vector_byte(address) : flash.read_array(address);
    flash_reads.push_back({.address = address, .value = value, .write = false});
    return value;
  }
  require(!resetting, "Only actual FLASH vector reads belong to the reset transaction");
  if (!Hcs08Gpio::contains(address) && !icg_register(address) && !flash_register(address) &&
      address != Hcs08SystemOptions::address) {
    return hcs08::Memory::read(address);
  }
  std::uint8_t value = 0;
  if (Hcs08Gpio::contains(address)) {
    value = gpio.read(address, context);
  } else if (icg_register(address)) {
    value = icg.read(address);
  } else if (flash_register(address)) {
    value = flash.read(address);
  } else {
    value = options.read();
  }
  peripheral_accesses.push_back({.address = address, .value = value, .write = false});
  return value;
}

void ApplicationMemory::write(const std::uint16_t address, const std::uint8_t value) {
  require(!resetting, "An instruction cannot store during a CPU vector transaction");
  if (address >= hf::first) {
    throw Fault("Unqualified startup FLASH write address=" + hex(address) +
                " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr));
  }
  if (!Hcs08Gpio::contains(address) && !icg_register(address) && !flash_register(address) &&
      address != Hcs08SystemOptions::address) {
    hcs08::Memory::write(address, value);
    return;
  }
  if (Hcs08Gpio::contains(address)) {
    gpio.write({.address = address, .value = value}, context);
  } else if (icg_register(address)) {
    icg.write({.address = address, .value = value});
  } else if (flash_register(address)) {
    flash.write({.address = address, .value = value});
  } else {
    options.write({.address = address, .value = value}, context);
  }
  peripheral_accesses.push_back({.address = address, .value = value, .write = true});
  ++generation;
}

std::uint8_t ApplicationMemory::peek(const std::uint16_t address) const {
  if (address < hf::first) { return hcs08::Memory::peek(address); }
  const auto offset = address - hf::first;
  return flash.image().known[offset] ? flash.image().data[offset] : 0;
}

void ApplicationMemory::check_execute(const InstructionFetch fetch) const {
  if (fetch.address < hf::first) {
    hcs08::Memory::check_execute(fetch);
    return;
  }
  require(fetch.length <= hcs08::space_bytes - fetch.address, "FLASH code fetch cannot wrap");
  for (unsigned n = 0; n < fetch.length; ++n) {
    static_cast<void>(flash.read_array(static_cast<std::uint16_t>(fetch.address + n)));
  }
}

Hcs08State warm_state(const unsigned seed) {
  constexpr Hcs08State first{
      .pc = 0x1234, .hx = 0x3322, .sp = 0x4567, .a = 0xa5, .ccr = hcs08::byte_mask};
  constexpr Hcs08State second{
      .pc = 0x5678, .hx = 0x44bb, .sp = 0x2456, .a = 0x5a, .ccr = Hcs08State::fixed_ccr};
  require(seed < 2, "Only the declared warm register histories are supported");
  return seed == 0 ? first : second;
}

void ram_contents(const ApplicationMemory &memory, const bool entered_main) {
  std::array<std::uint8_t, ram_end - ram_first> expected{};
  expected[0] = expected[6] = 7;
  expected[1] = expected[7] = hcs08::byte_mask;
  expected[fixed_frame - ram_first] = 1;
  expected[fixed_frame - ram_first + 1] = calibration_low;
  expected[expected.size() - 2] = 4;
  expected[expected.size() - 1] = 0x80;
  if (entered_main) {
    constexpr unsigned return_high = 0xe0;
    constexpr unsigned return_low = 0x5e;
    expected[stack - ram_first] = return_low;
    expected[stack - ram_first - 1] = return_high;
  }
  for (unsigned n = 0; n < expected.size(); ++n) {
    require(memory.known[ram_first + n] && memory.bytes[ram_first + n] == expected[n],
            "Independent original RAM/iterator/calibration/return-stack contents agree");
  }
}

void configured(const ApplicationMemory &memory, const Hcs08 &cpu) {
  constexpr std::array<Access, 11> writes{{{.address = 16, .value = 3, .write = true},
                                           {.address = 19, .value = 3, .write = true},
                                           {.address = 1, .value = 255, .write = true},
                                           {.address = 3, .value = 0, .write = true},
                                           {.address = 0, .value = 0, .write = true},
                                           {.address = 4, .value = 248, .write = true},
                                           {.address = 8, .value = 255, .write = true},
                                           {.address = 12, .value = 255, .write = true},
                                           {.address = 7, .value = 248, .write = true},
                                           {.address = 11, .value = 255, .write = true},
                                           {.address = 15, .value = 255, .write = true}}};
  constexpr std::array<Hcs08GpioRegisters, 5> ports{{{.pull_enable = 255},
                                                     {.latch = 248, .direction = 248},
                                                     {.latch = 255, .direction = 255},
                                                     {.latch = 31, .direction = 31},
                                                     {.latch = 3, .direction = 3}}};
  require(memory.peripheral_accesses == std::vector<Access>(writes.begin(), writes.end()),
          "All11 actual GPIO writes retain independent address/value/order expectations");
  for (unsigned n = 0; n < ports.size(); ++n) {
    require(memory.gpio.state(static_cast<Hcs08GpioPort>(n)) == ports[n],
            "All GPIO latch/direction/pull/slew fields retain original startup state");
  }
  require(cpu.state() == Hcs08State{.pc = input_pc,
                                    .hx = expected_hx,
                                    .sp = stack,
                                    .a = 3,
                                    .ccr_known = ccr_known,
                                    .ccr = configured_ccr} &&
              memory.retired == input_retired,
          "Independent full CPU endpoint before first actual strap input read");
  ram_contents(memory, false);
}

void same_endpoint(const ApplicationMemory &actual, const ApplicationMemory &expected) {
  require(actual.bytes == expected.bytes && actual.known == expected.known &&
              actual.retired == expected.retired && actual.accesses == expected.accesses &&
              actual.flash.image() == expected.flash.image() &&
              actual.flash.state() == expected.flash.state() &&
              actual.flash_reads == expected.flash_reads &&
              actual.icg.state() == expected.icg.state() &&
              actual.options.read() == expected.options.read() &&
              actual.options.written() == expected.options.written() &&
              actual.peripheral_accesses == expected.peripheral_accesses &&
              actual.resetting == expected.resetting,
          "Fresh joint owners reproduce all architectural memory/peripheral histories and NVM");
  for (unsigned n = 0; n < 5; ++n) {
    const auto port = static_cast<Hcs08GpioPort>(n);
    require(actual.gpio.state(port) == expected.gpio.state(port),
            "Fresh GPIO owners preserve every original register value");
  }
}
} // namespace blofeld::test::hcs08_clock_startup
