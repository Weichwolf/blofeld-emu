// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_flash.hpp"
#include "blofeld/mcu_machine.hpp"
#include "mcu_checkpoint_fixture.hpp"

#include <cstdint>

namespace blofeld::test {
namespace {
constexpr unsigned target = 0x8234;
constexpr unsigned target_offset = target - entry;
constexpr unsigned second_target = target + 2;
constexpr unsigned program = 0x20;
constexpr unsigned erase = 0x40;
constexpr unsigned launched = 0x80;
constexpr unsigned idle = 0xc0;
constexpr unsigned data = 0x1234;
constexpr unsigned second_data = 0x5678;
constexpr unsigned divider = 0x48;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned bus_hz = 24000000;
constexpr unsigned flash_status = 0x105;
constexpr unsigned flash_command = 0x106;

void prepare(McuBus &bus) {
  bus.observe_flash_clock({.oscillator_hz = oscillator_hz, .bus_hz = bus_hz});
  bus.write(hc12::flash_clock_divider, divider);
}

struct Operation {
  unsigned command = program;
  std::uint16_t address = target;
  unsigned value = data;
};

void select(McuBus &bus, const Operation operation = {}) {
  bus.write_memory({.address = operation.address, .size = 2, .value = operation.value});
  bus.write(flash_command, static_cast<std::uint8_t>(operation.command));
}

void finish(McuBus &bus) {
  bus.advance_flash(FlashMilestone::transfer_buffer);
  bus.advance_flash(FlashMilestone::complete_command);
}

void erase_target(McuBus &bus) {
  select(bus, {.command = erase});
  bus.write(flash_status, launched);
  finish(bus);
}

void test_selected_launched_and_active_jobs() {
  McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA);
  prepare(machine.bus());
  erase_target(machine.bus());
  machine.bus().write_memory({.address = target, .size = 2, .value = data});
  const auto staged = machine.checkpoint();
  machine.bus().write(flash_command, program);
  const auto selected = machine.checkpoint();
  machine.bus().write(flash_status, launched);
  const auto buffered = machine.checkpoint();
  machine.bus().advance_flash(FlashMilestone::transfer_buffer);
  const auto active = machine.checkpoint();
  machine.bus().advance_flash(FlashMilestone::complete_command);
  machine.restore(staged);
  machine.bus().write(flash_command, program);
  machine.bus().write(flash_status, launched);
  finish(machine.bus());
  require(machine.bus().read_memory({.address = target, .size = 2}) == data,
          "Restored staged word accepts its command and programs the expected data");
  machine.restore(selected);
  machine.bus().write(flash_status, launched);
  finish(machine.bus());
  require(machine.bus().read_memory({.address = target, .size = 2}) == data,
          "Restored selected command launches using the saved word and qualified clock");
  machine.restore(buffered);
  finish(machine.bus());
  require(machine.bus().read_memory({.address = target, .size = 2}) == data,
          "Restored launched buffer transfers and completes on its own cloned memory");
  machine.restore(active);
  machine.bus().advance_flash(FlashMilestone::complete_command);
  require(machine.bus().read_memory({.address = target, .size = 2}) == data &&
              !machine.bus().power_off_flash_image().fresh.test(target_offset / 2),
          "Restored active command completes once and consumes its preceding erase history");
  reject([&machine] { machine.bus().advance_flash(FlashMilestone::complete_command); },
         "A restored flash command cannot complete twice");
  machine.restore(staged);
  const auto untouched = machine.bus().power_off_flash_image();
  require(untouched.bytes[target_offset] == byte_mask && untouched.fresh.test(target_offset / 2),
          "Completions in other branches cannot mutate frozen NVM or erase history");
}

void test_queued_job_and_power_off() {
  McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA);
  prepare(machine.bus());
  erase_target(machine.bus());
  select(machine.bus());
  machine.bus().write(flash_status, launched);
  machine.bus().advance_flash(FlashMilestone::transfer_buffer);
  select(machine.bus(), {.address = second_target, .value = second_data});
  machine.bus().write(flash_status, launched);
  const auto queued = machine.checkpoint();
  const auto interrupted = machine.bus().power_off_flash_image();
  require(!interrupted.known.test(target_offset) && interrupted.known.test(target_offset + 2),
          "Power-off image invalidates the active word while leaving the queued word known");
  machine.bus().power_cycle();
  machine.restore(queued);
  machine.bus().advance_flash(FlashMilestone::complete_command);
  finish(machine.bus());
  require(machine.bus().read_memory({.address = target, .size = 2}) == data &&
              machine.bus().read_memory({.address = second_target, .size = 2}) == second_data &&
              machine.bus().read(flash_status) == idle,
          "Restored active and queued flash jobs retain independent offsets, data and order");
  machine.restore(queued);
  require(machine.bus().power_off_flash_image().known == interrupted.known,
          "Checkpoint flash reference is rebound to frozen memory after a source power cycle");
}

void test_latched_options_without_reset() {
  McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA);
  prepare(machine.bus());
  constexpr unsigned options_sector = 0xfe00;
  select(machine.bus(), {.command = erase, .address = options_sector});
  machine.bus().write(flash_status, launched);
  finish(machine.bus());
  constexpr unsigned unsecured = 0xfe;
  require(machine.bus().read(hc12::flash_security) == unsecured &&
              machine.bus().read(hc12::flash_security_field) == byte_mask,
          "Erasing option NVM does not replace previously latched FSEC before reset");
  const auto latched = machine.checkpoint();
  machine.bus().reset();
  require(machine.bus().read(hc12::flash_security) == byte_mask,
          "A real bus reset samples the now erased security option");
  machine.restore(latched);
  require(machine.bus().read(hc12::flash_security) == unsecured &&
              machine.bus().read(hc12::flash_security_field) == byte_mask,
          "Restoring a checkpoint copies live controller options without a cold reset");
}
}

void test_checkpoint_flash() {
  test_selected_launched_and_active_jobs();
  test_queued_job_and_power_off();
  test_latched_options_without_reset();
}
}
