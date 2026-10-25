// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hc12_minmax_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace blofeld::test {
namespace {
constexpr unsigned page_prefix = 0x18;
constexpr unsigned test_flags = 0x5a;

CpuState initial(const MinmaxOperation operation, const MinmaxValues values) {
  return {.pc = minmax_entry,
          .d = static_cast<std::uint16_t>(minmax_accumulator(operation, values.left)),
          .x = minmax_source,
          .y = minmax_initial_y,
          .sp = minmax_initial_stack,
          .ccr = static_cast<std::uint8_t>(values.flags)};
}

void changed_input(const MinmaxOperation operation) {
  MinmaxMemory memory;
  Hc12 cpu(memory, BLOFELD_DEFAULT_SLA);
  const std::array<std::uint8_t, 3> code{
      page_prefix, static_cast<std::uint8_t>(operation.opcode), 0};
  memory.install(code);
  const MinmaxValues values{
      .left = 0, .right = operation.word ? 0x8000U : 0x80U, .flags = test_flags};
  memory.operand(values.right, operation.word);
  memory.change_after_read = true;
  cpu.set_state(initial(operation, values));
  cpu.step();
  const auto selected = minmax_value(operation, values);
  require_minmax(cpu.state().d ==
                         minmax_accumulator(operation, operation.memory ? values.left : selected) &&
                     cpu.state().ccr == minmax_flags(values, operation.word) &&
                     memory.access_count == (operation.memory ? 2U : 1U) &&
                     memory.accesses[0].value == values.right,
                 "Min/max cannot reread a changing operand or recompute flags after selection");
  const unsigned after = operation.word
                             ? (static_cast<unsigned>(memory.bytes[memory.source]) << 8U) |
                                   memory.bytes[memory.source + 1]
                             : memory.bytes[memory.source];
  const auto changed = values.right ^ (operation.word ? 0xff00U : 0xffU);
  require_minmax(after == (operation.memory ? selected : changed),
                 "A read-side effect must remain visible unless the instruction actually writes");
}

void failed_operand(const MinmaxOperation operation, const bool write) {
  MinmaxMemory memory;
  std::vector<Hc12BusCycle> clocks;
  Hc12CycleBus bus(memory, [&clocks](const auto cycle) { clocks.push_back(cycle); });
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  const std::array<std::uint8_t, 3> code{
      page_prefix, static_cast<std::uint8_t>(operation.opcode), 0};
  memory.install(code);
  const MinmaxValues values{.left = operation.maximum ? 0U : minmax_byte_mask,
                            .right = operation.maximum ? minmax_byte_mask : 0U,
                            .flags = test_flags};
  memory.operand(values.right, operation.word);
  const auto before = memory.bytes;
  memory.reject_read = !write;
  memory.reject_write = write;
  cpu.set_state(initial(operation, values));
  bool rejected = false;
  try {
    cpu.step();
  } catch (const Fault &error) {
    const std::string message = error.what();
    rejected = message.contains(write ? "Own minmax write failure" : "Own minmax read failure") &&
               message.contains("pc=0x8000") && message.contains("ccr=0x5a");
  }
  require_minmax(!clocks.empty(), "A rejected min/max operand must expose its attempted cycle");
  const auto last = clocks.back();
  require_minmax(
      rejected && bus.faulted() && memory.bytes == before &&
          memory.access_count == (write ? 1U : 0U) && last.address == minmax_source &&
          last.size == (operation.word ? 2U : 1U) && clocks.size() == (write ? 4U : 2U),
      "A native min/max operand fault must stop at its actual manual slot without mutation");
  cpu.set_state(initial(operation, values));
  const auto cycles = bus.cycles();
  rejected = false;
  try {
    cpu.step();
  } catch (const Fault &error) {
    rejected = std::string(error.what()).contains("requires coordinated recovery");
  }
  require_minmax(rejected && bus.cycles() == cycles && memory.bytes == before,
                 "A failed min/max cycle cannot be retried through CPU debug replacement");
}

void configure_gpio(McuBus &bus, const unsigned value) {
  drive_interrupts(bus);
  bus.write(hc12::porta, static_cast<std::uint8_t>(value >> 8U));
  bus.write(hc12::portb, static_cast<std::uint8_t>(value & minmax_byte_mask));
  bus.write(hc12::ddra, minmax_byte_mask);
  bus.write(hc12::ddrb, minmax_byte_mask);
}

void check_gpio_events(const std::vector<BusEvent> &events,
                       const MinmaxOperation operation,
                       const MinmaxValues values) {
  const unsigned width = operation.word ? 2U : 1U;
  const unsigned source = operation.word ? hc12::porta : hc12::portb;
  // McuBus observers report writes, not reads. The flat native fixture checks
  // exact read counts separately; here every actual GPIO write must be visible.
  const auto expected_events = operation.memory ? static_cast<std::size_t>(width) : 0U;
  require_minmax(
      events.size() == expected_events,
      "Real GPIO min/max must perform every documented byte write, including unchanged data");
  const auto result = minmax_value(operation, values);
  for (unsigned index = 0; index < events.size(); ++index) {
    const auto shift = (width - 1 - index) * 8;
    require_minmax(events[index].write && events[index].address == source + index &&
                       events[index].value == ((result >> shift) & minmax_byte_mask),
                   "Real GPIO min/max writes retain their exact order, address and value");
  }
}

void gpio_case(const MinmaxOperation operation, const bool reversed) {
  const std::array<std::uint8_t, 3> code{
      page_prefix, static_cast<std::uint8_t>(operation.opcode), 0};
  const auto image = checkpoint_image(code);
  McuMachine machine(image, BLOFELD_DEFAULT_SLA);
  const unsigned low = operation.word ? 0x1000U : 0x10U;
  const unsigned high = operation.word ? 0x8000U : 0x80U;
  const MinmaxValues values{
      .left = reversed ? high : low, .right = reversed ? low : high, .flags = test_flags};
  configure_gpio(machine.bus(), values.right);
  auto before = initial(operation, values);
  before.x = operation.word ? hc12::porta : hc12::portb;
  machine.cpu().set_state(before);
  const auto checkpoint = machine.checkpoint();
  std::vector<BusEvent> events;
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    if (attempt != 0) { machine.restore(checkpoint); }
    events.clear();
    machine.bus().set_observer([&events](const auto event) {
      if (event.address <= hc12::portb) { events.push_back(event); }
    });
    machine.cpu().step();
    check_gpio_events(events, operation, values);
    const auto result = minmax_value(operation, values);
    const auto retained = operation.memory ? result : values.right;
    const auto port_a = machine.bus().observe_pads(McuPort::a, minmax_byte_mask);
    const auto port_b = machine.bus().observe_pads(McuPort::b, minmax_byte_mask);
    const auto high_byte = operation.word ? retained >> 8U : 0U;
    require_minmax(
        port_a.known == minmax_byte_mask && port_b.known == minmax_byte_mask &&
            port_a.levels == high_byte && port_b.levels == (retained & minmax_byte_mask),
        "Native min/max must leave actual GPIO output latches at independently expected values");
    const auto after = machine.cpu().state();
    const auto nvm = machine.bus().power_off_flash_image();
    require_minmax(
        after.pc == minmax_entry + code.size() && after.x == before.x && after.y == before.y &&
            after.sp == before.sp &&
            after.d == minmax_accumulator(operation, operation.memory ? values.left : result) &&
            after.ccr == minmax_flags(values, operation.word) && nvm.bytes == image.bytes &&
            nvm.known == image.known && nvm.fresh == image.fresh,
        "Trusted native replay must preserve GPIO expectations, CPU and all NVM state");
  }
}
}

unsigned minmax_side_effect_cases() {
  unsigned cases = 0;
  for (const auto operation : minmax_operations) {
    changed_input(operation);
    failed_operand(operation, false);
    cases += 2;
    if (operation.memory) {
      failed_operand(operation, true);
      ++cases;
    }
  }
  return cases;
}

unsigned minmax_machine_cases() {
  unsigned cases = 0;
  for (const auto operation : minmax_operations) {
    for (const auto reversed : {false, true}) {
      gpio_case(operation, reversed);
      ++cases;
    }
  }
  return cases;
}
}
