// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/gpio_pads.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/hcs08_bdc.hpp"
#include "blofeld/hcs08_bdc_wire.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_wire_fixture.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <ostream>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::keyboard_bdc_wire {
using hcs08::require;
constexpr unsigned helper_base = 0x1200, helper_bytes = 1018, caller = 0x2000,
                   returned = caller + 3;
constexpr unsigned host_stack = 0x2200, write_entry = 0x14ab, read_entry = 0x14c3;
constexpr unsigned byte_mask = 255, pin = 2, mapping = 0x21, initial_ccr = 0xd0;
constexpr unsigned reference_ratio = 8, maximum_steps = 10000, oscillator_hz = 12000000;
constexpr unsigned data_address = 0x200, reply_address = 0x201;
constexpr unsigned write_opcode = 0xc0, read_opcode = 0xe0;

struct Call {
  unsigned entry;
  unsigned argument;
};

struct Fixture {
  const Firmware &firmware;
  const std::vector<std::uint8_t> &helper;
  McuMachine host;
  hcs08_wire::Target target;
  Hcs08Bdc bdc{target.cpu, target.mapping, hcs08_wire::chip, Hcs08BdcReset::bkgd_held_low};
  Hcs08BdcWire wire;
  std::uint64_t steps = 0, received = 0, pin_writes = 0;
  std::vector<unsigned> expected;
  unsigned cursor = 0;
  ExecutionContext context;
  std::ostream &trace;
  std::function<void(std::uint64_t)> target_clock;

  Fixture(const Firmware &image,
          const std::vector<std::uint8_t> &code,
          const unsigned phase,
          std::ostream &output,
          const hcs08_programmer::Memory *snapshot = nullptr)
      : firmware(image),
        helper(code),
        host(McuFlashImage::from_known(image.mcu),
             BLOFELD_DEFAULT_SLA,
             McuRunContract{.clock = {.oscillator_hz = oscillator_hz,
                                      .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                            .memory_timing = McuRunMemory::internal_flash_reads}),
        target(snapshot),
        wire(bdc,
             hcs08_wire::wire_contract,
             Hcs08BdcDrive::high,
             {.period_ticks = static_cast<std::uint64_t>(reference_ratio) * 2,
              .first_edge_ticks = (static_cast<std::uint64_t>(phase) * 2) + 1}),
        trace(output) {
    require(phase < reference_ratio && helper.size() == helper_bytes,
            "Explicit original wire fixture extent/phase");
    auto &bus = host.bus();
    bus.write(hc12::initee, mapping);
    bus.retire_instruction();
    for (unsigned n = 0; n < helper.size(); ++n) {
      bus.write(static_cast<std::uint16_t>(helper_base + n), helper[n]);
    }
    constexpr std::array<std::uint8_t, 5> own_caller{0x16, 0x14, 0xab, 0x20, 0xfe};
    for (unsigned n = 0; n < own_caller.size(); ++n) {
      bus.write(static_cast<std::uint16_t>(caller + n), own_caller[n]);
    }
    bus.write(hc12::ddrb, byte_mask);
    bus.write(hc12::portb, pin);
    constexpr unsigned low_delay = 0x1600;
    constexpr unsigned high_delay = 0x1601;
    bus.write(low_delay, 9);
    bus.write(high_delay, 12); // Explicit post-SYNC helper-caller input, not a default board value.
    bind();
  }

  void bind() {
    host.bus().attach_gpio_pad_source(McuPort::b,
                                      [this](const std::uint32_t mask) { return pads(mask); });
    host.bus().set_observer([this](const BusEvent &event) { observe(event); });
    host.set_cycle_observer([this](const Hc12BusCycle &cycle) { advance(cycle); });
  }

  GpioPadSample pads(const std::uint32_t mask) const {
    const auto outputs = host.bus().gpio_outputs(McuPort::b);
    const auto drive = wire.target_drive();
    const auto selected = mask & pin;
    GpioNetDriver peer{.pull_up = selected};
    if (drive != Hcs08BdcDrive::released) {
      peer.active = selected;
      peer.known = selected;
      peer.levels = drive == Hcs08BdcDrive::high ? selected : 0;
      peer.pull_up = 0;
    }
    return resolve_gpio_net(mask,
                            {.active = outputs.mask & mask,
                             .known = outputs.known & outputs.mask & mask,
                             .levels = outputs.levels & outputs.mask & mask},
                            peer);
  }

  void observe(const BusEvent &event) {
    if (!event.write || (event.address != hc12::portb && event.address != hc12::ddrb)) { return; }
    const auto outputs = host.bus().gpio_outputs(McuPort::b);
    require((outputs.mask & pin & ~static_cast<unsigned>(outputs.known)) == 0,
            "Actual BDC host output must be known");
    auto drive = Hcs08BdcDrive::released;
    if ((outputs.mask & pin) != 0) {
      drive = (outputs.levels & pin) != 0 ? Hcs08BdcDrive::high : Hcs08BdcDrive::low;
    }
    wire.host_drive(drive, context, host.run_clock().bus_cycles() * 2);
    ++pin_writes;
    trace << "PIN cycles=" << host.run_clock().bus_cycles() << " pc=" << context.pc
          << " address=" << event.address << " value=" << static_cast<unsigned>(event.value)
          << " drive=" << static_cast<unsigned>(drive) << '\n';
  }

  void advance(const Hc12BusCycle cycle) {
    context = cycle.context;
    const auto boundary = (host.run_clock().bus_cycles() + 1) * 2;
    while (wire.state().next_clock_ticks <= boundary) {
      if (target_clock) { target_clock(wire.state().next_clock_ticks); }
      wire.clock();
      if (wire.state().received_bytes == received) { continue; }
      const auto control = bdc.state();
      require(wire.state().received_bytes == received + 1 && control.received > 0 &&
                  cursor < expected.size(),
              "Each received original byte has an independent expectation");
      const auto byte = control.frame.at(control.received - 1);
      require(byte == expected[cursor],
              "Original HC12 serialized byte differs at index=" + std::to_string(cursor) +
                  " actual=" + hex(byte) + " expected=" + hex(expected[cursor]));
      trace << "RX index=" << cursor << " value=" << static_cast<unsigned>(byte)
            << " BDC-cycles=" << wire.state().cycles << '\n';
      ++cursor;
      ++received;
    }
  }

  void step() {
    host.cpu().step();
    ++steps;
  }

  unsigned call(const Call invocation) {
    const auto entry = invocation.entry;
    const auto argument = invocation.argument;
    auto &bus = host.bus();
    bus.write(caller + 1, static_cast<std::uint8_t>(entry >> 8U));
    bus.write(caller + 2, static_cast<std::uint8_t>(entry & byte_mask));
    host.cpu().set_state({.pc = caller,
                          .d = static_cast<std::uint16_t>(argument),
                          .sp = host_stack,
                          .ccr = initial_ccr});
    const auto start = steps;
    cursor = 0;
    while (host.cpu().state().pc != returned) {
      require(steps - start < maximum_steps, "Original BDC byte helper must return");
      step();
    }
    const auto result = host.cpu().state().d & byte_mask;
    require(host.cpu().state().sp == host_stack && cursor == expected.size(),
            "Native helper restores its real return frame and sends the complete original frame");
    for (unsigned wait = 0; wire.state().perceived || wire.state().service_due; ++wait) {
      require(wait < 32 && host.cpu().state().pc == returned,
              "Only the actual own idle BRA advances the remaining reference bit tail");
      step();
    }
    return result;
  }

  void pair(const unsigned value) {
    host.bus().write_memory({.address = host_stack, .size = 2, .value = data_address});
    expected = {write_opcode, 2, 0, value};
    static_cast<void>(call({.entry = write_entry, .argument = value}));
    require(target.memory.known[data_address] && target.memory.bytes[data_address] == value,
            "Actual original C0 GPIO waveform writes the independently supplied target byte");
    const auto distinct = static_cast<std::uint8_t>(value ^ byte_mask);
    target.memory.install(reply_address, std::span(&distinct, 1));
    expected = {read_opcode, 2, 1};
    require(call({.entry = read_entry, .argument = reply_address}) == distinct,
            "Actual original E0/read GPIO waveform receives independently seeded target data");
    trace << "PAIR value=" << value << " reply=" << static_cast<unsigned>(distinct)
          << " commands=" << bdc.state().serviced << " rx=" << wire.state().received_bytes
          << " tx=" << wire.state().transmitted_bytes << " steps=" << steps << '\n';
  }
};
} // namespace blofeld::test::keyboard_bdc_wire
