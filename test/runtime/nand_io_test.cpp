// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dax_registers.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "runtime/nand_peer.hpp"
#include "storage/nand.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
using namespace blofeld;

constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned common_hz = 540000000;
constexpr unsigned first_sample = 0x42;
constexpr unsigned second_sample = 0x9a;
constexpr unsigned guest_pc = 0x1234;
constexpr unsigned power_ticks = 54000;
constexpr unsigned before_power = 53999;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned inactive = 0x300;
constexpr unsigned write_strobe = 0x100;
constexpr unsigned control_mask = 0xf00;
constexpr unsigned command_select = 0x400;
constexpr unsigned address_select = 0x800;
constexpr unsigned read_strobe = 0x200;

enum class Selection : std::uint16_t { command = command_select, address = address_select };

void require(const bool value, const char *message) {
  if (!value) { throw std::runtime_error(message); }
}

template <typename Action> void rejects(Action action) {
  try {
    action();
  } catch (const Fault &) { return; }
  throw std::runtime_error("Expected a contextual peripheral fault");
}

Nand image() {
  Nand device(NandReadContinuation::k9f1208_j);
  constexpr std::array<std::uint8_t, 2> samples{first_sample, second_sample};
  device.observe_bytes(0, 0, samples);
  device.observe_wp(true);
  return device;
}

struct Device {
  DspMachine dsp{{.extal_hz = oscillator_hz,
                  .timebase_hz = common_hz,
                  .switch_timing = DspClockSwitch::functional_region_boundary}};
  NandPeer nand{dsp, image()};
  NandPeerEvents events = nand.events();
  std::uint64_t now = 0;

  void write(const DspPort port, const DspGpioRegister reg, const unsigned value) {
    now += 1;
    nand.before_native(now);
    dsp.bus().gpio(port).write({.reg = reg, .value = value});
    nand.after_native(now, guest_pc);
  }

  void control(const unsigned value) { write(DspPort::c, DspGpioRegister::data, value); }

  void data(const unsigned value) { write(DspPort::f, DspGpioRegister::data, value); }

  void latch(const Selection selection, const unsigned byte) {
    const auto qualifier = static_cast<unsigned>(selection);
    data(byte);
    control(qualifier | inactive);
    control(qualifier | write_strobe);
    control(qualifier | inactive);
  }

  void complete() {
    const auto next = events.next_ticks();
    if (!next) { throw std::runtime_error("A pending operation must own a completion"); }
    now = *next;
    events.observe(now);
  }

  void prepare() {
    require(events.next_ticks() == power_ticks, "Power delay scales with the instrument timeline");
    rejects([&] { events.observe(before_power); });
    require(events.next_ticks() == power_ticks, "A rejected event leaves the deadline unchanged");
    complete();
    for (const auto port : {DspPort::c, DspPort::f}) {
      dsp.bus().gpio(port).set_contract(DspGpioContract::logical_pins);
      write(port, DspGpioRegister::control, 0);
    }
    control(inactive);
    write(DspPort::c, DspGpioRegister::direction, control_mask);
    write(DspPort::f, DspGpioRegister::direction, byte_mask);
  }
};

void transactions() {
  Device device;
  device.prepare();
  device.latch(Selection::command, 0);
  for (unsigned index = 0; index < 4; ++index) { device.latch(Selection::address, 0); }
  require(device.nand.protocol_copy().ready() == false, "Array read remains busy until completion");
  device.complete();
  device.control(inactive);
  device.write(DspPort::f, DspGpioRegister::direction, 0);
  device.write(DspPort::f, DspGpioRegister::control, byte_mask);
  device.control(read_strobe);
  require(device.dsp.bus().gpio(DspPort::f).read(DspGpioRegister::data, byte_mask) == first_sample,
          "First firmware read returns the original stored byte");
  require(device.dsp.bus().gpio(DspPort::f).read(DspGpioRegister::data, byte_mask) == first_sample,
          "Repeated register reads do not consume another NAND byte");
  device.control(inactive);
  device.control(read_strobe);
  require(device.dsp.bus().gpio(DspPort::f).read(DspGpioRegister::data, byte_mask) == second_sample,
          "A new firmware transaction advances the array cursor once");
  device.control(inactive);
  device.write(DspPort::f, DspGpioRegister::control, 0);
  device.write(DspPort::f, DspGpioRegister::direction, byte_mask);
  device.latch(Selection::command, byte_mask);
  require(device.nand.protocol_copy().pending() == NandPending::reset,
          "The firmware reset command schedules a real protocol operation");
  device.complete();
  require(device.nand.protocol_copy().ready() == true, "Reset completion publishes Ready");
}

void io_changes() {
  DspMachine dsp({.extal_hz = oscillator_hz,
                  .timebase_hz = common_hz,
                  .switch_timing = DspClockSwitch::functional_region_boundary});
  auto &bus = dsp.bus();
  constexpr unsigned data_register = 0xffffbd;
  bus.set_write_contract(DspWriteContract::functional_decode);
  bus.gpio(DspPort::c).set_contract(DspGpioContract::logical_pins);
  const auto revision = bus.io_revision();
  bus.write({.space = DspSpace::x, .address = 0, .value = first_sample});
  require(bus.io_revision() == revision, "Ordinary RAM does not publish device effects");
  bus.write({.space = DspSpace::x, .address = dax::non_audio, .value = 0});
  require(bus.io_revision() == revision,
          "Audio register writes do not resample unrelated interprocessor or NAND I/O");
  bus.write({.space = DspSpace::x, .address = data_register, .value = inactive});
  require(bus.io_revision() != revision, "CPU and DMA register stores share effect publication");
  const auto after_write = bus.io_revision();
  static_cast<void>(bus.access_peripheral_bit({.space = DspSpace::x,
                                               .address = data_register,
                                               .bit = 8,
                                               .operation = DspBitOperation::test}));
  require(bus.io_revision() == after_write, "Read-only bit tests do not report register writes");
  static_cast<void>(bus.access_peripheral_bit({.space = DspSpace::x,
                                               .address = data_register,
                                               .bit = 8,
                                               .operation = DspBitOperation::clear}));
  require(bus.io_revision() != after_write &&
              bus.gpio(DspPort::c).read(DspGpioRegister::data, inactive) == read_strobe,
          "A register bit write publishes the actual changed data");
  const auto before_reset = bus.io_revision();
  bus.reset_peripherals();
  require(bus.io_revision() != before_reset, "Peripheral reset also withdraws old I/O state");
}

void lifetimes() {
  DspMachine dsp({.extal_hz = oscillator_hz,
                  .timebase_hz = common_hz,
                  .switch_timing = DspClockSwitch::functional_region_boundary});
  auto peer = std::make_unique<NandPeer>(dsp, image());
  const auto events = peer->events();
  peer.reset();
  rejects([&] { static_cast<void>(events.next_ticks()); });
  rejects([&] { static_cast<void>(dsp.bus().gpio(DspPort::f).observe_pads(byte_mask)); });
}
}

int main() {
  try {
    transactions();
    lifetimes();
    io_changes();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
