// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_gpio_peer.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "esai_fixture.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
namespace reg = hc12;
using Register = DspGpioRegister;
constexpr unsigned byte_mask = 0xff, pins_c = 0xfff;
constexpr unsigned entry = 0x9000, nop = 0xa7, protection = 0x7f0d, security = 0x7f0f;
constexpr unsigned oscillator = 12000000, timebase = 540000000;
constexpr unsigned first = 0xa5, second = 0x5a, c_upper = 0xa00;
constexpr unsigned ready_pin = 1U << 9U, xirq_vector = 0xfff4;
constexpr unsigned priority = 0xffffff, irqa_vector = 0x10;
constexpr auto pins_q = reg::ptq + 1;

McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  bytes[protection] = byte_mask;
  constexpr unsigned unsecured = 0xfe;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

struct Fixture {
  explicit Fixture(const McuDspGpioWiring wiring = {})
      : mcu(image(),
            BLOFELD_DEFAULT_SLA,
            McuRunContract{.clock = {.oscillator_hz = oscillator,
                                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                           .memory_timing = McuRunMemory::unstretched_internal}),
        dsp({.extal_hz = oscillator,
             .timebase_hz = timebase,
             .switch_timing = DspClockSwitch::reference_cycle_boundary}) {
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 2, .levels = 2});
    dsp.bus().gpio(DspPort::c).set_contract(DspGpioContract::logical_pins);
    dsp.bus().gpio(DspPort::e).set_contract(DspGpioContract::logical_pins);
    peer = std::make_unique<McuDspGpioPeer>(mcu, dsp, wiring);
  }

  McuMachine mcu;
  OwnDspMachine dsp;
  std::unique_ptr<McuDspGpioPeer> peer;
};

void data_directions() {
  Fixture f;
  auto &c = f.dsp.bus().gpio(DspPort::c);
  c.write({.reg = Register::control, .value = byte_mask});
  // GPIO upper outputs remain local; only the low byte crosses the peer net.
  c.write({.reg = Register::direction, .value = pins_c & ~byte_mask});
  c.write({.value = c_upper});
  f.mcu.bus().write(reg::ddrq, byte_mask);
  for (unsigned value = 0; value <= byte_mask; ++value) {
    f.mcu.bus().write(reg::ptq, static_cast<std::uint8_t>(value));
    f.peer->synchronize();
    require(c.read(Register::data) == (c_upper | value), "Every MCU byte reaches actual DSP C");
    for (unsigned bit = 0; bit < 8; ++bit) {
      const auto mask = 1U << bit;
      const auto pad = c.observe_pads(mask);
      require(pad.known == mask && pad.levels == (value & mask) && pad.conflicts == 0,
              "Partial DSP pad reads mask the resolved full-byte drivers");
    }
  }
  f.mcu.bus().write(reg::ddrq, 0);
  c.write({.reg = Register::control});
  c.write({.reg = Register::direction, .value = pins_c});
  for (unsigned value = 0; value <= byte_mask; ++value) {
    c.write({.value = c_upper | value});
    f.peer->synchronize();
    require(f.mcu.bus().read(reg::ptq) == value && c.read(Register::data) == (c_upper | value),
            "Every DSP byte reaches actual MCU Q without leaking upper C pins");
  }
  c.write({.value = c_upper | first});
  f.peer->synchronize();
  c.write({.value = c_upper | second});
  require(f.mcu.bus().read(reg::ptq) == first,
          "Readers see the published boundary, not a later unsynchronized output");
  f.peer->synchronize();
  require(f.mcu.bus().read(reg::ptq) == second, "The next boundary publishes the real new value");
}

void unknown_and_contention() {
  Fixture f;
  require(f.mcu.bus().observe_pads(McuPort::q, byte_mask).known == 0,
          "Unspecified data bias cannot invent an idle byte");
  require(rejects([&] { static_cast<void>(f.mcu.bus().read(reg::ptq)); }).contains("Unknown"),
          "Unknown peer input fails through the actual MCU register read");
  auto &c = f.dsp.bus().gpio(DspPort::c);
  c.write({.reg = Register::direction, .value = byte_mask});
  f.peer->synchronize();
  require(f.mcu.bus().observe_pads(McuPort::q, byte_mask).known == 0,
          "An uninitialized DSP output latch stays unknown");
  c.write({.value = first});
  f.mcu.bus().write(reg::ptq, second);
  f.mcu.bus().write(reg::ddrq, byte_mask);
  f.peer->synchronize();
  const auto conflict = f.mcu.bus().observe_pads(McuPort::q, byte_mask);
  require(conflict.known == 0 && conflict.conflicts == byte_mask,
          "Opposing actual outputs produce visible conflicts, never a voltage winner");
  require(rejects([&] { static_cast<void>(f.mcu.bus().read(pins_q)); }).contains("conflicts=0xff"),
          "Contention fails at the actual register access");
  require(f.mcu.bus().read(reg::ptq) == second,
          "MCU output data reads retain the stored latch while pin-input reads expose contention");
  Fixture pulled({.data_pull_down = byte_mask});
  require(pulled.mcu.bus().read(reg::ptq) == 0, "Only declared idle pulls establish an idle byte");
  pulled.dsp.bus().gpio(DspPort::c).write({.reg = Register::direction, .value = byte_mask});
  pulled.peer->synchronize();
  require(pulled.mcu.bus().observe_pads(McuPort::q, byte_mask).known == 0,
          "Unknown active drivers defeat declared idle pulls");
}

void interrupts() {
  Fixture f;
  auto &bus = f.mcu.bus();
  auto &e = f.dsp.bus().gpio(DspPort::e);
  require(!bus.interrupt_request(0), "The actual MCU reset pull supplies idle XIRQ");
  e.write({.value = ready_pin});
  e.write({.reg = Register::direction, .value = ready_pin});
  f.peer->synchronize();
  require(!bus.interrupt_request(0), "High notification does not request an interrupt");
  e.write({.value = 0});
  f.peer->synchronize();
  const auto request = bus.interrupt_request(0);
  require(request && request->vector == xirq_vector && !bus.interrupt_request(0x40),
          "Real DSP E9 reaches the actual XIRQ controller with native masking");
  e.write({.reg = Register::direction});
  f.peer->synchronize();
  require(!bus.interrupt_request(0), "Released E9 uses the actual MCU pull");
  bus.write(reg::pucr, 0);
  f.peer->synchronize();
  require(rejects([&] { static_cast<void>(bus.interrupt_request(0)); }).contains("Unknown"),
          "Undriven notification without any pull remains visibly unknown");
  f.dsp.bus().write({.space = DspSpace::x, .address = priority, .value = 6});
  bus.write(reg::ptj, 1);
  bus.write(reg::ddrj, 1);
  f.peer->synchronize();
  require(!f.dsp.bus().interrupt_request(0), "Initial high J0 does not invent an IRQA edge");
  bus.write(reg::ptj, 0);
  f.peer->synchronize();
  require(f.dsp.bus().interrupt_request(0) ==
              DspInterruptRequest{.vector = irqa_vector, .level = 1},
          "The real J0 transition latches the DSP IRQA controller");
}

void restores_and_lifetime() {
  Fixture f;
  f.mcu.bus().write(reg::ptq, first);
  f.mcu.bus().write(reg::ddrq, byte_mask);
  f.peer->synchronize();
  const auto mcu_saved = f.mcu.checkpoint();
  const auto dsp_saved = f.dsp.checkpoint();
  f.mcu.bus().write(reg::ptq, second);
  f.peer->synchronize();
  f.mcu.restore(mcu_saved);
  f.dsp.restore(dsp_saved);
  require(rejects([&] { static_cast<void>(f.mcu.bus().read(pins_q)); }).contains("Detached"),
          "Native checkpoints never retain callbacks into a former owner");
  f.peer->rebind();
  require(f.mcu.bus().read(reg::ptq) == first &&
              f.dsp.bus().gpio(DspPort::c).observe_pads(byte_mask).levels == first,
          "Trusted coordinated restore explicitly reconnects both actual chips");
  f.mcu.begin_system_reset(McuResetSource::external, 3);
  f.peer->synchronize();
  require(f.mcu.bus().observe_pads(McuPort::q, byte_mask).known == 0,
          "MCU system reset reconnects its replacement bus and removes old output direction");
  f.dsp.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  f.peer->synchronize();
  require(f.dsp.bus().gpio(DspPort::c).observe_pads(byte_mask).known == 0,
          "DSP hardware reset preserves its existing binding without a duplicate attach");
  f.peer.reset();
  require(rejects([&] {
            static_cast<void>(f.mcu.bus().observe_pads(McuPort::q, 1));
          }).contains("Stale MCU/DSP") &&
              rejects([&] {
                static_cast<void>(f.dsp.bus().gpio(DspPort::c).observe_pads(1));
              }).contains("Stale MCU/DSP"),
          "Destroyed peer callbacks fail without accessing the dead owner");
}
}

int main() {
  try {
    data_directions();
    unknown_and_contention();
    interrupts();
    restores_and_lifetime();
    std::cout << "Native MCU/DSP GPIO peer checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
