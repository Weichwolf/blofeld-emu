// SPDX-License-Identifier: GPL-3.0-only
#include "dax_fixture.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <utility>

namespace blofeld::test::dax_test {
namespace {
void port_d_reset() {
  using Register = DspGpioRegister;
  DspGpio port(DspPort::d);
  require(port.read(Register::control) == 0 && port.read(Register::direction) == 0 &&
              port.read(Register::data) == 0 && port.observe_pads(3).known == 0,
          "Port D reset clears PDRD without qualifying disconnected physical pin voltages");
  for (unsigned bit = 1; bit <= 2; bit <<= 1U) {
    port.reset();
    port.write({.reg = Register::direction, .value = bit});
    require(port.outputs().known == bit && port.read(Register::data, bit) == 0,
            "Unlike Ports C/E/F, the Port D reset output latch has a documented zero value");
    port.write({.value = bit});
    require(port.read(Register::data, bit) == bit, "Port D GPIO writes drive a real selected pin");
    port.reset();
    require(port.read(Register::data) == 0, "Software reset clears a previously high Port D latch");
    port.write({.reg = Register::control, .value = bit});
    static_cast<void>(rejects([&] { static_cast<void>(port.read(Register::data, bit)); }));
    port.set_drive({.mask = bit, .levels = bit});
    require(port.read(Register::data, bit) == bit,
            "Port D input requires an explicit external level");
    port.set_drive({});
    port.write({.reg = Register::direction, .value = bit});
    require(port.outputs().peripheral == bit, "PDC=1/PC=1 selects DAX independently per pin");
  }
  static_cast<void>(rejects([&] { port.write({.reg = Register::control, .value = 4}); }));

  DspBus bus;
  require(bus.read({.space = DspSpace::x, .address = port_data}) == 0,
          "The actual X-space PDRD map has its independent documented reset");
  bus.write({.space = DspSpace::x, .address = port_direction, .value = 3});
  bus.write({.space = DspSpace::x, .address = port_data, .value = 3});
  bus.reset_peripherals();
  require(bus.read({.space = DspSpace::x, .address = port_data}) == 0 &&
              bus.gpio(DspPort::c).outputs().mask == 0 && bus.gpio(DspPort::e).outputs().mask == 0,
          "Bus reset clears Port D data without aliasing ESAI GPIO banks");
}

void pad_contract() {
  DspBus bus;
  const auto put = [&](const unsigned address, const unsigned value) {
    bus.write({.space = DspSpace::x, .address = address, .value = value});
  };
  put(control, 8);
  put(non_audio, 0);
  put(audio_a, first_sample);
  put(audio_a, other_sample);
  put(port_direction, 2);
  put(port_control, 2);
  require(rejects([&] { bus.observe_dax(inputs(false, 1)); }).contains("ACI"),
          "An external source requires the real ACI peripheral mux selection");
  put(port_direction, 3);
  put(port_control, 3);
  auto slow_core = inputs(false, 1);
  constexpr unsigned inadequate_hz = 1000000;
  slow_core.rates.core_hz = inadequate_hz;
  static_cast<void>(rejects([&] { bus.observe_dax(slow_core); }));
  bus.observe_dax(inputs(false, 1));
  require(bus.dax_output() == DspDaxOutput{.driven = true, .level = true},
          "The declared reference starts ADO high before its first generated preamble unit");
  bus.gpio(DspPort::d).set_drive({.mask = 2, .levels = 2});
  bus.observe_dax(inputs(true, 1));
  bus.observe_dax(inputs(false, 1));
  require(rejects([&] { bus.observe_dax(inputs(true, 1)); }).contains("contention"),
          "An opposed ADO driver rejects the exact first generated unit");
  const auto pending = bus.access_peripheral_bit(
      {.space = DspSpace::x, .address = status, .bit = 0, .operation = DspBitOperation::test});
  require(pending && !*pending, "Rejected output contention consumed neither upload nor XADE");
  bus.gpio(DspPort::d).set_drive({});
  bus.observe_dax(inputs(true, 1));
  require(bus.dax_output() == DspDaxOutput{.driven = true} &&
              bus.access_peripheral_bit({.space = DspSpace::x,
                                         .address = status,
                                         .bit = 0,
                                         .operation = DspBitOperation::test}) == true,
          "The same released edge commits one actual Z bit and a real empty request");
  auto changed = inputs(false, 1);
  changed.rates.core_hz = rates().core_hz;
  static_cast<void>(rejects([&] { bus.observe_dax(changed); }));
  static_cast<void>(rejects([&] { put(control, 0); }));

  put(port_control, 1);
  require(!bus.dax_output().driven,
          "Selecting only ACI keeps the internal DAX active but leaves ADO unselected");
}

void native_clock_failure() {
  DspMachine machine(dma_test::native_clocks);
  dma_test::NativeProgram code;
  for (const auto [address, value] : {std::pair{control, 0U},
                                      std::pair{non_audio, 0U},
                                      std::pair{audio_a, first_sample},
                                      std::pair{audio_b, other_sample},
                                      std::pair{port_direction, 2U},
                                      std::pair{port_control, 2U}}) {
    code.write({.space = DspSpace::x, .address = address, .value = value});
  }
  // FM MOVEP high-pp XSTR->X0; the literal read must reach the actual DAX.
  constexpr unsigned read_status_x0 = 0x084414;
  code.word(read_status_x0);
  dma_test::reset_native(machine, code);
  for (unsigned i = 0; i < 6; ++i) { dma_test::step_native(machine); }
  const auto error = rejects([&] { dma_test::step_native(machine); });
  require(error.contains("0xff000c") && error.contains("register read at X:0xffffd4") &&
              error.contains("clock"),
          "Actual native status access reports the PC, operation/address and missing clock");
  const auto failure = machine.io().failure();
  require(failure && failure->operation == DspIo::Operation::read && failure->address == status,
          "The real XSTR read latches its exact native I/O failure");
  const auto stopped = machine.cpu().registers();
  const auto progress = machine.cpu().state().progress;
  // The external device may keep observing clocks; a native I/O fault cannot resume.
  machine.bus().observe_dax(inputs(false));
  static_cast<void>(rejects([&] { dma_test::step_native(machine); }));
  require(machine.cpu().registers() == stopped && machine.cpu().state().progress == progress &&
              machine.io().failure(),
          "A genuine later clock observation cannot clear a latched native fault");
  DspMachine recovered(dma_test::native_clocks);
  dma_test::reset_native(recovered, code);
  for (unsigned i = 0; i < 6; ++i) { dma_test::step_native(recovered); }
  recovered.bus().observe_dax(inputs(false));
  dma_test::step_native(recovered);
  require(!recovered.io().failure() && recovered.cpu().registers().x == 0 &&
              recovered.cpu().state().progress.estimated_cycles == 13 &&
              recovered.cpu().state().progress.instructions == 7,
          "Fresh reset and real clock input admit the status read after literal12+1 units");
}
}

void native_checks() {
  port_d_reset();
  pad_contract();
  native_clock_failure();
}
}
