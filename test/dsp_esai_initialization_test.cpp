// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_esai_registers.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::esai;
constexpr auto reference = DspEsaiConfigurationContract::cold_unclocked_reference;
constexpr auto documented = DspEsaiConfigurationContract::documented_reset;
constexpr unsigned generated = 0x600301;
constexpr unsigned clock_register = 22;
constexpr unsigned format32 = 0x7d00;
constexpr unsigned invalid_format = 0x2c00;
constexpr unsigned reserved_clock = 0x800000;
constexpr unsigned direction_offset = 30;
constexpr unsigned mux_offset = 31;

void strict_default(const DspEsaiInstance instance) {
  DspEsai serial(instance);
  serial.set_active(true);
  static_cast<void>(rejects([&] { serial.write(clock_register, generated); }));
  static_cast<void>(rejects([&] { serial.write(control, normal24); }));
  static_cast<void>(rejects([&] { serial.write(common, esai::alignment); }));
  require(serial.read(clock_register) == 0 && serial.read(control) == 0 && serial.read(common) == 0,
          "Default selected-pin configuration remains strict and transactional");
  static_cast<void>(rejects([&] { serial.set_configuration_contract(reference); }));
  serial.set_active(false);
  serial.set_configuration_contract(reference);
  serial.set_configuration_contract(documented);
  serial.set_active(true);
  static_cast<void>(rejects([&] { serial.write(clock_register, generated); }));
}

void cold_order(const DspEsaiInstance instance, const bool pins_first) {
  DspEsai serial(instance);
  serial.set_configuration_contract(reference);
  if (pins_first) { serial.set_active(true); }
  serial.write(clock_register, generated);
  serial.write(control, format32);
  serial.write(common, 0x40);
  if (!pins_first) { serial.set_active(true); }
  require(serial.read(clock_register) == generated && serial.read(control) == format32 &&
              serial.read(common) == 0x40 && serial.read(status) == 0,
          "Explicit unclocked reference retains disabled clock/format registers in either order");
  require(serial.frames_started() == 0 && serial.slots_started() == 0 &&
              serial.outputs().driven == 0 && !serial.interrupt_vector(),
          "Cold programming invents no transfer, data or interrupt");
  static_cast<void>(rejects([&] { static_cast<void>(serial.clock_outputs()); }));
  static_cast<void>(rejects([&] { serial.observe({}); }));
  serial.write(control, format32 | 1U);
  require(serial.read(control) == (format32 | 1U),
          "Separate TE enable retains the programmed word format");
  static_cast<void>(rejects([&] { static_cast<void>(serial.read(status)); }));
  static_cast<void>(rejects([&] { static_cast<void>(serial.outputs()); }));
  static_cast<void>(
      rejects([&] { serial.write(clock_register, generated ^ esai::clock_polarity); }));
  serial.write(control, format32);
  static_cast<void>(rejects([&] { serial.write(clock_register, 0); }));
  require(serial.read(clock_register) == generated,
          "TE disable does not reopen the cold configuration window");
}

void cold_guards(const DspEsaiInstance instance) {
  DspEsai serial(instance);
  serial.set_configuration_contract(reference);
  serial.set_active(true);
  static_cast<void>(rejects([&] { serial.write(clock_register, reserved_clock); }));
  static_cast<void>(rejects([&] { serial.write(control, invalid_format); }));
  static_cast<void>(rejects([&] { serial.write(common, 1); }));
  static_cast<void>(rejects([&] { serial.write(esai::rcr, 1); }));
  static_cast<void>(rejects([&] { serial.write(control, normal24 | 1U); }));
  require(serial.read(control) == 0 && serial.read(clock_register) == 0 && serial.read(common) == 0,
          "Cold reference keeps reserved/unsupported fields and combined format/TE writes strict");
  serial.write(control, normal24);
  serial.write(0, input);
  serial.write(control, normal24 | 1U);
  baseline(serial);
  require(slot(serial)[0] == "101001011100001110011111" && serial.read(status) == empty_even_frame,
          "Only supplied frame/bit edges shift the independently expected primed word");
  static_cast<void>(rejects([&] { serial.write(common, esai::alignment); }));
  serial.write(control, normal24);
  serial.set_active(false);
  static_cast<void>(rejects([&] { serial.set_configuration_contract(reference); }));
  serial.set_active(true);
  static_cast<void>(rejects([&] { serial.write(control, normal24 | esai::shift_lsb); }));
  require(serial.frames_started() == 1 && serial.slots_started() == 1,
          "Disable and mux reset cannot erase prior transfer history to reopen the reference");
  serial.write(control, normal24 | esai::personal_reset);
  serial.write(clock_register, esai::clock_polarity);
  require(serial.read(clock_register) == esai::clock_polarity,
          "Documented personal reset still permits configuration after a real transfer");
}

void observed_inputs(const DspEsaiInstance instance) {
  constexpr std::array<DspEsaiInputs, 3> sources{DspEsaiInputs{.clock = false},
                                                 DspEsaiInputs{.frame_sync = false},
                                                 DspEsaiInputs{.high_clock = false}};
  for (const auto inputs : sources) {
    for (const bool before_mux : {false, true}) {
      DspEsai serial(instance);
      serial.set_configuration_contract(reference);
      if (!before_mux) { serial.set_active(true); }
      serial.observe(inputs);
      if (before_mux) { serial.set_active(true); }
      static_cast<void>(rejects([&] { serial.write(clock_register, generated); }));
      static_cast<void>(rejects([&] { serial.write(control, normal24); }));
      require(serial.read(clock_register) == 0 && serial.read(control) == 0,
              "Even a held known input closes the unclocked reference window on selected pins");
    }
  }
}

void copied_and_reset(const DspEsaiInstance instance) {
  DspEsai serial(instance);
  serial.set_configuration_contract(reference);
  serial.set_active(true);
  serial.write(control, normal24);
  auto copied = serial;
  copied.write(common, 0x40);
  require(serial.read(common) == 0 && copied.read(common) == 0x40,
          "Copy retains the explicit cold contract without sharing registers");
  copied.write(control, normal24 | 1U);
  serial.write(clock_register, esai::clock_polarity);
  copied.reset();
  copied.set_active(true);
  copied.write(clock_register, generated);
  require(copied.read(clock_register) == generated && copied.read(control) == 0,
          "Hardware/software reset reopens cold programming and retains the selected contract");
  copied.set_active(false);
  copied.set_configuration_contract(documented);
  copied.reset();
  copied.set_active(true);
  static_cast<void>(rejects([&] { copied.write(clock_register, generated); }));
}

void bus_profiles() {
  for (const auto instance : {DspEsaiInstance::primary, DspEsaiInstance::secondary}) {
    DspBus bus;
    bus.set_esai_configuration_contract(instance, reference);
    const bool primary = instance == DspEsaiInstance::primary;
    const auto space = primary ? DspSpace::x : DspSpace::y;
    const unsigned base = primary ? 0xffffa0 : 0xffff80;
    bus.write({.space = space, .address = base + direction_offset, .value = 8});
    bus.write({.space = space, .address = base + mux_offset, .value = 8});
    bus.write({.space = space, .address = base + clock_register, .value = generated});
    require(bus.read({.space = space, .address = base + clock_register}) == generated,
            "Both real register maps honor the explicit instance-specific cold contract");
    static_cast<void>(rejects([&] { bus.set_esai_configuration_contract(instance, documented); }));
    static_cast<void>(rejects([&] { static_cast<void>(bus.esai_clock_outputs(instance)); }));
    bus.reset_peripherals();
    bus.write({.space = space, .address = base + direction_offset, .value = 8});
    bus.write({.space = space, .address = base + mux_offset, .value = 8});
    bus.write({.space = space, .address = base + clock_register, .value = generated});
    require(bus.read({.space = space, .address = base + clock_register}) == generated,
            "Bus peripheral reset retains the instance contract while resetting GPIO selection");
  }
}
}

int main() {
  try {
    for (const auto instance : {DspEsaiInstance::primary, DspEsaiInstance::secondary}) {
      strict_default(instance);
      for (const bool pins_first : {false, true}) { cold_order(instance, pins_first); }
      cold_guards(instance);
      observed_inputs(instance);
      copied_and_reset(instance);
    }
    bus_profiles();
    std::cout << "ESAI explicit cold/unclocked configuration reference passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
