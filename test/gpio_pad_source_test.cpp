// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/gpio_pads.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <utility>

namespace {
namespace reg = blofeld::hc12;
constexpr unsigned mask = 255, first = 0xa5, second = 0xaa, upper = 0xf0, lower = 0x0f;
constexpr unsigned context_pc = 0x8123, context_ccr = 0x90;
constexpr unsigned dsp_mask = 0xfff;
constexpr std::array<std::uint8_t, 32768> flash{};

void require(const bool value, const std::string_view message) {
  if (!value) { throw blofeld::Fault(std::string(message)); }
}

void rejects(const auto &operation, const std::string_view label) {
  try {
    operation();
  } catch (const blofeld::Fault &error) {
    require(std::string_view(error.what()).contains(label), "Wrong pad-source diagnostic");
    return;
  }
  throw blofeld::Fault("Missing pad-source failure");
}

void net_expectations() {
  using blofeld::resolve_gpio_net;
  const auto bias = resolve_gpio_net(mask, {.pull_down = mask}, {});
  require(bias.known == mask && bias.levels == 0 && bias.conflicts == 0,
          "Explicit passive pull-downs define idle input voltage");
  const auto driven = resolve_gpio_net(
      mask, {.active = mask, .known = mask, .levels = first, .reduced = mask}, {.pull_up = mask});
  require(driven.known == mask && driven.levels == first && driven.conflicts == 0,
          "A reduced active output still dominates a passive pull");
  const auto agreed = resolve_gpio_net(mask,
                                       {.active = mask, .known = mask, .levels = first},
                                       {.active = mask, .known = mask, .levels = first});
  require(agreed.known == mask && agreed.levels == first, "Agreeing active drivers remain known");
  const auto opposed =
      resolve_gpio_net(mask,
                       {.active = mask, .known = mask, .levels = first, .reduced = mask},
                       {.active = mask, .known = mask, .levels = second});
  require(opposed.known == (mask ^ (first ^ second)) && opposed.levels == (first & second) &&
              opposed.conflicts == (first ^ second),
          "Opposed full/reduced outputs never receive a fabricated winner");
  rejects([&] { blofeld::require_known_gpio_pads(opposed, mask); }, "conflicts=0x00000f");
  const auto unknown = resolve_gpio_net(mask, {.active = upper}, {.pull_down = mask});
  require(unknown.known == lower && unknown.levels == 0,
          "An unknown active latch defeats weak bias knowledge");
  const auto pulls = resolve_gpio_net(mask, {.pull_up = mask}, {.pull_down = lower});
  require(pulls.known == upper && pulls.levels == upper && pulls.conflicts == lower,
          "Opposing passive pulls remain unresolved");
  rejects([&] { static_cast<void>(resolve_gpio_net(mask, {.known = 1}, {})); }, "driver");
  rejects([&] { static_cast<void>(resolve_gpio_net(mask, {.reduced = 1}, {})); }, "driver");
  rejects([&] { static_cast<void>(resolve_gpio_net(mask, {.pull_up = 1, .pull_down = 1}, {})); },
          "driver");
}

void binding_lifetime() {
  blofeld::GpioPadBinding binding;
  rejects([&] { binding.attach({}); }, "Invalid");
  binding.attach([](const std::uint32_t bits) {
    return blofeld::GpioPadSample{.known = bits, .levels = bits};
  });
  rejects([&] { binding.attach([](std::uint32_t) { return blofeld::GpioPadSample{}; }); },
          "already attached");
  const auto copy = binding;
  rejects([&] { static_cast<void>(copy.sample(1)); }, "Detached");
  require(binding.sample(1).levels == 1, "Copying never steals the live board connection");
  auto moved = std::move(binding);
  rejects([&] { static_cast<void>(moved.sample(1)); }, "Detached");
  moved.attach([](const std::uint32_t bits) { return blofeld::GpioPadSample{.known = bits}; });
  require(moved.sample(1).known == 1 && moved.sample(1).levels == 0,
          "A restored owner explicitly reconnects a fresh source");
  blofeld::GpioPadBinding invalid;
  invalid.attach([](const std::uint32_t bits) {
    return blofeld::GpioPadSample{.known = bits, .conflicts = bits};
  });
  rejects([&] { static_cast<void>(invalid.sample(1)); }, "Invalid resolved");
}

void mcu_register_and_pad() {
  blofeld::McuBus bus(flash);
  bus.begin_instruction({.pc = context_pc, .ccr = context_ccr});
  bus.write(reg::ptq, first);
  bus.write(reg::ddrq, mask);
  unsigned samples = 0;
  bus.attach_gpio_pad_source(blofeld::McuPort::q, [&](const std::uint32_t bits) {
    ++samples;
    return blofeld::GpioPadSample{.conflicts = bits};
  });
  bus.write(reg::ptq, second);
  require(bus.read(reg::ptq) == second && samples == 0,
          "A known MCU output-latch write/read consumes no unknown external voltage");
  rejects([&] { static_cast<void>(bus.read(reg::ptq + 1)); }, "pc=0x8123 ccr=0x90");
  require(samples == 1, "The actual pin-input register must query the external net");
  rejects([&] { bus.set_pin_drive(blofeld::McuPort::q, {}); }, "owns MCU");
  const auto clone = bus.clone();
  rejects([&] { static_cast<void>(clone->read(reg::ptq + 1)); }, "Detached");
  require(clone->read(reg::ptq) == second, "A detached checkpoint still retains its local latch");
  bus.write(reg::perq, mask);
  bus.write(reg::ppsq, mask);
  require(bus.gpio_biases(blofeld::McuPort::q).down == 0, "Outputs disable their input pulls");
  bus.write(reg::ddrq, 0);
  require(bus.gpio_biases(blofeld::McuPort::q).down == mask,
          "Native direction release exposes the programmed passive pull-downs");
  rejects([&] { static_cast<void>(bus.read(reg::ptq)); }, "Unknown MCU GPIO input");
  bus.reset();
  const auto after_reset = samples;
  rejects([&] { static_cast<void>(bus.read(reg::ptq)); }, "Unknown MCU GPIO input");
  require(samples == after_reset + 1, "MCU reset retains the actual board connection");
}

void dsp_register_and_pad() {
  using enum blofeld::DspGpioRegister;
  blofeld::DspGpio gpio(blofeld::DspPort::c);
  gpio.write({.reg = direction, .value = dsp_mask});
  gpio.write({.reg = data, .value = first});
  unsigned samples = 0;
  gpio.attach_pad_source([&](const std::uint32_t bits) {
    ++samples;
    return blofeld::GpioPadSample{.conflicts = bits};
  });
  gpio.write({.reg = data, .value = second});
  require(gpio.outputs().levels == second && samples == 0,
          "A defined DSP latch update does not read or manufacture pad voltage");
  rejects([&] { static_cast<void>(gpio.read(data, 1)); }, "Unknown DSP GPIO data read");
  const auto observation = gpio.observe_pads(1);
  require(observation.known == 0 && observation.conflicts == 1,
          "The board observes and audits an explicitly unresolved pin");
  rejects([&] { gpio.set_drive({}); }, "owns DSP");
  auto copy = gpio;
  rejects([&] { static_cast<void>(copy.read(data, 1)); }, "Detached");
  copy.attach_pad_source([&copy](const std::uint32_t bits) {
    const auto local = copy.outputs();
    return blofeld::GpioPadSample{.known = local.known & bits, .levels = local.levels & bits};
  });
  require(copy.read(data) == second, "A reconnected clone reads its fresh isolated net");
  gpio.reset();
  const auto after_reset = samples;
  rejects([&] { static_cast<void>(gpio.read(data, 1)); }, "Unknown DSP GPIO data read");
  require(samples == after_reset + 1, "DSP reset retains the actual board connection");
  blofeld::DspGpio legacy(blofeld::DspPort::c);
  legacy.write({.reg = direction, .value = dsp_mask});
  legacy.write({.reg = data, .value = first});
  legacy.set_drive({.mask = 1, .levels = 1});
  rejects([&] { legacy.write({.reg = data, .value = second}); }, "contention");
  require(legacy.outputs().levels == first, "Legacy opposing writes still fail before commit");
}
}

int main() {
  try {
    net_expectations();
    binding_lifetime();
    mcu_register_and_pad();
    dsp_register_and_pad();
    std::cout << "GPIO local registers and externally resolved unknown pads remain distinct.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
