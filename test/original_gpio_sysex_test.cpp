// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/firmware.hpp"
#include "gpio_parameter_caller.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "original_gpio_sysex_fixture.hpp"

#include <array>
#include <exception>
#include <fstream>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::gpio_sysex;
constexpr unsigned first_device = 17, second_device = 42;
constexpr std::array<std::array<unsigned, 2>, 5> profiles{
    {{0, 70}, {1, 70}, {2, 69}, {2, 70}, {3, 70}}};

int execute(const int argc, char **argv) {
  require(argc == 4, "Original SysEx test needs archive/report/trace");
  const auto firmware = Firmware::load(argv[1]);
  std::ofstream report(argv[2]);
  std::ofstream trace(argv[3]);
  require(report.is_open() && trace.is_open(), "Original SysEx outputs must open");
  report.exceptions(std::ios::badbit | std::ios::failbit);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  for (const auto [policy, parameter] : profiles) {
    for (const unsigned order : {0U, 1U}) {
      const Input input{.policy = policy,
                        .parameter = parameter,
                        .device = order == 0 ? first_device : second_device};
      constexpr std::array<unsigned, 1> values{incoming};
      const auto code =
          blofeld::test::gpio_parameter::caller_program(input.sound(initial), values, result);
      trace << "CASE policy=" << policy << " parameter=" << parameter << " order=" << order << '\n';
      Run run(firmware, input, trace);
      const auto nvm = run.pair.mcu.bus().power_off_flash_image();
      run.pair.verify_retained(firmware, nvm);
      run.install(order, code);
      run.execute();
      run.verify(code);
      run.pair.verify_retained(firmware, nvm);
      report << "CASE policy=" << policy << " parameter=" << parameter << " order=" << order
             << " device=" << input.device << " stores=" << run.stores
             << " sources=" << run.source_checks << " senders=" << run.send_calls
             << " mappers=" << run.mapper_calls << " uart-calls=" << run.uart_calls
             << " usb-events=" << run.usb_calls << " usb-packets=" << run.packets
             << " sci-irqs=" << run.sci_irqs << " usb-irqs=" << run.usb_irqs
             << " raw-bytes=" << run.raw.size() << " event-bytes=" << run.events.size()
             << " produced=" << run.produced << " consumed=" << run.consumed
             << " queries=" << run.pair.link.pad_queries
             << " unknown-queries=" << run.pair.link.unresolved_queries
             << " unresolved-events=" << run.pair.link.unresolved_events
             << " nvm-unchanged=1 program-unchanged=1 caller-unchanged=1 identity-valid=1 "
                "rings-valid=1 final-nak=1 board-rejected=1\n";
    }
  }
  return 0;
}
}

int main(const int argc, char **argv) {
  try {
    return execute(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
