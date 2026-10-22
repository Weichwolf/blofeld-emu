// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/firmware.hpp"
#include "blofeld/mcu_bus.hpp"
#include "gpio_parameter_caller.hpp"
#include "original_gpio_notification_fixture.hpp"
#include "original_gpio_pair_fixture.hpp"

#include <exception>
#include <fstream>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::gpio_notification;

int execute(const int argc, char **argv) {
  require(argc == 4, "Notification test needs original archive/report/trace");
  const auto firmware = Firmware::load(argv[1]);
  std::ofstream report(argv[2]);
  std::ofstream trace(argv[3]);
  require(report.is_open() && trace.is_open(), "Notification outputs must open");
  report.exceptions(std::ios::badbit | std::ios::failbit);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  for (const unsigned policy : {1U, 3U}) {
    for (const bool reject : {false, true}) {
      const Input input{.policy = policy, .reject = reject};
      const auto values = input.values();
      const auto code =
          blofeld::test::gpio_parameter::caller_program(input.sound(), values, result_base);
      for (const unsigned order : {0U, 1U}) {
        trace << "CASE policy=" << policy << " reject=" << reject << " order=" << order << '\n';
        Run run(firmware, input, trace);
        const auto nvm = run.pair.mcu.bus().power_off_flash_image();
        run.pair.verify_retained(firmware, nvm);
        run.install(code, order);
        run.execute();
        run.verify(code);
        run.pair.verify_retained(firmware, nvm);
        report << "CASE policy=" << policy << " reject=" << reject << " order=" << order
               << " writes=3 decisions=" << run.decisions << " stores=" << run.stores
               << " policy-reads=" << run.policy_reads << " sender-calls=" << run.sender_calls
               << " uart-calls=" << run.queue_calls << " sci-irqs=" << run.sci_irqs
               << " bytes=" << run.wire.bytes().size() << " produced=" << run.produced
               << " consumed=" << run.consumed << " queries=" << run.pair.link.pad_queries
               << " unknown-queries=" << run.pair.link.unresolved_queries
               << " unresolved-events=" << run.pair.link.unresolved_events
               << " cycles=" << run.pair.mcu.run_clock().bus_cycles()
               << " program-unchanged=1 nvm-unchanged=1 caller-unchanged=1 cache-valid=1 "
                  "ring-valid=1 board-rejected=1\n";
      }
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
