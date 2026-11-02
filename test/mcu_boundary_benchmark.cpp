// SPDX-License-Identifier: GPL-3.0-only
#include "board/detail/mcu_dsp_boundary.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned oscillator = 12000000, iterations = 200000;

void measure(const std::string_view label, const auto &read) {
  const auto expected = read();
  unsigned mismatches = 0;
  const auto start = std::chrono::steady_clock::now();
  for (unsigned index = 0; index < iterations; ++index) {
    mismatches += read() == expected ? 0U : 1U;
  }
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start);
  test::require(mismatches == 0 && elapsed.count() > 0,
                "Every full observed boundary must remain identical during measurement");
  std::cout << R"({"case":")" << label << R"(","queries":)" << iterations
            << R"(,"host_nanoseconds":)" << elapsed.count() << "}\n";
}
}

int main() {
  try {
    const McuRunContract contract{
        .clock = {.oscillator_hz = oscillator,
                  .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
        .memory_timing = McuRunMemory::unstretched_internal};
    McuMachine mcu(test::checkpoint_image(), BLOFELD_DEFAULT_SLA, contract);
    test::drive_interrupts(mcu.bus());
    const auto semantic = detail::mcu_boundary(mcu);
    const auto stamp = mcu.cpu().inspection_stamp();
    const auto full = [&] { return detail::mcu_boundary(mcu); };
    const auto integrity = [&] { return detail::mcu_integrity(mcu); };
    measure("semantic-1", full);
    measure("integrity-1", integrity);
    measure("integrity-2", integrity);
    measure("semantic-2", full);
    test::require(detail::mcu_boundary(mcu) == semantic && mcu.cpu().inspection_stamp() == stamp,
                  "Measurement preserves every architectural register, clock and integrity stamp");
    std::cout << "Read-only query microbenchmark only; no full-emulator or real-time claim.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
