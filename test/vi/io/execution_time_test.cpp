// SPDX-License-Identifier: GPL-3.0-only
#include "io/execution_time.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

int main() try {
  blofeld::vi::ExecutionTime time;
  constexpr std::uint64_t first = 123;
  constexpr std::uint64_t later = 456;
  if (time.mcu_wait_time() || time.dsp_origin() != 0) {
    throw std::runtime_error("Boot milestones start unobserved");
  }
  time.publish_mcu_wait(first);
  time.publish_mcu_wait(later);
  if (time.mcu_wait_time() != first) {
    throw std::runtime_error("Only the first actual MCU WAI establishes readiness");
  }
  time.release_dsp(first);
  if (time.dsp_origin() != first) { throw std::runtime_error("DSP reset release sets its origin"); }
  time.release_dsp(later);
  if (time.dsp_origin() != later || time.mcu_wait_time() != first) {
    throw std::runtime_error("A later DSP reset changes its origin while retaining MCU readiness");
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
