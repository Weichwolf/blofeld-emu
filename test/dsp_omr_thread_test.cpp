// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_omr_clock_fixture.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <thread>

namespace {
constexpr std::size_t workers = 2;
constexpr unsigned iterations = 8;

void concurrent_restores() {
  std::array<std::exception_ptr, workers> failures{};
  std::array<std::jthread, workers> threads;
  for (std::size_t slot = 0; slot < workers; ++slot) {
    threads.at(slot) = std::jthread([&, slot] {
      try {
        for (unsigned iteration = 0; iteration < iterations; ++iteration) {
          blofeld::test::omr_clock::restores();
        }
      } catch (...) { failures.at(slot) = std::current_exception(); }
    });
  }
  for (auto &thread : threads) { thread.join(); }
  for (const auto &failure : failures) {
    if (failure) { std::rethrow_exception(failure); }
  }
}
}

int main() {
  try {
    concurrent_restores();
    std::cout << "Two independent owners completed 224 OMR checkpoint suffixes concurrently\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
