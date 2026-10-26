// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_accumulator_clock_fixture.hpp"
#include "dsp_postloop_context_fixture.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <thread>

namespace {
namespace accumulator = blofeld::test::accumulator_clock;
namespace postloop = blofeld::test::postloop_context;
constexpr std::size_t workers = 2;
constexpr unsigned iterations = 2;

void exercise() {
  accumulator::restores();
  for (const auto source : {accumulator::Source::a, accumulator::Source::b}) {
    for (unsigned scale = 0; scale < 3; ++scale) {
      accumulator::one_case({.source = source,
                             .destination = accumulator::destinations[1],
                             .scale = scale,
                             .value = accumulator::values[1]});
    }
  }
  for (const auto form :
       {postloop::Form::direct, postloop::Form::short_immediate, postloop::Form::long_immediate}) {
    postloop::restores({.form = form});
  }
}

void concurrent_owners() {
  std::array<std::exception_ptr, workers> failures{};
  std::array<std::jthread, workers> threads;
  for (std::size_t slot = 0; slot < workers; ++slot) {
    threads.at(slot) = std::jthread([&, slot] {
      try {
        for (unsigned iteration = 0; iteration < iterations; ++iteration) { exercise(); }
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
    concurrent_owners();
    std::cout << "Two cold native owners passed 696 accumulator/loop suffixes and24 streams\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
