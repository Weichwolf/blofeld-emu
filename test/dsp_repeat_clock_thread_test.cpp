// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_norm_fixture.hpp"
#include "dsp_repeat_clock_fixture.hpp"

#include <array>
#include <barrier>
#include <cstddef>
#include <exception>
#include <iostream>
#include <thread>

int main() {
  try {
    constexpr unsigned owners = 2;
    std::array<std::exception_ptr, owners> failures{};
    std::array<unsigned, owners> suffixes{};
    std::barrier cold_start(static_cast<std::ptrdiff_t>(owners));
    {
      std::array<std::jthread, owners> workers;
      for (std::size_t owner = 0; owner < workers.size(); ++owner) {
        workers.at(owner) = std::jthread([&, owner] {
          try {
            cold_start.arrive_and_wait();
            for (unsigned iteration = 0; iteration < 2; ++iteration) {
              suffixes.at(owner) += blofeld::test::repeat_clock::one(
                  {.count = 3, .memory = true, .reg = true, .checkpoints = true});
              suffixes.at(owner) +=
                  blofeld::test::repeat_clock::one({.count = 2, .checkpoints = true});
            }
          } catch (...) { failures.at(owner) = std::current_exception(); }
        });
      }
    }
    for (const auto &failure : failures) {
      if (failure) { std::rethrow_exception(failure); }
    }
    constexpr unsigned expected_suffixes = 124;
    blofeld::test::require(suffixes.front() + suffixes.back() == expected_suffixes,
                           "Both cold owners must run the actual shared REP checkpoint oracle");
    std::cout << "2 cold owners,8 REP streams and124 joint checkpoint suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
