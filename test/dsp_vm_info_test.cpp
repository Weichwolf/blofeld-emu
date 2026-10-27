// SPDX-License-Identifier: GPL-3.0-only
#include <array>
#include <asmjit/core/virtmem.h>
#include <barrier>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace {
constexpr std::size_t workers = 16;
constexpr unsigned repeated_reads = 2048;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void cold_cache() {
  const auto host_page = ::getpagesize();
  require(host_page > 0, "OS must supply its actual page size");
  const auto expected_page = static_cast<std::uint32_t>(host_page);
  std::array<asmjit::VirtMem::Info, workers> initial{};
  std::array<std::exception_ptr, workers> failures{};
  std::barrier start(static_cast<std::ptrdiff_t>(workers));
  {
    std::array<std::jthread, workers> threads;
    for (std::size_t owner = 0; owner < workers; ++owner) {
      threads.at(owner) = std::jthread([&, owner] {
        start.arrive_and_wait();
        try {
          // No caller has entered VirtMem::info before the simultaneous cold calls.
          const auto first = asmjit::VirtMem::info();
          require(first.pageSize == expected_page && first.pageGranularity >= expected_page &&
                      first.pageGranularity % expected_page == 0,
                  "Cold native VM information must match OS pages and valid allocation alignment");
          initial.at(owner) = first;
          for (unsigned iteration = 0; iteration < repeated_reads; ++iteration) {
            const auto next = asmjit::VirtMem::info();
            require(next.pageSize == first.pageSize &&
                        next.pageGranularity == first.pageGranularity,
                    "Published native VM information must remain immutable for every caller");
          }
        } catch (...) { failures.at(owner) = std::current_exception(); }
      });
    }
  }
  for (const auto &failure : failures) {
    if (failure) { std::rethrow_exception(failure); }
  }
  for (const auto value : initial) {
    require(value.pageSize == expected_page &&
                value.pageGranularity == initial.front().pageGranularity,
            "Every cold owner must observe complete native VM information");
  }
}
} // namespace

int main() {
  try {
    cold_cache();
    std::cout << "Sixteen cold callers passed native OS page checks and32768 immutable reads.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
