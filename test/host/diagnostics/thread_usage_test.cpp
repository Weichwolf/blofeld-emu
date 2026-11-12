// SPDX-License-Identifier: GPL-3.0-only
#include "diagnostics/thread_usage.hpp"

#include <atomic>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
using blofeld::host::diagnostics::ThreadUsageReader;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::uint64_t work() {
  constexpr unsigned iterations = 1000000;
  constexpr unsigned left = 13;
  constexpr unsigned right = 7;
  constexpr unsigned final = 17;
  std::uint64_t value = 1;
  for (unsigned i = 0; i < iterations; ++i) {
    value ^= value << left;
    value ^= value >> right;
    value ^= value << final;
  }
  return value;
}

void native_owner() {
  const ThreadUsageReader reader;
  std::atomic<unsigned> phase{0};
  std::uint64_t result = 0;
  std::jthread worker([&] {
    phase.store(1);
    phase.notify_one();
    phase.wait(1);
    result = work();
    phase.store(3);
    phase.notify_one();
    phase.wait(3);
  });
  phase.wait(0);
  const auto thread = ThreadUsageReader::identify(worker.native_handle());
  const auto first = reader.read(thread);
  phase.store(2);
  phase.notify_one();
  phase.wait(2);
  const auto last = reader.read(thread);
  phase.store(4);
  phase.notify_one();
  worker.join();
  static_cast<void>(reader.read(thread));
  require(first.error == 0 && last.error == 0, "Regular-user counter reads succeed");
  require(first.count > 0 && last.count == first.count,
          "Native performance-level layout is stable");
  require(result != 0, "Native workload cannot disappear");
  std::uint64_t cpu = 0;
  std::uint64_t instructions = 0;
  std::uint64_t cycles = 0;
  for (unsigned i = 0; i < first.count; ++i) {
    const auto &before = first.levels[i];
    const auto &after = last.levels[i];
    require(after.cpu_nanoseconds >= before.cpu_nanoseconds &&
                after.instructions >= before.instructions && after.cycles >= before.cycles,
            "Per-owner counters are monotonic across native work");
    cpu += after.cpu_nanoseconds - before.cpu_nanoseconds;
    instructions += after.instructions - before.instructions;
    cycles += after.cycles - before.cycles;
  }
  require(cpu > 0, "Native work consumes owner CPU time");
#if defined(__APPLE__)
  require(first.pmc && last.pmc && first.identifier != 0 && first.identifier == last.identifier,
          "Darwin PMCs identify the same native owner, without task aggregation");
  require(instructions > 0 && cycles > 0, "Actual native instructions and cycles are counted");
#endif
}
}

int main() {
  try {
    native_owner();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
