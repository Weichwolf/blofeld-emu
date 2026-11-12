// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <thread>

namespace blofeld::host::diagnostics {
struct ThreadUsage {
  struct Level {
    std::uint64_t cpu_nanoseconds = 0, instructions = 0, cycles = 0;
  };

  static constexpr unsigned maximum_levels = 8;
  std::array<Level, maximum_levels> levels{};
  std::uint64_t identifier = 0;
  unsigned count = 0;
  int error = 0;
  bool pmc = false;
};

class ThreadUsageReader {
public:
  struct ThreadId {
    std::uint64_t identifier = 0;
    std::int64_t clock = 0;
    int error = 0;
  };

  ThreadUsageReader();
  [[nodiscard]] ThreadUsage read(ThreadId thread) const noexcept;
  [[nodiscard]] static ThreadId identify(std::thread::native_handle_type thread) noexcept;
  [[nodiscard]] static ThreadId current() noexcept;
  [[nodiscard]] static int name_current(const char *name) noexcept;

private:
#if defined(__APPLE__)
  unsigned levels_ = 1;
  std::uint32_t numerator_ = 1, denominator_ = 1;
#endif
};
}
