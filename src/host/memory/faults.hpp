// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <array>
#include <atomic>
#include <cstdint>
extern "C" {
#include <signal.h>
}

namespace blofeld::host::memory {
// Process scope, installed before workers and released after all owners stop.
class Faults {
public:
  using Claim = bool (*)(std::uintptr_t, bool, std::uintptr_t) noexcept;
  explicit Faults(Claim claim);
  ~Faults();
  Faults(const Faults &) = delete;
  Faults &operator=(const Faults &) = delete;

private:
  static void handle(int signal, siginfo_t *info, void *context) noexcept;
  static constexpr std::array signals{SIGSEGV, SIGBUS};
  std::array<struct sigaction, 2> previous_{};
  Claim claim_;
  static std::atomic<Faults *> active_;
};
}
