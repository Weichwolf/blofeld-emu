// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <bits/types/sigset_t.h>
#include <chrono>

namespace blofeld::host {
// Construct before starting workers; they inherit the blocked termination mask.
// The application consumes signals on its control thread, outside audio I/O.
class Signals {
public:
  Signals();
  ~Signals();
  Signals(const Signals &) = delete;
  Signals &operator=(const Signals &) = delete;
  [[nodiscard]] bool wait(std::chrono::milliseconds timeout) const;

private:
  sigset_t previous_{};
  int descriptor_ = -1;
};
}
