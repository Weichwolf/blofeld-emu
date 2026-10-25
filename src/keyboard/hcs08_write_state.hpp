// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace blofeld::detail::hcs08 {
struct PendingNativeWrite {
  MemoryWrite access;
  unsigned completed = 0;
};

struct NativeTemporary {
  std::uint64_t address;
  std::vector<std::uint8_t> bytes;
};

// Trusted in-process state; native operation/space pointers are reconstructed.
struct NativeInstructionState {
  std::uint64_t address, next;
  std::size_t index;
  unsigned steps;
  PendingNativeWrite write;
  std::vector<NativeTemporary> temporaries;
};
}
