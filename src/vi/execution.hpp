// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <exception>
#include <optional>

namespace blofeld::vi {
enum class Access : std::uint8_t { read, write, fetch, bit, execute, reset };

struct Failure {
  struct MediaAddress {
    std::uint32_t page = 0, column = 0;
  };

  Access operation = Access::execute;
  std::uint32_t pc = 0, address = 0, value = 0, code = 0;
  char space = '-';
  const char *reason = "";
  std::optional<MediaAddress> media = std::nullopt;
};

// Published only after an execution owner exits. No live Core/state inspection.
struct ExecutionReport {
  std::optional<Failure> failure;
  std::exception_ptr exception;
  std::optional<std::uint32_t> stopped_pc;
  std::uint64_t instructions = 0;
  std::uint64_t preparation_nanoseconds = 0, prepared_native_bytes = 0;
  std::optional<Failure> preparation_diagnostic;
};
}
