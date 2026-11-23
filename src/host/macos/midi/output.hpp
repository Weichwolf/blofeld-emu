// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <memory>
#include <span>

namespace blofeld::host::midi {
class Output {
public:
  explicit Output(std::int32_t destination);
  ~Output();
  Output(const Output &) = delete;
  Output &operator=(const Output &) = delete;

  void send(std::span<const std::uint32_t> words, std::uint64_t host_ns);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
