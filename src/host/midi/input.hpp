// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "midi/ump.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::host::midi {
struct Source {
  std::int32_t id = 0;
  std::string name;
};

// One selected native endpoint and group, one consumer; timestamps are monotonic nanoseconds.
class Input {
public:
  struct Route {
    std::int32_t source = 0;
    std::uint8_t group = 0;
  };

  [[nodiscard]] static std::vector<Source> sources();
  [[nodiscard]] static std::uint64_t now();
  explicit Input(Route route);
  ~Input();
  Input(const Input &) = delete;
  Input &operator=(const Input &) = delete;
  [[nodiscard]] std::optional<Packet> read();
  void check() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
