// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace blofeld {

struct Fault : std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct InstructionFetch {
  std::uint16_t address = 0;
  unsigned length = 0;
};

class Bus {
public:
  virtual ~Bus() = default;
  virtual std::uint8_t read(std::uint16_t address) = 0;
  virtual void write(std::uint16_t address, std::uint8_t value) = 0;
  [[nodiscard]] virtual std::uint8_t peek(std::uint16_t address) const = 0;
  [[nodiscard]] virtual std::uint64_t revision() const = 0;
  virtual void check_execute(InstructionFetch fetch) const = 0;
};

std::string hex(std::uint64_t value, unsigned width = 4);

} // namespace blofeld
