// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/ports.hpp"

#include "io/shi.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint16_t pim_first = 0x240, pim_end = 0x280, routing_address = 0x257;
constexpr std::uint16_t assignment_address = 0xa, mode_address = 0xb;
constexpr std::uint16_t pull_address = 0xc, reduced_address = 0xd;
constexpr std::uint8_t assignment_mask = 0xbc, assignment_clock = 0x10;
constexpr std::uint8_t core_controls = 0x93, routing_mask = 0x1f;
constexpr std::uint8_t normal_mode = 0x80, expansion_modes = 0x60, mode_controls = 0xa;
constexpr std::size_t first_pim = 3;
constexpr unsigned byte_mask = 0xff, e_direction = 0xfc;
constexpr std::array<unsigned, 11> valid{
    0xff, 0xff, 0xff, 0xf, 0x3f, 7, 0xff, 0xff, 0xff, 0xff, 0x3f};
constexpr std::array<unsigned, 11> tied{0, 0, 0, 0xc, 0x27, 0, 0xff, 0xff, 0, 0xf, 0x3e};

struct Register {
  std::size_t port = 0;
  unsigned kind = 0;
};

std::optional<Register> decode(const std::uint16_t offset) noexcept {
  switch (offset) {
    case 0: return Register{.port = 0, .kind = 0};
    case 1: return Register{.port = 1, .kind = 0};
    case 2: return Register{.port = 0, .kind = 2};
    case 3: return Register{.port = 1, .kind = 2};
    case 8: return Register{.port = 2, .kind = 0};
    case 9: return Register{.port = 2, .kind = 2};
    default: break;
  }
  if (offset < pim_first || offset >= pim_end || offset == routing_address) { return {}; }
  const auto relative = static_cast<unsigned>(offset - pim_first);
  return Register{.port = first_pim + (relative / 8), .kind = relative % 8};
}
}

Ports::Ports(Shi &shi) : boot_(shi) {
  // 9S12UF32DGV1 4.2.1.1: all Port E inputs have pulls after normal reset.
  ports_[2].pull = static_cast<std::uint8_t>(byte_mask);
  // MEBI output latches reset unknown; PIM output latches reset to zero.
  for (std::size_t index = first_pim; index < ports_.size(); ++index) {
    ports_[index].known = static_cast<std::uint8_t>(valid[index]);
  }
}

bool Ports::contains(const std::uint16_t offset) noexcept {
  return offset < 0x10 || (offset >= pim_first && offset < pim_end);
}

Ports::Sample Ports::pins(const std::size_t index) const noexcept {
  const auto &port = ports_[index];
  auto result =
      Sample{.known = port.pull, .value = port.pull & ~static_cast<unsigned>(port.polarity)};
  const auto outputs = static_cast<unsigned>(port.direction);
  result.known = (result.known & ~outputs) | (port.known & outputs);
  result.value = (result.value & ~outputs) | (port.latch & outputs);
  const auto external = static_cast<unsigned>(port.external.mask);
  result.known = (result.known & ~external) | (port.external.known & external);
  result.value = (result.value & ~external) | (port.external.value & external);
  if ((routing_ & 3) == 3) {
    result.known |= tied[index];
    result.value &= ~tied[index];
  }
  result.known |= byte_mask & ~valid[index];
  result.value &= valid[index];
  if (index == 2 && (assignment_ & assignment_clock) == 0) {
    // The clock output is not GPIO. Its sampling is not supplied by this bank.
    result.known &= ~static_cast<unsigned>(assignment_clock);
  }
  return result;
}

bool Ports::read(const std::uint16_t offset,
                 std::uint8_t &value,
                 const std::uint8_t mask) const noexcept {
  const auto decoded = decode(offset);
  if (decoded) {
    const auto &port = ports_[decoded->port];
    switch (decoded->kind) {
      case 0:
      case 1: {
        auto result = pins(decoded->port);
        if (decoded->kind == 0) {
          auto outputs = static_cast<unsigned>(port.direction);
          if (decoded->port == 2 && (assignment_ & assignment_clock) == 0) {
            outputs &= ~static_cast<unsigned>(assignment_clock);
          }
          result.known = (result.known & ~outputs) | (port.known & outputs);
          result.value = (result.value & ~outputs) | (port.latch & outputs);
        }
        if ((result.known & mask) != mask) { return false; }
        value = static_cast<std::uint8_t>(result.value & mask);
        return true;
      }
      case 2: value = port.direction; return true;
      case 3: value = port.reduced; return true;
      case 4: value = port.pull; return true;
      case 5: value = port.polarity; return true;
      case 6:
      case 7: value = 0; return true;
      default: return false;
    }
  }
  switch (offset) {
    case assignment_address: value = assignment_; return true;
    case mode_address: value = mode_; return true;
    case pull_address: value = pull_; return true;
    case reduced_address: value = reduced_; return true;
    case routing_address: value = routing_; return true;
    case 4:
    case 5:
    case 6:
    case 7:
    case 0xf: value = 0; return true;
    default: return false;
  }
}

bool Ports::write_control(const Write access) noexcept {
  switch (access.offset) {
    case assignment_address:
      assignment_ =
          static_cast<std::uint8_t>(assignment_written_ ? (assignment_ & ~assignment_clock) |
                                                              (access.value & assignment_clock)
                                                        : access.value & assignment_mask);
      assignment_written_ = true;
      return true;
    case mode_address:
      if (!mode_written_) {
        if ((access.value & expansion_modes) != 0) { return false; }
        mode_ = normal_mode | (access.value & mode_controls);
        mode_written_ = true;
      }
      return true;
    case pull_address:
      pull_ = access.value & core_controls;
      for (std::size_t index = 0; index < first_pim; ++index) {
        const unsigned select = index == 2 ? 0x10 : 1U << index;
        ports_[index].pull = (pull_ & select) != 0 ? static_cast<std::uint8_t>(byte_mask) : 0;
      }
      return true;
    case reduced_address: reduced_ = access.value & core_controls; return true;
    case routing_address: routing_ = access.value & routing_mask; return true;
    case 4:
    case 5:
    case 6:
    case 7:
    case 0xf: return true;
    default: return false;
  }
}

bool Ports::write(const Write access) noexcept {
  const auto decoded = decode(access.offset);
  if (!decoded) { return write_control(access); }
  auto &port = ports_[decoded->port];
  const auto value = static_cast<std::uint8_t>(access.value & valid[decoded->port]);
  switch (decoded->kind) {
    case 0:
      port.latch = value;
      port.known = static_cast<std::uint8_t>(valid[decoded->port]);
      break;
    case 1: return true;
    case 2:
      port.direction = static_cast<std::uint8_t>(decoded->port == 2 ? value & e_direction : value);
      break;
    case 3: port.reduced = value; return true;
    case 4: port.pull = value; return true;
    case 5: port.polarity = value; return true;
    case 6:
    case 7: return true;
    default: return false;
  }
  if (decoded->port != 1) { return true; }
  return boot_.write({.direction = port.direction, .known = port.known, .value = port.latch});
}

void Ports::input(const Port port, const Input sample) {
  const auto index = static_cast<std::size_t>(port);
  if (index >= ports_.size()) { throw std::out_of_range("Unknown MCU port"); }
  ports_[index].external = sample;
}
}
