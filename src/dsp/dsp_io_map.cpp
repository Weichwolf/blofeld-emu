// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_io_map.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"

#include <algorithm>
#include <array>
#include <span>

namespace blofeld {
namespace {
struct Range {
  unsigned first;
  unsigned last;
};

constexpr std::array x_registers{Range{.first = 0x82, .last = 0x94},
                                 Range{.first = 0xa0, .last = 0xa6},
                                 Range{.first = 0xa8, .last = 0xab},
                                 Range{.first = 0xb3, .last = 0xbf},
                                 Range{.first = 0xd0, .last = 0xf5},
                                 Range{.first = 0xfb, .last = 0xff}};
constexpr std::array y_registers{Range{.first = 0x80, .last = 0x86},
                                 Range{.first = 0x88, .last = 0x8b},
                                 Range{.first = 0x93, .last = 0xb8},
                                 Range{.first = 0xf5, .last = 0xf7}};
}

DspIoSelection dsp_io_selection(const DspAddress access) {
  constexpr unsigned maximum = 0xffffff;
  constexpr unsigned first_io = 0xffff80;
  if (access.address > maximum) { throw Fault("DSP I/O map address exceeds 24 bits"); }
  switch (access.space) {
    case DspSpace::p: return DspIoSelection::outside;
    case DspSpace::x:
    case DspSpace::y: break;
    default: throw Fault("Invalid DSP I/O map memory space");
  }
  if (access.address < first_io) { return DspIoSelection::outside; }
  constexpr unsigned offset_mask = 0xff;
  const auto offset = access.address & offset_mask;
  const auto ranges = access.space == DspSpace::x ? std::span<const Range>(x_registers)
                                                  : std::span<const Range>(y_registers);
  const bool assigned = std::ranges::any_of(ranges, [offset](const Range range) {
    return offset >= range.first && offset <= range.last;
  });
  return assigned ? DspIoSelection::assigned : DspIoSelection::reserved;
}
}
