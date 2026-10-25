// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace blofeld::test::banks {
namespace transfer = dma_test;
inline constexpr unsigned first = 0x123456;
inline constexpr unsigned second = 0x234567;
// Independent DSP56371UM memory-map expectations, not production definitions.
inline constexpr unsigned x_lower_last = 0x2fff;
inline constexpr unsigned y_lower_last = 0x1fff;
inline constexpr unsigned upper_begin = 0xc000;
inline constexpr unsigned upper_next = 0xe000;
inline constexpr unsigned x_upper_last = 0x11fff;
inline constexpr unsigned y_upper_last = 0x15fff;
inline constexpr unsigned p_lower_last = 0xfff;
inline constexpr unsigned p_rom_begin = 0xff0000;
inline constexpr unsigned x_hole = 0x3000;
inline constexpr unsigned section_last = 1023;
inline constexpr auto known = DspCoreBusKnowledge::reference_cycle_occupancy;
inline constexpr auto read_phase = DspDmaPhaseKind::source_read;
inline constexpr auto write_phase = DspDmaPhaseKind::destination_write;

struct Cycle {
  std::uint64_t index;
  unsigned requests = 0;
  DspDmaGrant grant = DspDmaGrant::available;
};

inline DspDmaInputs inputs(const Cycle cycle) {
  return {.cycle = cycle.index,
          .known_requests = 15,
          .requests = cycle.requests,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = cycle.grant};
}

inline DspCoreBusUse use(const std::span<const DspMemoryAccess> accesses = {}) {
  return {.accesses = accesses, .knowledge = known};
}

struct Transfer {
  unsigned source = transfer::ring_start;
  unsigned destination = transfer::destination;
  unsigned control = transfer::word_copy;
};

inline void configure(DspBus &bus, const Transfer setup = {}) {
  const std::array writes{
      DspStore{.space = DspSpace::x, .address = transfer::source0, .value = setup.source},
      DspStore{.space = DspSpace::x, .address = transfer::destination0, .value = setup.destination},
      DspStore{.space = DspSpace::x, .address = transfer::counter0},
      DspStore{.space = DspSpace::x, .address = transfer::control0, .value = setup.control}};
  for (const auto value : writes) { bus.write(value); }
}

void phase_checks();
void native_checks();
}
