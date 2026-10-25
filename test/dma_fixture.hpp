// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "esai_fixture.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld::test::dma_test {
using esai::rejects;
using esai::require;
inline constexpr unsigned source0 = 0xffffef;
inline constexpr unsigned destination0 = 0xffffee;
inline constexpr unsigned counter0 = 0xffffed;
inline constexpr unsigned control0 = 0xffffec;
inline constexpr unsigned status = 0xfffff4;
inline constexpr unsigned all_done = 0x3f;
inline constexpr unsigned zero_not_done = 0x3e;
inline constexpr unsigned zero_active = 0x13e;
inline constexpr unsigned offset0 = 0xfffff3;
inline constexpr unsigned offset1 = 0xfffff2;
inline constexpr unsigned offset2 = 0xfffff1;
inline constexpr unsigned offset3 = 0xfffff0;
inline constexpr unsigned enable = 0x800000;
inline constexpr unsigned software_copy = 0x9802d4; // DE,DTM3,X->Y,linear/linear; APR23 section2.
inline constexpr unsigned word_copy = 0x880244;     // DE,DTM1,IRQ A,X->Y,fixed/fixed.
inline constexpr unsigned repeat_ring = 0xa8b604;   // DE,DTM5,DRS22,modeC,source3D,fixedY.
inline constexpr unsigned ring_count = 0xfc0;
inline constexpr unsigned ring_start = 0x660;
inline constexpr unsigned ring_last = 0x69f;
inline constexpr unsigned ring_back = 0xffffc1; // -63 in24-bit modular address arithmetic.
inline constexpr unsigned destination = 0x900;
inline constexpr auto timing = DspDmaTiming::reference_sampled_cycles;
inline constexpr auto grant = DspDmaGrant::available;

inline void configure(DspDma &dma,
                      const unsigned control,
                      const unsigned count = 0,
                      const unsigned channel = 0) {
  dma.write(source0 - (channel * 4), ring_start);
  dma.write(destination0 - (channel * 4), destination);
  dma.write(counter0 - (channel * 4), count);
  dma.write(control0 - (channel * 4), control);
}

struct Harness {
  struct Write {
    DspDmaAddress address;
    std::uint32_t value;
  };

  DspDma dma;
  std::uint64_t cycle = 0;
  std::vector<DspDmaAddress> reads;
  std::vector<Write> writes;
  unsigned data = 0;
  bool fail_read = false;
  bool fail_write = false;

  [[nodiscard]] std::optional<DspDmaWord> tick(const std::uint32_t requests = 0,
                                               const bool wait = false) {
    const auto result =
        dma.advance({.cycle = cycle,
                     .known_requests = UINT32_MAX,
                     .requests = requests,
                     .timing = timing,
                     .grant = wait ? DspDmaGrant::wait : grant},
                    {.read =
                         [this](const DspDmaAddress address) {
                           if (fail_read) { throw Fault("Injected source bus failure"); }
                           reads.push_back(address);
                           return data == 0 ? address.address : data;
                         },
                     .write =
                         [this](const DspDmaAddress address, const std::uint32_t value) {
                           if (fail_write) { throw Fault("Injected destination bus failure"); }
                           writes.push_back({.address = address, .value = value});
                         }});
    ++cycle;
    return result;
  }
};

void native_checks();
void state_checks();
}
