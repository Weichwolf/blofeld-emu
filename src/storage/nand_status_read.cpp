// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_read_timing.hpp"

#include <cstdint>

namespace blofeld {
void NandReadTiming::refresh_status(const std::uint64_t ticks,
                                    const NandByte data,
                                    const NandStatusTiming contract) {
  check_time(ticks);
  if (contract != NandStatusTiming::reference_controller_boundary) {
    fail("dynamic status requires the explicit controller-boundary reference contract");
  }
  if (!initialized_ || ce_n_ || re_n_ || phase_ != Phase::reading) {
    fail("dynamic status requires an actual selected held RE-low read");
  }
  if ((data.value & data.known) != data.value || (data.known & ~Nand::status_bits) != 0) {
    fail("noncanonical or undefined status-pin knowledge");
  }
  // The logical register can change without a new RE/CE edge (Rev1.1 p33).
  // Initial access knowledge remains bounded by the genuine falling edge.
  // This explicit projection does not claim a measured asynchronous pad delay.
  ticks_ = ticks;
  data_ = data;
}
}
