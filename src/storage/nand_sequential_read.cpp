// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand.hpp"

#include <cstdint>

namespace blofeld {
namespace {
constexpr std::uint8_t read_second_half = 1;
constexpr std::uint8_t read_spare = 0x50;
}

Nand::Nand(const NandReadContinuation continuation) : continuation_(continuation) {
  if (continuation != NandReadContinuation::unqualified &&
      continuation != NandReadContinuation::k9f1208_p &&
      continuation != NandReadContinuation::k9f1208_j) {
    fail("read continuation", "invalid part profile");
  }
}

bool Nand::quiescent_read_pause() const {
  const bool command = command_ == 0 || command_ == read_second_half || command_ == read_spare;
  return continuation_ != NandReadContinuation::unqualified && ready_ == true && !status_mode_ &&
         command && (phase_ == Phase::data || phase_ == Phase::idle);
}

void Nand::observe_read_end(const NandReadEnd edge) {
  if (continuation_ == NandReadContinuation::unqualified) {
    fail("read end", "requires an explicit P-package or J-package read profile");
  }
  if (edge != NandReadEnd::re_high && edge != NandReadEnd::ce_high) {
    fail("read end", "invalid observed edge");
  }
  if (edge == NandReadEnd::ce_high) {
    // K9F1208U0C Rev1.1 p18/Fig5: CE may be inactive during data loading.
    if (phase_ == Phase::program_data) { return; }
    // Rev1.1 p33: identification mode persists until another command.
    if (phase_ == Phase::identification_data) { return; }
    if (phase_ == Phase::read_pending) {
      // Rev1.1 p26: P-package CE interrupts tR; J-package CE is don't-care.
      if (continuation_ == NandReadContinuation::k9f1208_p) { phase_ = Phase::read_abort_pending; }
      return;
    }
    // K9F1208U0C Rev1.1 p18/Fig6: CE may be inactive between data reads.
    // The completed page buffer and next column survive this pause; only
    // the pin timing owner gates data output. Busy abort remains separate.
    if (quiescent_read_pause()) { return; }
    if (phase_ != Phase::end && phase_ != Phase::block_end) {
      fail("read end", "unqualified CE deassertion outside pending read or final-byte output");
    }
    phase_ = Phase::idle;
    return;
  }
  // ID, status and ordinary byte release do not start/restart an array read.
  if (status_mode_ || phase_ == Phase::identification_data || phase_ == Phase::data ||
      phase_ == Phase::read_pending) {
    return;
  }
  if (phase_ != Phase::end) { fail("read end", "no final array byte awaiting release"); }
  // Only P-package parts automatically select another row (Rev1.1 p26).
  if (continuation_ == NandReadContinuation::k9f1208_j) {
    phase_ = Phase::idle;
    return;
  }
  if (page_ % block_pages == block_pages - 1) {
    phase_ = Phase::block_end;
    return;
  }
  ++page_;
  column_ = command_ == read_spare ? main_bytes : 0;
  if (command_ == read_second_half) {
    command_ = 0;
    pointer_ = 0;
  }
  phase_ = Phase::read_pending;
  ready_ = false;
}
}
