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
      continuation != NandReadContinuation::k9f1208_p) {
    fail("read continuation", "invalid part profile");
  }
}

void Nand::observe_read_end(const NandReadEnd edge) {
  if (continuation_ != NandReadContinuation::k9f1208_p) {
    fail("read end", "requires an explicit P-package sequential-read profile");
  }
  if (edge != NandReadEnd::re_high && edge != NandReadEnd::ce_high) {
    fail("read end", "invalid observed edge");
  }
  if (edge == NandReadEnd::ce_high) {
    if (phase_ == Phase::read_pending) {
      phase_ = Phase::read_abort_pending;
      return;
    }
    if (phase_ != Phase::end && phase_ != Phase::block_end) {
      fail("read end", "unqualified CE deassertion outside pending read or final-byte output");
    }
    phase_ = Phase::idle;
    return;
  }
  // Status output and ordinary byte release do not start/restart an array read.
  if (status_mode_ || phase_ == Phase::data || phase_ == Phase::read_pending) { return; }
  if (phase_ != Phase::end) { fail("read end", "no final array byte awaiting release"); }
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
