// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
constexpr unsigned byte_bits = 8;
constexpr unsigned erase_address_bytes = 3;
constexpr unsigned half_bytes = 256;
}

void Nand::observe_wp(const std::optional<bool> high) {
  const bool writing = phase_ == Phase::program_address || phase_ == Phase::program_data ||
                       phase_ == Phase::program_pending || phase_ == Phase::erase_address ||
                       phase_ == Phase::erase_pending;
  if (writing && high != wp_high_) {
    fail("WP observation", "unqualified WP change during write sequence");
  }
  wp_high_ = high;
}

void Nand::begin_program() {
  if (wp_high_ != true) { fail("program setup", "WP high must be explicitly observed"); }
  if (!pointer_) {
    fail("program setup", "unknown area pointer; supply an explicit read pointer command");
  }
  if (phase_ == Phase::program_address || phase_ == Phase::program_data ||
      phase_ == Phase::erase_address) {
    fail("program setup", "unqualified replacement of a staged write sequence");
  }
  program_ = {};
  address_cycles_ = 0;
  phase_ = Phase::program_address;
  status_mode_ = false;
}

void Nand::begin_erase() {
  if (wp_high_ != true) { fail("erase setup", "WP high must be explicitly observed"); }
  if (phase_ == Phase::program_address || phase_ == Phase::program_data ||
      phase_ == Phase::erase_address) {
    fail("erase setup", "unqualified replacement of a staged write sequence");
  }
  address_cycles_ = 0;
  page_ = 0;
  column_ = 0;
  phase_ = Phase::erase_address;
  status_mode_ = false;
}

void Nand::address_erase(const std::uint8_t value) {
  if (address_cycles_ == erase_address_bytes) { return; }
  const auto page = page_ | (static_cast<unsigned>(value) << (address_cycles_ * byte_bits));
  if (page >= pages) { fail("erase address", "row exceeds device"); }
  page_ = page;
  ++address_cycles_;
}

void Nand::write(const std::uint8_t value) {
  if (phase_ != Phase::program_data || status_mode_ || ready_ != true) {
    fail("data write", "no completed program address phase");
  }
  if (column_ == page_bytes) { fail("data write", "program data exceeds page"); }
  program_.bytes.at(program_.size) = value;
  ++program_.size;
  ++column_;
}

void Nand::check_write_permission(const bool erase) const {
  if (wp_high_ != true) { fail("write confirmation", "WP high must remain explicitly observed"); }
  const auto state = contents_.block(page_ / block_pages);
  if (state.valid != true) { fail("write confirmation", "unknown/invalid block condition"); }
  const auto protection = erase ? state.erase_protected : state.program_protected;
  if (!protection.has_value()) { fail("write confirmation", "unknown block protection"); }
  if (*protection) { fail("write confirmation", "unqualified protected-block write outcome"); }
}

void Nand::confirm_program() {
  if (phase_ != Phase::program_data || program_.size == 0) {
    fail("program confirmation", "no staged program data");
  }
  check_write_permission(false);
  auto before = contents_.snapshot(page_);
  if (!before.programs) {
    fail("program confirmation", "unknown program history; completed erase required");
  }
  const bool main = program_.start < main_bytes;
  const bool spare = program_.start + program_.size > main_bytes;
  constexpr unsigned main_limit = 1;
  constexpr unsigned spare_limit = 2;
  if (main && before.programs->main >= main_limit) {
    fail("program confirmation", "main partial-program limit");
  }
  if (spare && before.programs->spare >= spare_limit) {
    fail("program confirmation", "spare partial-program limit");
  }
  program_.before = before;
  contents_.invalidate_page(page_);
  phase_ = Phase::program_pending;
  ready_ = false;
  outcome_.reset();
  reset_status_ = false;
  if (pointer_ == half_bytes) { pointer_ = 0; }
}

void Nand::confirm_erase() {
  if (phase_ != Phase::erase_address || address_cycles_ != erase_address_bytes) {
    fail("erase confirmation", "no complete erase address");
  }
  check_write_permission(true);
  // No cell is known to have completed while the internal erase is busy.
  const auto block = page_ / block_pages;
  for (unsigned index = 0; index < block_pages; ++index) {
    contents_.invalidate_page((block * block_pages) + index);
  }
  phase_ = Phase::erase_pending;
  ready_ = false;
  outcome_.reset();
  reset_status_ = false;
}

void Nand::complete_program() {
  if (!program_.before || !program_.before->programs) {
    fail("program completion", "missing pre-program reference state");
  }
  auto after = *program_.before;
  for (unsigned index = 0; index < program_.size; ++index) {
    const auto position = program_.start + index;
    after.cells.at(position) =
        NandArray::programmed(after.cells.at(position), program_.bytes.at(index));
  }
  if (program_.start < main_bytes) { ++after.programs->main; }
  if (program_.start + program_.size > main_bytes) { ++after.programs->spare; }
  contents_.restore(page_, after);
}

void Nand::abort_active() {
  if (phase_ == Phase::program_pending) {
    contents_.invalidate_page(page_);
    contents_.observe_block(page_ / block_pages, {});
  } else if (phase_ == Phase::erase_pending) {
    contents_.invalidate_block(page_ / block_pages);
  }
}

void Nand::observe_completion(const NandOutcome outcome) {
  if (outcome != NandOutcome::pass && outcome != NandOutcome::fail) {
    fail("write completion", "invalid observed outcome");
  }
  if (phase_ != Phase::program_pending && phase_ != Phase::erase_pending) {
    fail("write completion", "no pending program/erase operation");
  }
  if (outcome == NandOutcome::fail) {
    abort_active();
  } else if (phase_ == Phase::program_pending) {
    complete_program();
  } else {
    contents_.erase_block(page_ / block_pages);
  }
  program_ = {};
  phase_ = Phase::idle;
  ready_ = true;
  outcome_ = outcome;
}

void Nand::power_loss() {
  abort_active();
  program_ = {};
  phase_ = Phase::cold;
  ready_.reset();
  wp_high_.reset();
  outcome_.reset();
  pointer_.reset();
  command_ = 0;
  address_cycles_ = 0;
  page_ = 0;
  column_ = 0;
  status_mode_ = false;
  reset_status_ = false;
}
}
