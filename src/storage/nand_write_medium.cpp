// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_write_medium.hpp"

#include "common/bus.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_operation_clock.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace blofeld {

namespace {
constexpr unsigned program_us = 500;
constexpr unsigned erase_us = 3000;
constexpr unsigned per_second = 1000000;
constexpr std::uint8_t setup_program = 0x80;
constexpr std::uint8_t confirm_program = 0x10;
constexpr std::uint8_t confirm_erase = 0xd0;
constexpr std::uint8_t reset_command = 0xff;
}

NandWriteMedium::NandWriteMedium(Nand nand, const std::uint32_t timebase, NandCellProfile profile)
    : clock_(std::move(nand), timebase, NandOperationContract::reference_maximum),
      timebase_(timebase) {
  if (profile.contract == NandCellContract::unqualified) {
    if (!profile.cells.empty()) { fail("inhibit map lacks a qualified cell model"); }
  } else {
    cells_.emplace(std::move(profile));
  }
}

[[noreturn]] void NandWriteMedium::fail(const std::string &reason) const {
  throw Fault("NAND write/verify operation page=" + hex(device().page(), 6) +
              " column=" + hex(device().column(), 3) + " ticks=" + std::to_string(ticks()) +
              " timebase=" + std::to_string(timebase_) + ": " + reason);
}

void NandWriteMedium::healthy() const {
  if (faulted()) { fail("incomplete operation requires coordinated recovery"); }
}

std::optional<NandCompletionEvent> NandWriteMedium::next_event() const {
  healthy();
  const auto core = clock_.next_event();
  if (core && job_) { fail("overlapping read/reset and write completion"); }
  return job_ ? std::optional{job_->event} : core;
}

NandWriteMedium::Job NandWriteMedium::prepare(const NandPending kind) const {
  const auto scaled =
      static_cast<std::uint64_t>(kind == NandPending::program ? program_us : erase_us) * timebase_;
  const auto duration = (scaled / per_second) + (scaled % per_second != 0 ? 1U : 0U);
  if (duration > std::numeric_limits<std::uint64_t>::max() - ticks()) {
    fail("completion overflow");
  }
  Job result{.event = {.ticks = ticks() + duration, .operation = kind},
             .first_page = device().page(),
             .column = program_column_,
             .data = staged_,
             .before = {}};
  const unsigned pages = kind == NandPending::erase ? Nand::block_pages : 1U;
  if (kind == NandPending::erase) {
    result.first_page = (result.first_page / Nand::block_pages) * Nand::block_pages;
    result.data.clear();
  } else if (staged_.empty() || program_page_ != result.first_page ||
             staged_.size() > Nand::page_bytes - program_column_) {
    fail("program confirmation lacks its genuinely staged data");
  }
  for (unsigned page = 0; page < pages; ++page) {
    Page copy{};
    for (unsigned column = 0; column < Nand::page_bytes; ++column) {
      copy.at(column) = device().cell({.page = result.first_page + page, .column = column});
    }
    result.before.push_back(copy);
  }
  return result;
}

void NandWriteMedium::merge(std::optional<bool> &verified, const std::optional<bool> next) {
  if (verified == false || next == false) {
    verified = false;
  } else if (!verified || !next) {
    verified.reset();
  }
}

void NandWriteMedium::execute(const Job &job) {
  if (!cells_) { fail("pending write lost its qualified cell model"); }
  auto after = job.before;
  std::optional<bool> verified = true;
  for (unsigned page = 0; page < after.size(); ++page) {
    const unsigned start = job.event.operation == NandPending::program ? job.column : 0U;
    const unsigned count = job.event.operation == NandPending::program
                               ? static_cast<unsigned>(job.data.size())
                               : Nand::page_bytes;
    for (unsigned i = 0; i < count; ++i) {
      const auto column = start + i;
      const auto inhibit = cells_->inhibit({.page = job.first_page + page, .column = column});
      const auto before = job.before.at(page).at(column);
      const auto transition = job.event.operation == NandPending::program
                                  ? NandCellModel::program(before, job.data.at(i), inhibit)
                                  : NandCellModel::erase(before, inhibit);
      after.at(page).at(column) = transition.after;
      merge(verified, transition.verified);
    }
  }
  if (!verified) { fail("cell verification depends on unknown inhibited bits"); }
  clock_.observe_completion(*verified ? NandOutcome::pass : NandOutcome::fail);
  for (unsigned page = 0; page < after.size(); ++page) {
    for (unsigned column = 0; column < Nand::page_bytes; ++column) {
      const NandLocation location{.page = job.first_page + page, .column = column};
      const auto cell = after.at(page).at(column);
      if (*verified && device().cell(location) != cell) {
        fail("verified cell/controller result mismatch");
      }
      if (!*verified) { clock_.observe_cell(location, cell); }
    }
  }
}

std::optional<NandCompletionEvent> NandWriteMedium::advance(const std::uint64_t at) {
  healthy();
  bad_ = true;
  if (at < ticks()) { fail("time moved backwards"); }
  std::optional<NandCompletionEvent> result;
  if (job_ && at >= job_->event.ticks) {
    if (clock_.advance(job_->event.ticks)) { fail("write overlapped another operation"); }
    result = job_->event;
    execute(*job_);
    job_.reset();
  }
  const auto core = clock_.advance(at);
  if (core && result) { fail("multiple overlapping operation completions"); }
  bad_ = false;
  return result ? result : core;
}

void NandWriteMedium::command(const std::uint8_t value) {
  healthy();
  bad_ = true;
  std::optional<Job> next;
  if (value == confirm_program || value == confirm_erase) {
    if (!cells_) { fail("program/erase completion requires an explicit cell model"); }
    next = prepare(value == confirm_program ? NandPending::program : NandPending::erase);
  }
  clock_.command(value);
  if (next) {
    job_ = std::move(next);
    staged_.clear();
  }
  if (value == reset_command) {
    job_.reset();
    staged_.clear();
  }
  if (value == setup_program) { staged_.clear(); }
  bad_ = false;
}

void NandWriteMedium::write(const std::uint8_t value) {
  healthy();
  bad_ = true;
  const auto page = device().page();
  const auto column = device().column();
  clock_.write(value);
  if (staged_.empty()) {
    program_page_ = page;
    program_column_ = column;
  }
  if (page != program_page_ || column != program_column_ + staged_.size()) {
    fail("noncontiguous native data staging");
  }
  staged_.push_back(value);
  bad_ = false;
}

void NandWriteMedium::power_loss() {
  healthy();
  bad_ = true;
  clock_.power_loss();
  job_.reset();
  staged_.clear();
  bad_ = false;
}

void NandWriteMedium::observe_power_ready() {
  healthy();
  clock_.observe_power_ready();
}

void NandWriteMedium::observe_identification(const NandIdentification identification) {
  healthy();
  clock_.observe_identification(identification);
}

void NandWriteMedium::observe_wp(const std::optional<bool> high) {
  healthy();
  clock_.observe_wp(high);
}

void NandWriteMedium::address(const std::uint8_t value) {
  healthy();
  clock_.address(value);
}

std::uint8_t NandWriteMedium::read(const std::uint8_t mask) {
  healthy();
  return clock_.read(mask);
}

void NandWriteMedium::observe_read_end(const NandReadEnd edge) {
  healthy();
  clock_.observe_read_end(edge);
}
}
