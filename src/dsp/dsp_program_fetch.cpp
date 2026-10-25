// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/detail/dsp_program_fetch.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_program_fetch.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
constexpr unsigned word_mask = 0xffffff;
}

void DspProgramFetch::start(const std::uint32_t pc, const std::uint32_t omr) {
  if (state_.active || pc > word_mask || omr > word_mask) {
    throw Fault("Unavailable native program-fetch start P:" + hex(pc, 6));
  }
  state_ = {.next_address = pc, .consume_address = pc, .omr = omr, .active = true};
  branch_.reset();
  branch_target_captured_ = false;
}

void DspProgramFetch::check_stage(const DspProgramFetchStage stage) const {
  if (!state_.active || state_.used == state_.words.size() ||
      (stage == DspProgramFetchStage::address && state_.pending_address) ||
      (stage == DspProgramFetchStage::read && !state_.pending_address) ||
      (stage != DspProgramFetchStage::address && stage != DspProgramFetchStage::read)) {
    throw Fault("Unavailable native program-fetch stage P:" + hex(state_.next_address, 6));
  }
}

void DspProgramFetch::issue() {
  check_stage(DspProgramFetchStage::address);
  state_.pending_address = state_.next_address;
  state_.next_address = (state_.next_address + 1) & word_mask;
}

void DspProgramFetch::capture(DspBus &bus) {
  check_stage(DspProgramFetchStage::read);
  const auto pending = state_.pending_address;
  if (!pending) { throw Fault("Native program-fetch read lost its issued address"); }
  const auto address = *pending;
  if (branch_ && !branch_target_captured_ && address != branch_->next_pc) {
    throw Fault("Native branch target F2 differs from its owned decision at P:" + hex(address, 6) +
                " pc=" + hex(branch_->word.address, 6) + " omr=" + hex(branch_->omr, 6));
  }
  const auto value = bus.read({.address = address});
  state_.words.at(state_.used++) = {.address = address, .value = value};
  state_.pending_address.reset();
  if (branch_) { branch_target_captured_ = true; }
}

std::optional<std::uint32_t> DspProgramFetch::peek(const std::uint32_t address) const {
  for (std::size_t index = 0; index < state_.used; ++index) {
    const auto word = state_.words.at(index);
    if (word.address == address) { return word.value; }
  }
  return std::nullopt;
}

void DspProgramFetch::check_consume(const std::uint32_t address) const {
  if (!state_.active || state_.used == 0 || state_.words.front().address != address ||
      state_.consume_address != address) {
    throw Fault("Native program-fetch admission awaits actual queued read P:" + hex(address, 6));
  }
}

std::uint32_t DspProgramFetch::consume(const std::uint32_t address) {
  check_consume(address);
  const auto value = state_.words.front().value;
  std::move(state_.words.begin() + 1,
            state_.words.begin() + static_cast<std::ptrdiff_t>(state_.used),
            state_.words.begin());
  state_.words.at(--state_.used) = {};
  state_.consume_address = (address + 1) & word_mask;
  return value;
}

void DspProgramFetch::redirect(const std::uint32_t pc, const std::uint32_t omr) {
  if (!state_.active) { throw Fault("Native program-fetch redirect requires an active frontend"); }
  state_ = {.next_address = pc, .consume_address = pc, .omr = omr, .active = true};
  branch_.reset();
  branch_target_captured_ = false;
}

void DspProgramFetch::predict_branch(const DspBranchDecision &decision) {
  if (!state_.active || branch_ || decision.omr != state_.omr || decision.next_pc > word_mask ||
      decision.extension.address != ((decision.word.address + 1) & word_mask)) {
    throw Fault("Unavailable native branch AG1 at P:" + hex(decision.word.address, 6) +
                " pc=" + hex(decision.word.address, 6) + " omr=" + hex(decision.omr, 6));
  }
  std::size_t keep = 0;
  for (std::size_t index = 0; index + 1 < state_.used; ++index) {
    if (state_.words.at(index) == decision.word &&
        state_.words.at(index + 1) == decision.extension) {
      keep = index + 2;
      break;
    }
  }
  if (keep == 0) {
    throw Fault("Native branch AG1 lost its actual F2 pair at P:" + hex(decision.word.address, 6) +
                " pc=" + hex(decision.word.address, 6) + " omr=" + hex(decision.omr, 6));
  }
  // Trim by queue position, including across the24-bit address wrap. Retain every
  // older word and this exact opcode/extension until native consumption.
  for (std::size_t index = keep; index < state_.used; ++index) { state_.words.at(index) = {}; }
  state_.used = keep;
  state_.pending_address.reset();
  state_.next_address = decision.next_pc;
  branch_ = decision;
  branch_target_captured_ = false;
}

bool DspProgramFetch::confirm_branch(const std::uint32_t pc,
                                     const std::uint32_t next_pc,
                                     const std::uint32_t omr,
                                     const bool redirected) {
  if (!branch_ || branch_->word.address != pc) { return false; }
  const auto decision = *branch_;
  const auto sequential = (decision.extension.address + 1) & word_mask;
  if (next_pc != decision.next_pc || omr != decision.omr || redirected != decision.taken ||
      !branch_target_captured_ || state_.used == 0 || state_.words.front().address != next_pc ||
      state_.consume_address != sequential) {
    throw Fault("Native branch retirement differs from its owned fetch decision pc=" + hex(pc, 6) +
                " next=" + hex(next_pc, 6) + " omr=" + hex(omr, 6));
  }
  // The target words are real F2 samples; confirming native execution must not
  // flush them or confuse a taken-to-sequential target with a false condition.
  state_.consume_address = next_pc;
  branch_.reset();
  branch_target_captured_ = false;
  return true;
}
}
