// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/detail/dsp_program_fetch.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_program_fetch.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

namespace blofeld {
namespace {
constexpr unsigned word_mask = 0xffffff;
}

void DspProgramFetch::clear_continuation() {
  occurrences_ = {};
  predictions_ = {};
  boundaries_ = {};
  consumed_ = {};
  loop_.reset();
  consumed_end_.reset();
  opening_.reset();
  observed_loop_.reset();
  branch_.reset();
  branch_occurrence_ = 0;
  branch_target_captured_ = false;
}

void DspProgramFetch::start(const std::uint32_t pc, const std::uint32_t omr) {
  if (state_.active || pc > word_mask || omr > word_mask) {
    throw Fault("Unavailable native program-fetch start P:" + hex(pc, 6));
  }
  sequence_ = std::make_shared<Sequence>();
  state_ = {.next_address = pc, .consume_address = pc, .omr = omr, .active = true};
  clear_continuation();
}

void DspProgramFetch::check_stage(const DspProgramFetchStage stage) const {
  if (!state_.active || state_.used == state_.words.size() ||
      (stage == DspProgramFetchStage::address && state_.pending_address) ||
      (stage == DspProgramFetchStage::read && !state_.pending_address) ||
      (stage != DspProgramFetchStage::address && stage != DspProgramFetchStage::read)) {
    throw Fault("Unavailable native program-fetch stage P:" + hex(state_.next_address, 6));
  }
  if (stage == DspProgramFetchStage::read && loop_) { loop_->check_fetch(*state_.pending_address); }
}

void DspProgramFetch::issue() {
  check_stage(DspProgramFetchStage::address);
  state_.pending_address = state_.next_address;
  state_.next_address = (state_.next_address + 1) & word_mask;
}

std::uint64_t DspProgramFetch::next_occurrence() {
  auto previous = sequence_->serial.load(std::memory_order_relaxed);
  for (;;) {
    if (previous == std::numeric_limits<std::uint64_t>::max()) {
      throw Fault("Native F2 identity exhausted read P:" + hex(state_.next_address, 6));
    }
    if (sequence_->serial.compare_exchange_weak(
            previous, previous + 1, std::memory_order_relaxed)) {
      return previous + 1;
    }
  }
}

void DspProgramFetch::capture(DspBus &bus) {
  check_stage(DspProgramFetchStage::read);
  const auto pending = state_.pending_address;
  if (!pending) { throw Fault("Native F2 read lost its actual issued P address"); }
  const auto address = *pending;
  if (branch_ && !branch_target_captured_ && address != branch_->next_pc) {
    throw Fault("Native branch target F2 differs from its owned decision at P:" + hex(address, 6) +
                " pc=" + hex(branch_->word.address, 6) + " omr=" + hex(branch_->omr, 6));
  }
  const auto identity = next_occurrence();
  unsigned value = 0;
  try {
    value = bus.read({.address = address});
  } catch (const std::exception &error) {
    throw Fault("Native F2 read P:" + hex(address, 6) + " pc=" + hex(state_.consume_address, 6) +
                " omr=" + hex(state_.omr, 6) + ": " + error.what());
  }
  const DspProgramWord word{.address = address, .value = value};
  if (loop_) {
    boundaries_.at(state_.used) = loop_->fetched(word);
    predictions_.at(state_.used) = loop_->prediction();
    state_.next_address = loop_->next_address();
  }
  occurrences_.at(state_.used) = identity;
  state_.words.at(state_.used++) = word;
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

std::optional<DspFetchedWord> DspProgramFetch::queued(const std::size_t index) const {
  if (index >= state_.used) { return std::nullopt; }
  return DspFetchedWord{.word = state_.words.at(index), .occurrence = occurrences_.at(index)};
}

DspFetchedWord DspProgramFetch::select(const std::uint32_t address,
                                       const std::uint64_t identity) const {
  std::optional<DspFetchedWord> result;
  for (std::size_t index = 0; index < state_.used; ++index) {
    const DspFetchedWord sample{.word = state_.words.at(index),
                                .occurrence = occurrences_.at(index)};
    if (sample.word.address != address || (identity != 0 && sample.occurrence != identity)) {
      continue;
    }
    if (result) {
      throw Fault("Ambiguous actual F2 read P:" + hex(address, 6) +
                  " pc=" + hex(state_.consume_address, 6) + " omr=" + hex(state_.omr, 6));
    }
    result = sample;
  }
  if (!state_.active || !result) {
    throw Fault("Unavailable actual F2 occurrence read P:" + hex(address, 6) +
                " pc=" + hex(state_.consume_address, 6) + " omr=" + hex(state_.omr, 6));
  }
  return *result;
}

DspFetchedInstruction DspProgramFetch::instruction(const DspFetchedWord opcode,
                                                   const unsigned words) const {
  const auto address = opcode.word.address;
  if (select(address, opcode.occurrence) != opcode) {
    throw Fault("Instruction selection differs from owned F2 read P:" + hex(address, 6) +
                " pc=" + hex(state_.consume_address, 6) + " omr=" + hex(state_.omr, 6));
  }
  if (words == 1) { return {.opcode = opcode, .extension = std::nullopt}; }
  if (words == 2) {
    for (std::size_t index = 0; index + 1 < state_.used; ++index) {
      if (occurrences_.at(index) != opcode.occurrence) { continue; }
      const DspFetchedWord next{.word = state_.words.at(index + 1),
                                .occurrence = occurrences_.at(index + 1)};
      if (next.word.address == ((address + 1) & word_mask)) {
        return {.opcode = opcode, .extension = next};
      }
    }
  }
  throw Fault("Missing adjacent actual F2 extension read P:" + hex((address + 1) & word_mask, 6) +
              " pc=" + hex(address, 6) + " omr=" + hex(state_.omr, 6));
}

void DspFetchedInstruction::check_front(const DspProgramFetch &fetch) const {
  if (opcode.occurrence == 0 || fetch.queued(0) != opcode ||
      (extension && fetch.queued(1) != extension)) {
    throw Fault(
        "Native E1 differs from owned actual F2 occurrence read P:" + hex(opcode.word.address, 6) +
        " pc=" + hex(opcode.word.address, 6) + " omr=" + hex(fetch.view().omr, 6));
  }
}

void DspFetchedInstruction::check_consumed(const std::array<DspFetchedWord, 2> &actual,
                                           const std::uint32_t omr) const {
  const bool matches =
      extension ? actual.front() == opcode && actual.back() == *extension : actual.back() == opcode;
  if (opcode.occurrence == 0 || !matches) {
    throw Fault("Native admission differs from owned consumed F2 occurrence read P:" +
                hex(opcode.word.address, 6) + " pc=" + hex(opcode.word.address, 6) +
                " omr=" + hex(omr, 6));
  }
}

std::optional<DspProgramWord> DspProgramFetch::occurrence(const std::uint64_t identity) const {
  for (std::size_t index = 0; index < state_.used; ++index) {
    if (identity != 0 && occurrences_.at(index) == identity) { return state_.words.at(index); }
  }
  return std::nullopt;
}

void DspProgramFetch::check_consume(const std::uint32_t address) const {
  if (!state_.active || state_.used == 0 || state_.words.front().address != address ||
      state_.consume_address != address || consumed_end_ || opening_) {
    throw Fault("Native program-fetch admission awaits actual queued read P:" + hex(address, 6));
  }
}

void DspProgramFetch::shift_queue() {
  const auto count = static_cast<std::ptrdiff_t>(state_.used);
  std::move(state_.words.begin() + 1, state_.words.begin() + count, state_.words.begin());
  std::move(occurrences_.begin() + 1, occurrences_.begin() + count, occurrences_.begin());
  std::move(predictions_.begin() + 1, predictions_.begin() + count, predictions_.begin());
  std::move(boundaries_.begin() + 1, boundaries_.begin() + count, boundaries_.begin());
  --state_.used;
  state_.words.at(state_.used) = {};
  occurrences_.at(state_.used) = 0;
  predictions_.at(state_.used).reset();
  boundaries_.at(state_.used).reset();
}

std::uint32_t DspProgramFetch::consume(const std::uint32_t address) {
  check_consume(address);
  const auto word = state_.words.front();
  consumed_[0] = consumed_[1];
  consumed_[1] = {.word = word, .occurrence = occurrences_.front()};
  consumed_end_ = boundaries_.front();
  state_.consume_address = consumed_end_ ? consumed_end_->expected.pc : ((address + 1) & word_mask);
  shift_queue();
  return word.value;
}

void DspProgramFetch::redirect(const std::uint32_t pc, const std::uint32_t omr) {
  if (!state_.active || pc > word_mask || omr > word_mask || opening_ || consumed_end_ ||
      (loop_ && (loop_->depth() != 0 || loop_->pending() != 0))) {
    throw Fault("Unqualified native fetch redirect P:" + hex(pc, 6) + " omr=" + hex(omr, 6));
  }
  state_ = {.next_address = pc, .consume_address = pc, .omr = omr, .active = true};
  clear_continuation();
}

std::size_t DspProgramFetch::branch_position(const DspBranchDecision &decision,
                                             const std::uint64_t identity) const {
  std::optional<std::size_t> found;
  for (std::size_t index = 0; index < state_.used; ++index) {
    if (state_.words.at(index) != decision.word ||
        (decision.extension &&
         (index + 1 >= state_.used || state_.words.at(index + 1) != *decision.extension)) ||
        (identity != 0 && occurrences_.at(index) != identity)) {
      continue;
    }
    if (found) {
      throw Fault("Ambiguous branch F2 occurrence read P:" + hex(decision.word.address, 6));
    }
    found = index;
  }
  if (!found) {
    throw Fault("Native branch lost actual F2 instruction read P:" + hex(decision.word.address, 6) +
                " pc=" + hex(decision.word.address, 6) + " omr=" + hex(decision.omr, 6));
  }
  return *found;
}

std::size_t DspProgramFetch::position(const DspFetchedWord captured) const {
  for (std::size_t index = 0; index < state_.used; ++index) {
    if (state_.words.at(index) == captured.word && occurrences_.at(index) == captured.occurrence &&
        captured.occurrence != 0) {
      return index;
    }
  }
  throw Fault("Unavailable captured F2 context read P:" + hex(captured.word.address, 6) +
              " pc=" + hex(state_.consume_address, 6) + " omr=" + hex(state_.omr, 6));
}

std::optional<std::uint32_t> DspProgramFetch::loop_last(const DspFetchedWord captured) const {
  const auto index = position(captured);
  if (boundaries_.at(index)) { return captured.word.address; }
  const auto &prediction = predictions_.at(index);
  return prediction ? prediction->loop_last() : std::nullopt;
}

std::uint32_t DspProgramFetch::continuation_after(const DspFetchedInstruction &captured) const {
  const auto words = captured.extension ? 2U : 1U;
  if (instruction(captured.opcode, words) != captured) {
    throw Fault("Continuation differs from actual F2 instruction read P:" +
                hex(captured.opcode.word.address, 6) + " pc=" + hex(state_.consume_address, 6) +
                " omr=" + hex(state_.omr, 6));
  }
  const auto last = captured.extension.value_or(captured.opcode);
  const auto &boundary = boundaries_.at(position(last));
  return boundary ? boundary->expected.pc : ((last.word.address + 1) & word_mask);
}

void DspProgramFetch::hold_after(const DspFetchedInstruction &captured) {
  if (!state_.active || branch_ || opening_ || !captured.extension ||
      instruction(captured.opcode, 2) != captured) {
    throw Fault(
        "Unavailable actual Decode fetch barrier read P:" + hex(captured.opcode.word.address, 6) +
        " pc=" + hex(state_.consume_address, 6) + " omr=" + hex(state_.omr, 6));
  }
  const auto keep = position(captured.opcode) + 2;
  const auto next = (captured.extension->word.address + 1) & word_mask;
  if (loop_) {
    const auto &prediction = predictions_.at(keep - 1);
    if (!prediction || boundaries_.at(keep - 2) || boundaries_.at(keep - 1)) {
      throw Fault("Unqualified Decode barrier at DSP loop end read P:" +
                  hex(captured.opcode.word.address, 6));
    }
    loop_->discard_after(*prediction, next);
  }
  for (std::size_t index = keep; index < state_.used; ++index) {
    state_.words.at(index) = {};
    occurrences_.at(index) = 0;
    predictions_.at(index).reset();
    boundaries_.at(index).reset();
  }
  state_.used = keep;
  state_.pending_address.reset();
  state_.next_address = next;
}

void DspProgramFetch::predict_branch(const DspBranchDecision &decision,
                                     const std::uint64_t identity) {
  const auto form = decode_branch_form(decision.word.value);
  const bool one_word = form && (form->displacement || form->address_register);
  if (!state_.active || branch_ || opening_ || decision.omr != state_.omr ||
      decision.next_pc > word_mask || decision.extension.has_value() == one_word ||
      (decision.extension &&
       decision.extension->address != ((decision.word.address + 1) & word_mask))) {
    throw Fault("Unavailable native branch AG1 read P:" + hex(decision.word.address, 6) +
                " pc=" + hex(decision.word.address, 6) + " omr=" + hex(decision.omr, 6));
  }
  const auto pair = branch_position(decision, identity);
  const auto keep = pair + (decision.extension ? 2U : 1U);
  if (loop_) {
    const auto &prediction = predictions_.at(keep - 1);
    if (!prediction || boundaries_.at(pair) || boundaries_.at(keep - 1)) {
      throw Fault("Unqualified branch at DSP loop end read P:" + hex(decision.word.address, 6));
    }
    loop_->discard_after(*prediction, decision.next_pc);
  }
  branch_occurrence_ = occurrences_.at(pair);
  for (std::size_t index = keep; index < state_.used; ++index) {
    state_.words.at(index) = {};
    occurrences_.at(index) = 0;
    predictions_.at(index).reset();
    boundaries_.at(index).reset();
  }
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
  const auto last = decision.extension.value_or(decision.word);
  const auto sequential = (last.address + 1) & word_mask;
  const auto consumed = decision.extension ? consumed_.front() : consumed_.back();
  if (next_pc != decision.next_pc || omr != decision.omr || redirected != decision.taken ||
      !branch_target_captured_ || state_.used == 0 || state_.words.front().address != next_pc ||
      state_.consume_address != sequential || consumed.occurrence != branch_occurrence_) {
    throw Fault("Native branch retirement differs from owned F2 pc=" + hex(pc, 6) +
                " next=" + hex(next_pc, 6) + " omr=" + hex(omr, 6));
  }
  state_.consume_address = next_pc;
  branch_.reset();
  branch_occurrence_ = 0;
  branch_target_captured_ = false;
  return true;
}

void DspProgramFetch::open_loop(const DspLoopOpening &opening,
                                const DspFetchedWord opcode,
                                const DspFetchedWord extension) {
  const auto body = (opening.word.address + 2) & word_mask;
  if (!state_.active || state_.used != 0 || state_.pending_address || branch_ || opening_ ||
      consumed_end_ || consumed_[0] != opcode || consumed_[1] != extension ||
      opcode.occurrence == 0 || opening.word != opcode.word ||
      opening.extension != extension.word || state_.consume_address != body ||
      state_.next_address != body || opening.before.omr != state_.omr) {
    throw Fault("DSP DO lost actual consumed F2 barrier read P:" + hex(opening.word.address, 6) +
                " pc=" + hex(opening.before.pc, 6) + " omr=" + hex(state_.omr, 6));
  }
  auto candidate = loop_ ? *loop_ : DspLoopFetch(opening.before);
  if (!loop_) {
    static_cast<void>(candidate.fetched(opcode.word));
    static_cast<void>(candidate.fetched(extension.word));
  }
  candidate.opened(opening);
  loop_ = std::move(candidate);
  opening_ = opening;
  state_.next_address = loop_->next_address();
  state_.consume_address = opening.after.pc;
}

bool DspProgramFetch::confirm_opening(const DspLoopArchitecture native_result,
                                      const bool redirected) {
  if (!opening_) { return false; }
  const auto &opening = *opening_;
  const auto body = (opening.word.address + 2) & word_mask;
  if (native_result != opening.after || redirected != (opening.after.pc != body) ||
      state_.consume_address != native_result.pc) {
    throw Fault("Native DO retirement differs from actual F2 pc=" + hex(opening.word.address, 6) +
                " omr=" + hex(native_result.omr, 6));
  }
  observed_loop_ = native_result;
  opening_.reset();
  return true;
}

bool DspProgramFetch::confirm_loop(const DspLoopArchitecture native_result, const bool redirected) {
  if (!consumed_end_) { return false; }
  if (!loop_ || redirected != consumed_end_->backedge ||
      state_.consume_address != native_result.pc || state_.used == 0 ||
      state_.words.front().address != native_result.pc) {
    throw Fault(
        "Native loop end differs from actual F2 read P:" + hex(consumed_end_->last.address, 6) +
        " pc=" + hex(native_result.pc, 6) + " omr=" + hex(native_result.omr, 6));
  }
  loop_->confirm(*consumed_end_, native_result);
  observed_loop_ = native_result;
  consumed_end_.reset();
  return true;
}

bool DspProgramFetch::accepts_loop_control(const DspLoopArchitecture native_result) const {
  if (!observed_loop_) { return false; }
  auto expected = *observed_loop_;
  expected.pc = native_result.pc;
  return expected == native_result;
}

bool DspProgramFetch::owns_loop_result(const DspLoopArchitecture native_result,
                                       const bool redirected) const {
  if (opening_) {
    const auto &opening = *opening_;
    const auto body = (opening.word.address + 2) & word_mask;
    if (native_result != opening.after || redirected != (opening.after.pc != body) ||
        state_.consume_address != native_result.pc) {
      throw Fault("Native held DO differs from its consumed F2 opening pc=" +
                  hex(opening.word.address, 6) + " omr=" + hex(native_result.omr, 6));
    }
    return true;
  }
  if (!consumed_end_) { return false; }
  const auto &end = *consumed_end_;
  if (!loop_ || consumed_.back().word != end.last || consumed_.back().occurrence == 0 ||
      native_result != end.expected || redirected != end.backedge ||
      state_.consume_address != native_result.pc) {
    throw Fault("Native held loop end differs from its consumed F2 receipt read P:" +
                hex(end.last.address, 6) + " pc=" + hex(native_result.pc, 6) +
                " omr=" + hex(native_result.omr, 6));
  }
  return true;
}

}
