// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/detail/dsp_program_fetch.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_repeat_fetch.hpp"

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
DspProgramWord DspProgramSample::require_known(const DspProgramReadContext context) const {
  if (!data) {
    throw Fault("Unknown DSP program sample read P:" + hex(address, 6) +
                " pc=" + hex(context.pc, 6) + " omr=" + hex(context.omr, 6));
  }
  return {.address = address, .value = *data};
}

DspFetchedWord DspFetchedSample::require_known(const DspProgramReadContext context) const {
  return {.word = sample.require_known(context), .occurrence = occurrence};
}

DspFetchedSample DspFetchedSample::retain(const DspFetchedWord known) {
  return {.sample = {.address = known.word.address, .data = known.word.value},
          .occurrence = known.occurrence};
}

namespace {
constexpr unsigned word_mask = 0xffffff;

// Retire only completed speculative loop metadata. Actual F1/F2 receipts,
// addresses, resource origins, branch decisions and CPU state remain owned.
void release_loop_history(
    std::optional<DspLoopFetch> &loop,
    std::optional<DspLoopArchitecture> &observed,
    std::array<std::optional<DspLoopFetch::Prediction>, dsp_program_fetch_capacity> &predictions,
    const std::array<std::optional<DspLoopBoundary>, dsp_program_fetch_capacity> &boundaries,
    const DspLoopArchitecture native_result) {
  if (!loop || loop->depth() != 0 || loop->pending() != 0) { return; }
  constexpr unsigned loop_flags = 0x18000;
  const bool boundary =
      std::ranges::any_of(boundaries, [](const auto &entry) { return entry.has_value(); });
  const bool active = std::ranges::any_of(
      predictions, [](const auto &entry) { return entry && entry->loop_last().has_value(); });
  if ((native_result.status & loop_flags) != 0 || boundary || active) {
    throw Fault("Completed loop retains active fetch history read P:" + hex(native_result.pc, 6) +
                " pc=" + hex(native_result.pc, 6) + " omr=" + hex(native_result.omr, 6));
  }
  loop.reset();
  observed.reset();
  predictions = {};
}
}

void DspProgramFetch::clear_continuation() {
  occurrences_ = {};
  origins_ = {};
  consumed_origins_ = {};
  context_.reset();
  predictions_ = {};
  boundaries_ = {};
  consumed_ = {};
  repeat_.reset();
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
  std::optional<std::uint32_t> value;
  DspMemoryResource resource;
  try {
    resource = bus.memory_resource({.target = {.address = address}});
    value = bus.sample_program(address);
  } catch (const std::exception &error) {
    throw Fault("Native F2 read P:" + hex(address, 6) + " pc=" + hex(state_.consume_address, 6) +
                " omr=" + hex(state_.omr, 6) + ": " + error.what());
  }
  const DspProgramSample word{.address = address, .data = value};
  if (loop_) {
    boundaries_.at(state_.used) = loop_->sampled(word);
    predictions_.at(state_.used) = loop_->prediction();
    state_.next_address = loop_->next_address();
  }
  origins_.at(state_.used) = {.omr = state_.omr, .resource = resource};
  occurrences_.at(state_.used) = identity;
  state_.words.at(state_.used++) = word;
  state_.pending_address.reset();
  if (branch_) { branch_target_captured_ = true; }
}

std::optional<std::uint32_t> DspProgramFetch::peek(const std::uint32_t address) const {
  for (std::size_t index = 0; index < state_.used; ++index) {
    const auto word = state_.words.at(index);
    if (word.address == address) { return word.data; }
  }
  return std::nullopt;
}

std::optional<DspFetchedWord> DspProgramFetch::queued(const std::size_t index) const {
  const auto value = sampled(index);
  return value ? std::optional(
                     value->require_known({.pc = state_.consume_address, .omr = state_.omr}))
               : std::nullopt;
}

std::optional<DspFetchedSample> DspProgramFetch::sampled(const std::size_t index) const {
  if (index >= state_.used) { return std::nullopt; }
  return DspFetchedSample{.sample = state_.words.at(index), .occurrence = occurrences_.at(index)};
}

DspFetchedWord DspProgramFetch::select(const std::uint32_t address,
                                       const std::uint64_t identity) const {
  std::optional<DspFetchedWord> result;
  for (std::size_t index = 0; index < state_.used; ++index) {
    if (state_.words.at(index).address != address ||
        (identity != 0 && occurrences_.at(index) != identity)) {
      continue;
    }
    const DspFetchedWord sample{.word = state_.words.at(index).require_known(
                                    {.pc = state_.consume_address, .omr = state_.omr}),
                                .occurrence = occurrences_.at(index)};
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
      const DspFetchedWord next{
          .word = state_.words.at(index + 1).require_known({.pc = address, .omr = state_.omr}),
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
    if (identity != 0 && occurrences_.at(index) == identity) {
      return state_.words.at(index).require_known(
          {.pc = state_.consume_address, .omr = state_.omr});
    }
  }
  return std::nullopt;
}

void DspProgramFetch::check_consume(const std::uint32_t address) const {
  if (!state_.active || state_.used == 0 || state_.words.front().address != address ||
      state_.consume_address != address || consumed_end_ || opening_) {
    throw Fault("Native program-fetch admission awaits actual queued read P:" + hex(address, 6));
  }
  static_cast<void>(
      state_.words.front().require_known({.pc = state_.consume_address, .omr = state_.omr}));
}

void DspProgramFetch::shift_queue() {
  const auto count = static_cast<std::ptrdiff_t>(state_.used);
  std::move(state_.words.begin() + 1, state_.words.begin() + count, state_.words.begin());
  std::move(occurrences_.begin() + 1, occurrences_.begin() + count, occurrences_.begin());
  std::move(origins_.begin() + 1, origins_.begin() + count, origins_.begin());
  std::move(predictions_.begin() + 1, predictions_.begin() + count, predictions_.begin());
  std::move(boundaries_.begin() + 1, boundaries_.begin() + count, boundaries_.begin());
  --state_.used;
  state_.words.at(state_.used) = {};
  occurrences_.at(state_.used) = 0;
  origins_.at(state_.used) = {};
  predictions_.at(state_.used).reset();
  boundaries_.at(state_.used).reset();
}

std::uint32_t DspProgramFetch::consume(const std::uint32_t address) {
  check_consume(address);
  const auto word =
      state_.words.front().require_known({.pc = state_.consume_address, .omr = state_.omr});
  if (repeat_) {
    auto &repeat = *repeat_;
    if (repeat.prefix_pending || repeat.consumed || address != repeat.current.body ||
        (repeat.body && repeat.body != queued(0))) {
      throw Fault("REP instruction latch ownership lost read P:" + hex(address, 6) +
                  " pc=" + hex(state_.consume_address, 6) + " omr=" + hex(state_.omr, 6));
    }
    repeat.body = queued(0);
    repeat.consumed = true;
  }
  consumed_origins_[0] = consumed_origins_[1];
  consumed_origins_[1] = origins_.front();
  consumed_[0] = consumed_[1];
  consumed_[1] = {.word = word, .occurrence = occurrences_.front()};
  if (repeat_) { return word.value; }
  consumed_end_ = boundaries_.front();
  state_.consume_address = consumed_end_ ? consumed_end_->expected.pc : ((address + 1) & word_mask);
  shift_queue();
  return word.value;
}

std::optional<DspRepeatControl> DspProgramFetch::repeat_control() const {
  return repeat_ ? std::optional(repeat_->current) : std::nullopt;
}

std::optional<DspFetchedWord> DspProgramFetch::repeat_body() const {
  if (!repeat_) { return std::nullopt; }
  const auto head = queued(0);
  return repeat_ && head && head->word.address == repeat_->current.body ? head : std::nullopt;
}

void DspProgramFetch::verify_repeat(const DspRepeatControl native_state) const {
  if (!repeat_ || repeat_->prefix_pending || native_state != repeat_->current || !repeat_body() ||
      (native_state.loaded && (!repeat_->consumed || !repeat_->body))) {
    throw Fault("Native REP continuation lost its owned instruction latch read P:" +
                hex(native_state.body, 6) + " pc=" + hex(native_state.pc, 6) +
                " omr=" + hex(native_state.omr, 6));
  }
}

bool DspProgramFetch::owns_repeat_result(const DspFetchCommit completed,
                                         const DspRepeatControl result) const {
  if (!repeat_) { return false; }
  auto candidate = *this;
  return candidate.confirm_repeat(completed, result);
}

void DspProgramFetch::stage_repeat(const DspRepeatControl before, const DspRepeatControl after) {
  const auto prefix = consumed_.back();
  const auto body = (prefix.word.address + 1) & word_mask;
  if (!state_.active || repeat_ || loop_ || opening_ || consumed_end_ || branch_ || context_ ||
      prefix.occurrence == 0 || !fetch_repeat_prefix(prefix.word.value) ||
      before.pc != prefix.word.address || before.omr != state_.omr || before.remaining != 0 ||
      after.pc != body || after.body != body || after.omr != before.omr || after.remaining == 0 ||
      after.remaining > word_mask || after.lc != after.remaining || after.saved_lc != before.lc ||
      after.loaded || state_.consume_address != body) {
    throw Fault("Unqualified native REP opening read P:" + hex(prefix.word.address, 6) +
                " pc=" + hex(before.pc, 6) + " omr=" + hex(before.omr, 6));
  }
  repeat_ = Repeat{.prefix = prefix, .current = after, .body = std::nullopt};
}

bool DspProgramFetch::confirm_repeat(const DspFetchCommit completed,
                                     const DspRepeatControl native_result) {
  if (!repeat_) { return false; }
  auto &repeat = *repeat_;
  if (repeat.prefix_pending) {
    if (completed.pc != repeat.prefix.word.address || completed.redirected ||
        completed.next_pc != repeat.current.pc || completed.omr != repeat.current.omr ||
        native_result != repeat.current) {
      throw Fault("Native REP prefix differs from held result read P:" +
                  hex(repeat.prefix.word.address, 6) + " pc=" + hex(completed.pc, 6) +
                  " omr=" + hex(completed.omr, 6));
    }
    repeat.prefix_pending = false;
    return true;
  }
  auto expected = repeat.current;
  --expected.remaining;
  expected.loaded = expected.remaining != 0;
  expected.next = (expected.body + 1) & word_mask;
  expected.pc = expected.loaded ? expected.body : expected.next;
  expected.lc = expected.loaded ? expected.remaining : expected.saved_lc;
  if (repeat.body) { expected.opcode = repeat.body->word.value; }
  expected.extension = 0;
  if (!repeat.consumed || !repeat.body || queued(0) != repeat.body ||
      completed.pc != expected.body || completed.next_pc != expected.pc || completed.redirected ||
      completed.omr != expected.omr || native_result != expected) {
    throw Fault(
        "Native REP body differs from owned instruction latch read P:" + hex(expected.body, 6) +
        " pc=" + hex(completed.pc, 6) + " omr=" + hex(completed.omr, 6));
  }
  repeat.current = native_result;
  if (expected.remaining == 0) {
    state_.consume_address = expected.next;
    shift_queue();
    repeat_.reset();
  }
  return true;
}

void DspProgramFetch::redirect(const std::uint32_t pc, const std::uint32_t omr) {
  if (!state_.active || pc > word_mask || omr > word_mask || opening_ || consumed_end_ ||
      (loop_ && (loop_->depth() != 0 || loop_->pending() != 0)) || repeat_) {
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

DspLoopArchitecture DspProgramFetch::subroutine_source(const DspFetchedWord captured,
                                                       DspLoopArchitecture observed) const {
  const auto index = position(captured);
  const auto form = decode_branch_form(captured.word.value);
  if (!form || (!form->subroutine_call && !form->subroutine_return)) {
    throw Fault("Unavailable owned subroutine projection read P:" + hex(captured.word.address, 6) +
                " pc=" + hex(observed.pc, 6) + " omr=" + hex(observed.omr, 6));
  }
  if (loop_) {
    const auto &prediction = predictions_.at(index);
    if (!accepts_loop_control(observed) || !prediction || boundaries_.at(index)) {
      throw Fault("Unqualified owned subroutine context read P:" + hex(captured.word.address, 6) +
                  " pc=" + hex(observed.pc, 6) + " omr=" + hex(observed.omr, 6));
    }
    // F2 already owns these older implicit loop boundaries. This metadata
    // controls fetch only; genuine native E1/E2 must independently confirm it.
    observed = prediction->control();
  }
  observed.pc = captured.word.address;
  return observed;
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
    origins_.at(index) = {};
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
  const bool one_word = form && form->words() == 1;
  if (form && form->absolute_short && loop_ && (loop_->depth() != 0 || loop_->pending() != 0)) {
    // FM A.2.6.1 requires additional jump/LA boundary interlocks. Keep this
    // new form visible until its active-DO target timing is qualified.
    throw Fault("Unqualified short Jcc/active DO read P:" + hex(decision.word.address, 6) +
                " pc=" + hex(decision.word.address, 6) + " omr=" + hex(decision.omr, 6));
  }
  if (form &&
      (form->absolute_extension ||
       ((form->subroutine_call || form->subroutine_return) && !decision.subroutine)) &&
      loop_ && (loop_->depth() != 0 || loop_->pending() != 0)) {
    const auto *const operation = form->absolute_extension
                                      ? "Unqualified absolute JMP/active DO read P:"
                                      : "Unqualified subroutine/active DO read P:";
    throw Fault(operation + hex(decision.word.address, 6) + " pc=" + hex(decision.word.address, 6) +
                " omr=" + hex(decision.omr, 6));
  }
  if (!state_.active || branch_ || opening_ ||
      decision.omr != execution_omr(select(decision.word.address, identity)) ||
      decision.next_pc > word_mask || decision.extension.has_value() == one_word ||
      (form && form->absolute_extension &&
       (!decision.taken || !decision.extension || decision.next_pc != decision.extension->value)) ||
      (decision.extension &&
       decision.extension->address != ((decision.word.address + 1) & word_mask))) {
    throw Fault("Unavailable native branch AG1 read P:" + hex(decision.word.address, 6) +
                " pc=" + hex(decision.word.address, 6) + " omr=" + hex(decision.omr, 6));
  }
  const auto pair = branch_position(decision, identity);
  const auto keep = pair + (decision.extension ? 2U : 1U);
  predict_loop_branch(decision, pair, keep);
  branch_occurrence_ = occurrences_.at(pair);
  for (std::size_t index = keep; index < state_.used; ++index) {
    state_.words.at(index) = {};
    occurrences_.at(index) = 0;
    origins_.at(index) = {};
    predictions_.at(index).reset();
    boundaries_.at(index).reset();
  }
  state_.used = keep;
  state_.pending_address.reset();
  state_.next_address = decision.next_pc;
  branch_ = decision;
  branch_target_captured_ = false;
}

void DspProgramFetch::predict_loop_branch(const DspBranchDecision &decision,
                                          const std::size_t pair,
                                          const std::size_t keep) {
  if (!loop_) { return; }
  const auto &prediction = predictions_.at(keep - 1);
  if (!prediction || boundaries_.at(pair) || boundaries_.at(keep - 1)) {
    throw Fault("Unqualified branch at DSP loop end read P:" + hex(decision.word.address, 6));
  }
  auto candidate = *loop_;
  candidate.discard_after(*prediction, decision.next_pc);
  const auto form = decode_branch_form(decision.word.value);
  if (form && (form->subroutine_call || form->subroutine_return)) {
    if (!decision.subroutine) {
      throw Fault("Missing DSP subroutine forecast read P:" + hex(decision.word.address, 6) +
                  " pc=" + hex(decision.word.address, 6) + " omr=" + hex(decision.omr, 6));
    }
    candidate.predict_subroutine(decision.word, *decision.subroutine, form->subroutine_call);
  }
  loop_ = std::move(candidate);
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

bool DspProgramFetch::confirm_subroutine(const DspFetchCommit completed,
                                         const DspLoopArchitecture native_result) {
  if (!branch_ || !branch_->subroutine || branch_->word.address != completed.pc) { return false; }
  const auto &control = *branch_->subroutine;
  const auto form = decode_branch_form(branch_->word.value);
  if (!form || (!form->subroutine_call && !form->subroutine_return) ||
      control.after != native_result || (observed_loop_ && !accepts_loop_control(control.before))) {
    throw Fault("Native subroutine control differs from owned F2 pc=" + hex(completed.pc, 6) +
                " omr=" + hex(completed.omr, 6));
  }
  if (!confirm_branch(completed.pc, completed.next_pc, completed.omr, completed.redirected)) {
    throw Fault("Native subroutine lost branch ownership pc=" + hex(completed.pc, 6) +
                " omr=" + hex(completed.omr, 6));
  }
  if (observed_loop_) { observed_loop_ = native_result; }
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
  release_loop_history(loop_, observed_loop_, predictions_, boundaries_, native_result);
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
  release_loop_history(loop_, observed_loop_, predictions_, boundaries_, native_result);
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
