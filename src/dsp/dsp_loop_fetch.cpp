// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_loop_fetch.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_program_fetch.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned loop_flag = 0x8000;
constexpr unsigned forever_flag = 0x10000;
constexpr unsigned loop_flags = loop_flag | forever_flag;
constexpr unsigned safe_stack_pointer = 13;
constexpr unsigned do_forever = 0x000203;
constexpr unsigned unsupported_status = 0xa2000; // CE, SA, SC.
constexpr unsigned condition_mask = 0xff;
constexpr unsigned max_stack_count = 31;

std::string context(const DspProgramWord word, const DspLoopArchitecture &cpu) {
  return "read P:" + hex(word.address, 6) + " pc=" + hex(cpu.pc, 6) + " omr=" + hex(cpu.omr, 6);
}

bool qualified(const unsigned opcode) {
  constexpr unsigned immediate_mask = 0xff00f0;
  constexpr unsigned immediate_do = 0x060080;
  constexpr unsigned register_mask = 0xffc0ff;
  constexpr unsigned register_do = 0x06c000;
  constexpr unsigned forbidden_ssh = 0x3c;
  constexpr unsigned source_mask = 0x3f;
  return opcode <= word_mask &&
         ((opcode & immediate_mask) == immediate_do || opcode == do_forever ||
          ((opcode & register_mask) == register_do &&
           ((opcode >> 8) & source_mask) != forbidden_ssh));
}

bool same_control(DspLoopArchitecture first, DspLoopArchitecture second) {
  first.pc = 0;
  second.pc = 0;
  return first == second;
}
}

DspLoopFetch::DspLoopFetch(const DspLoopArchitecture initial)
    : forecast_(initial), next_(initial.pc) {
  if ((initial.status & (loop_flags | unsupported_status | condition_mask)) != 0 ||
      initial.pc > word_mask || initial.omr > word_mask || initial.la > word_mask ||
      initial.lc > word_mask || initial.sp > word_mask || initial.sc > max_stack_count) {
    throw Fault("Unqualified initial DSP loop fetch " + context({.address = initial.pc}, initial));
  }
}

void DspLoopFetch::opened(const DspLoopOpening &opening) {
  const auto before = opening.before;
  const auto after = opening.after;
  const auto word = opening.word;
  const auto body = (word.address + 2) & word_mask;
  const auto last = opening.extension.value;
  const auto end = (last + 1) & word_mask;
  const bool forever = word.value == do_forever;
  if (!qualified(word.value) || word.address != before.pc ||
      opening.extension.address != ((word.address + 1) & word_mask) || last > word_mask ||
      body > last || pending_ != 0 || next_ != body || !same_control(before, forecast_) ||
      revision_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("Unqualified actual DSP DO opening " + context(word, before));
  }
  if (depth_ != 0) {
    const auto &parent = frames_.at(depth_ - 1);
    if (word.address < parent.body || word.address + 2 >= parent.last || last >= parent.last) {
      throw Fault("Unqualified nested DSP loop extent " + context(word, before));
    }
  }
  if (!forever && after.pc == end) {
    auto expected = before;
    expected.pc = end;
    if (after != expected) {
      throw Fault("Annulled DSP DO changed control " + context(word, before));
    }
    next_ = end;
    ++revision_;
    return;
  }
  auto expected = before;
  expected.pc = body;
  expected.la = last;
  expected.lc = forever ? before.lc : after.lc;
  expected.sp += 2;
  expected.sc += 2;
  expected.status = (before.status & ~loop_flags) | loop_flag | (forever ? forever_flag : 0U);
  if ((!forever && after.lc == 0) || after != expected || before.sp > safe_stack_pointer ||
      depth_ == frames_.size()) {
    throw Fault("Unqualified native DSP loop frame " + context(word, before));
  }
  frames_.at(depth_++) = {.restore = before, .body = body, .last = last, .forever = forever};
  forecast_ = after;
  ++revision_;
}

std::uint64_t DspLoopFetch::next_occurrence() {
  auto previous = sequence_->serial.load(std::memory_order_relaxed);
  for (;;) {
    if (previous == std::numeric_limits<std::uint64_t>::max()) {
      throw Fault("DSP loop occurrence identity exhausted " +
                  context({.address = next_}, forecast_));
    }
    if (sequence_->serial.compare_exchange_weak(
            previous, previous + 1, std::memory_order_relaxed)) {
      return previous + 1;
    }
  }
}

void DspLoopFetch::check_fetch(const std::uint32_t address) const {
  const bool end = depth_ != 0 && address == frames_.at(depth_ - 1).last;
  if (address != next_ || (end && pending_ == boundaries_.size())) {
    throw Fault("Unavailable DSP loop F2 " + context({.address = address}, forecast_));
  }
}

std::optional<DspLoopBoundary> DspLoopFetch::fetched(const DspProgramWord word) {
  check_fetch(word.address);
  if (word.address != next_ || word.value > word_mask ||
      occurrence_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("Unqualified DSP loop F2 occurrence " + context(word, forecast_));
  }
  const bool end = depth_ != 0 && word.address == frames_.at(depth_ - 1).last;
  if (end && pending_ == boundaries_.size()) {
    throw Fault("DSP loop boundary queue exhausted " + context(word, forecast_));
  }
  occurrence_ = next_occurrence();
  std::move(history_.begin() + 1, history_.end(), history_.begin());
  history_.back() = occurrence_;
  next_ = (word.address + 1) & word_mask;
  if (!end) { return std::nullopt; }
  const auto frame = frames_.at(depth_ - 1);
  const bool backedge = frame.forever || forecast_.lc != 1;
  if (!frame.forever && forecast_.lc == 1) {
    forecast_ = frame.restore;
    --depth_;
    frames_.at(depth_) = {};
  } else {
    if (!frame.forever) { --forecast_.lc; }
    next_ = frame.body;
  }
  forecast_.pc = next_;
  const DspLoopBoundary result{
      .last = word, .occurrence = occurrence_, .expected = forecast_, .backedge = backedge};
  boundaries_.at(pending_++) = result;
  return result;
}

void DspLoopFetch::confirm(const DspLoopBoundary &boundary,
                           const DspLoopArchitecture native_result) {
  if (pending_ == 0 || boundary != boundaries_.front() || boundary.expected != native_result) {
    throw Fault("Native DSP loop end differs from owned F2 " +
                context(boundary.last, native_result));
  }
  confirmed_ = boundary.occurrence;
  std::move(boundaries_.begin() + 1,
            boundaries_.begin() + static_cast<std::ptrdiff_t>(pending_),
            boundaries_.begin());
  boundaries_.at(--pending_) = {};
}

DspLoopFetch::Prediction DspLoopFetch::prediction() const {
  Prediction result;
  result.frames_ = frames_;
  result.forecast_ = forecast_;
  result.sequence_ = sequence_;
  result.cut_ = occurrence_;
  result.revision_ = revision_;
  result.depth_ = depth_;
  return result;
}

void DspLoopFetch::discard_after(const Prediction &prediction, const std::uint32_t next_pc) {
  const bool retained = prediction.cut_ == 0 || std::ranges::contains(history_, prediction.cut_);
  if (prediction.sequence_ != sequence_ || prediction.revision_ != revision_ || !retained ||
      prediction.cut_ < confirmed_ || prediction.cut_ > occurrence_ || next_pc > word_mask ||
      revision_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("Stale, foreign or confirmed DSP loop discard " +
                context({.address = next_pc}, forecast_));
  }
  while (pending_ != 0 && boundaries_.at(pending_ - 1).occurrence > prediction.cut_) {
    boundaries_.at(--pending_) = {};
  }
  for (auto &entry : history_) {
    if (entry > prediction.cut_) { entry = 0; }
  }
  frames_ = prediction.frames_;
  forecast_ = prediction.forecast_;
  depth_ = prediction.depth_;
  occurrence_ = prediction.cut_;
  next_ = next_pc;
  ++revision_;
}
}
