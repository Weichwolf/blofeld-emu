// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_loop_fetch.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_loop_source.hpp"
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
         (relative_loop(opcode) || (opcode & immediate_mask) == immediate_do ||
          opcode == do_forever ||
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
  const bool relative = relative_loop(word.value);
  const auto last =
      relative ? (word.address + opening.extension.value) & word_mask : opening.extension.value;
  const auto end = (last + 1) & word_mask;
  const bool forever = word.value == do_forever;
  if (!qualified(word.value) || word.address != before.pc ||
      opening.extension.address != ((word.address + 1) & word_mask) || last > word_mask ||
      body > last ||
      (relative &&
       (word.address > word_mask - 2 || opening.extension.value > word_mask - word.address)) ||
      pending_ != 0 || next_ != body || !same_control(before, forecast_) ||
      revision_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("Unqualified actual DSP DO opening " + context(word, before));
  }
  if (depth_ != 0) {
    const auto &parent = frames_.at(depth_ - 1);
    const bool lexical =
        word.address >= parent.body && word.address + 2 < parent.last && last < parent.last;
    const bool called =
        calls_ > parent.calls_at_entry && (last < parent.body || word.address > parent.last);
    if (!lexical && !called) {
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
  frames_.at(depth_++) = {
      .restore = before, .body = body, .last = last, .forever = forever, .calls_at_entry = calls_};
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
  if (address != next_ || (end && (pending_ == boundaries_.size() ||
                                   calls_ != frames_.at(depth_ - 1).calls_at_entry))) {
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
  result.calls_ = calls_;
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
  calls_ = prediction.calls_;
  occurrence_ = prediction.cut_;
  next_ = next_pc;
  ++revision_;
}

void DspLoopFetch::predict_subroutine(const DspProgramWord word,
                                      const DspSubroutineControl &control,
                                      const bool call) {
  constexpr unsigned last_stack_frame = 15;
  constexpr unsigned restricted_end_words = 2; // FM A.3.1: LA-2,LA-1,LA.
  if (!same_control(control.before, forecast_) || control.before.sp > last_stack_frame ||
      (!call && control.before.sp == 0)) {
    throw Fault("Unqualified subroutine/active DO read P:" + hex(word.address, 6) +
                " pc=" + hex(control.before.pc, 6) + " omr=" + hex(control.before.omr, 6));
  }
  auto expected = control.before;
  expected.pc = next_;
  expected.sp = call ? expected.sp + 1 : expected.sp - 1;
  expected.sc = (call ? expected.sc + 1 : expected.sc + max_stack_count) & max_stack_count;
  // calls_ counts calls since this loop owner began. An older native caller
  // can return after every tracked DO frame closes; active loops still require
  // a call recorded inside their own scope.
  bool unsafe = control.after != expected || (call && control.before.sp >= last_stack_frame) ||
                (call ? calls_ >= last_stack_frame : (calls_ == 0 && depth_ != 0));
  // Original1.25 requires finite short BSR at LA-1/-2;independent software
  // windows qualify only this bounded exception to FM A.3.1 (see reference manifest).
  constexpr unsigned short_mask = 0xfffc20;
  constexpr unsigned short_bsr = 0x050800;
  const bool short_call = call && word.value <= word_mask && (word.value & short_mask) == short_bsr;
  for (std::size_t index = 0; index < depth_; ++index) {
    const auto &frame = frames_.at(index);
    unsafe = unsafe ||
             (word.address >= frame.body && word.address <= frame.last &&
              frame.last - word.address <= restricted_end_words &&
              (!short_call || frame.forever || word.address == frame.last)) ||
             (call && next_ >= frame.body && next_ <= frame.last);
  }
  if (!call && depth_ != 0) {
    const auto &frame = frames_.at(depth_ - 1);
    unsafe = unsafe || calls_ <= frame.calls_at_entry || control.before.sp <= frame.restore.sp + 2;
  }
  if (unsafe) {
    throw Fault("Unqualified subroutine/active DO read P:" + hex(word.address, 6) +
                " pc=" + hex(control.before.pc, 6) + " omr=" + hex(control.before.omr, 6));
  }
  forecast_ = control.after;
  if (call) {
    ++calls_;
  } else if (calls_ != 0) {
    --calls_;
  }
}
}
