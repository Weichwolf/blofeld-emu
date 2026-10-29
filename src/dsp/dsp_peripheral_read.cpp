// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_peripheral_read.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_peripheral_read_form.hpp"
#include "dsp/dsp_peripheral_read_stage.hpp"

#include <cstdint>
#include <exception>
#include <optional>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned word_mask = 0xffffff;

bool same_address(const DspAddress first, const DspAddress second) {
  return first.space == second.space && first.address == second.address;
}
}

std::string DspPeripheralRead::context() const {
  return std::string("read ") + (form_.target.space == DspSpace::x ? "X:" : "Y:") +
         hex(form_.target.address, 6) + " pc=" + hex(word_.address, 6) + " omr=" + hex(omr_, 6);
}

DspPeripheralRead DspPeripheralRead::capture(DspBus &bus,
                                             const DspProgramFetch &fetch,
                                             const DspPeripheralReadPoint point,
                                             const std::uint64_t occurrence) {
  const auto address = point.address;
  const auto opcode = fetch.select(address, occurrence);
  const auto form = decode_peripheral_read(opcode.word.value);
  if (!fetch.active() || !form) {
    throw Fault("Peripheral read requires an actual qualified F2 word at P:" + hex(address, 6));
  }
  DspPeripheralRead result;
  result.fetched_ = fetch.instruction(opcode, form->words);
  result.word_ = result.fetched_.opcode.word;
  result.form_ = *form;
  result.sample_omr_ = fetch.view().omr;
  result.omr_ = fetch.execution_omr(opcode);
  result.epoch_ = point.epoch;
  if (result.fetched_.extension) { result.extension_ = result.fetched_.extension->word; }
  try {
    if (form->bit && DspBus::handles_peripheral_bit(form->target)) {
      const auto value = bus.access_peripheral_bit(
          {.space = form->target.space, .address = form->target.address, .bit = *form->bit});
      if (!value) { throw Fault("Peripheral bit route changed before capture"); }
      result.kind_ = DspNativeAccessKind::bit;
      result.value_ = *value ? 1U : 0U;
    } else {
      result.value_ = bus.read(form->target);
    }
  } catch (const std::exception &error) {
    throw Fault("Early peripheral " + result.context() + ": " + error.what());
  }
  return result;
}

void DspPeripheralRead::follow_context(const DspFetchContext &transition) {
  const auto writer = transition.writer.extension.value_or(transition.writer.opcode);
  if (fetched_.opcode.occurrence <= writer.occurrence) { return; }
  if (!active_ || (omr_ != transition.before && omr_ != transition.after)) {
    throw Fault("Peripheral context lost its actual preceding native OMR result: " + context());
  }
  // The physical sample and its original context remain immutable. Only the
  // later native execution context follows this actual older control result.
  omr_ = transition.after;
}

bool DspPeripheralRead::forward(const DspProgramWord producer,
                                const DspNativeAccess &completed_store,
                                const std::uint64_t epoch) {
  const auto form = decode_memory_stage(producer.value);
  if (!active_ || completed_store.kind != DspNativeAccessKind::write ||
      completed_store.stage != DspNativeAccessStage::after || !completed_store.value ||
      *completed_store.value > word_mask || completed_store.pc != producer.address ||
      completed_store.omr != omr_ || !form || !form->peripheral ||
      !same_address(*form->peripheral, completed_store.target)) {
    throw Fault("Peripheral forwarding lost its actual native producer: " + context());
  }
  if (!form_.bit || !same_address(form_.target, completed_store.target) || epoch < epoch_) {
    return false;
  }
  constexpr unsigned read_to_execute = 2;
  if (epoch - epoch_ > read_to_execute || (forward_epoch_ && epoch <= *forward_epoch_)) {
    throw Fault("Unqualified late/repeated peripheral forwarding: " + context());
  }
  const auto value = *completed_store.value;
  value_ = kind_ == DspNativeAccessKind::bit ? ((value >> *form_.bit) & 1U) : value;
  forward_epoch_ = epoch;
  return true;
}

void DspPeripheralRead::check_word(const DspNativeAccess &access,
                                   const DspProgramWord word,
                                   const DspNativeAccessKind kind,
                                   const std::uint64_t ordinal) const {
  if (access.kind != kind || access.stage != DspNativeAccessStage::after ||
      access.target.space != DspSpace::p || access.target.address != word.address ||
      access.value != word.value || access.pc != word_.address || access.omr != omr_ ||
      access.ordinal != ordinal || access.bit) {
    throw Fault("Peripheral read differs from its actual native F2 pair: " + context());
  }
}

std::uint32_t DspPeripheralRead::consume(const DspNativeAccess &opcode,
                                         const std::optional<DspNativeAccess> &extension,
                                         const DspNativeAccess &pending,
                                         const std::uint64_t epoch) {
  constexpr unsigned read_to_execute = 2;
  if (!active_ || epoch < epoch_ || epoch - epoch_ != read_to_execute) {
    throw Fault("Unavailable peripheral response/phase: " + context());
  }
  check_word(opcode, word_, DspNativeAccessKind::opcode, 0);
  if (extension.has_value() != extension_.has_value()) {
    throw Fault("Peripheral read lost its actual native extension form: " + context());
  }
  if (extension) { check_word(*extension, *extension_, DspNativeAccessKind::extension, 1); }
  std::optional<DspNativeAccessBit> expected_bit;
  if (kind_ == DspNativeAccessKind::bit) {
    if (!form_.bit) { throw Fault("Peripheral bit response lost its bit index: " + context()); }
    expected_bit = DspNativeAccessBit{.index = *form_.bit};
  }
  if (pending.kind != kind_ || pending.stage != DspNativeAccessStage::before || pending.value ||
      !same_address(pending.target, form_.target) || pending.pc != word_.address ||
      pending.omr != omr_ || pending.ordinal != form_.words || pending.bit != expected_bit) {
    throw Fault("Peripheral response differs from actual native read: " + context());
  }
  active_ = false;
  return value_;
}
}

namespace blofeld {
DspBranchDecision DspPeripheralRead::preview_branch(const std::uint64_t epoch) const {
  if (!active_ || !form_.bit || !extension_ || epoch < epoch_ || epoch - epoch_ != 1) {
    throw Fault("Unavailable owned peripheral branch preview: " + context());
  }
  const bool value =
      kind_ == DspNativeAccessKind::bit ? value_ != 0 : ((value_ >> *form_.bit) & 1U) != 0;
  const bool taken = value == form_.set;
  const auto target =
      form_.relative ? (word_.address + extension_->value) & word_mask : extension_->value;
  return {.word = word_,
          .extension = extension_,
          .omr = omr_,
          .next_pc = taken ? target : ((extension_->address + 1) & word_mask),
          .taken = taken};
}
}
