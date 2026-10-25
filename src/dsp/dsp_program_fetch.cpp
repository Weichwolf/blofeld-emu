// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/detail/dsp_program_fetch.hpp"

#include "common/bus.hpp"
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
  const auto value = bus.read({.address = address});
  state_.words.at(state_.used++) = {.address = address, .value = value};
  state_.pending_address.reset();
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
}
}
