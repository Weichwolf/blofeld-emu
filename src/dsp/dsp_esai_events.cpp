// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_esai_registers.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>

namespace blofeld {
unsigned DspEsai::launches_until_event() const {
  const auto layout = format();
  const auto period = layout.slot * frame_slots();
  const bool bit_sync = (control_ & esai::bit_sync) != 0;
  const auto distance = [&](const unsigned phase) {
    return ((phase + period - frame_position_) % period) + 1;
  };
  // Generated FST boundaries and the bit-delayed SAISR frame flag are visible
  // independently of data transfer. Slot loads retain mask/underrun/IRQ effects.
  unsigned launches = std::min(distance(0), distance(bit_sync ? 1U : layout.slot));
  if (bit_sync) { launches = std::min(launches, distance(layout.slot + 1)); }
  if (!frame_clocked_ || pending_frame_) { return 1; }
  if (frame_) { launches = std::min(launches, layout.slot - position_ + 1); }
  return launches;
}

std::uint64_t DspEsai::high_edges_for_toggles(const std::uint64_t toggles) const {
  if (toggles == 0 || !observed_.high_clock) {
    throw Fault(
        "Periodic DSP ESAI divider requires a known HCKT level and positive toggle count at " +
        address(esai::tccr));
  }
  const auto divider = high_clock_divisor();
  const bool polarity = (clock_control_ & esai::high_clock_polarity) != 0;
  const auto first = *observed_.high_clock == polarity ? 1U : 2U;
  const auto first_toggle = first + (2 * (divider - high_phase_ - 1));
  return first_toggle + ((toggles - 1) * 2 * divider);
}

std::optional<std::uint64_t> DspEsai::high_clock_edges_until_event() const {
  if (!high_clock_event_valid_) {
    high_clock_event_ = compute_high_clock_event();
    high_clock_event_valid_ = true;
  }
  return high_clock_event_;
}

std::optional<std::uint64_t> DspEsai::compute_high_clock_event() const {
  validate();
  if (!generator_running() && !needs_transmit_clock()) { return std::nullopt; }
  if (!clock_generator()) {
    throw Fault("Periodic DSP ESAI HCKT requires generated SCKT at " + address(esai::tccr));
  }
  if (!observed_.high_clock || observed_.timing != DspEsaiClockTiming::reference_reset_edges) {
    throw Fault("Periodic DSP ESAI HCKT requires its initial level at " + address(esai::tccr));
  }
  const bool launch_level = (clock_control_ & esai::clock_polarity) == 0;
  const std::uint64_t first_launch = generated_clock_ != launch_level ? 1U : 2U;
  auto toggles =
      frame_generator() ? first_launch + (std::uint64_t{2} * (launches_until_event() - 1)) : 1;
  if ((control_ & esai::bit_sync) != 0 && needs_transmit_clock() &&
      (generated_sync_ || previous_sync_)) {
    // Sampling FST can arm a frame or diagnose an overlong bit-sync pulse.
    const std::uint64_t first_sample = generated_clock_ == launch_level ? 1U : 2U;
    toggles = std::min(toggles, first_sample);
  }
  return high_edges_for_toggles(toggles);
}

void DspEsai::skip_quiet_clock_toggles(const std::uint64_t toggles) {
  const bool launch_level = (clock_control_ & esai::clock_polarity) == 0;
  const auto launches =
      (toggles / 2) + (toggles % 2 != 0 && generated_clock_ != launch_level ? 1U : 0U);
  if (launches >= launches_until_event()) {
    throw Fault("DSP ESAI quiet span crosses a transmitter event at " + address(esai::tccr));
  }
  cold_configuration_ = false;
  generated_clock_ = generated_clock_ != (toggles % 2 != 0);
  previous_clock_ = generated_clock_;
  if (launches == 0) { return; }
  const auto count = static_cast<unsigned>(launches);
  const auto period = format().slot * frame_slots();
  frame_position_ = (frame_position_ + count) % period;
  update_frame_flag();
  if (frame_ && needs_transmit_clock()) {
    position_ += count;
    --position_;
    emit_bit();
    ++position_;
  }
}

void DspEsai::advance_high_clock_checked(const std::uint64_t edges) {
  if (edges == 0) { return; }
  const auto deadline = high_clock_edges_until_event();
  if (deadline && edges > *deadline) {
    throw Fault("Periodic DSP ESAI span crosses a transmitter event at " + address(esai::tccr));
  }
  if (!observed_.high_clock) {
    throw Fault("Periodic DSP ESAI span requires a known HCKT baseline at " + address(esai::tccr));
  }
  const auto final_level = *observed_.high_clock != (edges % 2 != 0);
  if (!deadline) {
    auto final = observed_;
    final.high_clock = final_level;
    observe_checked(final);
    high_clock_event_ = std::nullopt;
    high_clock_event_valid_ = true;
    return;
  }
  // Apply the same baseline/configuration checks once for this owned span.
  observe_checked(observed_);
  std::uint64_t completed = 0;
  const auto first_toggle = high_edges_for_toggles(1);
  if (frame_generator() && edges >= first_toggle) {
    const auto toggles = 1 + ((edges - first_toggle) / (std::uint64_t{2} * high_clock_divisor()));
    if (toggles > 1) {
      completed = high_edges_for_toggles(toggles - 1);
      skip_quiet_clock_toggles(toggles - 1);
      observed_.high_clock = *observed_.high_clock != (completed % 2 != 0);
      high_phase_ = 0;
    }
  }
  const bool polarity = (clock_control_ & esai::high_clock_polarity) != 0;
  while (completed < edges) {
    const auto to_toggle = high_edges_for_toggles(1);
    const auto count = std::min(edges - completed, to_toggle);
    const auto first = *observed_.high_clock == polarity ? 1U : 2U;
    const auto selected = count < first ? 0 : 1 + ((count - first) / 2);
    high_phase_ += static_cast<unsigned>(selected);
    observed_.high_clock = *observed_.high_clock != (count % 2 != 0);
    completed += count;
    if (count != to_toggle) { break; }
    high_phase_ = 0;
    generated_clock_ = !generated_clock_;
    clock_edge(generated_clock_, observed_.frame_sync);
  }
  // Quiet prefixes retain the same next event; only its remaining distance changes.
  high_clock_event_valid_ = edges < *deadline;
  if (high_clock_event_valid_) { high_clock_event_ = *deadline - edges; }
}

void DspEsai::advance_high_clock(const std::uint64_t edges) {
  auto next = *this;
  next.advance_high_clock_checked(edges);
  *this = next;
}
}
