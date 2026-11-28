// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/foreground.hpp"

#include "dsp/dax.hpp"
#include "dsp/dma.hpp"
#include "dsp/esai.hpp"
#include "dsp/io.hpp"
#include "dsp/timers.hpp"

#include <cstdint>
#include <dsp56300/memory.hpp>
#include <initializer_list>
#include <limits>

namespace blofeld::vi::dsp {
bool Io::foreground_timer_quiet(const std::uint64_t end, const bool first) const noexcept {
  Timers timers = timers_;
  if (!timers.advance(end) || timers.interrupt()) { return false; }
  auto dma = dma_;
  dma.trigger_sources(timers.requests() << timer_request_first);
  if (first) { dma.trigger_sources(audio_requests()); }
  return !dma.pending();
}

bool Io::foreground_audio_quiet(const std::uint64_t end, const ForegroundStores stores) const {
  Esai esai = esai_1_;
  Dax dax = dax_;
  if (!audio_.preview_clock(esai, dax, end) || dax.interrupt()) { return false; }
  auto dma = dma_;
  dma.trigger_sources((static_cast<unsigned>(esai.request()) << esai1_transmit_request) |
                      (static_cast<unsigned>(dax.request()) << dax_request));
  return dma.service(
             dma_pc_,
             [&](const Dma::Address input, dsp56300::Word &value) {
               const auto word = memory_.physical_word(input.space, input.address);
               return word && !stores.contains(stores.context, *word) &&
                      memory_.read(input.space, input.address, value);
             },
             [&](const Dma::Address target, const dsp56300::Word value) {
               return write_audio_dma(target, value, esai, dax);
             }) &&
         dma.interrupts() == 0 && !dax.interrupt();
}

ForegroundCrossing Io::foreground_crossing(const std::uint64_t grant,
                                           const std::uint64_t cycles,
                                           const ForegroundStores stores) const {
  using Crossing = ForegroundCrossing;
  if (core_ == nullptr || stores.contains == nullptr || cycles <= grant || stop_.stop_requested() ||
      reset_.pending(epoch_) || audio_dirty_ || timers_.requests() != 0 || dma_.pending() ||
      dma_.failure()) {
    return Crossing::blocked;
  }
  const auto now = device_cycles();
  if (cycles > std::numeric_limits<std::uint64_t>::max() - now) { return Crossing::blocked; }
  const auto end = now + cycles;
  for (const auto deadline : {media_deadline_, midi_deadline_, peer_deadline_}) {
    if (deadline && *deadline <= end) { return Crossing::blocked; }
  }
  if (dma_.interrupts() != 0 || dax_.interrupt() || timers_.interrupt() || peer_interrupt()) {
    return Crossing::blocked;
  }

  const auto audio_deadline = audio_.deadline();
  const auto timer_deadline = timers_.deadline();
  const bool timer_due = timer_deadline && *timer_deadline <= end;
  const bool timer_first = timer_due && (!audio_deadline || *timer_deadline < *audio_deadline);
  const auto first = timer_first ? timer_deadline : audio_deadline;
  if (!first || *first != now + grant) { return Crossing::blocked; }

  bool ready = false;
  if (controller_.ready(ready) && !timers_.input_is(1, ready)) { return Crossing::timer_effect; }
  if (timer_due && !foreground_timer_quiet(end, timer_first)) { return Crossing::timer_effect; }
  if (!audio_deadline || *audio_deadline > end) { return Crossing::quiet; }
  return foreground_audio_quiet(end, stores) ? Crossing::quiet : Crossing::blocked;
}
}
