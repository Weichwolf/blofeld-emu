// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_machine.hpp"
#include "blofeld/hcs08_bdc.hpp"
#include "blofeld/hcs08_bdc_wire.hpp"
#include "blofeld/hcs08_run_clock.hpp"
#include "blofeld/lcd_image.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard_chain_fixture.hpp"
#include "keyboard_handoff_fixture.hpp"
#include "keyboard_reset_fixture.hpp"
#include "lcd_fixture.hpp"
#include "original_update_forward_fixture.hpp"

#include <memory>

namespace blofeld::test::keyboard_chain {
using hcs08::require;

Saved Chain::checkpoint() const {
  require(sender.pair.mcu.cpu().state().pc == text_entry && texts == 0 && text_returns == 0,
          "Joint chain snapshot follows actual reset/SYNC and precedes the original saving call");
  return {.pair = sender.pair.checkpoint(),
          .peer = peer->checkpoint(),
          .display = *display,
          .sender = {.interrupted = sender.interrupted,
                     .loaded = sender.loaded,
                     .sent = sender.sent,
                     .received = sender.received,
                     .initial_reads = sender.initial_reads,
                     .returned = sender.returned,
                     .irq_entries = sender.irq_entries,
                     .irq_returns = sender.irq_returns,
                     .irq_queries = sender.irq_queries,
                     .irq_unknown = sender.irq_unknown},
          .trace_counts = filter.counts(),
          .samples = samples,
          .bytes = bytes,
          .cursor = cursor,
          .texts = texts,
          .text_returns = text_returns,
          .uploaded = uploaded,
          .buffered = buffered};
}

void Chain::restore(const Saved &saved) {
  auto &pair = sender.pair;
  pair.start(scenario.order == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp,
             scenario.phase,
             kh::prefix_end);
  peer->restore(saved.peer);
  // Replacing both owners drops every old board callback. Reconnect all peers.
  pair.restore_owners(saved.pair);
  pair.reconnect(saved.pair);
  display = std::make_unique<LcdFixture>(pair.mcu.bus(), saved.display);
  sender.interrupted = saved.sender.interrupted;
  sender.loaded = saved.sender.loaded;
  sender.sent = saved.sender.sent;
  sender.received = saved.sender.received;
  sender.initial_reads = saved.sender.initial_reads;
  sender.returned = saved.sender.returned;
  sender.irq_entries = saved.sender.irq_entries;
  sender.irq_returns = saved.sender.irq_returns;
  sender.irq_queries = saved.sender.irq_queries;
  sender.irq_unknown = saved.sender.irq_unknown;
  samples = saved.samples;
  bytes = saved.bytes;
  cursor = saved.cursor;
  texts = saved.texts;
  text_returns = saved.text_returns;
  uploaded = saved.uploaded;
  buffered = saved.buffered;
  filter.restore_counts(saved.trace_counts);
  connect();
}

void Chain::compare_endpoint(const Chain &other) const {
  const auto &pair = sender.pair;
  const auto &op = other.sender.pair;
  require(update_forward::registers(pair.mcu.cpu().state()) ==
                  update_forward::registers(op.mcu.cpu().state()) &&
              pair.dsp.cpu().debug_state() == op.dsp.cpu().debug_state() &&
              pair.schedule->end_ticks() == op.schedule->end_ticks() &&
              pair.schedule->ties() == op.schedule->ties() &&
              pair.schedule->completed_steps() == op.schedule->completed_steps() &&
              peer->endpoint() == other.peer->endpoint() &&
              peer->target.memory.bytes == other.peer->target.memory.bytes &&
              peer->target.memory.known == other.peer->target.memory.known &&
              peer->target.memory.flash.image() == other.peer->target.memory.flash.image() &&
              peer->target.memory.flash.state() == other.peer->target.memory.flash.state() &&
              peer->controller().state() == other.peer->controller().state() &&
              peer->transport().state() == other.peer->transport().state() &&
              peer->run_clock().state() == other.peer->run_clock().state() &&
              peer->reset_sequence().state() == other.peer->reset_sequence().state() &&
              capture_lcd(display->controller(), {.width = 128, .reverse_commons = true}) ==
                  capture_lcd(other.display->controller(), {.width = 128, .reverse_commons = true}),
          "Fresh three-CPU/reset/wire/clock/LCD owners reproduce complete native endpoints");
}
} // namespace blofeld::test::keyboard_chain
