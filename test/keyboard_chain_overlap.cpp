// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_schedule.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard/hcs08_run_clock.hpp"
#include "keyboard_chain_fixture.hpp"
#include "keyboard_handoff_fixture.hpp"

#include <ostream>
#include <string>

namespace blofeld::test::keyboard_chain {
using hcs08::require;
constexpr unsigned saving_writes = 1172, saving_reads = 296, send_bit = 0x143f;

void Chain::until_saving() {
  while (sender.pair.mcu.cpu().state().pc != text_entry) {
    require(sender.pair.link.msteps < max_steps, "Original reset/SYNC reaches its real text call");
    step();
  }
  require(
      cursor == 1 && peer->target_owner().bdc_state().serviced == 0 &&
          peer->target_owner().wire_state().service_due.has_value() &&
          peer->target_owner().wire_state().syncs == 2 && peer->ram_known().none() &&
          peer->target_owner().run_state().retired_instructions == 0,
      "Actual reset/SYNC precedes drawing with BACKGROUND pending and no supplied target upload");
  trace << "SAVING_BOUNDARY pc=" << sender.pair.mcu.cpu().state().pc
        << " sp=" << sender.pair.mcu.cpu().state().sp << " rx=" << cursor
        << " commands=" << peer->target_owner().bdc_state().serviced << '\n';
}

void Chain::until_overlap() {
  const auto reason = hcs08::rejected([&] { until_buffer(); });
  const auto &pair = sender.pair;
  const auto wire = peer->target_owner().wire_state();
  require(reason.contains("pc=0x143f") && reason.contains("MCU write at 0x0001") &&
              reason.contains("host falling edge before the preceding bit/SYNC finished") &&
              pair.mcu.cpu().state().pc == send_bit && peer->target_owner().faulted() &&
              pair.mcu.timing_faulted() && pair.schedule->faulted(),
          "Declared direct LCD/BKGD reference refuses the unsupported original bit sequence");
  require(texts == 2 && text_returns == 1 && samples == 4 && bytes == 3 && cursor == 1 &&
              sender.sent == kh::complete_samples && sender.loaded == kh::checksum_word + 2 &&
              !uploaded && !buffered && peer->target_owner().bdc_state().serviced == 1 &&
              wire.syncs == 2 && wire.received_bytes == 1 && wire.transmitted_bytes == 0 &&
              peer->ram_known().none() &&
              peer->target_owner().run_state().retired_instructions == 0 &&
              display->counts().writes == saving_writes && display->counts().reads == saving_reads,
          "Complete real drawing precedes the strict wire failure without any trim/page/GO result");
  require(filter.counts().dsp == pair.link.dsteps && filter.counts().mcu == pair.link.msteps + 1,
          "Compressed trace accounts for every native DSP completion and MCU step attempt");
  trace << "DIAGNOSTIC " << reason << '\n';
  trace << "WIRE_FRONTIER pc=" << pair.mcu.cpu().state().pc << " rx=" << cursor
        << " commands=" << peer->target_owner().bdc_state().serviced << " syncs=" << wire.syncs
        << " writes=" << display->counts().writes << " reads=" << display->counts().reads
        << " host-steps=" << pair.link.msteps << " target-retired=0 uploaded=0 buffered=0\n";
  guards();
}
} // namespace blofeld::test::keyboard_chain
