// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard_chain_fixture.hpp"
#include "keyboard_handoff_fixture.hpp"
#include "keyboard_reset_fixture.hpp"
#include "lcd_fixture.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_machine.hpp"

#include <algorithm>
#include <cstdint>
#include <ostream>
#include <string>

namespace blofeld::test::keyboard_chain {
using hcs08::require;
constexpr unsigned upload_bytes = 164, stream_bytes = 517, stream_samples = 692, sent_chunks = 2052,
                   loaded_words = 514;
constexpr unsigned buffer_rx = 2709, buffer_commands = 679, target_buffer = 0x200;
constexpr unsigned saving_pointer = 0x15d0, saving_x = 8, saving_y = 16, saving_font = 2;
constexpr unsigned measure_return = 0xa73c, text_arguments = 7, recursive_depth = 13;
constexpr unsigned low_delay = 0x1600, high_delay = 0x1601;
constexpr unsigned byte_mask = 255, trim_slot = 0x1602, trim = 0xa5;

void Chain::step() {
  auto &pair = sender.pair;
  const auto before = pair.mcu.cpu().state();
  if (before.pc == text_entry) {
    const auto returned = pair.mcu.bus().read_memory({.address = before.sp, .size = 2});
    const auto mode = before.d & byte_mask;
    require((texts == 0 && returned == text_return && before.sp == frame - text_arguments &&
             mode == 0) ||
                (texts == 1 && returned == measure_return &&
                 before.sp == frame - text_arguments - recursive_depth && mode == 3),
            "Original callback and recursive width measurement retain their actual frames");
    const auto argument = [&](const unsigned offset) {
      return pair.mcu.bus().peek(static_cast<std::uint16_t>(before.sp + offset));
    };
    const auto pointer = pair.mcu.bus().read_memory(
        {.address = static_cast<std::uint16_t>(before.sp + 2), .size = 2});
    require(pointer == saving_pointer && argument(4) == saving_y && argument(5) == saving_x &&
                argument(6) == saving_font,
            "Original outer/recursive text arguments retain the actual stack ABI");
    ++texts;
    trace << "TEXT mode=" << mode << " return=" << returned << " pointer=" << pointer
          << " sp=" << before.sp << " x=" << static_cast<unsigned>(argument(5))
          << " y=" << static_cast<unsigned>(argument(4))
          << " font=" << static_cast<unsigned>(argument(6)) << '\n';
  }
  pair.step();
  const auto after = pair.mcu.cpu().state();
  if (after.pc == text_return) {
    require(texts == 2 && text_returns == 0, "Original resident text function really returns");
    ++text_returns;
    trace << "TEXT_RETURN sp=" << after.sp << '\n';
  }
  if (std::ranges::find(kh::sample_pcs, before.pc) != kh::sample_pcs.end()) {
    const auto index = samples % 4;
    require(before.pc == kh::sample_pcs.at(index),
            "Original live cache refill samples its four actual pin groups in order");
    const auto value = index == 0 || index == 3 ? after.d >> 8U : after.d & byte_mask;
    const auto expected_value =
        sender.chunk(kh::resident_samples + samples) | (index % 2 == 0 ? 128U : 0U);
    require(value == expected_value,
            "Original helper samples the actual source's next held DSP data");
    trace << "STREAM_SAMPLE index=" << samples << " value=" << value << '\n';
    ++samples;
  }
  if (before.pc == byte_return) {
    require((after.d & byte_mask) == source_byte(data_offset + bytes),
            "Actual original byte/cache return differs from complete sender payload");
    trace << "STREAM_BYTE index=" << bytes << " value=" << (after.d & byte_mask) << '\n';
    ++bytes;
  }
  if (after.pc == page_ready && !uploaded) {
    require(text_returns == 1 && peer->target.memory.known.count() == upload_bytes &&
                pair.mcu.bus().peek(trim_slot) == trim,
            "Actual reset/drawing/trim read precede complete original RAM upload");
    uploaded = true;
    trace << "UPLOAD commands=" << peer->controller().state().serviced << " rx=" << peer->received
          << " known=" << peer->target.memory.known.count() << '\n';
  }
}

void Chain::until_buffer() {
  auto &pair = sender.pair;
  while (pair.mcu.cpu().state().pc != buffer_ready) {
    require(pair.link.msteps < max_steps,
            "Complete original helper reaches the first native page buffer");
    step();
  }
  require(pair.mcu.bus().peek(low_delay) == 9 && pair.mcu.bus().peek(high_delay) == 12 &&
              pair.mcu.cpu().state().sp == frame && uploaded && bytes == stream_bytes &&
              samples == stream_samples && sender.sent == sent_chunks &&
              sender.loaded == loaded_words && cursor == buffer_rx &&
              peer->controller().state().serviced == buffer_commands,
          "Original DSP stream/address/cache/BDC upload reaches one coherent first-page boundary");
  for (unsigned n = 0; n < page_bytes; ++n) {
    require(peer->target.memory.known[target_buffer + n] &&
                peer->target.memory.bytes[target_buffer + n] == source_byte(page_offset + 2 + n),
            "Every original page byte reaches target RAM through the actual three-CPU transport");
  }
  buffered = true;
  trace << "BUFFER sp=" << pair.mcu.cpu().state().sp << " bytes=" << bytes << " samples=" << samples
        << " sent=" << sender.sent << " loaded=" << sender.loaded << " rx=" << cursor
        << " commands=" << peer->controller().state().serviced << '\n';
}

void Chain::guards() {
  auto &pair = sender.pair;
  const auto image = pair.mcu.bus().power_off_flash_image();
  require(std::ranges::equal(image.bytes, firmware.mcu) && image.known.all() && image.fresh.none(),
          "Original full programmer prefix preserves all MCU NVM/knowledge/history");
  for (unsigned n = 0; n < helper.size(); ++n) {
    require(pair.mcu.bus().peek(static_cast<std::uint16_t>(kr::helper_base + n)) == helper[n],
            "Entire actually copied helper remains unchanged");
  }
  constexpr unsigned vector = 0xfffe;
  const auto target_image = peer->target.memory.flash.image();
  require(!peer->target.cpu.state().pc_known && !target_image.known[vector - hcs08_flash::first] &&
              !target_image.known[vector + 1 - hcs08_flash::first],
          "Continuous original chain preserves pre-vector target PC and NVM unknownness");
  require(pair.link.unresolved_queries == 0 && sender.irq_unknown == 0 &&
              peer->target.memory.transfers == 0 && !peer->target.memory.flash.state().active,
          "No unknown consumed pad, successful target NVM transfer or completion is supplied");
  for (const unsigned index : {2U, 3U}) {
    const auto layout = kh::layouts.at(index);
    for (unsigned n = 0; n < layout.stored; ++n) {
      require(pair.dsp.bus().read({.space = index == 2 ? DspSpace::y : DspSpace::p,
                                   .address = layout.address + n}) ==
                  sender.blocks.at(index).expected.at(n),
              "Complete original downloaded sender and source remain unchanged");
    }
  }
  detail.flush();
  require(hcs08::rejected([&] {
            pair.reject_board_qualification();
          }).contains("Board qualification failed"),
          "Logical transport cannot qualify its unresolved board interval");
  trace << "RETAINED nvm=1 helper=1 source=1 unknown=0 transfers=0 board-rejected=1 omitted-dsp="
        << filter.counts().dsp << " omitted-mcu=" << filter.counts().mcu
        << " omitted-known-pads=" << filter.counts().pads << '\n';
}
}
