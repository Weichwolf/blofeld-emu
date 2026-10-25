// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard_initial_fixture.hpp"

#include "board/gpio_pads.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "original_update_forward_fixture.hpp"
#include "original_update_transport_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace blofeld::test::keyboard_initial {
namespace fw = update_forward;

Fixture::Fixture(const Firmware &image,
                 const std::vector<transport::Block> &input,
                 const transport::Templates &templates,
                 const unsigned expected_branch,
                 std::ostream &trace,
                 const unsigned phase)
    : pair(image, gp::cases.back(), trace),
      firmware(image),
      blocks(input),
      branch(expected_branch) {
  const auto &main = image.dsp.at(main_record);
  require(main.space == 0 && main.address <= main_patch &&
              main_patch - main.address < main.words.size(),
          "The complete pinned original main-loop record contains the patch target");
  for (unsigned n = 0; n < main.words.size(); ++n) {
    pair.dsp.bus().write(
        {.space = DspSpace::p, .address = main.address + n, .value = main.words[n]});
  }
  for (unsigned n = 0; n < 64; ++n) {
    pair.dsp.bus().write({.space = DspSpace::x, .address = scratch + n, .value = gp::data_canary});
  }
  const auto epoch = pair.dsp.run_clock().native_epoch();
  const auto before = pair.dsp.cpu().debug_state().native_instructions;
  trace << "PRELOAD block=0\n";
  transport::Transfer data(image,
                           blocks.at(0),
                           layouts.at(0),
                           templates,
                           trace,
                           {.owner = &pair.dsp, .from = pair.foreground});
  data.run();
  trace << "PRELOAD block=1\n";
  transport::Transfer program(
      image, blocks.at(1), layouts.at(1), templates, trace, {.owner = &pair.dsp, .reuse = true});
  program.run();
  require(pair.dsp.run_clock().native_epoch() == epoch &&
              pair.dsp.cpu().state().pc == fw::update_entry && pair.dsp.cpu().state().sp == 1 &&
              pair.dsp.cpu().state().r[0] == transport::context &&
              pair.dsp.cpu().debug_state().native_instructions - before ==
                  data.steps + program.steps,
          "Both native downloads preserve the original launch/context and one DSP epoch");
  returned = program.returned;
  own_dsp = program.caller_words;
  own_dsp.back() =
      returned; // Own caller waits after the actual sender RTS, without another packet.
  const std::array idle{transport::absolute_jump, returned};
  pair.dsp.bus().install_rom({.start = returned, .words = idle});
  trace << "PRELOAD_END pc=" << pair.dsp.cpu().state().pc << " epoch=" << epoch
        << " steps=" << data.steps + program.steps << '\n';
  align(phase);
  connect();
  pair.additional_completion.observe = [this](const auto &step) { observe(step); };
}

void Fixture::align(const unsigned phase) {
  const auto original = pair.mcu.cpu().state();
  pair.mcu.cpu().set_state({.pc = gp::alignment_caller, .sp = gp::stack, .ccr = gp::alignment_ccr});
  require(gp::common_hz % pair.mcu.run_clock().timebase_hz() == 0, "Integral explicit clock ratio");
  const auto ratio = gp::common_hz / pair.mcu.run_clock().timebase_hz();
  unsigned steps = 0;
  constexpr unsigned limit = 1000000;
  while (pair.mcu.run_clock().elapsed_ticks() * ratio <
         pair.dsp.run_clock().elapsed_ticks() + phase) {
    require(++steps < limit, "Own busy loop reaches the declared common-time horizon");
    pair.mcu.cpu().step();
  }
  pair.mcu.cpu().set_state(original);
  pair.sync();
}

void Fixture::connect() {
  auto &remote = pair.dsp.bus().gpio(DspPort::e);
  remote.write({.reg = DspGpioRegister::direction, .value = irq_strobe, .mask = irq_strobe});
  remote.write({.reg = DspGpioRegister::data, .value = irq_strobe, .mask = irq_strobe});
  pair.mcu.bus().set_pin_drive(McuPort::b, {.mask = 8});
  display = std::make_unique<LcdFixture>(pair.mcu.bus());
  pair.additional_bus_write = [this](const BusEvent &event) { display->observe(event); };
  pair.mcu.bus().write(gp::bus_flags, 1);
  own_mcu = fw::caller_program(true);
  for (unsigned n = 0; n < own_mcu.size(); ++n) {
    pair.mcu.bus().write(static_cast<std::uint16_t>(caller + n), own_mcu[n]);
  }
  pair.mcu.bus().write(hc12::ddrq, fw::handshake_strobe);
  pair.mcu.bus().write(hc12::ptq, fw::handshake_strobe);
  auto state = pair.mcu.cpu().state();
  state.pc = caller;
  state.ccr = fw::foreground_ccr;
  pair.mcu.cpu().set_state(state);
  pair.mcu.bus().set_pin_drive(McuPort::e, {});
  attach_irq();
  pair.sync();
}

void Fixture::attach_irq() {
  pair.mcu.bus().attach_gpio_pad_source(McuPort::e, [this](const std::uint32_t bits) {
    const auto output = pair.dsp.bus().gpio(DspPort::e).outputs();
    const auto local = pair.mcu.bus().gpio_outputs(McuPort::e);
    require((local.mask & bits & 3U) == 0, "Declared IRQ inputs cannot have local strong outputs");
    const auto irq_known = 2U | ((output.mask & output.known & irq_strobe) != 0 ? 1U : 0U);
    const auto irq_levels = 2U | ((output.levels & irq_strobe) != 0 ? 1U : 0U);
    const auto known = (irq_known | (local.known & ~3U)) & bits;
    if ((bits & 3U) != 0) {
      ++irq_queries;
      irq_unknown += static_cast<unsigned>((irq_known & bits) != (bits & 3U));
    }
    return GpioPadSample{.known = known, .levels = (irq_levels | (local.levels & ~3U)) & known};
  });
}

unsigned Fixture::chunk(const unsigned index) const {
  return (blocks.at(0).expected.at(index / 4) >> (high_group - 6U * (index % 4))) & data_mask;
}

void Fixture::observe(const DspScheduledStep &step) {
  pair.mcu.bus().observe_interrupt_pads();
  const auto before = step.plan.before.cpu;
  const auto after = pair.dsp.cpu().state();
  if (before.pc == source_read) {
    require(loaded < source_words && before.r[0] == source + loaded &&
                after.r[0] == source + loaded + 1 &&
                ((after.a >> word_bits) & gp::word_mask) == blocks[0].expected.at(loaded),
            "Original initial sender reads only its41 declared source words");
    pair.trace << "SOURCE index=" << loaded << " address=" << source + loaded
               << " value=" << blocks[0].expected.at(loaded) << '\n';
    ++loaded;
  }
  if (std::ranges::find(chunk_pcs, before.pc) != chunk_pcs.end()) {
    const auto value = (before.a >> extension_shift) & byte_mask;
    const auto output = pair.dsp.bus().gpio(DspPort::c).outputs();
    require(sent < source_words * 4 && value == chunk(sent) &&
                (output.mask & output.known & fw::data_outputs) == fw::data_outputs &&
                (output.levels & fw::data_outputs) == value,
            "Native first-stage MOVEP pins match the independent six-bit stream");
    pair.trace << "CHUNK index=" << sent << " value=" << value << " pc=" << before.pc << '\n';
    ++sent;
  }
  if (before.pc == final_ack && after.pc != final_ack) {
    ++acknowledgements;
    pair.trace << "ACK pc=" << before.pc << " next=" << after.pc << '\n';
  }
  if (before.pc == sender_rts) {
    require(after.pc == returned && after.sp == 0,
            "Actual initial sender RTS returns to own idle caller");
    ++sender_returns;
    pair.trace << "SENDER_RETURN pc=" << after.pc << " sp=" << after.sp << '\n';
  }
}

void Fixture::step() {
  const auto before = pair.mcu.cpu().state();
  drawing();
  pair.trace << "M " << pair.mcu.run_clock().elapsed_ticks() << ' ' << hex(before.pc) << '\n';
  const auto result = pair.mcu.cpu().step();
  ++pair.link.msteps;
  pair.schedule->end_instruction();
  const auto after = pair.mcu.cpu().state();
  if (before.pc == indicator_entry) {
    pair.trace << "INDICATOR_READ index=" << indicators << " value=" << (after.d & byte_mask)
               << '\n';
  }
  if (result.interrupt_vector) {
    require(result.interrupt_vector == fw::xirq_vector && !result.instruction_executed &&
                after.pc == fw::request_irq && after.sp == before.sp - fw::interrupt_frame_bytes,
            "Original XIRQ entry supplies its genuine interrupt frame");
    interrupted = before;
    ++irq_entries;
  }
  if (before.pc == fw::request_rti) {
    if (!interrupted) { throw Fault("Original RTI has no captured interrupt entry"); }
    require(fw::registers(after) == fw::registers(interrupted.value()),
            "Original XIRQ RTI restores caller");
    ++irq_returns;
  }
  if (before.pc == fw::receiver_sample) {
    pair.trace << "CONSUMER index=" << received << " value=" << (after.d & data_mask) << '\n';
    if (received < source_words * 4) {
      require((after.d & data_mask) == chunk(received), "Native copy reads actual sender chunks");
    } else {
      const auto driver = pair.dsp.bus().gpio(DspPort::c).outputs();
      require((after.d & data_mask) == 0 && (driver.mask & fw::data_outputs) == 0 &&
                  (driver.peripheral & fw::data_outputs) == 0 &&
                  (pair.link.dsp_drive.pull_down & fw::data_outputs) == fw::data_outputs &&
                  (pair.link.mcu_drive.active & fw::data_outputs) == 0 &&
                  ((pair.link.mcu_drive.pull_up | pair.link.mcu_drive.pull_down) &
                   fw::data_outputs) == 0 &&
                  loaded == source_words && sent == source_words * 4 && sender_returns == 1,
              "Post-sender samples come only from the explicit passive Port-C bias");
      pair.trace << "PASSIVE_SAMPLE index=" << received - (source_words * 4) << " value=0\n";
    }
    ++received;
  }
  if (before.pc == stream_return && after.pc >= trailer_begin && after.pc < trailer_end) {
    require(helper_returns == 1 && trailer_bytes < 9 && (after.d & byte_mask) == 0,
            "The resident loader drains exactly its two cached and seven passive zero bytes");
    pair.trace << "TRAILER_BYTE index=" << trailer_bytes << " value=" << (after.d & byte_mask)
               << " return=" << after.pc << '\n';
    ++trailer_bytes;
  }
  if (before.pc == helper_rts) {
    require(after.pc == helper_return,
            "The original initial helper returns through the resident frame");
    ++helper_returns;
    pair.trace << "HELPER_RETURN pc=" << after.pc << " sp=" << after.sp
               << " flags=" << static_cast<unsigned>(pair.mcu.bus().read(gp::bus_flags)) << '\n';
  }
}

void Fixture::verify_copy() {
  for (unsigned n = 0; n < helper_bytes + 32; ++n) {
    const auto offset = n + 2;
    const auto expected =
        n < helper_bytes
            ? (blocks[0].expected.at(offset / 3) >> (16U - 8U * (offset % 3))) & byte_mask
            : gp::ram_canary;
    require(pair.mcu.bus().read(static_cast<std::uint16_t>(target + n)) == expected,
            "Resident copy preserves every original119 helper byte and untouched RAM tail");
  }
}

void Fixture::execute(const unsigned order, const unsigned phase) {
  const auto end = caller + static_cast<unsigned>(own_mcu.size()) - 2;
  pair.start(
      order == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp, phase, end);
  bool entered = false;
  while (pair.link.msteps < instruction_limit) {
    if (!entered && pair.mcu.cpu().state().pc == target) {
      const auto state = pair.mcu.cpu().state();
      verify_copy();
      require(state.d == service_table && state.sp == entry_sp && irq_entries == 1 &&
                  irq_returns == 1 && sent == source_words * 4 && received == sent &&
                  loaded == source_words,
              "Native initial sender/copy/JSR reaches the exact119-byte helper boundary");
      pair.trace << "ENTRY pc=" << state.pc << " d=" << state.d << " sp=" << state.sp << '\n';
      entered = true;
    }
    step();
    if (helper_returns == 1) {
      require(entered && pair.mcu.cpu().state().pc == helper_return && trailer_bytes == 0 &&
                  received == source_words * 4 && sender_returns == 1 && acknowledgements == 1 &&
                  indicators == 5 && texts == 2 && rectangles == 1 &&
                  display->counts().writes == lcd_writes && display->counts().reads == lcd_reads,
              "Original sender/helper returns precede the resident trailer and retain the drawing");
      pair.trace << "RETURN_BOUNDARY pc=" << pair.mcu.cpu().state().pc
                 << " dsp-pc=" << pair.dsp.cpu().state().pc << '\n';
      return;
    }
  }
  throw Fault("Original initial Keyboard sender/helper exceeded its bound");
}

void Fixture::finish() {
  const auto end = caller + static_cast<unsigned>(own_mcu.size()) - 2;
  for (unsigned n = 0; n < instruction_limit; ++n) {
    step();
    if (pair.mcu.cpu().state().pc != end) { continue; }
    const auto cpu = pair.mcu.cpu().state();
    const auto driver = pair.mcu.bus().gpio_outputs(McuPort::q);
    constexpr unsigned zero_flag = 4;
    constexpr unsigned interrupt_mask = 0x10;
    require(trailer_bytes == 9 && received == source_words * 4 + 12 && driver.mask == 0 &&
                pair.mcu.bus().read(gp::bus_flags) == final_flags && cpu.sp == gp::stack &&
                (cpu.ccr & (zero_flag | interrupt_mask)) == zero_flag &&
                pair.dsp.cpu().state().pc == returned && pair.dsp.cpu().state().sp == 0 &&
                pair.link.unresolved_queries == 0 && irq_unknown == 0,
            "Original trailer/release/CLI/RTS completes with the real passive zero bus");
    pair.trace << "INITIAL_END mcu-pc=" << end << " dsp-pc=" << pair.dsp.cpu().state().pc
               << " samples=" << received << " chunks=" << sent << " loaded=" << loaded
               << " helper-returned=" << helper_returns << " sender-returned=" << sender_returns
               << " ack=" << acknowledgements << " indicators=" << indicators << " texts=" << texts
               << " rectangles=" << rectangles << " trailer=" << trailer_bytes
               << " writes=" << display->counts().writes << " reads=" << display->counts().reads
               << " flags=" << static_cast<unsigned>(pair.mcu.bus().read(gp::bus_flags))
               << " queries=" << pair.link.pad_queries << " irq-queries=" << irq_queries
               << " unresolved-events=" << pair.link.unresolved_events
               << " unknown=" << pair.link.unresolved_queries << " irq-unknown=" << irq_unknown
               << '\n';
    return;
  }
  throw Fault("Original initial Keyboard resident trailer exceeded its bound");
}

void Fixture::retain() {
  verify_copy();
  const auto flash = pair.mcu.bus().power_off_flash_image();
  require(std::ranges::equal(flash.bytes, firmware.mcu) && flash.known.all() && flash.fresh.none(),
          "Initial display helper preserves all MCU NVM bytes, knowledge and erase history");
  const auto &main = firmware.dsp.at(main_record);
  for (unsigned n = 0; n < main.words.size(); ++n) {
    const auto address = main.address + n;
    require(pair.dsp.bus().inspect_program(address) ==
                (address == main_patch ? branch : main.words[n]),
            "The original main loop has exactly its independently assembled branch patch");
  }
  for (unsigned n = 0; n < 64; ++n) {
    require(pair.dsp.bus().read({.space = DspSpace::x, .address = scratch + n}) == 0,
            "Original REP clears each of the64 declared scratch words");
  }
  for (const unsigned index : {0U, 1U}) {
    const auto layout = layouts.at(index);
    for (unsigned n = 0; n < layout.stored; ++n) {
      require(pair.dsp.bus().read({.space = index == 0 ? DspSpace::y : DspSpace::p,
                                   .address = layout.address + n}) ==
                  blocks.at(index).expected.at(n),
              "Every original downloaded source/program word remains unchanged");
    }
  }
  for (unsigned n = 0; n < own_dsp.size(); ++n) {
    require(pair.dsp.bus().inspect_program(transport::staging_rom + n) == own_dsp[n],
            "Own native packet/idle caller remains exact");
  }
  for (unsigned n = 0; n < own_mcu.size(); ++n) {
    require(pair.mcu.bus().read(static_cast<std::uint16_t>(caller + n)) == own_mcu[n],
            "Own request/display caller remains outside the transferred helper");
  }
  for (const unsigned index : {transport::context_record, transport::command_record}) {
    const auto &record = firmware.dsp.at(index);
    for (unsigned n = 0; n < record.words.size(); ++n) {
      const auto address = record.address + n;
      if (index == transport::command_record &&
          (address == transport::store_site || address == transport::return_site)) {
        continue;
      }
      require(pair.dsp.bus().read({.space = static_cast<DspSpace>(record.space),
                                   .address = address}) == record.words[n],
              "Original download decoder/context stays exact outside its two native live patches");
    }
  }
  bool rejected = false;
  try {
    pair.reject_board_qualification();
  } catch (const Fault &error) {
    rejected = std::string(error.what()).contains("Board qualification failed");
  }
  require(rejected, "Known consumed samples do not qualify unresolved board voltage intervals");
  pair.trace << "RETAINED helper=1 source=1 program=1 main-loop=1 nvm=1 scratch=1 callers=1"
                " board-rejected=1\n";
}
} // namespace blofeld::test::keyboard_initial
