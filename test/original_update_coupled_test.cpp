// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/lcd_image.hpp"
#include "blofeld/private_file.hpp"
#include "dsp_norm_fixture.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "original_update_display_fixture.hpp"
#include "original_update_forward_fixture.hpp"
#include "original_update_transport_fixture.hpp"

#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {
namespace forward = blofeld::test::update_forward;
namespace transport = blofeld::test::update_transport;
namespace display = blofeld::test::update_display;
using blofeld::test::require;
constexpr unsigned main_payload = 4, main_program = 5;
constexpr unsigned continuation_limit = 240000, alignment_limit = 1000000;
constexpr unsigned clear_entry = 0x8d70, clear_return = 0x1209, text_entry = 0xa6ee;
constexpr unsigned rectangle_entry = 0x8bfd, rectangle_return = 0x124e, prepared_entry = 0x1256;
constexpr unsigned delay_entry = 0x128b, wait_entry = 0x1291, cop_entry = 0x1298;
constexpr unsigned delay_iterations = 30000, prepared_flags = 0xd1;
constexpr unsigned rectangle_y = 23, rectangle_height = 18;
constexpr unsigned lcd_writes = 1768, lcd_reads = 1488;
constexpr unsigned complete_source = 0x144bb, waiting_chunk = 0x2e6b;

void preload(blofeld::test::gpio_pair::OriginalGpioPair &pair,
             const blofeld::Firmware &firmware,
             const std::vector<transport::Block> &blocks,
             const transport::Templates &templates,
             std::vector<unsigned> &caller,
             std::ostream &trace) {
  const auto epoch = pair.dsp.run_clock().native_epoch();
  const auto before = pair.dsp.cpu().debug_state().native_instructions;
  trace << "PRELOAD block=4\n";
  transport::Transfer data(firmware,
                           blocks.at(main_payload),
                           transport::layouts.at(main_payload),
                           templates,
                           trace,
                           {.owner = &pair.dsp, .from = pair.foreground});
  data.run();
  trace << "PRELOAD block=5\n";
  transport::Transfer program(firmware,
                              blocks.at(main_program),
                              transport::layouts.at(main_program),
                              templates,
                              trace,
                              {.owner = &pair.dsp, .reuse = true});
  program.run();
  const auto delta = pair.dsp.cpu().debug_state().native_instructions - before;
  require(
      pair.dsp.run_clock().native_epoch() == epoch &&
          pair.dsp.cpu().state().pc == forward::update_entry && pair.dsp.cpu().state().sp == 1 &&
          pair.dsp.cpu().state().r[0] == transport::context && delta == data.steps + program.steps,
      "Both native downloads retain one DSP epoch, every instruction and the actual launch frame");
  caller = program.caller_words;
  pair.input.value = transport::completed_status; // Oracle for the real native X:076c status write.
  pair.align_prefix(alignment_limit);
  trace << "PRELOAD_END pc=" << pair.dsp.cpu().state().pc << " epoch=" << epoch
        << " steps=" << delta << " status="
        << pair.dsp.bus().read({.space = blofeld::DspSpace::x, .address = transport::status_slot})
        << " r0=" << pair.dsp.cpu().state().r[0] << " sp=" << pair.dsp.cpu().state().sp << '\n';
}

struct Drawing {
  unsigned texts = 0, rectangles = 0, delay = 0, waits = 0;
  bool clearing = false, rectangle_done = false, prepared = false;

  static bool root(forward::Forward &run) {
    const auto returned =
        run.pair.mcu.bus().read_memory({.address = run.pair.mcu.cpu().state().sp, .size = 2});
    return returned >= forward::target && returned < forward::target + forward::expected_bytes;
  }

  static unsigned argument(forward::Forward &run, const unsigned offset) {
    return run.pair.mcu.bus().read(
        static_cast<std::uint16_t>(run.pair.mcu.cpu().state().sp + offset));
  }

  void text(forward::Forward &run) {
    const auto cpu = run.pair.mcu.cpu().state();
    const auto pointer = run.pair.mcu.bus().read_memory(
        {.address = static_cast<std::uint16_t>(cpu.sp + 2), .size = 2});
    require(pointer >= forward::target && pointer < forward::target + forward::expected_bytes,
            "Original helper text remains inside its transferred RAM object");
    run.pair.trace << "TEXT index=" << texts << " pointer=" << pointer << " y=" << argument(run, 4)
                   << " x=" << argument(run, 5) << " font=" << argument(run, 6)
                   << " mode=" << (cpu.d & forward::mask) << " value="
                   << display::read_text(run.pair.mcu.bus(), static_cast<unsigned>(pointer))
                   << '\n';
    ++texts;
  }

  void observe(forward::Forward &run) {
    const auto cpu = run.pair.mcu.cpu().state();
    if (cpu.pc == clear_entry) { clearing = true; }
    if (clearing && cpu.pc == clear_return) {
      display::check_clear(run);
      run.pair.trace << "CLEAR writes=" << run.display->counts().writes
                     << " reads=" << run.display->counts().reads << '\n';
      clearing = false;
    }
    if (cpu.pc == text_entry && root(run)) { text(run); }
    if (cpu.pc == rectangle_entry && root(run)) {
      require(argument(run, 5) == 0 && argument(run, 4) == rectangle_y && argument(run, 3) == 128 &&
                  argument(run, 2) == rectangle_height && (cpu.d & forward::mask) == 2,
              "Original rectangle arguments XOR the full128 by18 band at y23");
      run.pair.trace << "RECTANGLE x=0 y=23 width=128 height=18 mode=2\n";
      ++rectangles;
    }
    if (cpu.pc == rectangle_return) {
      require(texts == 2 && rectangles == 1 && run.display->counts().writes == lcd_writes &&
                  run.display->counts().reads == lcd_reads,
              "Original rectangle has exact native read/modify/write counts");
      rectangle_done = true;
    }
    if (cpu.pc == prepared_entry) {
      const auto flags = run.pair.mcu.bus().read(forward::bus_flags);
      require(rectangle_done && flags == prepared_flags,
              "Original helper prepared/activity flags after drawing: " + blofeld::hex(flags, 2));
      prepared = true;
      run.pair.trace << "PREPARED flags=" << prepared_flags << '\n';
    }
    delay += static_cast<unsigned>(cpu.pc == delay_entry);
    waits += static_cast<unsigned>(cpu.pc == wait_entry);
  }
};

void unknown(blofeld::DspBus &bus, const blofeld::DspAddress address) {
  bool rejected = false;
  try {
    static_cast<void>(bus.read(address));
  } catch (const blofeld::Fault &error) {
    rejected = std::string_view(error.what()).contains("Unknown DSP memory read");
  }
  require(rejected, "RAM outside the native download remains unprovided");
}

void storage(forward::Forward &run,
             const blofeld::McuFlashImage &nvm,
             const std::vector<transport::Block> &blocks,
             const std::vector<unsigned> &caller) {
  const auto after = run.pair.mcu.bus().power_off_flash_image();
  require(after.bytes == nvm.bytes && after.known == nvm.known && after.fresh == nvm.fresh,
          "Original error drawing and attempted restart retain all MCU NVM and history");
  for (const unsigned block : {main_payload, main_program}) {
    const auto layout = transport::layouts.at(block);
    const auto space = block == main_payload ? blofeld::DspSpace::y : blofeld::DspSpace::p;
    for (unsigned n = 0; n < blocks.at(block).expected.size(); ++n) {
      require(run.pair.dsp.bus().read({.space = space, .address = layout.address + n}) ==
                  blocks.at(block).expected[n],
              "Every natively downloaded program/payload/padding word remains exact");
    }
    unknown(run.pair.dsp.bus(), {.space = space, .address = layout.address + layout.stored});
  }
  for (unsigned n = 0; n < caller.size(); ++n) {
    require(run.pair.dsp.bus().inspect_program(transport::staging_rom + n) == caller[n],
            "Own native download caller remains exact");
  }
  for (unsigned n = 0; n < run.own_caller.size(); ++n) {
    require(run.pair.mcu.bus().read(static_cast<std::uint16_t>(forward::caller + n)) ==
                run.own_caller[n],
            "Own MCU caller remains exact");
  }
  const auto &bytes = run.firmware.update_blocks.at(main_payload);
  const auto offset = (forward::source - forward::payload_base) * forward::word_bytes;
  for (unsigned n = 0; n < forward::guard_bytes; ++n) {
    const auto value =
        n < forward::expected_bytes ? bytes[offset + forward::size_bytes + n] : forward::ram_canary;
    require(run.pair.mcu.bus().read(static_cast<std::uint16_t>(forward::target + n)) == value,
            "Received MCU helper and untouched RAM tail remain exact");
  }
  for (const unsigned index : {transport::context_record, transport::command_record}) {
    const auto &record = run.firmware.dsp.at(index);
    for (unsigned n = 0; n < record.words.size(); ++n) {
      const auto address = record.address + n;
      if (index == transport::command_record &&
          (address == transport::store_site || address == transport::return_site)) {
        continue;
      }
      require(run.pair.dsp.bus().read({.space = static_cast<blofeld::DspSpace>(record.space),
                                       .address = address}) == record.words[n],
              "Original decoder/context outside live patches remain unchanged");
    }
  }
}

void quarantine(forward::Forward &run) {
  const auto mcu = forward::registers(run.pair.mcu.cpu().state());
  const auto dsp = run.pair.dsp.cpu().debug_state();
  const auto ticks = run.pair.dsp.run_clock().elapsed_ticks();
  const auto mcu_ticks = run.pair.mcu.run_clock().elapsed_ticks();
  const auto mcu_cycles = run.pair.mcu.run_clock().bus_cycles();
  bool mcu_rejected = false;
  bool joint_rejected = false;
  try {
    static_cast<void>(run.pair.mcu.cpu().step());
  } catch (const blofeld::Fault &) { mcu_rejected = true; }
  try {
    run.pair.schedule->end_instruction();
  } catch (const blofeld::Fault &) { joint_rejected = true; }
  require(mcu_rejected && joint_rejected && run.pair.mcu.timing_faulted() &&
              run.pair.schedule->faulted() && !run.pair.dsp.timing_faulted() &&
              forward::registers(run.pair.mcu.cpu().state()) == mcu &&
              run.pair.dsp.cpu().debug_state() == dsp &&
              run.pair.dsp.run_clock().elapsed_ticks() == ticks &&
              run.pair.mcu.run_clock().elapsed_ticks() == mcu_ticks &&
              run.pair.mcu.run_clock().bus_cycles() == mcu_cycles,
          "MCU and joint scheduling reject further execution while the independent DSP state stays "
          "held");
  run.pair.trace << "QUARANTINE mcu=1 joint=1 dsp-held=1 dsp-faulted=0\n";
}

void continuation(forward::Forward &run,
                  const blofeld::McuFlashImage &nvm,
                  const std::vector<transport::Block> &blocks,
                  const std::vector<unsigned> &caller,
                  const std::string &path) {
  Drawing drawing;
  constexpr unsigned entry_flags = 0x91;
  const auto initial_flags = run.pair.mcu.bus().read(forward::bus_flags);
  require(initial_flags == entry_flags,
          "Original receiver active/display flags: " + blofeld::hex(initial_flags, 2));
  run.pair.trace << "ENTRY_FLAGS flags=" << static_cast<unsigned>(initial_flags) << '\n';
  for (unsigned n = 0; n < continuation_limit; ++n) {
    drawing.observe(run);
    try {
      run.native_step();
    } catch (const blofeld::Fault &error) {
      const std::string_view message(error.what());
      const auto mcu = run.pair.mcu.cpu().state();
      const auto dsp = run.pair.dsp.cpu().state();
      run.pair.trace << "FAULT_STATE mcu-pc=" << mcu.pc << " mcu-d=" << mcu.d
                     << " mcu-sp=" << mcu.sp << " mcu-x=" << mcu.x << " dsp-pc=" << dsp.pc
                     << " cursor=" << dsp.r[1] << " delay=" << drawing.delay
                     << " waits=" << drawing.waits << '\n';
      require(
          message.contains("MCU write at 0x003c") && message.contains("pc=0x1298") &&
              message.contains("Unimplemented MCU peripheral write at 0x003c") &&
              drawing.prepared && drawing.delay == delay_iterations && drawing.waits == 1 &&
              mcu.pc == cop_entry && mcu.x == delay_iterations && mcu.sp == forward::stack - 14 &&
              (mcu.d & forward::mask) == 1 && dsp.pc == waiting_chunk &&
              dsp.r[1] == complete_source - 1 && run.loaded == forward::prefetched_words + 1 &&
              run.sent == forward::stream_samples + 2 && run.received == forward::expected_samples,
          "Only the real COPCTL attempt after all30000 delay iterations may end this path: " +
              std::string(error.what()));
      run.pair.trace << "COUPLED_FAULT " << error.what() << '\n';
      quarantine(run);
      storage(run, nvm, blocks, caller);
      const auto image =
          blofeld::capture_lcd(run.display->controller(), {.width = 128, .reverse_commons = true});
      blofeld::write_private_file(path + ".pbm", blofeld::encode_lcd_pbm(image));
      blofeld::write_private_file(path + ".png", blofeld::encode_lcd_png(image));
      require(run.display->counts().writes == lcd_writes &&
                  run.display->counts().reads == lcd_reads &&
                  run.pair.mcu.bus().read(forward::bus_flags) == prepared_flags &&
                  run.pair.link.unresolved_queries == 0 && run.unknown_irq_queries == 0,
              "Final raster, prepared state and every actual consumed pad remain exact");
      run.pair.trace << "COUPLED_RESULT texts=" << drawing.texts
                     << " rectangles=" << drawing.rectangles << " writes=" << lcd_writes
                     << " reads=" << lcd_reads << " delay=" << drawing.delay
                     << " waits=" << drawing.waits << " prepared=" << prepared_flags
                     << " source-loads=" << run.loaded << " chunks=" << run.sent
                     << " samples=" << run.received << " dsp-pc=" << dsp.pc
                     << " cursor=" << dsp.r[1] << " queries=" << run.pair.link.pad_queries
                     << " irq-queries=" << run.irq_queries
                     << " unknown-queries=" << run.pair.link.unresolved_queries
                     << " irq-unknown=" << run.unknown_irq_queries
                     << " unresolved-events=" << run.pair.link.unresolved_events
                     << " storage=1 unknown-tail=" << complete_source << " helper-returned=0\n";
      return;
    }
  }
  throw blofeld::Fault("Coupled continuation instruction limit");
}
}

int main(const int argc, char **argv) {
  try {
    require(argc == 5, "Expected original firmware, private packets, own templates and trace");
    const auto firmware = blofeld::Firmware::load(argv[1]);
    const auto blocks = transport::read_blocks(argv[2]);
    const auto templates = transport::read_templates(argv[3]);
    const auto &payload = blocks.at(main_payload);
    const std::vector<unsigned> tail(payload.expected.begin() +
                                         transport::layouts.at(main_payload).declared,
                                     payload.expected.end());
    std::ofstream trace(argv[4]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    for (const unsigned order : {0U, 1U}) {
      for (const unsigned phase : {0U, forward::phase_offset}) {
        trace << "CASE order=" << order << " phase=" << phase << '\n';
        std::vector<unsigned> caller;
        const forward::PayloadPreparation prepare = [&](auto &pair) {
          preload(pair, firmware, blocks, templates, caller, trace);
        };
        forward::Forward run(firmware, trace, true, prepare, tail);
        const auto nvm = run.pair.mcu.bus().power_off_flash_image();
        run.execute(order, phase);
        run.verify(nvm);
        continuation(run,
                     nvm,
                     blocks,
                     caller,
                     std::string(argv[4]) + "." + std::to_string(order) + "." +
                         std::to_string(phase));
      }
    }
    std::cout << "Four native downloads and coupled error-helper COPCTL frontiers verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
