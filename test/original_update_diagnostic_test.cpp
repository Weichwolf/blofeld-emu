// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/lcd_image.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/private_file.hpp"
#include "dsp_norm_fixture.hpp"
#include "lcd_fixture.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "original_update_display_fixture.hpp"
#include "original_update_forward_fixture.hpp"

#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test::update_forward;
using namespace blofeld::test::update_display;
constexpr unsigned continuation_limit = 200000;
constexpr unsigned display_clear = 0x8d70;
constexpr unsigned clear_return = 0x1209;
constexpr unsigned text_entry = 0xa6ee;
constexpr unsigned word_argument = 2;
constexpr unsigned y_argument = 4;
constexpr unsigned x_argument = 5;
constexpr unsigned font_argument = 6;
constexpr unsigned missing_source = source + prefetched_words;
constexpr unsigned prepared_flag = 0x40;

void storage_guards(Forward &run, const McuFlashImage &nvm) {
  const auto after = run.pair.mcu.bus().power_off_flash_image();
  blofeld::test::require(after.bytes == nvm.bytes && after.known == nvm.known &&
                             after.fresh == nvm.fresh,
                         "Error drawing preserves every MCU nonvolatile byte and history");
  const auto &bytes = run.firmware.update_blocks.at(payload_block);
  const auto offset = (source - payload_base) * word_bytes;
  for (unsigned n = 0; n < guard_bytes; ++n) {
    const auto expected = n < expected_bytes ? bytes[offset + size_bytes + n] : ram_canary;
    blofeld::test::require(run.pair.mcu.bus().read(static_cast<std::uint16_t>(target + n)) ==
                               expected,
                           "Executed original helper and its RAM tail remain exact");
  }
  const auto &helper = run.firmware.update_blocks.at(helper_block);
  for (unsigned n = 0; n < helper_words; ++n) {
    blofeld::test::require(
        run.pair.dsp.bus().read({.space = DspSpace::p, .address = update_entry + n}) ==
            word(helper, n),
        "Original DSP helper remains unchanged after the fault");
  }
  for (unsigned n = 0; n < bytes.size() / word_bytes; ++n) {
    const auto address = payload_base + n;
    blofeld::test::require(run.pair.dsp.bus().read({.space = DspSpace::y, .address = address}) ==
                               (address == marker ? 0 : word(bytes, n)),
                           "Every prepared source word remains unchanged");
  }
  for (unsigned n = 0; n < run.own_caller.size(); ++n) {
    blofeld::test::require(run.pair.mcu.bus().read(static_cast<std::uint16_t>(caller + n)) ==
                               run.own_caller[n],
                           "Own diagnostic caller remains unchanged");
  }
  bool unknown = false;
  try {
    static_cast<void>(run.pair.dsp.bus().read({.space = DspSpace::y, .address = missing_source}));
  } catch (const Fault &error) {
    unknown = std::string_view(error.what()).contains("Unknown DSP memory read");
  }
  blofeld::test::require(unknown, "The missing word remains unknown after the native failure");
}

void quarantine(Forward &run) {
  const auto mcu = registers(run.pair.mcu.cpu().state());
  const auto dsp = run.pair.dsp.cpu().debug_state();
  bool mcu_rejected = false;
  bool dsp_rejected = false;
  try {
    static_cast<void>(run.pair.mcu.cpu().step());
  } catch (const Fault &) { mcu_rejected = true; }
  try {
    run.pair.dsp.cpu().step();
  } catch (const Fault &) { dsp_rejected = true; }
  blofeld::test::require(mcu_rejected && dsp_rejected &&
                             registers(run.pair.mcu.cpu().state()) == mcu &&
                             run.pair.dsp.cpu().debug_state() == dsp,
                         "Both failed clocked owners reject all further native progress");
  run.pair.trace << "QUARANTINE mcu=1 dsp=1 unchanged=1\n";
}

void diagnostic(Forward &run, const McuFlashImage &nvm, const std::string &path) {
  unsigned texts = 0;
  bool clearing = false;
  for (unsigned steps = 0; steps < continuation_limit; ++steps) {
    const auto cpu = run.pair.mcu.cpu().state();
    if (cpu.pc == display_clear) { clearing = true; }
    if (clearing && cpu.pc == clear_return) {
      check_clear(run);
      run.pair.trace << "CLEAR writes=" << run.display->counts().writes
                     << " reads=" << run.display->counts().reads << '\n';
      clearing = false;
    }
    if (cpu.pc == text_entry) {
      const auto returned = run.pair.mcu.bus().read_memory({.address = cpu.sp, .size = 2});
      if (returned >= target && returned < target + expected_bytes) {
        const auto pointer = run.pair.mcu.bus().read_memory(
            {.address = static_cast<std::uint16_t>(cpu.sp + word_argument), .size = 2});
        blofeld::test::require(pointer >= target && pointer < target + expected_bytes,
                               "Original diagnostic text stays inside the loaded helper");
        const auto y = run.pair.mcu.bus().read(static_cast<std::uint16_t>(cpu.sp + y_argument));
        const auto x = run.pair.mcu.bus().read(static_cast<std::uint16_t>(cpu.sp + x_argument));
        const auto font =
            run.pair.mcu.bus().read(static_cast<std::uint16_t>(cpu.sp + font_argument));
        run.pair.trace << "TEXT index=" << texts << " pointer=" << pointer
                       << " y=" << static_cast<unsigned>(y) << " x=" << static_cast<unsigned>(x)
                       << " font=" << static_cast<unsigned>(font) << " mode=" << (cpu.d & mask)
                       << " value=" << read_text(run.pair.mcu.bus(), static_cast<unsigned>(pointer))
                       << '\n';
        ++texts;
      }
    }
    try {
      run.native_step();
    } catch (const Fault &error) {
      const std::string_view message(error.what());
      blofeld::test::require(
          message.contains("DSP reference step pc=" + hex(stream_read, 6)) &&
              message.contains("Unknown DSP memory read at Y:" + hex(missing_source, 6)) &&
              run.pair.mcu.timing_faulted() && run.pair.dsp.timing_faulted() &&
              run.pair.schedule->faulted() && texts == 2,
          "Only the actual post-object DSP source read may end this diagnostic");
      run.pair.trace << "DIAGNOSTIC_FAULT " << error.what() << '\n';
      blofeld::test::require(
          run.sent == stream_samples && run.received == expected_samples &&
              run.loaded == prefetched_words &&
              run.pair.dsp.cpu().state().r[1] == missing_source + 1 &&
              run.pair.dsp.cpu().state().pc == stream_read + 1 &&
              (run.pair.mcu.bus().read(bus_flags) & prepared_flag) == 0 &&
              run.pair.link.unresolved_queries == 0 && run.unknown_irq_queries == 0,
          "Real checksum chunks and rejected source prefetch retain their state");
      quarantine(run);
      storage_guards(run, nvm);
      const auto image =
          capture_lcd(run.display->controller(), {.width = 128, .reverse_commons = true});
      write_private_file(path + ".pbm", encode_lcd_pbm(image));
      write_private_file(path + ".png", encode_lcd_png(image));
      run.pair.trace << "DIAG_RESULT texts=" << texts << " writes=" << run.display->counts().writes
                     << " reads=" << run.display->counts().reads << " chunks=" << run.sent
                     << " samples=" << run.received << " source-loads=" << run.loaded
                     << " queries=" << run.pair.link.pad_queries
                     << " irq-queries=" << run.irq_queries
                     << " unknown-queries=" << run.pair.link.unresolved_queries
                     << " irq-unknown=" << run.unknown_irq_queries
                     << " unresolved-events=" << run.pair.link.unresolved_events
                     << " missing=" << missing_source
                     << " cursor=" << run.pair.dsp.cpu().state().r[1]
                     << " prepared=0 nvm-unchanged=1 program-unchanged=1 payload-unchanged=1"
                        " caller-unchanged=1 helper-unchanged=1 ram-tail-unchanged=1\n";
      return;
    }
  }
  throw Fault("Original diagnostic continuation budget");
}
}

int main(const int argc, char **argv) {
  try {
    blofeld::test::require(argc == 3,
                           "Original update diagnostic needs firmware and private trace");
    const auto firmware = blofeld::Firmware::load(argv[1]);
    std::ofstream trace(argv[2]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    for (const unsigned order : {0U, 1U}) {
      for (const unsigned phase : {0U, phase_offset}) {
        trace << "CASE order=" << order << " phase=" << phase << '\n';
        blofeld::test::update_forward::Forward run(firmware, trace, true);
        const auto nvm = run.pair.mcu.bus().power_off_flash_image();
        run.execute(order, phase);
        run.verify(nvm);
        diagnostic(run,
                   nvm,
                   std::string(argv[2]) + "." + std::to_string(order) + "." +
                       std::to_string(phase));
      }
    }
    std::cout << "Four original MCU error-helper frames and post-object DSP faults verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
