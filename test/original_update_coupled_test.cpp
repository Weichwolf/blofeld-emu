// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/lcd_image.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_cop.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_reset.hpp"
#include "blofeld/mcu_run_clock.hpp"
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
#include <optional>
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
constexpr unsigned delay_entry = 0x128b, wait_entry = 0x1291, cop_entry = 0x129c;
constexpr unsigned delay_iterations = 30000, prepared_flags = 0xd1, restart_accumulators = 0xf001;
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

struct CopObservation {
  std::optional<std::uint64_t> enabled;

  void bind(forward::Forward &run) {
    const auto previous = run.pair.additional_bus_write;
    run.pair.additional_bus_write = [this, &run, previous](const blofeld::BusEvent &event) {
      if (previous) { previous(event); }
      if (event.address != blofeld::hc12::cop_control) { return; }
      const auto state = run.pair.mcu.bus().cop_state();
      require(!enabled && event.value == 1 && state.control == 1 && state.phase == 0,
              "Original COP enable transaction starts the normal counter once");
      enabled = run.pair.mcu.run_clock().oscillator_cycles();
      run.pair.trace << "COP_ENABLED control=1 phase=0 oscillator=" << *enabled << '\n';
    };
  }

  void verify(forward::Forward &run) const {
    const auto state = run.pair.mcu.bus().cop_state();
    const auto oscillator = run.pair.mcu.run_clock().oscillator_cycles();
    require(enabled && state.control == 1 && state.phase == oscillator - *enabled &&
                state.phase < 16384 && state.reset == blofeld::McuCopReset::invalid_service,
            "Original invalid service requests reset after exactly the observed OSC edges");
    run.pair.trace << "COP_REQUEST control=1 phase=" << state.phase << " oscillator=" << oscillator
                   << " invalid-service=1 service=240\n";
  }
};

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

void reset_continuation(forward::Forward &run, const blofeld::McuFlashImage &nvm) {
  constexpr unsigned reset_edges = 195;
  constexpr unsigned oscillator_delta = 205;
  constexpr unsigned reset_ticks = 1024;
  constexpr unsigned vector_cycles = 5;
  constexpr unsigned resident_entry = 0x8000;
  constexpr unsigned first_write = 0x8002;

  const auto dsp = run.pair.dsp.cpu().debug_state();
  const auto dsp_ticks = run.pair.dsp.run_clock().elapsed_ticks();
  auto &mcu = run.pair.mcu;
  const auto ticks = mcu.run_clock().elapsed_ticks();
  const auto osc = mcu.run_clock().oscillator_cycles();
  const auto bus = mcu.run_clock().bus_cycles();
  run.display.reset(); // Finished screenshot fixture; it must not retain a replaced bus reference.
  mcu.begin_system_reset(blofeld::McuResetSource::cop, 3);
  unsigned vector = 0;
  std::string slots;
  mcu.set_cycle_observer([&](const blofeld::Hc12BusCycle cycle) {
    slots += cycle.kind;
    if (cycle.kind == 'V' && cycle.address) { vector = *cycle.address; }
  });
  require(
      mcu.advance_system_reset(reset_edges, blofeld::McuResetPad::high) == reset_edges &&
          !mcu.system_reset_active() && !mcu.timing_faulted() && slots == "VfPPP" &&
          vector == blofeld::hc12::cop_reset_vector && mcu.cpu().state().pc == resident_entry &&
          mcu.cpu().state().sp == blofeld::hc12::reset_sp &&
          mcu.cpu().state().ccr == blofeld::hc12::reset_ccr &&
          mcu.run_clock().elapsed_ticks() - ticks == reset_ticks &&
          mcu.run_clock().oscillator_cycles() - osc == oscillator_delta &&
          mcu.run_clock().bus_cycles() - bus == vector_cycles &&
          mcu.run_clock().native_epoch() == 2,
      "Original helper request applies the qualified MCU-only pulse and native COP vector fetch");
  bool refused = false;
  try {
    run.pair.schedule->end_instruction();
  } catch (const blofeld::Fault &) { refused = true; }
  require(refused && run.pair.schedule->faulted(),
          "Old board scheduling cannot reuse a changed MCU epoch");
  const auto after = mcu.bus().power_off_flash_image();
  require(after.bytes == nvm.bytes && after.known == nvm.known && after.fresh == nvm.fresh,
          "Actual MCU reset preserves every NVM byte, known bit and erase-history bit");
  run.pair.trace
      << "SYSTEM_RESET edges=195 vector=" << vector
      << " target=32768 ticks=1024 osc=205 bus=5 epoch=2 dsp-held=1 storage=1 joint-refused=1\n";
  mcu.set_cycle_observer({});
  mcu.cpu().step();
  require(
      mcu.cpu().state().pc == first_write,
      "Unmodified resident loader retires its first instruction before the first register write");
  bool unknown = false;
  try {
    mcu.cpu().step();
  } catch (const blofeld::Fault &error) {
    const std::string_view message(error.what());
    unknown = message.contains("MCU write at 0x0012") && message.contains("pc=0x8002") &&
              message.contains("Unimplemented MCU ECLK output observation at 0x0008");
    require(unknown,
            "Only the unqualified reset-board observation may stop native startup: " +
                std::string(error.what()));
    run.pair.trace << "RESET_BOOT_FAULT " << error.what() << '\n';
  }
  require(unknown && mcu.timing_faulted() && run.pair.dsp.cpu().debug_state() == dsp &&
              run.pair.dsp.run_clock().elapsed_ticks() == dsp_ticks,
          "Post-reset wiring remains unprovided; the independent DSP is held without guessed reset "
          "coupling");
  run.pair.trace
      << "RESET_BOOT start=32768 stop=32770 instructions=1 next-observation-unqualified=1\n";
}

void continuation(forward::Forward &run,
                  const blofeld::McuFlashImage &nvm,
                  const std::vector<transport::Block> &blocks,
                  const std::vector<unsigned> &caller,
                  const std::string &path,
                  const CopObservation &cop) {
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
          message.contains("MCU write at 0x003f") && message.contains("pc=0x129c") &&
              message.contains("Unqualified MCU COP system reset (invalid ARMCOP service)") &&
              drawing.prepared && drawing.delay == delay_iterations && drawing.waits == 1 &&
              mcu.pc == cop_entry && mcu.x == delay_iterations && mcu.sp == forward::stack - 14 &&
              mcu.d == restart_accumulators && dsp.pc == waiting_chunk &&
              dsp.r[1] == complete_source - 1 && run.loaded == forward::prefetched_words + 1 &&
              run.sent == forward::stream_samples + 2 && run.received == forward::expected_samples,
          "Only the original invalid ARMCOP service after all30000 delay iterations ends this "
          "path: " +
              std::string(error.what()));
      run.pair.trace << "COUPLED_FAULT " << error.what() << '\n';
      cop.verify(run);
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
      reset_continuation(run, nvm);
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
        CopObservation cop;
        forward::Forward run(firmware, trace, true, prepare, tail);
        const auto nvm = run.pair.mcu.bus().power_off_flash_image();
        run.execute(order, phase);
        run.verify(nvm);
        cop.bind(run);
        continuation(run,
                     nvm,
                     blocks,
                     caller,
                     std::string(argv[4]) + "." + std::to_string(order) + "." +
                         std::to_string(phase),
                     cop);
      }
    }
    std::cout
        << "Four native downloads, COP reset pulses and resident-loader continuations verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
