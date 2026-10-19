// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "coupled_boot_fixture.hpp"
#include "dsp_boot_reference.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <sstream>
#include <string>

namespace blofeld::test::coupled {
namespace {
void verify_unknown_ready(McuBus &bus) {
  require(bus.observe_pads(McuPort::u, 1).known == 0,
          "Releasing PU0 must not manufacture a DSP-ready level");
  try {
    static_cast<void>(bus.read(port_u));
  } catch (const Fault &fault) {
    const std::string message = fault.what();
    require(message.contains("0x0278") && message.contains("mask=0x01"),
            "Unobserved PU0 must report the actual address and unknown mask");
    return;
  }
  throw Fault("Unobserved PU0 unexpectedly became known");
}

unsigned cold_start(McuMachine &mcu, std::ostream &trace) {
  constexpr unsigned fixture_e_inputs = 0xfb;
  mcu.bus().set_pin_drive(McuPort::e, {.mask = fixture_e_inputs, .levels = 3});
  unsigned completed = 0;
  unsigned ready_visits = 0;
  bool phy_sampled = false;
  while (true) {
    constexpr unsigned instruction_limit = 2000000;
    require(completed < instruction_limit, "Cold original MCU boot exceeded its instruction limit");
    const auto before = mcu.cpu().state();
    if (before.pc == phy_poll && !phy_sampled) {
      mcu.bus().observe_phy({.edges = 15});
      phy_sampled = true;
    }
    if (before.pc == ready_poll) {
      if (ready_visits == 0) {
        verify_unknown_ready(mcu.bus());
        // Controlled inputs, not observed Blofeld routing or ready behavior.
        mcu.bus().set_pin_drive(McuPort::u, {.mask = 1});
        mcu.bus().set_pin_drive(McuPort::b, {.mask = other_b_inputs});
      }
      if (++ready_visits == 3) {
        mcu.bus().set_pin_drive(McuPort::u, {.mask = 1, .levels = 1});
        require(phy_sampled, "Native MCU reset must traverse its own clock-lock polling");
        trace << "cold-mcu-instructions=" << completed << " checkpoint-pc=" << hex(before.pc)
              << " ready-low-visits=2\n";
        return completed;
      }
    }
    const auto result = mcu.cpu().step();
    require(result.instruction_executed && !result.interrupt_vector,
            "Untimed cold MCU fixture executes without synthetic interrupts");
    ++completed;
  }
}

void verify_loaded_stage(DspBus &bus, const std::span<const std::uint8_t> mcu) {
  const auto frame = mcu.subspan(frame_offset);
  require(dsp_boot_word(std::span<const std::uint8_t, 3>(frame.first(3))) == stage_words &&
              dsp_boot_word(std::span<const std::uint8_t, 3>(frame.subspan(3, 3))) == stage_base,
          "Original MCU image supplies the independently identified frame header");
  for (unsigned index = 0; index < stage_words; ++index) {
    const auto expected = dsp_boot_word(std::span<const std::uint8_t, 3>(
        frame.subspan((static_cast<std::size_t>(index) + 2) * 3, 3)));
    require(bus.read({.address = stage_base + index}) == expected,
            "Native DSP must receive every original stage word from executing MCU GPIO stores");
  }
  constexpr unsigned receive_not_empty = 0x020000;
  constexpr unsigned receive_overrun = 0x100000;
  require((bus.shi().read(hcsr) & (receive_not_empty | receive_overrun)) == receive_not_empty,
          "Original ROM must consume the frame without overrun and retain the extra MCU word");
  const auto trailing = dsp_boot_word(std::span<const std::uint8_t, 3>(
      frame.subspan((static_cast<std::size_t>(stage_words) + 2) * 3, 3)));
  require(bus.shi().read(hrx) == trailing && (bus.shi().read(hcsr) & receive_not_empty) == 0,
          "Exactly the trailing original MCU word remains after the native ROM's load loop");
}

void verify_clear_frontier(DspBus &bus, DspCore &dsp, Link &link) {
  while (dsp.state().r[4] != long_ram_end) { link.step_dsp(); }
  const auto state = dsp.state();
  constexpr unsigned original_memory_mode = 0x600389;
  require(state.pc == clear_pc && state.lc == clear_count - long_ram_end &&
              state.omr == original_memory_mode,
          "Native loaded stage must reach the mapped clear-loop frontier without a loop skip");
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = 0; address < long_ram_end; ++address) {
      require(bus.read({.space = space, .address = address}) == 0,
              "Native long stores must clear every mapped low X/Y pair");
    }
  }
  constexpr unsigned pll_operand = 0x70;
  require(bus.pll().read() == bus.read({.address = pll_operand}),
          "Original first-stage MOVEP must program its own PCTL operand");
}

std::string run_experiment(McuMachine &mcu,
                           const std::span<const std::uint8_t> image,
                           const std::span<const std::uint32_t> rom,
                           const Experiment experiment) {
  DspBus bus;
  bus.install_rom({.start = bootstrap_base, .words = rom});
  DspCore dsp(bus);
  std::ostringstream trace;
  Link link(mcu, bus, dsp, trace, experiment);
  link.start();
  mcu.bus().set_observer([&link](const BusEvent &event) { link.observe(event); });
  unsigned completed = 0;
  // Independent original_mcu_sender trace: 1856593 total minus 1672478 before this checkpoint.
  constexpr unsigned sender_instructions = 184115;
  try {
    while (mcu.cpu().state().pc != sender_end) {
      require(completed < sender_instructions,
              "Original MCU transmission exceeded its independently observed count at PC=" +
                  hex(mcu.cpu().state().pc));
      const auto result = mcu.cpu().step();
      require(result.instruction_executed && !result.interrupt_vector,
              "Original sender must execute its own instructions without synthetic interrupts");
      ++completed;
      link.after_mcu_instruction();
    }
  } catch (...) {
    mcu.bus().set_observer({});
    throw;
  }
  mcu.bus().set_observer({});
  require(completed == sender_instructions,
          "Coupling must preserve the original sender instruction count exactly");
  link.verify_transmission();
  verify_loaded_stage(bus, image);
  verify_clear_frontier(bus, dsp, link);
  const auto state = mcu.cpu().state();
  trace << "sender-instructions=" << completed << " end-pc=" << hex(state.pc)
        << " d=" << hex(state.d) << " x=" << hex(state.x) << " y=" << hex(state.y)
        << " sp=" << hex(state.sp) << " ccr=" << hex(state.ccr, 2)
        << " dsp-instructions=" << link.dsp_steps() << '\n';
  return trace.str();
}

std::string verify_rejection(McuMachine &mcu,
                             const std::span<const std::uint8_t> image,
                             const std::span<const std::uint32_t> rom,
                             const Experiment experiment,
                             const std::string &expected) {
  try {
    static_cast<void>(run_experiment(mcu, image, rom, experiment));
  } catch (const Fault &fault) {
    const std::string message = fault.what();
    require(message.contains(expected) && message.contains("mcu-pc=") &&
                message.contains("dsp-pc=") && message.contains("MCU write at 0x0001"),
            "Coupled pin failure must identify both CPUs, bus operation and original reason: " +
                message);
    return message;
  }
  throw Fault("Invalid original-coupling experiment unexpectedly completed");
}

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path rom;
  std::filesystem::path trace;
};

int run(const Inputs &inputs) {
  const auto firmware = Firmware::load(inputs.firmware);
  const auto rom = read_spi_boot_prefix(inputs.rom);
  McuMachine mcu(McuFlashImage::from_known(firmware.mcu), BLOFELD_DEFAULT_SLA);
  std::ofstream trace(inputs.trace);
  require(trace.is_open(), "Cannot create private coupled CPU trace");
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  cold_start(mcu, trace);
  const auto checkpoint = mcu.checkpoint();
  trace << "rejected-unresolved="
        << verify_rejection(mcu, firmware.mcu, rom, {}, "Ambiguous simultaneous") << '\n';
  mcu.restore(checkpoint);
  trace << "rejected-select-before-clock="
        << verify_rejection(mcu,
                            firmware.mcu,
                            rom,
                            {.order = PadOrder::select_before_clock},
                            "configured idle SCK")
        << '\n';
  mcu.restore(checkpoint);
  const Experiment clock_first{.order = PadOrder::clock_before_select};
  const auto first = run_experiment(mcu, firmware.mcu, rom, clock_first);
  trace << "explicit-clock-before-select cadence=1\n" << first;
  mcu.restore(checkpoint);
  const auto replay = run_experiment(mcu, firmware.mcu, rom, clock_first);
  require(first == replay,
          "Trusted MCU checkpoint must replay every coupled CPU/pin event exactly");
  trace << "exact-coupled-replay=true\n";
  mcu.restore(checkpoint);
  trace << "explicit-clock-before-select cadence=4\n"
        << run_experiment(
               mcu,
               firmware.mcu,
               rom,
               {.order = PadOrder::clock_before_select, .dsp_steps_per_mcu_instruction = 4});
  mcu.restore(checkpoint);
  trace << "rejected-unknown-mosi="
        << verify_rejection(mcu,
                            firmware.mcu,
                            rom,
                            {.order = PadOrder::clock_before_select, .break_mosi = true},
                            "Unknown DSP SHI MOSI")
        << '\n';
  std::cout << "Native original MCU GPIO loaded all 94 DSP words through executed documented ROM; "
               "two explicit instruction interleavings, exact checkpoint replay and three strict "
               "pin-failure cases passed. Controlled one-way pad-order experiment, not a timed "
               "or hardware-verified Blofeld boot.\n";
  return 0;
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 4) {
      throw blofeld::Fault("Usage: coupled-boot-test firmware.zip private-rom.bin private.trace");
    }
    return blofeld::test::coupled::run(
        {.firmware = arguments[1], .rom = arguments[2], .trace = arguments[3]});
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
