// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using namespace blofeld;
using namespace dsp56300;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .switch_timing = DspClockSwitch::reference_instruction_boundary};
constexpr Word boot_entry = 0xff0000;
constexpr Word ram_entry = 0x100;
constexpr Word jump_absolute = 0x0af080;
constexpr Word immediate_x0 = 0x44f400;
constexpr Word movec_sr = 0x05f439;
constexpr Word irqa_vector = 0x10;
constexpr Word priority_register = 0xffffff;
constexpr unsigned hold_ticks = 90, recovery_ticks = 135;

void require(const bool value, const char *message) {
  if (!value) { throw std::runtime_error(message); }
}

template <class Operation> void rejects(Operation operation, const char *message) {
  try {
    operation();
  } catch (const std::logic_error &) { return; } catch (const blofeld::Fault &) {
    return;
  }
  throw std::runtime_error(message);
}

void reset(OwnDspMachine &machine) {
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3});
  require(machine.advance_hardware_reset_ticks(hold_ticks) == hold_ticks, "Reset hold differs");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(recovery_ticks) == recovery_ticks,
          "Reset recovery differs");
}

void read_word(OwnDspFetch &fetch, const Word address) {
  fetch.issue();
  require(fetch.view().pending_address == address, "F1 did not own the expected address");
  fetch.capture();
  require(!fetch.view().pending_address, "F2 did not complete its actual sample");
}

void retire(OwnDspMachine &machine) {
  for (unsigned count = 0; count < 32; ++count) {
    const auto pending = machine.cpu().pending_instruction();
    if (!pending) { throw std::runtime_error("Native instruction lost its effect"); }
    const auto result = machine.advance(pending->ticket);
    require(result.fault.kind == FaultKind::none, "Native staged effect failed");
    if (result.retired) { return; }
  }
  throw std::runtime_error("Native instruction exceeded its effect bound");
}

void actual_fetch() {
  OwnDspMachine machine(clocks);
  const std::array<Word, 2> rom{jump_absolute, ram_entry};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = ram_entry, .value = immediate_x0});
  machine.bus().write({.address = ram_entry + 1, .value = 7});
  reset(machine);
  const auto ticks = machine.clock().elapsed_ticks();
  OwnDspFetch fetch(machine);
  rejects([&] { const OwnDspFetch duplicate(machine); }, "A second fetch owner was accepted");
  rejects([&] { static_cast<void>(machine.begin_fetch()); }, "Competing native fetch was accepted");
  rejects([&] { static_cast<void>(machine.admit_interrupt({.address = boot_entry, .opcode = 0})); },
          "Unqualified interrupt admission was accepted");
  read_word(fetch, boot_entry);
  rejects([&] { static_cast<void>(fetch.begin()); }, "Missing F2 extension was accepted");
  read_word(fetch, boot_entry + 1);
  require(fetch.begin().fault.kind == FaultKind::none, "Real captured jump was rejected");
  rejects([&] { fetch.synchronize(); }, "An unretired instruction was synchronized");
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().pc == ram_entry && fetch.view().next_address == ram_entry,
          "Actual jump did not redirect the F1/F2 owner");
  read_word(fetch, ram_entry);
  read_word(fetch, ram_entry + 1);
  machine.bus().write({.address = ram_entry + 1, .value = 11});
  require(fetch.begin().fault.kind == FaultKind::none, "Captured immediate was rejected");
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().x == 7 && machine.cpu().registers().pc == ram_entry + 2 &&
              machine.cpu().progress().instructions == 2 &&
              machine.clock().elapsed_ticks() == ticks,
          "CPU must execute the original F2 sample without manufacturing clock time");
  read_word(fetch, ram_entry + 2);
  try {
    static_cast<void>(fetch.begin());
    throw std::runtime_error("Unknown program memory was accepted");
  } catch (const blofeld::Fault &error) {
    const std::string detail(error.what());
    require(detail.contains("read P:0x000102") && detail.contains("pc=0x000102"),
            "Unknown F2 sample lost its operation, address or PC");
  }
}

DspDmaInputs dma_input(const unsigned cycle, const DspDmaGrant grant = DspDmaGrant::available) {
  return {.cycle = cycle,
          .known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = grant};
}

void continuous_fetch_pipeline() {
  constexpr unsigned cycles_before_admission = 10;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 12> rom{};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  const auto before = machine.cpu().registers();
  OwnDspFetch fetch(machine);
  for (unsigned cycle = 0; cycle < cycles_before_admission; ++cycle) {
    const auto result =
        fetch.advance_dma_pipeline(dma_input(cycle), false, OwnDspNativeCycleModel::unqualified);
    const auto expected_used = cycle < 8 ? cycle : 8U;
    require(fetch.view().used == expected_used && !result.native &&
                machine.cpu().registers() == before,
            "Automatic F1/F2 must not execute an unselected CPU effect");
    if (cycle < 8) {
      require(result.issued_address == boot_entry + cycle && result.captured == (cycle != 0),
              "F1 must issue sequentially after the previous actual F2 sample");
    } else {
      require(!result.issued_address && result.captured == (cycle == 8),
              "A full owned fetch queue must apply backpressure without another F1");
    }
  }
  require(machine.clock().native_cycle_units() == cycles_before_admission &&
              machine.clock().timer_clk2_edges() == cycles_before_admission / 2,
          "Continuous fetch and backpressure cycles must advance the real reference clock");
  static_cast<void>(fetch.begin());
  const auto pending = machine.cpu().pending_instruction();
  const auto issued =
      fetch.advance_dma_pipeline(dma_input(10), false, OwnDspNativeCycleModel::unqualified);
  const auto captured =
      fetch.advance_dma_pipeline(dma_input(11), false, OwnDspNativeCycleModel::unqualified);
  const auto held = machine.cpu().pending_instruction();
  require(issued.issued_address == boot_entry + 8 && !issued.captured && captured.captured &&
              !captured.issued_address && fetch.view().used == 8 && pending && held &&
              pending->effect == held->effect,
          "Released queue capacity must resume F1/F2 while preserving the held native effect");
  bool retired = false;
  unsigned next_cycle = 12;
  constexpr unsigned maximum_cycles = 20;
  for (; next_cycle < maximum_cycles && !retired; ++next_cycle) {
    const auto result = fetch.advance_dma_pipeline(
        dma_input(next_cycle), true, OwnDspNativeCycleModel::isolated_effect);
    if (!result.native) { throw std::runtime_error("Selected native effect was not admitted"); }
    require(!result.issued_address && !result.captured,
            "A full queue must hold F1 while selected native effects advance");
    retired = result.native->retired;
  }
  require(retired && machine.cpu().registers().pc == boot_entry + 1 &&
              machine.clock().native_cycle_units() == next_cycle,
          "Selected actual CPU effects must eventually retire without fabricated timing");
  fetch.synchronize();
}

void automatic_execution() {
  constexpr Word add_x0 = 0x200040;
  constexpr unsigned instruction_count = 3;
  constexpr unsigned maximum_cycles = 80;
  constexpr unsigned accumulator_shift = 24;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 16> rom{immediate_x0, 7, add_x0, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  rejects(
      [&] {
        static_cast<void>(
            fetch.advance_dma_automatic(dma_input(0), OwnDspAutomaticTiming::unqualified));
      },
      "Automatic execution requires an explicit timing hypothesis");
  unsigned starts = 0;
  unsigned retirements = 0;
  unsigned cycle = 0;
  for (; cycle < maximum_cycles && retirements < instruction_count; ++cycle) {
    const auto result =
        fetch.advance_dma_automatic(dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle);
    if (result.instruction_started) { ++starts; }
    if (result.native && result.native->retired) { ++retirements; }
    require(machine.clock().native_cycle_units() == cycle + 1,
            "Each automatic cycle must advance exactly one supplied clock period");
  }
  require(
      starts == instruction_count && retirements == instruction_count &&
          machine.cpu().progress().instructions == instruction_count &&
          machine.cpu().registers().pc == boot_entry + 4 && machine.cpu().registers().x == 7 &&
          machine.cpu().registers().a == (std::uint64_t{7} << accumulator_shift),
      "Automatic fetch and native effects must execute real immediate, ADD and NOP instructions");
}

void automatic_peripheral_write() {
  constexpr Word movep_immediate_once = 0x08f4bc;
  constexpr Word value = 0x234567;
  constexpr unsigned maximum_cycles = 30;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 8> rom{movep_immediate_once, value, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  unsigned cycle = 0;
  while (machine.cpu().progress().instructions == 0 && cycle < maximum_cycles) {
    static_cast<void>(
        fetch.advance_dma_automatic(dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle));
    ++cycle;
  }
  require(machine.cpu().progress().instructions == 1 && machine.bus().once_latch() == value &&
              machine.cpu().registers().pc == boot_entry + 2,
          "Automatic F2 overlap must execute the real immediate OnCE peripheral write");
}

void automatic_shi_status_test() {
  constexpr Word jset_status_bit_two = 0x0191a2;
  constexpr unsigned maximum_cycles = 30;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 8> rom{jset_status_bit_two, boot_entry + 5, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  unsigned cycle = 0;
  while (machine.cpu().progress().instructions == 0 && cycle < maximum_cycles) {
    static_cast<void>(
        fetch.advance_dma_automatic(dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle));
    ++cycle;
  }
  require(machine.cpu().progress().instructions == 1 &&
              machine.cpu().registers().pc == boot_entry + 2,
          "Real SHI status test must retain the actual clear bit without invented input");
}

void automatic_gpio_bit() {
  constexpr Word bset_port_c_bit_three = 0x013d23;
  constexpr Word selected_pin = 1U << 3U;
  constexpr unsigned maximum_cycles = 30;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 8> rom{bset_port_c_bit_three, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  auto &port = machine.bus().gpio(DspPort::c);
  port.write({.reg = DspGpioRegister::direction, .value = selected_pin});
  port.write({.reg = DspGpioRegister::data, .value = 0});
  OwnDspFetch fetch(machine);
  unsigned cycle = 0;
  while (machine.cpu().progress().instructions == 0 && cycle < maximum_cycles) {
    const auto result =
        fetch.advance_dma_automatic(dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle);
    require(!result.native || result.native->fault.kind == FaultKind::none,
            "Actual native GPIO bit operation failed");
    ++cycle;
  }
  const auto output = port.outputs();
  require(machine.cpu().progress().instructions == 1 &&
              machine.cpu().registers().pc == boot_entry + 1 &&
              (output.known & selected_pin) != 0 && (output.levels & selected_pin) != 0,
          "Native BSET must change the actual configured GPIO output");
}

void automatic_gpio_unknown_input() {
  constexpr Word bset_port_c_bit_three = 0x013d23;
  constexpr Word port_data = 0xffffbd;
  constexpr unsigned maximum_cycles = 30;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 8> rom{bset_port_c_bit_three, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  for (unsigned cycle = 0; cycle < maximum_cycles; ++cycle) {
    const auto result =
        fetch.advance_dma_automatic(dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle);
    if (result.native && result.native->fault.kind != FaultKind::none) {
      const auto failure = machine.io().failure();
      require(result.native->fault.kind == FaultKind::io_bit && failure &&
                  failure->address == port_data && failure->pc == boot_entry && failure->cause &&
                  machine.cpu().progress().instructions == 0,
              "Unknown GPIO input must retain the real bit fault and remain unretired");
      return;
    }
  }
  throw std::runtime_error("Unknown GPIO input was silently accepted");
}

void automatic_gpio_read(const bool known_input) {
  constexpr Word movep_port_c_y0 = 0x0446dd;
  constexpr Word data_register = 0xffffbd;
  constexpr unsigned port_mask = 0xfff;
  constexpr unsigned input = 0xa5;
  constexpr unsigned maximum_cycles = 30;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 8> rom{movep_port_c_y0, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  auto &port = machine.bus().gpio(DspPort::c);
  port.write({.reg = DspGpioRegister::control, .value = port_mask});
  if (known_input) { port.set_drive({.mask = port_mask, .levels = input}); }
  OwnDspFetch fetch(machine);
  for (unsigned cycle = 0; cycle < maximum_cycles; ++cycle) {
    const auto result =
        fetch.advance_dma_automatic(dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle);
    if (result.native && result.native->fault.kind != FaultKind::none) {
      require(!known_input && result.native->fault.kind == FaultKind::memory_read &&
                  result.native->fault.address == data_register && machine.io().failure(),
              "GPIO sample must preserve the actual unknown-input fault");
      return;
    }
    if (machine.cpu().progress().instructions == 1) {
      require(known_input && machine.cpu().registers().y == input,
              "Native MOVEP must publish the actual GPIO input");
      return;
    }
  }
  throw std::runtime_error("Native GPIO sample failed to finish within its bound");
}

void automatic_shi_receive() {
  constexpr Word movep_hrx_y0 = 0x044694;
  constexpr Word status_register = 0xffff91;
  constexpr Word receive_register = 0xffff94;
  constexpr Word serial_control = 0xa9;
  constexpr Word received = 0x345678;
  constexpr unsigned maximum_cycles = 30;
  constexpr unsigned word_bits = 24;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 8> rom{movep_hrx_y0, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  auto &shi = machine.bus().shi();
  shi.observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = status_register, .value = serial_control});
  shi.observe_spi({.selected = true, .mosi = false});
  for (unsigned bit = word_bits; bit != 0; --bit) {
    const bool level = ((received >> (bit - 1)) & 1U) != 0;
    shi.observe_spi({.selected = true, .clock = true, .mosi = level});
    shi.observe_spi({.selected = true, .clock = false, .mosi = level});
  }
  shi.observe_spi({.selected = false, .mosi = false});
  OwnDspFetch fetch(machine);
  unsigned cycle = 0;
  while (machine.cpu().progress().instructions == 0 && cycle < maximum_cycles) {
    const auto result =
        fetch.advance_dma_automatic(dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle);
    require(!result.native || result.native->fault.kind == FaultKind::none,
            "The actual SHI FIFO read must not fail");
    ++cycle;
  }
  require(machine.cpu().progress().instructions == 1 && machine.cpu().registers().y == received &&
              machine.cpu().registers().pc == boot_entry + 1,
          "The native instruction must read and publish the actual SPI word");
  rejects(
      [&] {
        static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = receive_register}));
      },
      "The actual native FIFO read must consume the word exactly once");
}

void automatic_program_write() {
  constexpr Word movem_x0_absolute = 0x077084;
  constexpr Word target = 0x100;
  constexpr Word value = 0x13579b;
  constexpr unsigned maximum_cycles = 20;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 12> rom{immediate_x0, value, movem_x0_absolute, target, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, boot_entry + 2);
  read_word(fetch, boot_entry + 3);
  static_cast<void>(fetch.begin());
  const auto pending = machine.cpu().pending_instruction();
  if (!pending || pending->effect.kind != EffectKind::write ||
      pending->effect.space != Space::program || pending->effect.address != target) {
    throw std::runtime_error("MOVEM did not reach its actual P write");
  }
  fetch.issue();
  const auto before = fetch.view();
  const auto first = fetch.advance_dma_automatic(
      dma_input(0), OwnDspAutomaticTiming::one_effect_per_cycle_program_write_priority);
  require(before.pending_address == boot_entry + 4 && !first.captured && first.native &&
              fetch.view().pending_address == before.pending_address &&
              machine.bus().read({.address = target}) == value,
          "Program write priority must hold F2 and perform the actual native P write");
  unsigned cycle = 1;
  for (; cycle < maximum_cycles && machine.cpu().progress().instructions < 2; ++cycle) {
    static_cast<void>(fetch.advance_dma_automatic(
        dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle_program_write_priority));
  }
  require(machine.cpu().progress().instructions == 2 &&
              machine.cpu().registers().pc == boot_entry + 4,
          "The actual MOVEM must retire once after its real program-memory write");
}

void automatic_program_write_fetch_guard() {
  constexpr Word movem_x0_absolute = 0x077084;
  constexpr Word target = ram_entry + 2;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 2> rom{jump_absolute, ram_entry};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = ram_entry, .value = movem_x0_absolute});
  machine.bus().write({.address = ram_entry + 1, .value = target});
  machine.bus().write({.address = target, .value = 0});
  reset(machine);
  static_cast<void>(machine.begin_fetch());
  retire(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, ram_entry);
  read_word(fetch, ram_entry + 1);
  static_cast<void>(fetch.begin());
  fetch.issue();
  try {
    static_cast<void>(fetch.advance_dma_automatic(
        dma_input(0), OwnDspAutomaticTiming::one_effect_per_cycle_program_write_priority));
    throw std::runtime_error("Write to an issued program word was accepted");
  } catch (const blofeld::Fault &error) {
    const std::string detail(error.what());
    require(detail.contains("write to issued/fetched P:0x000102") &&
                detail.contains("pc=0x000100") && fetch.view().pending_address == target &&
                machine.clock().native_cycle_units() == 0 && !machine.bus().last_dma_cycle(),
            "Unqualified program coherence must refuse before either bus operation or clock");
  }
}

void automatic_omr_transition() {
  constexpr Word movec_omr = 0x05f43a;
  constexpr Word changed_omr = 0x304;
  constexpr unsigned maximum_cycles = 30;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 8> rom{movec_omr, changed_omr, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  bool saw_held_stale = false;
  for (unsigned cycle = 0; cycle < maximum_cycles && machine.cpu().progress().instructions == 0;
       ++cycle) {
    const auto before = fetch.view();
    const bool stale = before.omr != machine.cpu().registers().omr;
    const auto result =
        fetch.advance_dma_automatic(dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle);
    if (stale) {
      saw_held_stale = true;
      require(!result.captured && !result.issued_address,
              "A stale F2 context must be held through the real OMR retirement");
    }
  }
  require(saw_held_stale && machine.cpu().progress().instructions == 1 &&
              machine.cpu().registers().omr == changed_omr && fetch.view().omr == changed_omr &&
              fetch.view().next_address == boot_entry + 2 && !fetch.view().pending_address,
          "Actual OMR retirement must replace the stale fetch context before further sampling");
}

void shi_status_dma_guard() {
  constexpr Word jset_status_bit_two = 0x0191a2;
  constexpr Word dma_control = 0xffffec;
  constexpr Word irq_triggered_copy = 0x880244;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 4> rom{jset_status_bit_two, boot_entry + 3, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  const auto pending = machine.cpu().pending_instruction();
  if (!pending || pending->effect.kind != EffectKind::bit) {
    throw std::runtime_error("SHI branch did not reach its actual bit test");
  }
  fetch.issue();
  for (const auto address : {dma_control + 3, dma_control + 2, dma_control + 1}) {
    machine.bus().write({.space = DspSpace::x, .address = address});
  }
  machine.bus().write({.space = DspSpace::x, .address = dma_control, .value = irq_triggered_copy});
  try {
    static_cast<void>(
        fetch.advance_dma_capture_native(dma_input(0), OwnDspNativeCycleModel::isolated_effect));
    throw std::runtime_error("Unqualified active DMA/SHI overlap was accepted");
  } catch (const blofeld::Fault &error) {
    const std::string detail(error.what());
    const auto held = machine.cpu().pending_instruction();
    require(detail.contains("native=read X:0xffff91") && detail.contains("pc=0xff0000") && held &&
                held->effect == pending->effect && !machine.bus().last_dma_cycle() &&
                machine.clock().native_cycle_units() == 0,
            "Active DMA/SHI overlap must refuse before the actual bit test or a DMA sample");
  }
}

void automatic_interrupt_frontier() {
  constexpr unsigned maximum_cycles = 40;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 8> rom{movec_sr, 0, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  unsigned cycle = 0;
  while (machine.cpu().progress().instructions == 0 && cycle < maximum_cycles) {
    static_cast<void>(
        fetch.advance_dma_automatic(dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle));
    ++cycle;
  }
  require(machine.cpu().progress().instructions == 1 && machine.cpu().registers().sr == 0,
          "Real MOVEC must unmask the interrupt before the frontier test");
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(true);
  machine.bus().observe_irqa(false);
  const auto before = machine.cpu().registers();
  try {
    static_cast<void>(
        fetch.advance_dma_automatic(dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle));
    throw std::runtime_error("Automatic execution skipped a real pending interrupt");
  } catch (const blofeld::Fault &error) {
    const std::string detail(error.what());
    require(detail.contains("interrupt vector=0x000010") && detail.contains("pc=0xff0002") &&
                machine.cpu().registers() == before && machine.bus().interrupt_request(0) &&
                machine.clock().native_cycle_units() == cycle,
            "An unqualified IRQ must remain pending without starting an instruction or clock");
  }
}

void automatic_interrupt_vector(const bool missing_second) {
  constexpr unsigned maximum_cycles = 100;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 8> rom{movec_sr, 0, immediate_x0, 7, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = irqa_vector, .value = 0});
  if (!missing_second) { machine.bus().write({.address = irqa_vector + 1, .value = 0}); }
  reset(machine);
  OwnDspFetch fetch(machine);
  unsigned cycle = 0;
  constexpr auto timing =
      OwnDspAutomaticTiming::one_effect_per_cycle_write_priority_with_interrupts;
  while (machine.cpu().progress().instructions == 0 && cycle < maximum_cycles) {
    static_cast<void>(fetch.advance_dma_automatic(dma_input(cycle++), timing));
  }
  require(machine.cpu().registers().sr == 0, "Interrupt must be unmasked by actual MOVEC");
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(true);
  machine.bus().observe_irqa(false);
  bool saw_first = false;
  bool saw_second = false;
  for (; cycle < maximum_cycles; ++cycle) {
    const auto result = fetch.advance_dma_automatic(dma_input(cycle), timing);
    if (result.native && result.native->retired &&
        machine.cpu().instruction_fetch().exception_vector &&
        machine.cpu().instruction_fetch().address == irqa_vector + 1) {
      saw_first = true;
      require(machine.bus().interrupt_request(0).has_value(),
              "First actual vector read must not acknowledge the interrupt");
    }
    if (saw_first && result.native && result.native->fault.kind == FaultKind::memory_read) {
      require(missing_second && result.native->fault.address == irqa_vector + 1 &&
                  machine.bus().interrupt_request(0).has_value(),
              "Missing second vector word must fault and retain its IRQ");
      saw_second = true;
      break;
    }
    if (saw_first && !machine.bus().interrupt_request(0)) { saw_second = true; }
    if (!missing_second && saw_second && machine.cpu().registers().x == 7) { break; }
  }
  require(saw_first && saw_second && (missing_second || machine.cpu().registers().x == 7),
          "Automatic IRQ must execute both actual vector words and resume its foreground");
}

void automatic_interrupt_missing_first() {
  constexpr unsigned maximum_cycles = 80;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 8> rom{movec_sr, 0, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = irqa_vector + 1, .value = 0});
  reset(machine);
  OwnDspFetch fetch(machine);
  unsigned cycle = 0;
  constexpr auto timing =
      OwnDspAutomaticTiming::one_effect_per_cycle_write_priority_with_interrupts;
  while (machine.cpu().progress().instructions == 0 && cycle < maximum_cycles) {
    static_cast<void>(fetch.advance_dma_automatic(dma_input(cycle++), timing));
  }
  require(machine.cpu().registers().sr == 0, "Interrupt must be unmasked by actual MOVEC");
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(true);
  machine.bus().observe_irqa(false);
  for (; cycle < maximum_cycles; ++cycle) {
    const auto result = fetch.advance_dma_automatic(dma_input(cycle), timing);
    if (result.native && result.native->fault.kind == FaultKind::memory_read) {
      require(result.native->fault.address == irqa_vector &&
                  result.native->fault.space == Space::program && machine.io().failure() &&
                  machine.bus().interrupt_request(0).has_value(),
              "Unknown first vector must fault with its actual address and retain IRQ ownership");
      return;
    }
  }
  throw std::runtime_error("Automatic IRQ skipped the missing first vector word");
}

void timeline_boundaries() {
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 8> rom{immediate_x0, 7, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  constexpr std::uint64_t epoch = 1000;
  constexpr std::uint64_t period = 45;
  OwnDspTimeline timeline(machine, OwnDspAutomaticTiming::one_effect_per_cycle, epoch);
  const auto first = epoch + hold_ticks + recovery_ticks + period;
  require(timeline.next_deadline() == first, "Deadline must include reset, epoch and DSP period");
  require(!timeline.advance_one(first, OwnDspBoundary::before, dma_input(0)) &&
              machine.clock().native_cycle_units() == 0,
          "Before boundary must preserve a simultaneous DSP cycle");
  const auto started = timeline.advance_one(first, OwnDspBoundary::through, dma_input(0));
  require(started && machine.clock().native_cycle_units() == 1 &&
              timeline.next_deadline() == first + period,
          "Through boundary must execute exactly one real DSP cycle");
  rejects(
      [&] { static_cast<void>(timeline.advance_one(first, OwnDspBoundary::before, dma_input(1))); },
      "Timeline rewound a consumed boundary");
  const auto later = first + (2 * period);
  require(timeline.advance_one(later, OwnDspBoundary::through, dma_input(1)).has_value() &&
              timeline.advance_one(later, OwnDspBoundary::through, dma_input(2)).has_value() &&
              !timeline.advance_one(later, OwnDspBoundary::through, dma_input(3)),
          "A shared horizon must admit each due DSP period once");
  require(machine.clock().native_cycle_units() == 3 && machine.bus().last_dma_cycle() == 2,
          "Timeline must preserve the real DSP and DMA cycle counts");
}

void timeline_program_tie(const bool external_first) {
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 2> rom{jump_absolute, ram_entry};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = ram_entry, .value = immediate_x0});
  machine.bus().write({.address = ram_entry + 1, .value = 7});
  machine.bus().write({.address = ram_entry + 2, .value = 0});
  reset(machine);
  static_cast<void>(machine.begin_fetch());
  retire(machine);
  OwnDspTimeline timeline(machine, OwnDspAutomaticTiming::one_effect_per_cycle, 0);
  for (unsigned cycle = 0; cycle < 2; ++cycle) {
    require(
        timeline.advance_one(timeline.next_deadline(), OwnDspBoundary::through, dma_input(cycle))
            .has_value(),
        "Actual opcode fetch cycle must execute");
  }
  const auto tie = timeline.next_deadline();
  if (external_first) {
    require(!timeline.advance_one(tie, OwnDspBoundary::before, dma_input(2)),
            "DSP sampled before the selected external-first boundary");
    machine.bus().write({.address = ram_entry + 1, .value = 9});
  }
  const auto captured = timeline.advance_one(tie, OwnDspBoundary::through, dma_input(2));
  require(captured && captured->captured, "The selected tie must perform the real F2 fetch");
  if (!external_first) { machine.bus().write({.address = ram_entry + 1, .value = 9}); }
  for (unsigned cycle = 3; cycle < 12 && machine.cpu().registers().pc == ram_entry; ++cycle) {
    static_cast<void>(
        timeline.advance_one(timeline.next_deadline(), OwnDspBoundary::through, dma_input(cycle)));
  }
  require(machine.cpu().registers().x == (external_first ? 9U : 7U),
          "F2 must execute the value observed in the selected external/DSP order");
}

void timeline_fault() {
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 1> rom{immediate_x0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspTimeline timeline(machine, OwnDspAutomaticTiming::one_effect_per_cycle, 0);
  for (unsigned cycle = 0; cycle < 10; ++cycle) {
    try {
      static_cast<void>(timeline.advance_one(
          timeline.next_deadline(), OwnDspBoundary::through, dma_input(cycle)));
    } catch (const blofeld::Fault &error) {
      require(std::string(error.what()).contains("0xff0001") && timeline.faulted(),
              "Unknown actual program read must report its address and quarantine the timeline");
      rejects([&] { static_cast<void>(timeline.next_deadline()); },
              "Faulted timeline accepted another native step");
      return;
    }
  }
  throw std::runtime_error("Timeline skipped an unknown program extension");
}

void timeline_initial_reset() {
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  constexpr std::uint64_t epoch = 1000;
  constexpr std::uint64_t partial_hold = 44;
  constexpr std::uint64_t excess = 100;
  constexpr std::uint64_t period = 45;
  const OwnDspResetInputs reset_inputs{
      .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3};
  OwnDspTimeline timeline(
      machine, OwnDspAutomaticTiming::one_effect_per_cycle, epoch, reset_inputs);
  rejects([&] { static_cast<void>(timeline.next_deadline()); },
          "Held reset cannot offer an executing DSP period");
  require(timeline.advance_reset_to(epoch + partial_hold) == partial_hold,
          "Reset must retain fractional EXTAL time");
  rejects([&] { timeline.release_hardware_reset(); }, "Reset released before two EXTAL periods");
  require(timeline.advance_reset_to(epoch + hold_ticks) == hold_ticks - partial_hold,
          "Reset hold must advance the remaining supplied ticks");
  timeline.release_hardware_reset();
  require(timeline.advance_reset_to(epoch + hold_ticks + recovery_ticks + excess) ==
                  recovery_ticks &&
              machine.clock().elapsed_ticks() == hold_ticks + recovery_ticks &&
              machine.cpu().progress().instructions == 0 && !machine.bus().last_dma_cycle(),
          "Recovery must stop exactly at completion without reading absent ROM or executing");
  const std::array<Word, 8> rom{immediate_x0, 7, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  require(timeline.next_deadline() == epoch + hold_ticks + recovery_ticks + period,
          "First execution deadline must follow actual reset recovery");
  for (unsigned cycle = 0; cycle < 10 && machine.cpu().registers().x != 7; ++cycle) {
    static_cast<void>(
        timeline.advance_one(timeline.next_deadline(), OwnDspBoundary::through, dma_input(cycle)));
  }
  require(machine.cpu().registers().x == 7, "Actual reset entry must execute the supplied ROM");
}

void timeline_reset_stale_owner() {
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  OwnDspTimeline timeline(
      machine,
      OwnDspAutomaticTiming::one_effect_per_cycle,
      0,
      OwnDspResetInputs{.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3});
  const auto checkpoint = machine.checkpoint();
  machine.restore(checkpoint);
  rejects([&] { static_cast<void>(timeline.advance_reset_to(hold_ticks)); },
          "Restored held reset must invalidate timeline ownership");
}

void timeline_stale_owner() {
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 1> rom{0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  const auto checkpoint = machine.checkpoint();
  OwnDspTimeline timeline(machine, OwnDspAutomaticTiming::one_effect_per_cycle, 0);
  machine.restore(checkpoint);
  rejects([&] { static_cast<void>(timeline.next_deadline()); },
          "Timeline accepted a restored machine with stale fetch ownership");
}

void dma_fetch_conflict() {
  constexpr Word source_register = 0xffffef;
  constexpr Word destination_register = 0xffffee;
  constexpr Word counter_register = 0xffffed;
  constexpr Word control_register = 0xffffec;
  constexpr Word source = 0x120;
  constexpr Word program_copy = 0x9802da;
  constexpr Word original_opcode = 0x200000;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 2> rom{jump_absolute, ram_entry};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = source, .value = 0});
  machine.bus().write({.address = ram_entry, .value = original_opcode});
  reset(machine);
  static_cast<void>(machine.begin_fetch());
  retire(machine);
  require(machine.cpu().registers().pc == ram_entry, "Native jump did not reach RAM");
  for (const auto write : std::array{
           DspStore{.space = DspSpace::x, .address = source_register, .value = source},
           DspStore{.space = DspSpace::x, .address = destination_register, .value = ram_entry},
           DspStore{.space = DspSpace::x, .address = counter_register},
           DspStore{.space = DspSpace::x, .address = control_register, .value = program_copy}}) {
    machine.bus().write(write);
  }
  const auto source_read = machine.advance_dma_reference_cycle(dma_input(0));
  require(source_read.phase && source_read.phase->kind == DspDmaPhaseKind::source_read,
          "The genuine DMA P source must be captured before F2");
  OwnDspFetch fetch(machine);
  rejects([&] { static_cast<void>(fetch.advance_dma_capture(dma_input(1))); },
          "F2 cycle without an issued address was accepted");
  require(machine.clock().native_cycle_units() == 1 && machine.bus().last_dma_cycle() == 0,
          "Rejected F2 admission must not consume DMA time");
  fetch.issue();
  const auto waiting = fetch.advance_dma_pipeline(
      dma_input(1, DspDmaGrant::wait), false, OwnDspNativeCycleModel::unqualified);
  const auto written =
      fetch.advance_dma_pipeline(dma_input(2), false, OwnDspNativeCycleModel::unqualified);
  require(waiting.dma.core_stall && !waiting.dma.word && !waiting.captured &&
              written.dma.core_stall && written.dma.word && !written.captured &&
              !waiting.issued_address && !written.issued_address &&
              fetch.view().pending_address == ram_entry && fetch.view().used == 0 &&
              machine.bus().read({.address = ram_entry}) == 0,
          "A real DMA wait and write must hold the issued F2 address");
  const auto captured =
      fetch.advance_dma_pipeline(dma_input(3), false, OwnDspNativeCycleModel::unqualified);
  require(captured.captured && captured.issued_address == ram_entry + 1 &&
              !captured.dma.core_stall && fetch.view().used == 1 &&
              fetch.view().words.front() == DspProgramWord{.address = ram_entry, .value = 0} &&
              machine.clock().native_cycle_units() == 4 && machine.clock().timer_clk2_edges() == 2,
          "F2 must sample the actual DMA-written instruction on the next available cycle");
  require(fetch.begin().fault.kind == FaultKind::none, "DMA-written F2 opcode was not admitted");
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().pc == ram_entry + 1,
          "The actual sampled DMA-written opcode must retire on the own core");
}

void dma_fetch_native_overlap() {
  constexpr Word source_register = 0xffffef;
  constexpr Word destination_register = 0xffffee;
  constexpr Word counter_register = 0xffffed;
  constexpr Word control_register = 0xffffec;
  constexpr Word source = 0x120;
  constexpr Word program_copy = 0x9802da;
  constexpr Word load_x0_absolute = 0x44f000;
  constexpr Word data_address = 0x500;
  constexpr Word data = 0x123456;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 2> rom{jump_absolute, ram_entry};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  for (const auto write :
       std::array{DspStore{.address = source, .value = 0},
                  DspStore{.address = ram_entry, .value = load_x0_absolute},
                  DspStore{.address = ram_entry + 1, .value = data_address},
                  DspStore{.address = ram_entry + 2, .value = immediate_x0},
                  DspStore{.space = DspSpace::x, .address = data_address, .value = data}}) {
    machine.bus().write(write);
  }
  reset(machine);
  static_cast<void>(machine.begin_fetch());
  retire(machine);
  for (const auto write : std::array{
           DspStore{.space = DspSpace::x, .address = source_register, .value = source},
           DspStore{.space = DspSpace::x, .address = destination_register, .value = ram_entry + 2},
           DspStore{.space = DspSpace::x, .address = counter_register},
           DspStore{.space = DspSpace::x, .address = control_register, .value = program_copy}}) {
    machine.bus().write(write);
  }
  const auto source_read = machine.advance_dma_reference_cycle(dma_input(0));
  require(source_read.phase && source_read.phase->kind == DspDmaPhaseKind::source_read,
          "DMA must really read its source before testing the destination overlap");
  OwnDspFetch fetch(machine);
  read_word(fetch, ram_entry);
  read_word(fetch, ram_entry + 1);
  static_cast<void>(fetch.begin());
  auto pending = machine.cpu().pending_instruction();
  if (!pending || pending->effect.kind != EffectKind::read || pending->effect.space != Space::x ||
      pending->effect.address != data_address) {
    throw std::runtime_error("Native load did not reach its actual X read");
  }
  fetch.issue();
  rejects(
      [&] {
        static_cast<void>(
            fetch.advance_dma_capture_native(dma_input(1), OwnDspNativeCycleModel::unqualified));
      },
      "Unqualified overlapping cycle was accepted");
  const auto waiting = fetch.advance_dma_capture_native(dma_input(1, DspDmaGrant::wait),
                                                        OwnDspNativeCycleModel::isolated_effect);
  const auto written =
      fetch.advance_dma_capture_native(dma_input(2), OwnDspNativeCycleModel::isolated_effect);
  const auto held = machine.cpu().pending_instruction();
  require(waiting.dma.core_stall && !waiting.captured && !waiting.native &&
              written.dma.core_stall && written.dma.word && !written.captured && !written.native &&
              fetch.view().pending_address == ram_entry + 2 && held &&
              held->effect == pending->effect,
          "The selected global stall must retain both actual F2 and native X read");
  const auto completed =
      fetch.advance_dma_capture_native(dma_input(3), OwnDspNativeCycleModel::isolated_effect);
  require(completed.captured && completed.native &&
              completed.native->fault.kind == FaultKind::none &&
              fetch.view().words.front() == DspProgramWord{.address = ram_entry + 2, .value = 0} &&
              machine.clock().native_cycle_units() == 4 && machine.clock().timer_clk2_edges() == 2,
          "Released cycle must perform the actual F2 read and independent native X read");
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().x == data, "Native instruction must publish the real X word");
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().pc == ram_entry + 3,
          "DMA-written opcode captured during the X load must execute on the own core");
}

void dma_fetch_native_program_conflict() {
  constexpr Word movem_x0 = 0x07f084;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 3> rom{movem_x0, ram_entry, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = ram_entry, .value = 7});
  reset(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  const auto pending = machine.cpu().pending_instruction();
  if (!pending || pending->effect.kind != EffectKind::read ||
      pending->effect.space != Space::program || pending->effect.address != ram_entry) {
    throw std::runtime_error("Native MOVEM did not reach its actual program read");
  }
  fetch.issue();
  try {
    static_cast<void>(
        fetch.advance_dma_capture_native(dma_input(0), OwnDspNativeCycleModel::isolated_effect));
    throw std::runtime_error("Two unqualified program bus reads were accepted");
  } catch (const blofeld::Fault &error) {
    const std::string detail(error.what());
    const auto held = machine.cpu().pending_instruction();
    require(detail.contains("read P:0xff0002") && detail.contains("native=read P:0x000100") &&
                detail.contains("pc=0xff0000") && held && held->effect == pending->effect &&
                fetch.view().pending_address == boot_entry + 2 &&
                machine.clock().native_cycle_units() == 0 && !machine.bus().last_dma_cycle(),
            "Unqualified dual program reads must retain addresses and leave both effects pending");
  }
}

void cache_fallback() {
  constexpr Word cache_status = 0xc80300;
  constexpr Word normal_status = 0xc00300;
  const std::array<Word, 7> rom{
      movec_sr, cache_status, immediate_x0, 7, movec_sr, normal_status, 0};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  const auto ticks = machine.clock().elapsed_ticks();
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  require(machine.cpu().registers().sr == cache_status, "Native cache status did not retire");
  fetch.synchronize();
  rejects([&] { fetch.issue(); }, "F2 prefetch bypassed native cache ownership");
  auto progress = fetch.begin();
  auto pending = progress.pending;
  if (!pending) { throw std::runtime_error("Native cache fetch has no effect"); }
  require(pending->effect.kind == EffectKind::fetch_opcode &&
              pending->effect.address == boot_entry + 2,
          "Enabled cache must use the actual native program fetch");
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().x == 7 && machine.cpu().registers().pc == boot_entry + 4,
          "Native CE-enabled fetch must execute the actual immediate");
  progress = fetch.begin();
  pending = progress.pending;
  if (!pending) { throw std::runtime_error("Native cache fetch has no disable effect"); }
  require(pending->effect.kind == EffectKind::fetch_opcode &&
              pending->effect.address == boot_entry + 4,
          "Native cache owner must fetch the actual CE-disable instruction");
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().sr == normal_status &&
              fetch.view().next_address == boot_entry + 6,
          "CE disable must restore the actual F2 address");
  read_word(fetch, boot_entry + 6);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().pc == boot_entry + 7 &&
              machine.clock().elapsed_ticks() == ticks,
          "Captured F2 execution must resume after the real CE transition");
}

void cache_unmapped_program() {
  constexpr Word cache_status = 0xc80300;
  constexpr Word external = 0x200000;
  const std::array<Word, 4> rom{movec_sr, cache_status, jump_absolute, external};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  const auto started = fetch.begin();
  const auto pending = started.pending;
  if (!pending) { throw std::runtime_error("Unmapped native fetch has no effect"); }
  require(pending->effect.kind == EffectKind::fetch_opcode && pending->effect.address == external,
          "CE-enabled core must request the actual unmapped P word");
  const auto result = machine.advance(pending->ticket);
  require(result.fault.kind == FaultKind::memory_read && result.fault.space == Space::program &&
              result.fault.address == external && result.fault.pc == external &&
              machine.io().failure(),
          "Unknown external P must retain its real operation, address and PC");
}

enum class Acknowledgement : std::uint8_t { pending, accepted };

void vector_read(OwnDspMachine &machine,
                 const Word expected,
                 const Acknowledgement acknowledgement) {
  const auto pending = machine.cpu().pending_instruction();
  if (!pending) { throw std::runtime_error("Native vector lost its fetch"); }
  require(pending->effect.kind == EffectKind::fetch_opcode && pending->effect.address == expected &&
              pending->effect.pc == expected && pending->effect.space == Space::program,
          "Native vector must own the actual program-bus access");
  const auto result = machine.advance(pending->ticket);
  require(result.fault.kind == FaultKind::none && machine.bus().interrupt_request(0).has_value() ==
                                                      (acknowledgement == Acknowledgement::pending),
          "Interrupt acknowledgement must follow the second actual vector read");
}

void interrupt_vector(const bool missing_second) {
  const std::array<Word, 4> rom{movec_sr, 0, immediate_x0, 7};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = irqa_vector, .value = 0});
  if (!missing_second) { machine.bus().write({.address = irqa_vector + 1, .value = 0}); }
  reset(machine);
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(true);
  machine.bus().observe_irqa(false);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  require(!fetch.admit_interrupt() && machine.bus().interrupt_request(0).has_value(),
          "Masked interrupt must remain pending with the actual F2 foreground");
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, boot_entry + 2);
  read_word(fetch, boot_entry + 3);
  require(fetch.admit_interrupt(), "Unmasked real IRQA was not admitted");
  rejects([&] { fetch.issue(); }, "F2 prefetch was accepted during native vector ownership");
  static_cast<void>(fetch.begin());
  vector_read(machine, irqa_vector, Acknowledgement::pending);
  retire(machine);
  fetch.synchronize();
  static_cast<void>(fetch.begin());
  if (missing_second) {
    const auto pending = machine.cpu().pending_instruction();
    if (!pending) { throw std::runtime_error("Second vector read is missing"); }
    const auto result = machine.advance(pending->ticket);
    require(result.fault.kind == FaultKind::memory_read && result.fault.space == Space::program &&
                result.fault.address == irqa_vector + 1 && result.fault.pc == irqa_vector + 1 &&
                machine.io().failure() && machine.bus().interrupt_request(0).has_value(),
            "Unknown vector word must fail with its actual address and retain IRQ ownership");
    return;
  }
  vector_read(machine, irqa_vector + 1, Acknowledgement::accepted);
  retire(machine);
  fetch.synchronize();
  require(fetch.view().next_address == boot_entry + 2 &&
              machine.cpu().registers().pc == boot_entry + 2,
          "Fast vector must resume the actual foreground PC");
  read_word(fetch, boot_entry + 2);
  read_word(fetch, boot_entry + 3);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().x == 7, "Foreground must execute after the actual vector");
}

void interrupt_call(const bool short_vector) {
  constexpr Word long_call = 0x0bf080;
  constexpr Word short_call = 0x0d0011;
  constexpr Word handler = 0x200;
  const std::array<Word, 3> rom{movec_sr, 0, 0};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = irqa_vector, .value = short_vector ? short_call : long_call});
  machine.bus().write({.address = irqa_vector + 1, .value = handler});
  machine.bus().write({.address = handler, .value = 0});
  reset(machine);
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(true);
  machine.bus().observe_irqa(false);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, boot_entry + 2);
  require(fetch.admit_interrupt(), "Real IRQA call was not admitted");
  static_cast<void>(fetch.begin());
  vector_read(machine, irqa_vector, Acknowledgement::pending);
  if (short_vector) {
    retire(machine);
    require(machine.cpu().registers().pc == irqa_vector + 1 && machine.bus().interrupt_request(0),
            "Short vector must retain its actual unacknowledged source");
    try {
      fetch.synchronize();
      throw std::runtime_error("Unqualified one-word vector was accepted");
    } catch (const blofeld::Fault &error) {
      require(std::string(error.what()).contains("interrupt vector continuation"),
              "Unqualified vector must report its real continuation");
    }
    return;
  }
  const auto extension = machine.cpu().pending_instruction();
  if (!extension) { throw std::runtime_error("Long vector extension is missing"); }
  require(extension->effect.kind == EffectKind::fetch_extension &&
              extension->effect.address == irqa_vector + 1,
          "Long vector must read the actual second vector word");
  const auto result = machine.advance(extension->ticket);
  require(result.fault.kind == FaultKind::none && !machine.bus().interrupt_request(0),
          "Real second vector word must acknowledge the source");
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().pc == handler && machine.cpu().registers().sp == 1 &&
              fetch.view().next_address == handler,
          "Long interrupt call must install its real frame and handler fetch");
  read_word(fetch, handler);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().registers().pc == handler + 1,
          "The native interrupt handler must execute its actual instruction");
}

void missing_first_vector() {
  const std::array<Word, 3> rom{movec_sr, 0, 0};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = irqa_vector + 1, .value = 0});
  reset(machine);
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(true);
  machine.bus().observe_irqa(false);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, boot_entry + 2);
  require(fetch.admit_interrupt(), "Real IRQ with unknown vector was not admitted");
  static_cast<void>(fetch.begin());
  const auto pending = machine.cpu().pending_instruction();
  if (!pending) { throw std::runtime_error("First vector read is missing"); }
  const auto result = machine.advance(pending->ticket);
  require(result.fault.kind == FaultKind::memory_read && result.fault.space == Space::program &&
              result.fault.address == irqa_vector && result.fault.pc == irqa_vector &&
              machine.io().failure() && machine.bus().interrupt_request(0).has_value(),
          "Unknown first vector must retain IRQ and report its true read and PC");
}

void software_vector() {
  constexpr Word trap = 6;
  constexpr Word trap_vector = 8;
  const std::array<Word, 2> rom{trap, 0};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = trap_vector, .value = 0});
  machine.bus().write({.address = trap_vector + 1, .value = 0});
  reset(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  require(machine.cpu().instruction_fetch().exception_vector &&
              fetch.view().next_address == trap_vector,
          "The native TRAP must redirect to its actual software vector");
  for (const auto address : {trap_vector, trap_vector + 1}) {
    read_word(fetch, address);
    static_cast<void>(fetch.begin());
    retire(machine);
    fetch.synchronize();
  }
  require(!machine.cpu().instruction_fetch().exception_vector &&
              machine.cpu().registers().pc == boot_entry + 1 &&
              fetch.view().next_address == boot_entry + 1,
          "Software-vector completion must return to the actual foreground");
}

void repeat_latch() {
  constexpr Word repeat_three = 0x0603a0;
  constexpr Word add_x0 = 0x200040;
  const std::array<Word, 2> rom{jump_absolute, ram_entry};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  machine.bus().write({.address = ram_entry, .value = immediate_x0});
  machine.bus().write({.address = ram_entry + 1, .value = 7});
  machine.bus().write({.address = ram_entry + 2, .value = repeat_three});
  machine.bus().write({.address = ram_entry + 3, .value = add_x0});
  reset(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, ram_entry);
  read_word(fetch, ram_entry + 1);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, ram_entry + 2);
  static_cast<void>(fetch.begin());
  retire(machine);
  fetch.synchronize();
  read_word(fetch, ram_entry + 3);
  machine.bus().write({.address = ram_entry + 3, .value = 0});
  static_cast<void>(fetch.begin());
  retire(machine);
  rejects([&] { fetch.issue(); }, "F1 after retirement bypassed REP synchronization");
  fetch.synchronize();
  for (unsigned iteration = 0; iteration < 2; ++iteration) {
    require(machine.cpu().instruction_fetch().latched_opcode == add_x0 &&
                fetch.view().consume_address == ram_entry + 4,
            "REP must own the actual original F2 opcode latch");
    rejects([&] { fetch.issue(); }, "Unqualified REP prefetch was accepted");
    static_cast<void>(fetch.begin());
    retire(machine);
    fetch.synchronize();
  }
  constexpr unsigned fractional_shift = 24;
  constexpr std::uint64_t expected_sum = 21;
  require(machine.cpu().registers().a == (expected_sum << fractional_shift) &&
              machine.cpu().registers().pc == ram_entry + 4 &&
              machine.cpu().progress().instructions == 6 && fetch.view().used == 0,
          "All REP iterations must execute the genuine latch despite overwritten RAM");
}

void automatic_repeat_latch() {
  constexpr Word repeat_three = 0x0603a0;
  constexpr Word add_x0 = 0x200040;
  constexpr unsigned maximum_cycles = 100;
  constexpr unsigned fractional_shift = 24;
  constexpr std::uint64_t expected_sum = 21;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 2> rom{jump_absolute, ram_entry};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  for (const auto write : std::array{DspStore{.address = ram_entry, .value = immediate_x0},
                                     DspStore{.address = ram_entry + 1, .value = 7},
                                     DspStore{.address = ram_entry + 2, .value = repeat_three},
                                     DspStore{.address = ram_entry + 3, .value = add_x0},
                                     DspStore{.address = ram_entry + 4, .value = 0}}) {
    machine.bus().write(write);
  }
  reset(machine);
  static_cast<void>(machine.begin_fetch());
  retire(machine);
  OwnDspFetch fetch(machine);
  bool replaced = false;
  unsigned held_cycles = 0;
  for (unsigned cycle = 0; cycle < maximum_cycles && machine.cpu().progress().instructions < 6;
       ++cycle) {
    const auto pending = machine.cpu().pending_instruction();
    if (!pending && machine.cpu().instruction_fetch().latched_opcode == add_x0 && !replaced) {
      machine.bus().write({.address = ram_entry + 3, .value = 0});
      replaced = true;
    }
    const bool repeating = replaced && machine.cpu().registers().pc == ram_entry + 3;
    const auto result =
        fetch.advance_dma_automatic(dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle);
    if (repeating) {
      ++held_cycles;
      require(!result.captured && !result.issued_address,
              "A native latched REP iteration must not fabricate an F1/F2 access");
    }
  }
  require(
      replaced && held_cycles != 0 && machine.cpu().progress().instructions == 6 &&
          machine.cpu().registers().a == (expected_sum << fractional_shift) &&
          machine.cpu().registers().pc == ram_entry + 4,
      "Automatic REP must execute the real captured opcode three times despite RAM replacement");
}

void automatic_repeat_dual_memory() {
  constexpr Word load_r1_absolute = 0x61f400;
  constexpr Word repeat_three = 0x0603a0;
  constexpr Word store_a_long_postincrement = 0x485900;
  constexpr Word target = 0x100;
  constexpr Word initial = 0x345678;
  constexpr unsigned maximum_cycles = 120;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 8> rom{
      load_r1_absolute, target, repeat_three, store_a_long_postincrement, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  for (unsigned index = 0; index < 4; ++index) {
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      machine.bus().write({.space = space, .address = target + index, .value = initial});
    }
  }
  OwnDspFetch fetch(machine);
  for (unsigned cycle = 0; cycle < maximum_cycles && machine.cpu().progress().instructions < 5;
       ++cycle) {
    const auto result =
        fetch.advance_dma_automatic(dma_input(cycle), OwnDspAutomaticTiming::one_effect_per_cycle);
    require(!result.native || result.native->fault.kind == FaultKind::none,
            "Repeated native long-store effect failed");
  }
  require(machine.cpu().progress().instructions == 5 &&
              machine.cpu().registers().r[1] == target + 3 &&
              machine.cpu().registers().pc == boot_entry + 4,
          "Native REP must execute exactly three long stores and address updates");
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned index = 0; index < 3; ++index) {
      require(machine.bus().read({.space = space, .address = target + index}) == 0,
              "Every X/Y word must be actually written by the repeated native store");
    }
    require(machine.bus().read({.space = space, .address = target + 3}) == initial,
            "REP must not write an extra X/Y pair");
  }
}

void native_repeat_recovery() {
  constexpr Word repeat_three = 0x0603a0;
  const std::array<Word, 2> rom{repeat_three, 0};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  {
    OwnDspFetch fetch(machine);
    read_word(fetch, boot_entry);
    static_cast<void>(fetch.begin());
    retire(machine);
    fetch.synchronize();
    read_word(fetch, boot_entry + 1);
    static_cast<void>(fetch.begin());
    retire(machine);
    fetch.synchronize();
  }
  require(machine.cpu().instruction_fetch().latched_opcode == 0,
          "Native REP must still own the original NOP latch");
  OwnDspFetch resumed(machine);
  rejects([&] { resumed.issue(); }, "A resumed native REP latch accepted an F2 read");
  const auto progress = resumed.begin();
  const auto pending = progress.pending;
  if (!pending) { throw std::runtime_error("Native REP continuation has no effect"); }
  require(pending->effect.kind != EffectKind::fetch_opcode,
          "A native REP latch must not fabricate another opcode fetch");
  retire(machine);
  resumed.synchronize();
  require(machine.cpu().progress().instructions == 3,
          "Resumed ownership must preserve the actual native REP continuation");
}

void automatic_native_repeat_recovery() {
  constexpr Word repeat_three = 0x0603a0;
  constexpr unsigned maximum_cycles = 30;
  auto cycle_clocks = clocks;
  cycle_clocks.switch_timing = DspClockSwitch::reference_cycle_boundary;
  OwnDspMachine machine(cycle_clocks);
  const std::array<Word, 2> rom{repeat_three, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  {
    OwnDspFetch first(machine);
    read_word(first, boot_entry);
    static_cast<void>(first.begin());
    retire(machine);
    first.synchronize();
    read_word(first, boot_entry + 1);
    static_cast<void>(first.begin());
    retire(machine);
    first.synchronize();
  }
  require(machine.cpu().instruction_fetch().latched_opcode == 0,
          "Restored owner must start from the native latched NOP");
  OwnDspFetch resumed(machine);
  unsigned cycle = 0;
  while (machine.cpu().progress().instructions < 4 && cycle < maximum_cycles) {
    const auto result = resumed.advance_dma_automatic(dma_input(cycle),
                                                      OwnDspAutomaticTiming::one_effect_per_cycle);
    require(!result.captured && !result.issued_address,
            "Native REP continuation must not manufacture program reads");
    ++cycle;
  }
  require(machine.cpu().progress().instructions == 4 &&
              machine.cpu().registers().pc == boot_entry + 2 &&
              !machine.cpu().instruction_fetch().latched_opcode &&
              resumed.view().next_address == boot_entry + 2,
          "The native latch must complete both actual remaining iterations before F1 resumes");
}

void changed_memory_context() {
  constexpr Word movec_omr = 0x05f43a;
  constexpr Word changed_omr = 0x304;
  const std::array<Word, 3> rom{movec_omr, changed_omr, 0};
  OwnDspMachine machine(clocks);
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  OwnDspFetch fetch(machine);
  read_word(fetch, boot_entry);
  read_word(fetch, boot_entry + 1);
  fetch.issue();
  static_cast<void>(fetch.begin());
  for (unsigned count = 0; count < 32 && machine.cpu().registers().omr != changed_omr; ++count) {
    const auto pending = machine.cpu().pending_instruction();
    if (!pending) { throw std::runtime_error("OMR instruction lost its effect"); }
    const auto result = machine.advance(pending->ticket);
    require(result.fault.kind == FaultKind::none && !result.retired,
            "OMR change must publish before retirement");
  }
  require(machine.cpu().registers().omr == changed_omr, "OMR transition did not publish");
  try {
    fetch.capture();
    throw std::runtime_error("F2 sampled with a stale memory context");
  } catch (const blofeld::Fault &error) {
    require(std::string(error.what()).contains("F2 memory context read P:0xff0002"),
            "Stale F2 context lost its actual pending address");
  }
  retire(machine);
  fetch.synchronize();
  require(fetch.view().omr == changed_omr && fetch.view().next_address == boot_entry + 2 &&
              !fetch.view().pending_address,
          "Real OMR retirement must discard the stale F1 and reset its context");
}

void invalidation() {
  OwnDspMachine machine(clocks);
  const std::array<Word, 1> rom{0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = rom});
  reset(machine);
  const auto checkpoint = machine.checkpoint();
  OwnDspFetch old(machine);
  read_word(old, boot_entry);
  machine.restore(checkpoint);
  rejects([&] { static_cast<void>(old.view()); }, "Restore retained stale F2 ownership");
  OwnDspFetch fresh(machine);
  read_word(fresh, boot_entry);
  static_cast<void>(fresh.begin());
  reset(machine);
  rejects([&] { fresh.issue(); }, "Hardware reset retained stale F2 ownership");
  OwnDspFetch after_reset(machine);
  read_word(after_reset, boot_entry);
}
}

int main() {
  try {
    actual_fetch();
    continuous_fetch_pipeline();
    automatic_execution();
    automatic_peripheral_write();
    automatic_shi_status_test();
    automatic_shi_receive();
    automatic_gpio_read(true);
    automatic_gpio_read(false);
    automatic_gpio_bit();
    automatic_gpio_unknown_input();
    automatic_program_write();
    automatic_program_write_fetch_guard();
    automatic_omr_transition();
    shi_status_dma_guard();
    automatic_interrupt_frontier();
    automatic_interrupt_vector(false);
    automatic_interrupt_vector(true);
    automatic_interrupt_missing_first();
    timeline_boundaries();
    timeline_program_tie(false);
    timeline_program_tie(true);
    timeline_fault();
    timeline_stale_owner();
    timeline_initial_reset();
    timeline_reset_stale_owner();
    dma_fetch_conflict();
    dma_fetch_native_overlap();
    dma_fetch_native_program_conflict();
    invalidation();
    cache_fallback();
    cache_unmapped_program();
    repeat_latch();
    automatic_repeat_latch();
    automatic_repeat_dual_memory();
    native_repeat_recovery();
    automatic_native_repeat_recovery();
    changed_memory_context();
    interrupt_vector(false);
    interrupt_vector(true);
    interrupt_call(false);
    interrupt_call(true);
    missing_first_vector();
    software_vector();
    std::cout << "Own DSP uses actual owned F1/F2 samples.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
