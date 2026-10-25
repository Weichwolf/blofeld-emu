// SPDX-License-Identifier: GPL-3.0-only
#include "original_owned_clock_fixture.hpp"

#include "common/bus.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

namespace blofeld::test::owned_clock {
namespace {
constexpr unsigned phy_poll = 0x8049;
constexpr unsigned first_loopback = 0x8029;
constexpr unsigned second_loopback = 0x8036;
constexpr unsigned application = 0x8200;
constexpr unsigned release_inputs = 0x820e;
constexpr unsigned capture_enable = 0x87e8;
constexpr unsigned ready_poll = 0xb98d;
constexpr unsigned display_initialization = 0x8b94;
constexpr unsigned initial_e_mask = 0xfb;
constexpr unsigned application_e_mask = 0xeb;
constexpr unsigned other_b_inputs = 0xe8;
constexpr unsigned released_b_inputs = 0x1e;
constexpr unsigned ready_inputs = 0x3f;
constexpr unsigned receive_pad = 4;
constexpr unsigned low_flash = 0x4000;
constexpr unsigned last_address = 0xffff;
constexpr unsigned register_trace_end = 0x70;
constexpr unsigned root_limit = 80000000;
constexpr unsigned root_steps = 53388708;
constexpr unsigned root_retired = 53382260;
constexpr unsigned root_interrupts = 6448;
constexpr unsigned progress_period = 1000000;

void require_idle_receive(const McuMachine &machine) {
  const auto idle = machine.bus().observe_pads(McuPort::r, receive_pad);
  require(idle.known == receive_pad && idle.levels == receive_pad,
          "Original reference fixture requires an explicitly observed idle PR2");
}
}

std::filesystem::path artifact(const std::filesystem::path &trace, const std::string_view suffix) {
  auto result = trace;
  result += suffix;
  return result;
}

bool same_cpu(const CpuState &first, const CpuState &second) {
  return first.pc == second.pc && first.sp == second.sp && first.d == second.d &&
         first.x == second.x && first.y == second.y && first.ccr == second.ccr;
}

void check_nvm(const McuMachine &machine, const McuFlashImage &initial) {
  const auto current = machine.bus().power_off_flash_image();
  require(current.bytes == initial.bytes && current.known == initial.known &&
              current.fresh == initial.fresh,
          "Original clocked execution must preserve all MCU NVM bytes, knowledge and history");
}

void Inputs::observe(McuMachine &machine) {
  auto &board = machine.bus();
  const auto pc = machine.cpu().state().pc;
  if (pc == phy_poll && !phy) {
    board.observe_phy({.edges = 15});
    phy = true;
  }
  if (pc == first_loopback || pc == second_loopback) {
    const auto sample = board.observe_pads(McuPort::e, 4);
    require(sample.known == 4, "E2 must be known before observing its E3 loopback");
    const auto levels = static_cast<std::uint8_t>(3U | ((sample.levels & 4U) != 0 ? 8U : 0U));
    board.set_pin_drive(McuPort::e, {.mask = initial_e_mask, .levels = levels});
  }
  if (pc == application) {
    board.set_pin_drive(McuPort::q, {.floating = byte_mask});
    board.observe_usb_decoder({});
    board.set_pin_drive(McuPort::e, {.mask = application_e_mask, .levels = 3, .floating = 0x10});
    board.set_pin_drive(McuPort::b, {.mask = other_b_inputs});
  }
  if (pc == release_inputs) { board.set_pin_drive(McuPort::b, {.mask = released_b_inputs}); }
  if (pc == capture_enable && !capture) {
    board.set_pin_drive(McuPort::t, {.mask = 1, .levels = 1});
    machine.set_timer_inputs(1);
    capture = true;
  }
  if (pc == ready_poll) { board.set_pin_drive(McuPort::u, {.mask = ready_inputs, .levels = 1}); }
  if (pc == display_initialization) { board.set_pin_drive(McuPort::b, {.mask = 8}); }
}

void Census::before(McuMachine &machine) {
  const auto pc = machine.cpu().state().pc;
  auto &row = rows[pc];
  if (!row.instruction.empty()) { return; }
  row.instruction = machine.cpu().disassemble();
  for (unsigned index = 0; index < row.bytes.size(); ++index) {
    row.bytes[index] = machine.bus().peek(static_cast<std::uint16_t>(pc + index));
  }
}

void Census::retire(const CpuState before, const CpuStep result) {
  if (!result.instruction_executed) { return; }
  constexpr unsigned arithmetic_flags = 15;
  ++rows.at(before.pc).flags[before.ccr & arithmetic_flags];
  ++retired;
}

void Census::write(const std::filesystem::path &path) const {
  std::ofstream stream(path);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  for (const auto &[pc, row] : rows) {
    stream << "{\"pc\":" << pc << ",\"instruction\":" << std::quoted(row.instruction)
           << ",\"bytes\":[";
    for (unsigned index = 0; index < row.bytes.size(); ++index) {
      if (index != 0) { stream << ','; }
      stream << row.bytes[index];
    }
    stream << "],\"flags\":[";
    for (unsigned index = 0; index < row.flags.size(); ++index) {
      if (index != 0) { stream << ','; }
      stream << row.flags[index];
    }
    stream << "]}\n";
  }
}

void FlashPhases::observe(const Hc12BusCycle cycle, const std::uint64_t before) {
  if (cycle.word_part == Hc12WordPart::whole) { return; }
  require(cycle.address.has_value() && cycle.size == 1, "Expanded Flash phase needs one byte");
  if (!cycle.address) { throw Fault("Missing Flash byte phase address"); }
  const unsigned address = *cycle.address;
  if (cycle.word_part == Hc12WordPart::high) {
    require(!pending && address >= low_flash && address != last_address && (address & 1U) != 0,
            "Expanded Flash high byte must begin an odd, non-wrapping mapped word");
    pending = std::pair<unsigned, unsigned>{cycle.context.pc, address};
    high_before = before;
    ++high;
  } else {
    require(pending && pending->first == cycle.context.pc && pending->second + 1 == address &&
                before == high_before + 1,
            "Expanded Flash low byte must follow its high address and clock immediately");
    if (!pending) { throw Fault("Missing Flash high-byte phase"); }
    ++words[*pending];
    pending.reset();
    ++low;
  }
}

void FlashPhases::write(const std::filesystem::path &path) const {
  std::ofstream stream(path);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream << "{\"high\":" << high << ",\"low\":" << low
         << ",\"pending\":" << (pending ? "true" : "false") << ",\"words\":[";
  bool first = true;
  for (const auto &[key, count] : words) {
    if (!first) { stream << ','; }
    first = false;
    stream << "{\"pc\":" << key.first << ",\"address\":" << key.second << ",\"count\":" << count
           << '}';
  }
  stream << "]}\n";
}

void Interrupts::observe(const CpuState before,
                         const CpuStep result,
                         const Hc12 &cpu,
                         const unsigned opcode) {
  if (result.interrupt_vector) {
    if (result.stack_reused) {
      require(foreground && result.interrupt_vector == realtime_vector &&
                  result.instruction_executed && opcode == return_interrupt,
              "Original RTI chaining must retain the existing foreground frame");
      ++entries;
      ++returns;
      ++reused;
      return;
    }
    require(!foreground && result.interrupt_vector == realtime_vector && !result.stack_reused &&
                !result.instruction_executed,
            "Original cold foreground must enter real, non-nested RTI interrupts");
    foreground = before;
    ++entries;
  }
  if (foreground && opcode == return_interrupt && result.instruction_executed) {
    require(same_cpu(*foreground, cpu.state()),
            "Every original cold-start RTI must restore the complete foreground state");
    foreground.reset();
    ++returns;
  }
}

void attach(McuMachine &machine, LcdFixture &display, FlashPhases &phases, std::ostream &trace) {
  machine.set_cycle_observer([&machine, &phases](const auto cycle) {
    phases.observe(cycle, machine.run_clock().bus_cycles());
    require_idle_receive(machine);
  });
  machine.set_wait_observer([&machine](auto /*cycle*/) { require_idle_receive(machine); });
  machine.bus().set_observer([&machine, &display, &trace](const auto &event) {
    display.observe(event);
    const bool serial_config =
        event.address >= hc12::sci_baud_high && event.address <= hc12::sci_control2;
    if (event.write && (event.address < register_trace_end || serial_config)) {
      trace << "cycle=" << machine.run_clock().bus_cycles() << " write=" << hex(event.address)
            << ':' << hex(event.value, 2) << '\n';
    }
  });
}

void detach(McuMachine &machine) {
  machine.bus().set_observer({});
  machine.set_cycle_observer({});
  machine.set_wait_observer({});
}

PrefixCheckpoint
run_prefix(McuMachine &machine, const std::filesystem::path &path, std::ostream &trace) {
  const auto initial = machine.bus().power_off_flash_image();
  machine.bus().set_pin_drive(McuPort::e, {.mask = initial_e_mask, .levels = 3});
  machine.bus().set_pin_drive(McuPort::q, {.mask = byte_mask});
  machine.bus().set_pin_drive(McuPort::r, {.mask = receive_pad, .levels = receive_pad});
  LcdFixture display(machine.bus());
  FlashPhases phases;
  Census census;
  Inputs inputs;
  Interrupts interrupts;
  attach(machine, display, phases, trace);
  std::optional<PrefixCheckpoint> frozen;
  unsigned steps = 0;
  try {
    for (; steps < root_limit; ++steps) {
      const auto before = machine.cpu().state();
      if (before.pc == input_read) {
        frozen.emplace(PrefixCheckpoint{.machine = machine.checkpoint(),
                                        .display = LcdFixture(machine.bus(), display)});
      }
      if (steps % progress_period == 0) {
        std::cout << "clocked cold start steps=" << steps
                  << " cycles=" << machine.run_clock().bus_cycles() << '\n';
        std::cout.flush();
      }
      inputs.observe(machine);
      census.before(machine);
      const auto opcode = machine.bus().peek(before.pc);
      const auto result = machine.cpu().step();
      census.retire(before, result);
      interrupts.observe(before, result, machine.cpu(), opcode);
    }
  } catch (const Fault &fault) {
    census.write(artifact(path, ".root.census.jsonl"));
    phases.write(artifact(path, ".root.phases.json"));
    trace << "root steps=" << steps << " retired=" << census.retired
          << " entries=" << interrupts.entries << " cycles=" << machine.run_clock().bus_cycles()
          << " ticks=" << machine.run_clock().elapsed_ticks()
          << " OSC=" << machine.run_clock().oscillator_cycles() << " fault=" << fault.what()
          << '\n';
    detach(machine);
    if (!frozen || steps != root_steps || census.retired != root_retired ||
        interrupts.entries != root_interrupts || interrupts.returns != root_interrupts ||
        interrupts.reused != 0 || machine.run_clock().bus_cycles() != prefix_cycles + 1 ||
        phases.low != 0 || !machine.timing_faulted() ||
        !std::string_view(fault.what()).contains("Unknown MCU GPIO input at 0x0248 mask=0x08")) {
      throw Fault("Unexpected original cold-start frontier: " + std::string(fault.what()));
    }
    check_nvm(machine, initial);
    if (!frozen) { throw Fault("Missing original pre-input checkpoint"); }
    return *frozen;
  }
  detach(machine);
  throw Fault("Original clocked reset exceeded its bounded strict-input frontier");
}
}
