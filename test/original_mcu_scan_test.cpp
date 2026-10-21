// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>

namespace {
constexpr unsigned byte_mask = 0xff;
constexpr unsigned caller = 0x1700;
constexpr unsigned returned = caller + 3;
constexpr unsigned stack = 0x1ff0;
constexpr unsigned ram_begin = 0x1200;
constexpr unsigned ram_end = 0x2000;
constexpr unsigned extended_delay = 0xb9ba;
constexpr unsigned byte_delay = 0xb9c2;
constexpr unsigned scan_outputs = 0x8a42;
constexpr unsigned inner_return = 0xb9bd;
constexpr unsigned jsr = 0x16;
constexpr unsigned a_input = 0xa500;
constexpr unsigned x_input = 0x369c;
constexpr unsigned y_input = 0xc963;
constexpr unsigned upper_ccr = 0xf0;
constexpr unsigned negative = 8;
constexpr unsigned carry = 1;
constexpr unsigned instruction_limit = 520;
constexpr unsigned expected_cases = 2788;
constexpr unsigned extended_instructions = 261;
constexpr unsigned extended_cycles = 787;
constexpr unsigned scan_cycles = 19;
constexpr std::array<unsigned, 3> ports{0x250, 0x270, 0x240};
constexpr std::array<unsigned, 3> levels{1, 0, 1};
constexpr std::array<unsigned, 7> boundary_counts{0, 1, 2, 127, 128, 254, 255};
constexpr std::array<unsigned, 4> flag_patterns{0x50, 0x75, 0xda, 0xff};

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw blofeld::Fault(std::string(message)); }
}

bool directed(const unsigned count, const unsigned flags) {
  return std::ranges::contains(boundary_counts, count) ||
         std::ranges::contains(flag_patterns, flags);
}

unsigned ram_value(const unsigned at) {
  return ((at * 13U) ^ (at >> 3U)) & byte_mask;
}

struct Case {
  unsigned entry = 0;
  unsigned input = 0;
  unsigned flags = 0;
};

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path trace;
};

class Fixture {
public:
  Fixture(const blofeld::Firmware &firmware, std::ostream &trace)
      : board_(firmware.mcu),
        timed_(board_, [this](const auto cycle) { observe_cycle(cycle); }),
        cpu_(timed_, BLOFELD_DEFAULT_SLA),
        trace_(trace) {
    // Diagnostic RAM and inactive IRQ/XIRQ inputs, not recovered startup state.
    for (unsigned at = ram_begin; at < ram_end; ++at) {
      board_.write(static_cast<std::uint16_t>(at), static_cast<std::uint8_t>(ram_value(at)));
    }
    board_.set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
    require(board_.observe_pads(blofeld::McuPort::j, 7).known == 0 &&
                board_.observe_pads(blofeld::McuPort::s, byte_mask).known == 0 &&
                board_.observe_pads(blofeld::McuPort::t, 15).known == 0,
            "Reset scan ports retain unknown external inputs; no pad observations are supplied");
  }

  void call(const Case &request);

  void unchanged(const blofeld::Firmware &firmware) const {
    const auto image = board_.power_off_flash_image();
    require(std::ranges::equal(image.bytes, firmware.mcu) && image.known.all() &&
                image.fresh.none(),
            "Complete original MCU flash and erase history must remain unchanged");
  }

private:
  void observe_cycle(blofeld::Hc12BusCycle cycle);
  void prepare(const Case &request);
  void guards() const;
  blofeld::McuBus board_;
  blofeld::Hc12CycleBus timed_;
  blofeld::Hc12 cpu_;
  std::ostream &trace_;
  unsigned writes_ = 0;
  unsigned stack_reads_ = 0;
  unsigned stack_writes_ = 0;
  unsigned port_cycles_ = 0;
  bool nested_ = false;
};

void Fixture::observe_cycle(const blofeld::Hc12BusCycle cycle) {
  if (!cycle.address) { return; }
  const auto at = *cycle.address;
  if (at >= stack - 4 && at <= stack - 2 && cycle.size == 2) {
    require((at & 1U) == 0, "Declared internal stack accesses are aligned words");
    if (cycle.kind == 'S') {
      require(stack_writes_ < 2 && at == stack - (2 * (stack_writes_ + 1)),
              "Native JSR pushes its frame at the independently expected stack address");
      ++stack_writes_;
    } else {
      require(cycle.kind == 'U', "Only native call/return stack operands are admitted");
      const unsigned frames = nested_ ? 2 : 1;
      require(stack_reads_ < frames && at == stack - (2 * (frames - stack_reads_)),
              "Native RTS consumes its genuine inner or outer frame in order");
      ++stack_reads_;
    }
    return;
  }
  require(std::ranges::contains(ports, at) && cycle.size == 1 && cycle.kind == 'w',
          "Original helpers may only write output latches; no input-pad read is allowed");
  ++port_cycles_;
}

void Fixture::prepare(const Case &request) {
  const auto [entry, input, flags] = request;
  board_.write(caller, jsr);
  board_.write(caller + 1, static_cast<std::uint8_t>(entry >> 8U));
  board_.write(caller + 2, static_cast<std::uint8_t>(entry & byte_mask));
  board_.set_observer({});
  for (const auto at : ports) {
    board_.write(static_cast<std::uint16_t>(at), static_cast<std::uint8_t>(input));
  }
  board_.set_observer([this](const auto &event) {
    require(writes_ < ports.size() && event.write && event.address == ports.at(writes_) &&
                event.value == levels.at(writes_),
            "Original scan output writes must be PTJ=1, PTS=0, PTT=1 in order");
    ++writes_;
  });
  writes_ = stack_reads_ = stack_writes_ = port_cycles_ = 0;
  nested_ = entry == extended_delay;
  cpu_.set_state({.pc = caller,
                  .d = static_cast<std::uint16_t>(a_input | input),
                  .x = x_input,
                  .y = y_input,
                  .sp = stack,
                  .ccr = static_cast<std::uint8_t>(flags)});
}

void Fixture::guards() const {
  for (unsigned at = ram_begin; at < ram_end; ++at) {
    if ((at >= caller && at < returned) || (at >= stack - 4 && at < stack)) { continue; }
    require(board_.peek(static_cast<std::uint16_t>(at)) == ram_value(at),
            "Original helpers must preserve all declared RAM outside real call frames");
  }
  require(board_.peek(stack - 2) == returned >> 8U &&
              board_.peek(stack - 1) == (returned & byte_mask),
          "Outer native JSR leaves its actual big-endian return address");
}

void Fixture::call(const Case &request) {
  const auto [entry, input, flags] = request;
  prepare(request);
  const auto start = timed_.cycles();
  unsigned instructions = 0;
  while (cpu_.state().pc != returned) {
    require(instructions < instruction_limit, "Original helper must return within its bound");
    const auto result = cpu_.step();
    require(result.instruction_executed && !result.interrupt_vector,
            "Declared inactive interrupt inputs permit only original call execution");
    ++instructions;
  }
  const auto state = cpu_.state();
  const unsigned turns = input == 0 ? byte_mask + 1 : input;
  const bool extended = entry == extended_delay;
  const bool scan = entry == scan_outputs;
  unsigned expected_flags = flags;
  if (scan) { expected_flags &= upper_ccr; }
  if (extended) { expected_flags = (flags & upper_ccr) | negative | (flags & carry); }
  const unsigned expected_instructions = scan ? 6 : turns + (extended ? extended_instructions : 2U);
  // S12CPUV2: DBNE=3, RTS=5, JSR=4, LDAB #imm=1, JMP ext=3.
  const unsigned expected_cycle_count =
      scan ? scan_cycles : (3 * turns) + (extended ? extended_cycles : 9U);
  require(
      state.d == (a_input | static_cast<unsigned>(scan)) && state.x == x_input &&
          state.y == y_input && state.sp == stack && state.ccr == expected_flags &&
          instructions == expected_instructions && timed_.cycles() - start == expected_cycle_count,
      "Original helper state/instruction/core-cycle result differs entry=" + blofeld::hex(entry) +
          " input=" + blofeld::hex(input) + " flags=" + blofeld::hex(flags) + " actual-ccr=" +
          blofeld::hex(state.ccr) + " cycles=" + std::to_string(timed_.cycles() - start));
  require(stack_reads_ == (extended ? 2U : 1U) && stack_writes_ == stack_reads_ &&
              writes_ == (scan ? 3U : 0U) && port_cycles_ == writes_,
          "Only genuine nested frames and declared port writes are permitted");
  if (extended) {
    require(board_.peek(stack - 4) == inner_return >> 8U &&
                board_.peek(stack - 3) == (inner_return & byte_mask),
            "Extended delay executes its original inner JSR/RTS before the tail jump");
  }
  guards();
  trace_ << "CASE " << std::hex << entry << ' ' << input << ' ' << flags << ' ' << state.pc << ' '
         << state.d << ' ' << state.x << ' ' << state.y << ' ' << state.sp << ' '
         << static_cast<unsigned>(state.ccr) << std::dec << ' ' << instructions << ' '
         << timed_.cycles() - start << ' ' << writes_ << '\n';
}

void run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  std::ofstream trace(inputs.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  Fixture fixture(firmware, trace);
  for (const auto entry : {byte_delay, extended_delay, scan_outputs}) {
    unsigned cases = 0;
    for (unsigned input = 0; input <= byte_mask; ++input) {
      for (unsigned flags = 0; flags <= byte_mask; ++flags) {
        if (!directed(input, flags)) { continue; }
        fixture.call({.entry = entry, .input = input, .flags = flags});
        ++cases;
      }
    }
    require(cases == expected_cases, "Complete deterministic counter/flag matrix is required");
  }
  fixture.unchanged(firmware);
  std::cout << "Verified 8364 original MCU delay/scan calls, independent CPU/core-cycle "
               "expectations, real JSR/RTS and RAM/NVM guards. No boot, physical delay, "
               "scan-input or interrupt-scheduling qualification.\n";
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) { throw blofeld::Fault("Usage: original-mcu-scan firmware.zip private.trace"); }
    run({.firmware = arguments[1], .trace = arguments[2]});
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
