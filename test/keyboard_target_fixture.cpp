// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard_target_fixture.hpp"

#include "blofeld/firmware.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_bdc.hpp"
#include "blofeld/hcs08_bdc_wire.hpp"
#include "blofeld/hcs08_flash.hpp"
#include "blofeld/hcs08_run_clock.hpp"
#include "blofeld/mcu_machine.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_wire_fixture.hpp"
#include "keyboard_bdc_wire_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace blofeld::test::keyboard_target {
using hcs08::require;
namespace hp = hcs08_programmer;
namespace hf = hcs08_flash;
constexpr auto timing = Hcs08RunContract::reference_instruction_retirement;
constexpr unsigned prepare_bound = 1500000, launch_bound = 20000, secure_option = 0xfd;
constexpr unsigned pc_command = 0x4b, sp_command = 0x4f, stack_low = 0x7c, expected_stack = 0x1277;

Runner::Runner(const Firmware &firmware,
               const std::vector<std::uint8_t> &helper,
               const unsigned phase,
               const bool secured,
               std::ostream &trace,
               const hcs08_programmer::Memory *snapshot)
    : pair(firmware, helper, phase, trace, snapshot),
      clock(pair.target.memory, pair.bdc, timing),
      secure(secured) {
  auto &memory = pair.target.memory;
  if (snapshot == nullptr) {
    for (unsigned address = hp::ram_first; address < hp::ram_end; ++address) {
      memory.known.reset(address);
      memory.bytes[address] = canary;
    }
    ++memory.generation;
    ++memory.code_generation;
    memory.accesses.clear();
  }
  if (secure && snapshot == nullptr) {
    auto image = memory.flash.image();
    image.data[hf::nv_options - hf::first] = secure_option;
    memory.flash = Hcs08Flash(image, hcs08_wire::chip);
  }
  initial_flash = memory.flash.image();
  pair.target.cpu.set_state(
      {.pc = initial_pc, .hx = initial_hx, .sp = initial_sp, .a = initial_a, .ccr = initial_ccr});
  pair.target_clock = [this](const std::uint64_t ticks) { target_tick(ticks); };
}

void Runner::target_tick(const std::uint64_t ticks) {
  if (!pair.bdc.state().active && !go_checked) {
    require(pair.bdc.cpu_state() == Hcs08State{.pc = hp::algorithm,
                                               .hx = initial_hx,
                                               .sp = hp::ram_end - 4,
                                               .a = initial_a,
                                               .ccr = initial_ccr},
            "Actual native HC12 PC/SP/GO frames replace distinct initial registers and preserve "
            "the rest");
    go_checked = true;
    pair.trace << "GO pc=" << pair.target.cpu.state().pc << " sp=" << pair.target.cpu.state().sp
               << " hx=" << pair.target.cpu.state().hx << " ticks=" << ticks << '\n';
  }
  if (detailed) {
    pair.trace << "TICK ticks=" << ticks << " offered=" << clock.state().offered_periods
               << " executed=" << clock.state().executing_periods
               << " remaining=" << clock.state().remaining_periods
               << " mode=" << pair.bdc.state().active << '\n';
  }
  const auto before = clock.state().retired_instructions;
  clock.advance(); // Explicit reference tie order: CPU period ends before BDC service on this edge.
  if (clock.state().retired_instructions != before) {
    const auto cpu = pair.target.cpu.state();
    pair.trace << "CPU n=" << clock.state().retired_instructions
               << " periods=" << clock.state().executing_periods << " pc=" << cpu.pc
               << " hx=" << cpu.hx << " sp=" << cpu.sp << " a=" << static_cast<unsigned>(cpu.a)
               << " ccr=" << static_cast<unsigned>(cpu.ccr) << '\n';
  }
}

void Runner::step() {
  if (detailed) {
    const auto cpu = pair.host.cpu().state();
    pair.trace << "HOST pc=" << cpu.pc << " d=" << cpu.d << " x=" << cpu.x << " y=" << cpu.y
               << " sp=" << cpu.sp << " ccr=" << static_cast<unsigned>(cpu.ccr)
               << " cycles=" << pair.host.run_clock().bus_cycles() << '\n';
  }
  pair.step();
}

void Runner::prepare() {
  pair.expected = {keyboard_bdc_wire::read_opcode,
                   trim_address >> 8U,
                   trim_address & keyboard_bdc_wire::byte_mask};
  for (unsigned address = target_first; address < target_last; ++address) {
    const auto byte =
        address < hp::algorithm ? 0 : pair.helper[algorithm_offset + address - hp::algorithm];
    pair.expected.insert(pair.expected.end(),
                         {keyboard_bdc_wire::write_opcode,
                          address >> 8U,
                          address & keyboard_bdc_wire::byte_mask,
                          static_cast<unsigned>(byte)});
  }
  pair.cursor = 0;
  pair.host.cpu().set_state(
      {.pc = prepare_entry, .sp = frame, .ccr = keyboard_bdc_wire::initial_ccr});
  while (pair.host.cpu().state().pc != prepare_end) {
    require(pair.steps < prepare_bound,
            "Original complete controller RAM-upload loop has a finite bound");
    step();
  }
  const auto &memory = pair.target.memory;
  for (unsigned address = hp::ram_first; address < hp::ram_end; ++address) {
    const bool written = !secure && address < target_last;
    unsigned value = canary;
    if (written) {
      value =
          address < hp::algorithm ? 0U : pair.helper.at(algorithm_offset + address - hp::algorithm);
    }
    require(memory.known[address] == written && memory.bytes[address] == value,
            "Original upload writes exactly32 clear bytes and132 code bytes, or none when secured");
  }
  constexpr unsigned uploaded = target_last - target_first;
  constexpr unsigned commands = uploaded + 1;
  require(
      pair.host.cpu().state().sp == frame && pair.cursor == pair.expected.size() &&
          pair.bdc.state().serviced == commands && pair.wire.state().transmitted_bytes == 1 &&
          pair.host.bus().peek(saved_trim) == (secure ? 0 : expected_trim) &&
          pair.host.bus().read_memory({.address = frame + 2, .size = 2}) == target_last &&
          pair.host.bus().read_memory({.address = frame + 9, .size = 2}) ==
              source_algorithm + algorithm_bytes,
      "Original source pointer, destination, saved actual trim and native frame complete together");
  pair.trace << "PREP commands=" << pair.bdc.state().serviced
             << " rx=" << pair.wire.state().received_bytes
             << " tx=" << pair.wire.state().transmitted_bytes << " known=" << memory.known.count()
             << " trim=" << static_cast<unsigned>(pair.host.bus().peek(saved_trim))
             << " steps=" << pair.steps << '\n';
}

void Runner::buffer(const std::vector<std::uint8_t> &records) {
  for (unsigned n = 0; n < page_bytes; ++n) {
    const auto address = hp::buffer + n;
    pair.host.bus().write_memory(
        {.address = keyboard_bdc_wire::host_stack, .size = 2, .value = address});
    pair.expected = {keyboard_bdc_wire::write_opcode,
                     address >> 8U,
                     address & keyboard_bdc_wire::byte_mask,
                     records[n + 2]};
    static_cast<void>(
        pair.call({.entry = keyboard_bdc_wire::write_entry, .argument = records[n + 2]}));
  }
  for (unsigned n = 0; n < page_bytes; ++n) {
    require(pair.target.memory.known[hp::buffer + n] == !secure &&
                pair.target.memory.bytes[hp::buffer + n] == (secure ? canary : records[n + 2]),
            "Original byte helper reaches every page-buffer byte through actual GPIO or secure "
            "blocking");
  }
  pair.trace << "BUFFER commands=" << pair.bdc.state().serviced
             << " rx=" << pair.wire.state().received_bytes
             << " tx=" << pair.wire.state().transmitted_bytes << " steps=" << pair.steps << '\n';
}

void Runner::start(const std::vector<std::uint8_t> &records) {
  pair.host.bus().write_memory({.address = frame + 11,
                                .size = 2,
                                .value = (static_cast<unsigned>(records[0]) << 8U) | records[1]});
  pair.expected = {keyboard_bdc_wire::write_opcode,
                   0,
                   target_first,
                   records[0],
                   keyboard_bdc_wire::write_opcode,
                   0,
                   target_first + 1,
                   0,
                   pc_command,
                   0,
                   hp::algorithm,
                   sp_command,
                   4,
                   stack_low,
                   8};
  pair.cursor = 0;
  pair.host.cpu().set_state(
      {.pc = start_entry, .sp = frame, .ccr = keyboard_bdc_wire::initial_ccr});
}

std::string Runner::endpoint() const {
  const auto cpu = pair.target.cpu.state();
  const auto host = pair.host.cpu().state();
  std::ostringstream out;
  out << cpu.pc << ' ' << cpu.hx << ' ' << cpu.sp << ' ' << static_cast<unsigned>(cpu.a) << ' '
      << static_cast<unsigned>(cpu.ccr) << ' ' << host.pc << ' ' << host.d << ' ' << host.x << ' '
      << host.y << ' ' << host.sp << ' ' << static_cast<unsigned>(host.ccr) << ' '
      << pair.host.run_clock().bus_cycles() << ' ' << pair.host.run_clock().elapsed_ticks() << ' '
      << clock.state().offered_periods << ' ' << clock.state().executing_periods << ' '
      << clock.state().retired_instructions << ' ' << pair.wire.state().time_ticks << ' '
      << pair.wire.state().cycles << ' ' << pair.bdc.state().serviced << ' ' << pair.steps;
  return out.str();
}

void Runner::until_fault() {
  const auto begin = pair.steps;
  const auto reason = hcs08::rejected([&] {
    while (true) {
      require(pair.steps - begin < launch_bound,
              "Original start/poll prefix retains its step bound");
      step();
    }
  });
  require(go_checked && pair.host.timing_faulted() && clock.faulted() &&
              reason.contains(secure ? "fetch address=0x00a0" : "address=0x1825") &&
              clock.state().executing_periods == (secure ? 0U : strict_periods) &&
              clock.state().retired_instructions == (secure ? 0U : strict_retired) &&
              pair.target.memory.transfers == 0 && pair.cursor == pair.expected.size(),
          "Actual native GO proceeds independently of host polls to the strict CPU/FLASH frontier");
  if (!secure) {
    require(pair.target.cpu.faulted() && pair.bdc.faulted() && pair.wire.faulted() &&
                pair.target.cpu.state().pc == strict_site &&
                pair.target.cpu.state().sp == expected_stack,
            "Unchanged original H/TXS and strict FSTAT failure quarantine every coupled execution "
            "owner");
  }
  pair.trace << "FAULT secure=" << secure << " target-pc=" << pair.target.cpu.state().pc
             << " retired=" << clock.state().retired_instructions
             << " executing=" << clock.state().executing_periods
             << " commands=" << pair.bdc.state().serviced
             << " rx=" << pair.wire.state().received_bytes
             << " tx=" << pair.wire.state().transmitted_bytes << " steps=" << pair.steps << '\n';
  const auto before = endpoint();
  require(!hcs08::rejected([&] { pair.host.cpu().step(); }).empty() && endpoint() == before,
          "Failed native host cannot advance either target owner or source state on retry");
  guards();
}

void Runner::guards() const {
  const auto host_image = pair.host.bus().power_off_flash_image();
  require(std::ranges::equal(host_image.bytes, pair.firmware.mcu) && host_image.known.all() &&
              host_image.fresh.none(),
          "All original MCU NVM bytes/knowledge/history remain unchanged");
  for (unsigned n = 0; n < pair.helper.size(); ++n) {
    require(pair.host.bus().peek(static_cast<std::uint16_t>(keyboard_bdc_wire::helper_base + n)) ==
                pair.helper[n],
            "All original HC12 helper and embedded HCS08 algorithm bytes remain unchanged");
  }
  require(pair.target.memory.flash.image() == initial_flash,
          "No target erase/program or unknown NVM retention has been fabricated");
  const auto flash = pair.target.memory.flash.state();
  constexpr unsigned divider = 0x1e;
  require(flash.divider_loaded == !secure && flash.divider == (secure ? 0U : divider) &&
              !flash.active && flash.errors == 0,
          "Only the actual native prefix loads FCDIV; no command or error result is synthesized");
  const auto &accesses = pair.target.memory.accesses;
  if (secure) {
    require(accesses.empty(),
            "Secured BDC RAM writes retain unknown RAM without successful accesses");
    return;
  }
  constexpr unsigned uploaded = target_last - target_first;
  constexpr unsigned writes = uploaded + page_bytes;
  require(accesses.size() == writes + 5 && pair.target.cpu.state().sp == expected_stack,
          "Exact native/BDC write footprint includes two page-pointer writes and three accepted "
          "registers");
  for (unsigned n = 0; n < writes; ++n) {
    const auto address = n < uploaded ? target_first + n : hp::buffer + n - uploaded;
    require(accesses[n].write && accesses[n].address == address,
            "Every actual BDC RAM write retains its independent order/address");
  }
  constexpr std::array<unsigned, 5> addresses{0x80, 0x81, 0x1820, 0x1821, 0x1824};
  constexpr std::array<unsigned, 5> values{0xe0, 0, 0x1e, 2, 255};
  for (unsigned n = 0; n < addresses.size(); ++n) {
    const auto access = accesses[writes + n];
    require(
        access.write && access.address == addresses[n] && access.value == values[n],
        "Original pointer/CPU register stores are observed rather than supplied as ready results");
  }
}

Saved Runner::checkpoint() const {
  return {.host = pair.host.checkpoint(),
          .memory = pair.target.memory,
          .cpu = pair.target.cpu.checkpoint(),
          .bdc = pair.bdc.checkpoint(),
          .wire = pair.wire.checkpoint(),
          .clock = clock.checkpoint(),
          .expected = pair.expected,
          .cursor = pair.cursor,
          .steps = pair.steps,
          .received = pair.received,
          .pin_writes = pair.pin_writes,
          .go_checked = go_checked};
}

void Runner::restore(const Saved &saved) {
  require(pair.target.memory.bytes == saved.memory.bytes &&
              pair.target.memory.known == saved.memory.known &&
              pair.target.memory.flash.image() == saved.memory.flash.image(),
          "Fresh target owner must be constructed from the complete saved memory/peripheral state");
  pair.target.cpu.restore_checkpoint(saved.cpu);
  pair.bdc.restore_checkpoint(saved.bdc);
  pair.wire.restore_checkpoint(saved.wire);
  clock.restore_checkpoint(saved.clock);
  pair.host.restore(saved.host);
  pair.expected = saved.expected;
  pair.cursor = saved.cursor;
  pair.steps = saved.steps;
  pair.received = saved.received;
  pair.pin_writes = saved.pin_writes;
  go_checked = saved.go_checked;
  detailed = true;
  pair.bind();
}
} // namespace blofeld::test::keyboard_target
