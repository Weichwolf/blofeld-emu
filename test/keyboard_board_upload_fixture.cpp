// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard_board_upload_fixture.hpp"

#include "board/hcs08_board_wire.hpp"
#include "firmware/firmware.hpp"
#include "hcs08_board_wire_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard_board_transport_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace blofeld::test::keyboard_board_upload {
using hcs08::require;
namespace hp = hcs08_programmer;
namespace hf = hcs08_flash;
constexpr unsigned prepare_bound = 1500000, launch_bound = 20000, secure_option = 0xfd;
constexpr unsigned pc_command = 0x4b, sp_command = 0x4f, stack_low = 0x7c, expected_stack = 0x77;

Runner::Runner(const Firmware &firmware,
               const std::vector<std::uint8_t> &helper,
               const unsigned phase,
               const bool secured,
               std::ostream &trace,
               const Hcs08BoardWireCheckpoint *snapshot)
    : pair(firmware, helper, phase, trace, preimage(secured), snapshot), secure(secured) {
  initial_flash = pair.target.memory().flash_image();
  pair.target_clock = [this](const std::uint64_t ticks) { target_tick(ticks); };
}

Hcs08FlashImage Runner::preimage(const bool secure) {
  auto image = board_wire::blank(secure ? secure_option : 2);
  image.data[trim_address - hf::first] = expected_trim;
  image.fresh.reset(trim_address - hf::first);
  return image;
}

void Runner::target_tick(const std::uint64_t ticks) {
  if (!pair.target.bdc_state().active && !go_checked) {
    constexpr unsigned reset_flags = 0x68;
    constexpr unsigned high_known = 0xff00;
    require(
        pair.target.cpu_state() == Hcs08State{.pc = hp::algorithm,
                                              .sp = hp::ram_end - 4,
                                              .ccr_known = reset_flags,
                                              .ccr = reset_flags,
                                              .a_known = 0,
                                              .hx_known = high_known},
        "Actual original PC/SP/GO frames retain native reset A/X knowledge without a target seed");
    go_checked = true;
    pair.trace << "GO pc=" << pair.target.cpu_state().pc << " sp=" << pair.target.cpu_state().sp
               << " hx=" << pair.target.cpu_state().hx
               << " a-known=" << static_cast<unsigned>(pair.target.cpu_state().a_known)
               << " hx-known=" << pair.target.cpu_state().hx_known
               << " ccr-known=" << static_cast<unsigned>(pair.target.cpu_state().ccr_known)
               << " ccr=" << static_cast<unsigned>(pair.target.cpu_state().ccr)
               << " ticks=" << ticks << '\n';
  }
  if (detailed) {
    pair.trace << "TICK ticks=" << ticks << " offered=" << pair.target.run_state().offered_periods
               << " executed=" << pair.target.run_state().executing_periods
               << " remaining=" << pair.target.run_state().remaining_periods
               << " mode=" << pair.target.bdc_state().active << '\n';
  }
  if (pair.target.bdc_state().active) { return; }
  const auto before = pair.target.run_state().retired_instructions;
  pair.target
      .advance_reference_period(); // Declared reference tie:CPU ends before BDC on this edge.
  if (pair.target.run_state().retired_instructions != before) {
    const auto cpu = pair.target.cpu_state();
    pair.trace << "CPU n=" << pair.target.run_state().retired_instructions
               << " periods=" << pair.target.run_state().executing_periods << " pc=" << cpu.pc
               << " hx=" << cpu.hx << " sp=" << cpu.sp << " a=" << static_cast<unsigned>(cpu.a)
               << " ccr=" << static_cast<unsigned>(cpu.ccr)
               << " a-known=" << static_cast<unsigned>(cpu.a_known) << " hx-known=" << cpu.hx_known
               << " ccr-known=" << static_cast<unsigned>(cpu.ccr_known) << '\n';
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
  pair.expected = {keyboard_board_transport::read_opcode,
                   trim_address >> 8U,
                   trim_address & keyboard_board_transport::byte_mask};
  for (unsigned address = target_first; address < target_last; ++address) {
    const auto byte =
        address < hp::algorithm ? 0 : pair.helper[algorithm_offset + address - hp::algorithm];
    pair.expected.insert(pair.expected.end(),
                         {keyboard_board_transport::write_opcode,
                          address >> 8U,
                          address & keyboard_board_transport::byte_mask,
                          static_cast<unsigned>(byte)});
  }
  pair.cursor = 0;
  pair.host.cpu().set_state(
      {.pc = prepare_entry, .sp = frame, .ccr = keyboard_board_transport::initial_ccr});
  while (pair.host.cpu().state().pc != prepare_end) {
    require(pair.steps < prepare_bound,
            "Original complete controller RAM-upload loop has a finite bound");
    step();
  }
  const auto &memory = pair.target.memory();
  for (unsigned address = hp::ram_first; address < hp::ram_end; ++address) {
    const bool written = !secure && address < target_last;
    unsigned value = 0;
    if (written) {
      value =
          address < hp::algorithm ? 0U : pair.helper.at(algorithm_offset + address - hp::algorithm);
    }
    require(memory.inspect_ram(static_cast<std::uint16_t>(address)).known ==
                    (written ? keyboard_board_transport::byte_mask : 0U) &&
                memory.inspect_ram(static_cast<std::uint16_t>(address)).value == value,
            "Original upload writes exactly32 clear bytes and132 code bytes, or none when secured");
  }
  constexpr unsigned uploaded = target_last - target_first;
  constexpr unsigned commands = uploaded + 1;
  require(
      pair.host.cpu().state().sp == frame && pair.cursor == pair.expected.size() &&
          pair.target.bdc_state().serviced == commands &&
          pair.target.wire_state().transmitted_bytes == 1 &&
          pair.host.bus().peek(saved_trim) == (secure ? 0 : expected_trim) &&
          pair.host.bus().read_memory({.address = frame + 2, .size = 2}) == target_last &&
          pair.host.bus().read_memory({.address = frame + 9, .size = 2}) ==
              source_algorithm + algorithm_bytes,
      "Original source pointer, destination, saved actual trim and native frame complete together");
  pair.trace << "PREP commands=" << pair.target.bdc_state().serviced
             << " rx=" << pair.target.wire_state().received_bytes
             << " tx=" << pair.target.wire_state().transmitted_bytes
             << " known=" << (secure ? 0U : uploaded)
             << " trim=" << static_cast<unsigned>(pair.host.bus().peek(saved_trim))
             << " steps=" << pair.steps << '\n';
}

void Runner::buffer(const std::vector<std::uint8_t> &records) {
  for (unsigned n = 0; n < page_bytes; ++n) {
    const auto address = hp::buffer + n;
    pair.host.bus().write_memory(
        {.address = keyboard_board_transport::host_stack, .size = 2, .value = address});
    pair.expected = {keyboard_board_transport::write_opcode,
                     address >> 8U,
                     address & keyboard_board_transport::byte_mask,
                     records[n + 2]};
    static_cast<void>(
        pair.call({.entry = keyboard_board_transport::write_entry, .argument = records[n + 2]}));
  }
  for (unsigned n = 0; n < page_bytes; ++n) {
    require(
        pair.target.memory().inspect_ram(static_cast<std::uint16_t>(hp::buffer + n)).known ==
                (secure ? 0U : keyboard_board_transport::byte_mask) &&
            pair.target.memory().inspect_ram(static_cast<std::uint16_t>(hp::buffer + n)).value ==
                (secure ? 0 : records[n + 2]),
        "Original byte helper reaches every page-buffer byte through actual GPIO or secure "
        "blocking");
  }
  pair.trace << "BUFFER commands=" << pair.target.bdc_state().serviced
             << " rx=" << pair.target.wire_state().received_bytes
             << " tx=" << pair.target.wire_state().transmitted_bytes << " steps=" << pair.steps
             << '\n';
}

void Runner::start(const std::vector<std::uint8_t> &records) {
  pair.host.bus().write_memory({.address = frame + 11,
                                .size = 2,
                                .value = (static_cast<unsigned>(records[0]) << 8U) | records[1]});
  pair.expected = {keyboard_board_transport::write_opcode,
                   0,
                   target_first,
                   records[0],
                   keyboard_board_transport::write_opcode,
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
      {.pc = start_entry, .sp = frame, .ccr = keyboard_board_transport::initial_ccr});
}

std::string Runner::endpoint() const {
  const auto cpu = pair.target.cpu_state();
  const auto host = pair.host.cpu().state();
  std::ostringstream out;
  out << cpu.pc << ' ' << cpu.hx << ' ' << cpu.sp << ' ' << static_cast<unsigned>(cpu.a) << ' '
      << static_cast<unsigned>(cpu.ccr) << ' ' << host.pc << ' ' << host.d << ' ' << host.x << ' '
      << host.y << ' ' << host.sp << ' ' << static_cast<unsigned>(host.ccr) << ' '
      << pair.host.run_clock().bus_cycles() << ' ' << pair.host.run_clock().elapsed_ticks() << ' '
      << pair.target.run_state().offered_periods << ' ' << pair.target.run_state().executing_periods
      << ' ' << pair.target.run_state().retired_instructions << ' '
      << pair.target.wire_state().time_ticks << ' ' << pair.target.wire_state().cycles << ' '
      << pair.target.bdc_state().serviced << ' ' << pair.steps;
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
  require(go_checked && pair.host.timing_faulted() && pair.target.faulted() &&
              reason.contains(secure ? "fetch address=0x00a0" : "address=0x1825") &&
              pair.target.run_state().executing_periods == (secure ? 0U : strict_periods) &&
              pair.target.run_state().retired_instructions == (secure ? 0U : strict_retired) &&
              !pair.target.memory().flash_state().active && pair.cursor == pair.expected.size(),
          "Actual native GO proceeds independently of host polls to the strict CPU/FLASH frontier");
  if (!secure) {
    require(pair.target.faulted() && pair.target.cpu_state().pc == strict_site &&
                pair.target.cpu_state().sp == expected_stack,
            "Unchanged original H/TXS and strict FSTAT failure quarantine every coupled execution "
            "owner");
  }
  pair.trace << "FAULT secure=" << secure << " target-pc=" << pair.target.cpu_state().pc
             << " retired=" << pair.target.run_state().retired_instructions
             << " executing=" << pair.target.run_state().executing_periods
             << " commands=" << pair.target.bdc_state().serviced
             << " rx=" << pair.target.wire_state().received_bytes
             << " tx=" << pair.target.wire_state().transmitted_bytes << " steps=" << pair.steps
             << '\n';
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
    require(pair.host.bus().peek(static_cast<std::uint16_t>(keyboard_board_transport::helper_base +
                                                            n)) == pair.helper[n],
            "All original HC12 helper and embedded HCS08 algorithm bytes remain unchanged");
  }
  require(pair.target.memory().flash_image() == initial_flash,
          "No target erase/program or unknown NVM retention has been fabricated");
  const auto flash = pair.target.memory().flash_state();
  constexpr unsigned divider = 0x1e;
  require(flash.divider_loaded == !secure && flash.divider == (secure ? 0U : divider) &&
              !flash.active && flash.errors == 0,
          "Only the actual native prefix loads FCDIV; no command or error result is synthesized");
  for (unsigned address = hp::ram_first; address < hp::ram_end; ++address) {
    const bool known = !secure && ((address >= target_first && address < target_last) ||
                                   (address >= hp::buffer && address < hp::buffer + page_bytes));
    require(pair.target.memory().inspect_ram(static_cast<std::uint16_t>(address)).known ==
                (known ? keyboard_board_transport::byte_mask : 0U),
            "Production target retains every independently expected SRAM known mask");
  }
}

Saved Runner::checkpoint() const {
  return {.host = pair.host.checkpoint(),
          .target = pair.target.checkpoint(),
          .expected = pair.expected,
          .cursor = pair.cursor,
          .steps = pair.steps,
          .received = pair.received,
          .pin_writes = pair.pin_writes,
          .go_checked = go_checked};
}

void Runner::restore(const Saved &saved) {
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
} // namespace blofeld::test::keyboard_board_upload
