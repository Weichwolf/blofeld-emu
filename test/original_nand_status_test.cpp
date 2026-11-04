// SPDX-License-Identifier: GPL-3.0-only
#include "board/nand_bus_decode.hpp"
#include "board/nand_peer.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "firmware/firmware.hpp"
#include "nand_native_run.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_image.hpp"
#include "storage/nand_timing.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned memory_mode = 0x600389, movec_omr = 0x05f43a;
constexpr unsigned extal = 12000000, active_pointer = 0x2a16, buffer = 0x2800;
constexpr unsigned boot = 0xff0000, entry = 0x1a29, read_edge = 0x1a39;
constexpr unsigned read_data = 0x1a3e, record_end = 0x1a44, last_reply = 0x2a19;
constexpr unsigned bus_control = 0x08f4bb, port_c = 0x07f43d, port_f = 0x09f4b5;
constexpr unsigned direction_c = 0x07f43e, function_c = 0x07f43f;
constexpr unsigned bus_value = 0x01ffff, c_outputs = 0xf00, c_functions = 0xff;
constexpr unsigned f_outputs = 0x4ff, f_functions = 0x300;
constexpr unsigned direction_f = 0x09f4b6, function_f = 0x09f4b7;
constexpr unsigned jump_absolute = 0x0af080, maximum_periods = 1000000;
constexpr unsigned inactive = 0x300, command_controls = 0x700, address_controls = 0xb00;
constexpr unsigned we_bit = 0x200, byte_mask = 0xff, program_data = 0xa4;
constexpr unsigned program_command = 0x80, program_confirm = 0x10;
constexpr unsigned erase_command = 0x60, erase_confirm = 0xd0, reset_command = 0xff;
constexpr unsigned program_wait = 7000, erase_wait = 40000, reset_wait = 256;
constexpr unsigned ready_wp = 0xc0, fail_bit = 1, status_unknown = 0xfe;
constexpr std::uint64_t timebase = 1080000000;

struct Case {
  const char *name;
  bool erase = false;
  bool inhibited = false;
  bool reset = false;
  bool unknown = false;
  bool ordered = false;
  bool candidate_timing = false;
};

constexpr std::array cases{
    Case{.name = "program-pass"},
    Case{.name = "program-fail", .inhibited = true},
    Case{.name = "erase-pass", .erase = true},
    Case{.name = "erase-fail", .erase = true, .inhibited = true},
    Case{.name = "reset", .reset = true},
    Case{.name = "undefined-fail-refuses", .unknown = true},
    Case{.name = "ordered-CLE-refuses", .reset = true, .ordered = true},
    Case{.name = "candidate-CLE-refuses", .reset = true, .candidate_timing = true}};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

Nand device(const Case &test) {
  NandImage image;
  NandArray::Page page;
  page.cells.fill(
      {.value = static_cast<std::uint8_t>(test.erase ? 0U : byte_mask), .known = byte_mask});
  page.programs = test.erase ? NandProgramCount{.main = 1, .spare = 1} : NandProgramCount{};
  for (unsigned row = 0; row < Nand::block_pages; ++row) { image.pages.emplace(row, page); }
  image.blocks.emplace(
      0, NandBlock{.valid = true, .program_protected = false, .erase_protected = false});
  Nand result(image, NandReadContinuation::k9f1208_j);
  result.observe_wp(true);
  return result;
}

NandCellProfile physics(const Case &test) {
  NandCellProfile result{.contract = NandCellContract::reference_pulse_inhibit, .cells = {}};
  if (test.inhibited) {
    result.cells.emplace(
        0,
        NandCellInhibit{.program = static_cast<std::uint8_t>(test.erase ? 0U : fail_bit),
                        .erase = static_cast<std::uint8_t>(test.erase ? fail_bit : 0U)});
  }
  return result;
}

void emit(std::vector<std::uint32_t> &words, const unsigned opcode, const unsigned value) {
  words.push_back(opcode);
  words.push_back(value);
}

void latch(std::vector<std::uint32_t> &words, const unsigned controls, const unsigned value) {
  emit(words, port_c, controls);
  emit(words, port_f, value);
  emit(words, port_c, controls & ~we_bit);
  words.push_back(0);
  emit(words, port_c, controls);
}

void operation(std::vector<std::uint32_t> &words, const Case &test) {
  if (test.unknown) { return; }
  if (test.reset) {
    latch(words, command_controls, reset_command);
  } else {
    latch(words, command_controls, test.erase ? erase_command : program_command);
    const auto cycles = test.erase ? 3U : 4U;
    for (unsigned index = 0; index < cycles; ++index) { latch(words, address_controls, 0); }
    if (!test.erase) { latch(words, inactive, program_data); }
    latch(words, command_controls, test.erase ? erase_confirm : program_confirm);
  }
  emit(words, port_c, inactive);
  auto wait = program_wait;
  if (test.reset) {
    wait = reset_wait;
  } else if (test.erase) {
    wait = erase_wait;
  }
  words.insert(words.end(), wait, 0);
}

void install(OwnDspMachine &machine, const Firmware &firmware, const Case &test) {
  machine.bus().observe_omr(memory_mode);
  machine.bus().advance_instruction_cycles(3);
  std::vector<std::uint32_t> words{movec_omr, memory_mode, 0, 0, 0};
  emit(words, bus_control, bus_value);
  emit(words, direction_c, c_outputs);
  emit(words, function_c, c_functions);
  emit(words, port_c, inactive);
  emit(words, function_f, f_functions);
  emit(words, direction_f, f_outputs);
  operation(words, test);
  emit(words, jump_absolute, entry);
  machine.bus().install_rom({.start = boot, .words = words});
  unsigned count = 0;
  for (const auto &record : firmware.dsp) {
    if (record.space != 0) { continue; }
    for (unsigned index = 0; index < record.words.size(); ++index) {
      const auto address = record.address + index;
      if (address >= entry && address < record_end) {
        machine.bus().write(
            {.space = DspSpace::p, .address = address, .value = record.words[index]});
        ++count;
      }
    }
  }
  require(count == record_end - entry, "Original status slice is complete");
  machine.bus().write({.space = DspSpace::x, .address = active_pointer, .value = buffer});
}

void check_refusal(const Case &test,
                   OwnDspMachine &machine,
                   NandPeer &peer,
                   const std::string &text) {
  if (test.unknown && text.contains("pc=" + hex(read_data, 6))) {
    require(peer.detailed().output.data.known == ready_wp,
            "The protocol/timing keep only defined Ready/WP status knowledge");
    require(machine.bus().gpio(DspPort::f).observe_pads(byte_mask).known == status_unknown,
            "The reference pad choice leaves fail0 unknown");
  } else {
    require((test.ordered || test.candidate_timing) && text.contains("pc=" + hex(read_edge, 6)) &&
                text.contains("WE high and ALE/CLE low"),
            "Unexpected original status refusal: " + text);
  }
  std::cout << "PASS " << test.name << " visible refusal=" << text << '\n';
}

void check_endpoint(const Case &test, OwnDspMachine &machine, NandPeer &peer) {
  require(!test.unknown && !test.ordered && !test.candidate_timing,
          "Negative status case must refuse");
  const auto protocol = peer.protocol_copy();
  const auto reply = machine.bus().read({.space = DspSpace::x, .address = last_reply});
  const auto inhibited = test.inhibited ? fail_bit : 0U;
  const auto expected = ready_wp | inhibited;
  require(reply == expected && protocol.ready() == true && protocol.pending() == NandPending::none,
          "Original full-byte status stores the independently expected actual operation result");
  const auto expected_known = test.reset ? byte_mask : ready_wp | fail_bit;
  require(protocol.status().known == expected_known,
          "The underlying NAND's status knowledge remains unchanged");
  if (!test.reset) {
    const auto expected_cell = test.erase ? byte_mask - inhibited : program_data | inhibited;
    require(protocol.cell({.page = 0, .column = 0}) ==
                NandByte{.value = static_cast<std::uint8_t>(expected_cell), .known = byte_mask},
            "Status fail bit agrees with independently computed physical cell contents");
  }
}

void run(const Firmware &firmware, const Case &test) {
  OwnDspMachine machine({.extal_hz = extal,
                         .timebase_hz = timebase,
                         .switch_timing = DspClockSwitch::reference_cycle_boundary});
  for (const auto port : {DspPort::c, DspPort::f}) {
    machine.bus().gpio(port).set_contract(DspGpioContract::logical_pins);
  }
  install(machine, firmware, test);
  NandPeer peer(
      machine,
      device(test),
      physics(test),
      test.ordered ? NandBusContract::ordered_pins : NandBusContract::functional_transactions,
      test.candidate_timing ? NandTiming::candidate_minima : NandTiming::functional_protocol);
  test::NandNativeRun schedule(
      machine,
      peer,
      {.identity = std::make_shared<const PeripheralEventIdentity>(),
       .next_ticks = []() -> std::optional<std::uint64_t> { return std::nullopt; },
       .observe = [](std::uint64_t) { throw Fault("Unexpected external event"); }});
  std::optional<NandPeerSnapshot> nand_cut;
  std::optional<test::NandNativeCheckpoint> native_cut;
  bool restored = false;
  for (unsigned period = 0; period < maximum_periods; ++period) {
    if (!native_cut && !test.unknown && machine.cpu().registers().pc == read_edge + 1 &&
        peer.detailed().previous_re == false) {
      nand_cut.emplace(peer.checkpoint());
      native_cut.emplace(schedule.checkpoint());
    }
    try {
      static_cast<void>(schedule.tick());
    } catch (const Fault &failure) {
      check_refusal(test, machine, peer, failure.what());
      return;
    }
    if (machine.cpu().registers().pc == record_end) {
      check_endpoint(test, machine, peer);
      if (!nand_cut) { throw Fault("Original status transition lacks its NAND snapshot"); }
      if (!native_cut) { throw Fault("Original status transition lacks its native snapshot"); }
      if (!restored) {
        peer.restore(nand_cut.value());
        schedule.restore(native_cut.value());
        restored = true;
        continue;
      }
      const auto reply = machine.bus().read({.space = DspSpace::x, .address = last_reply});
      std::cout << "PASS " << test.name << " original-reply=" << hex(reply, 2)
                << " restored-status-edge=1 operations=" << machine.cpu().progress().instructions
                << '\n';
      return;
    }
  }
  throw Fault("Original status probe exceeded its period bound");
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Expected original firmware ZIP");
    const auto firmware = Firmware::load(arguments[1]);
    for (const auto &test : cases) { run(firmware, test); }
    return 0;
  } catch (const std::exception &failure) {
    std::cerr << failure.what() << '\n';
    return 1;
  }
}
