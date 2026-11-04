// SPDX-License-Identifier: GPL-3.0-only
// Original sample helper plus runtime queue; explicit isolated RAM/board premises.
#include "board/nand_bus_decode.hpp"
#include "board/nand_peer.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
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
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000, mode = 0x600389, mask = 0xffffff;
constexpr unsigned helper_base = 0x2e00, helper_words = 133;
constexpr unsigned descriptor = 0x100, hook = 0x300, flags = 6;
constexpr unsigned queue_entry = 0x1856, output_base = 0x2800, state_base = 0xc000;
constexpr unsigned first_slot = 0x10000, group_words = 5640, data_words = 5632;
constexpr unsigned page_words = 176, page_bytes = 528, pages = 32;
constexpr unsigned old_cookie = 0x91, neighbor_cookie = 0x42;
constexpr unsigned max_ticks = 4 * 2000000, byte_mask = 0xff, byte_bits = 8;
constexpr unsigned jsr_absolute = 0x0bf080, jmp_absolute = 0x0af080;
constexpr unsigned timer_priority = 0x100;
constexpr unsigned producer = 0x2a14, consumer = 0x2a15, queue_base = 0x2700;
constexpr unsigned inhibited_bit = 1, timer_vector = 0x58, reply = 0x2a19;

struct Group {
  unsigned slot = 0;
  std::vector<unsigned> words;
};

using Groups = std::array<Group, 4>;

void require(const bool condition, const std::string_view reason) {
  if (!condition) { throw std::runtime_error(std::string(reason)); }
}

unsigned read_be(std::istream &input, const unsigned count) {
  unsigned result = 0;
  for (unsigned i = 0; i < count; ++i) {
    const int value = input.get();
    require(value != std::char_traits<char>::eof(), "Truncated pinned input");
    result = (result << byte_bits) | static_cast<unsigned>(value);
  }
  return result;
}

void put(DspBus &bus, const DspSpace space, const unsigned address, const unsigned value) {
  bus.write({.space = space, .address = address, .value = value});
}

unsigned get(DspBus &bus, const unsigned address) {
  return bus.read({.space = DspSpace::x, .address = address});
}

void install(DspBus &bus,
             const Firmware &firmware,
             const std::span<const unsigned> helper,
             const Groups &groups) {
  bus.set_write_contract(DspWriteContract::functional_decode);
  for (const auto port : {DspPort::c, DspPort::f}) {
    bus.gpio(port).set_contract(DspGpioContract::logical_pins);
  }
  bus.observe_omr(mode);
  bus.advance_instruction_cycles(3);
  // Declared initialized laboratory RAM, not recovered boot state.
  constexpr unsigned x_words = 0x3000;
  constexpr unsigned y_words = 0x2000;
  for (unsigned i = 0; i < x_words; ++i) { put(bus, DspSpace::x, i, 0); }
  for (unsigned i = 0; i < y_words; ++i) { put(bus, DspSpace::y, i, 0); }
  for (const auto &record : firmware.dsp) {
    for (unsigned i = 0; i < record.words.size(); ++i) {
      put(bus, static_cast<DspSpace>(record.space), record.address + i, record.words[i]);
    }
  }
  for (unsigned i = 0; i < helper.size(); ++i) {
    put(bus, DspSpace::p, helper_base + i, helper[i]);
  }
  for (const auto &group : groups) {
    for (unsigned i = 0; i < group.words.size(); ++i) {
      put(bus, DspSpace::y, group.slot + i, group.words[i]);
    }
  }
  for (const auto field :
       {std::pair{0U, 4U}, {1U, hook}, {5U, flags}, {6U, queue_entry}, {7U, output_base}}) {
    put(bus, DspSpace::x, descriptor + field.first, field.second);
  }
  put(bus, DspSpace::x, flags, 0);
  // The original installer patches hook to its foreground poller. Its saved
  // continuation is hook-2; these two authored NOPs form a genuine native loop.
  put(bus, DspSpace::p, hook - 2, 0);
  put(bus, DspSpace::p, hook - 1, 0);
  require(get(bus, producer) == queue_base && get(bus, consumer) == queue_base,
          "Original queue cursors must initially agree");
}

class Caller {
public:
  void operand(const unsigned opcode, const unsigned value) {
    words_.push_back(opcode);
    words_.push_back(value);
  }

  void word(const unsigned value) { words_.push_back(value); }

  void nops() {
    for (unsigned i = 0; i < 3; ++i) { word(0); }
  }

  void install(DspBus &bus) const { bus.install_rom({.start = boot, .words = words_}); }

private:
  std::vector<std::uint32_t> words_;
};

void caller(DspBus &bus) {
  Caller code;
  // Authored FM literal encodings, matching existing native NAND fixtures.
  constexpr unsigned x1_immediate = 0x45f400;
  constexpr unsigned x1_omr = 0x04c5ba;
  constexpr unsigned x1_sr = 0x04c5b9;
  constexpr unsigned modifier = 0x05f420;
  constexpr unsigned r0_immediate = 0x60f400;
  constexpr unsigned bus_control = 0x08f4bb;
  constexpr unsigned direction_c = 0x07f43e;
  constexpr unsigned function_c = 0x07f43f;
  constexpr unsigned data_c = 0x07f43d;
  constexpr unsigned direction_f = 0x09f4b6;
  constexpr unsigned function_f = 0x09f4b7;
  constexpr unsigned data_f = 0x09f4b5;
  constexpr unsigned iprp = 0x08f4be;
  constexpr unsigned timer_load = 0x07f40a;
  constexpr unsigned timer_compare = 0x07f409;
  constexpr unsigned bus_value = 0x1ffff;
  constexpr unsigned controls = 0xf00;
  constexpr unsigned inputs = 0xff;
  constexpr unsigned inactive = 0x300;
  constexpr unsigned write_direction = 0x4ff;
  code.nops();
  code.operand(x1_immediate, mode);
  code.word(x1_omr);
  code.nops();
  for (unsigned i = 0; i < 8; ++i) { code.operand(modifier + i, mask); }
  code.operand(bus_control, bus_value);
  code.operand(direction_c, controls);
  code.operand(function_c, inputs);
  code.operand(data_c, inactive);
  code.operand(function_f, inactive);
  code.operand(direction_f, write_direction);
  code.operand(data_f, 0);
  code.operand(timer_load, 0);
  code.operand(timer_compare, 1);
  code.operand(iprp, timer_priority);
  code.operand(x1_immediate, 0);
  code.word(x1_sr);
  code.operand(r0_immediate, descriptor);
  code.operand(jsr_absolute, helper_base);
  code.operand(jmp_absolute, hook - 2);
  code.install(bus);
}

std::vector<unsigned> guards(const Groups &groups) {
  std::vector<unsigned> result{groups.front().words[data_words] - pages};
  for (const auto &group : groups) {
    const auto row = group.words[data_words] + pages;
    bool target = false;
    for (const auto &other : groups) { target = target || other.words[data_words] == row; }
    if (!target) { result.push_back(row); }
  }
  return result;
}

Nand device(const Groups &groups) {
  NandImage image;
  NandArray::Page page;
  page.cells.fill({.value = old_cookie, .known = byte_mask});
  page.programs = NandProgramCount{.main = 1, .spare = 1};
  for (const auto &group : groups) {
    const auto row = group.words[data_words];
    for (unsigned i = 0; i < pages; ++i) { image.pages.emplace(row + i, page); }
    image.blocks.emplace(
        row / pages,
        NandBlock{.valid = true, .program_protected = false, .erase_protected = false});
  }
  page.cells.fill({.value = neighbor_cookie, .known = byte_mask});
  for (const auto row : guards(groups)) {
    for (unsigned i = 0; i < pages; ++i) { image.pages.emplace(row + i, page); }
  }
  Nand nand(image, NandReadContinuation::k9f1208_p);
  nand.observe_wp(true); // Explicit external laboratory write-enable input.
  return nand;
}

void verify(const NandPeer &peer,
            const unsigned row,
            const std::span<const unsigned> words,
            const bool inhibited) {
  const auto nand = peer.protocol_copy();
  for (unsigned page = 0; page < pages; ++page) {
    for (unsigned column = 0; column < page_bytes; ++column) {
      const auto index = (page * page_words) + (column / 3);
      auto expected = (words[index] >> ((2 - column % 3) * byte_bits)) & byte_mask;
      if (inhibited && page == pages - 1 && column == 0) {
        require((expected & inhibited_bit) == 0,
                "Declared inhibited bit must need a program pulse");
        expected |= inhibited_bit;
      }
      require(nand.cell({.page = row + page, .column = column}) ==
                  NandByte{.value = static_cast<std::uint8_t>(expected), .known = byte_mask},
              "Actual programmed cell differs from independent complete input");
    }
  }
}

void verify_guards(const NandPeer &peer, const Groups &groups) {
  const auto nand = peer.protocol_copy();
  for (const auto row : guards(groups)) {
    for (unsigned page = 0; page < pages; ++page) {
      for (unsigned column = 0; column < page_bytes; ++column) {
        require(nand.cell({.page = row + page, .column = column}) ==
                    NandByte{.value = neighbor_cookie, .known = byte_mask},
                "Guard block changed");
      }
    }
  }
}

void verify_code(DspBus &bus, const Firmware &firmware, const std::span<const unsigned> helper) {
  constexpr unsigned service = 0x2e27;
  constexpr unsigned status_read = 0x2e29;
  constexpr unsigned status_write = 0x2e30;
  constexpr unsigned continuation = 0x2e42;
  constexpr unsigned poll = 0x2e28;
  // The original distribution intentionally replaces P:$10..11 in a later
  // record. Compare the final ordered projection, preserving that replacement.
  std::map<unsigned, unsigned> original;
  for (const auto &record : firmware.dsp) {
    if (record.space != 0) { continue; }
    for (unsigned i = 0; i < record.words.size(); ++i) {
      original[record.address + i] = record.words[i];
    }
  }
  for (const auto &[address, expected] : original) {
    if ((address >= hook - 2 && address < hook + 2) ||
        (address >= helper_base && address < helper_base + helper_words)) {
      continue;
    }
    require(bus.peek_program(address) == expected,
            "Original resident program changed at P:" + hex(address, 6));
  }
  for (unsigned i = 0; i < helper.size(); ++i) {
    const auto address = helper_base + i;
    auto expected = helper[i];
    if (address == service) { expected = queue_entry; }
    if (address == status_read || address == status_write) { expected = flags; }
    if (address == continuation) { expected = hook - 2; }
    require(bus.peek_program(address) == expected, "Unexpected original helper modification");
  }
  require(bus.peek_program(hook) == jmp_absolute && bus.peek_program(hook + 1) == poll,
          "Original foreground hook was not installed");
}

void execute(const Firmware &firmware,
             const std::span<const unsigned> helper,
             const Groups &groups,
             const bool inhibited,
             const std::filesystem::path &trace) {
  constexpr unsigned extal = 12000000;
  constexpr std::uint64_t timebase = 1080000000;
  OwnDspMachine machine({.extal_hz = extal,
                         .timebase_hz = timebase,
                         .switch_timing = DspClockSwitch::reference_cycle_boundary});
  install(machine.bus(), firmware, helper, groups);
  caller(machine.bus());
  const auto row = groups.back().words[data_words];
  NandCellProfile profile{.contract = NandCellContract::reference_pulse_inhibit, .cells = {}};
  if (inhibited) {
    profile.cells.emplace((row + pages - 1) * page_bytes,
                          NandCellInhibit{.program = inhibited_bit, .erase = 0});
  }
  NandPeer peer(machine,
                device(groups),
                std::move(profile),
                NandBusContract::functional_transactions,
                NandTiming::functional_protocol);
  std::ofstream telemetry(trace);
  require(telemetry.is_open(), "Cannot open original sample NAND trace");
  peer.set_telemetry(&telemetry);
  test::NandNativeRun run(
      machine,
      peer,
      {.identity = std::make_shared<const PeripheralEventIdentity>(),
       .next_ticks = []() -> std::optional<std::uint64_t> { return std::nullopt; },
       .observe = [](std::uint64_t) { throw Fault("No external event declared"); }});
  bool installed = false;
  unsigned interrupts = 0;
  unsigned completed = 0;
  try {
    for (unsigned tick = 0; tick < max_ticks; ++tick) {
      const auto cycle = run.tick();
      if (cycle.native && cycle.native->retired && cycle.native_pc == timer_vector) {
        ++interrupts;
      }
      installed = installed || machine.cpu().registers().pc == hook - 2;
      const auto count = installed ? get(machine.bus(), state_base + 6) : 0U;
      if (count != completed) {
        require(count == completed + 1 && count <= groups.size(), "Original block counter order");
        const auto &group = groups[completed];
        verify(peer, group.words[data_words], group.words, inhibited && count == groups.size());
        completed = count;
        std::cout << "BLOCK count=" << count << " slot=" << group.slot
                  << " row=" << group.words[data_words] << " cycles=" << tick + 1 << '\n';
      }
      if (completed == groups.size() && machine.cpu().registers().pc == hook - 2) {
        verify_guards(peer, groups);
        verify_code(machine.bus(), firmware, helper);
        constexpr unsigned status_ready = 0xc0;
        require(get(machine.bus(), reply) == status_ready + (inhibited ? 1U : 0U),
                "Original completion counter is distinct from actual sampled status");
        require(peer.detailed().base.pending == NandPending::none && interrupts == pages * 4 + 4,
                "Every original transfer must finish its actual Timer1 interrupt");
        std::cout << "COMPLETE blocks=" << completed << " cycles=" << tick + 1
                  << " instructions=" << machine.cpu().progress().instructions
                  << " timer-entries=" << interrupts << " inhibited=" << inhibited << '\n';
        return;
      }
    }
    throw Fault("Original sample queue exceeded explicit native cycle bound");
  } catch (const std::exception &error) {
    const auto cpu = machine.cpu().registers();
    std::cerr << "FRONTIER pc=" << hex(cpu.pc, 6) << " omr=" << hex(cpu.omr, 6)
              << " ticks=" << machine.clock().elapsed_ticks()
              << " instructions=" << machine.cpu().progress().instructions
              << " installed=" << installed << " producer=" << hex(get(machine.bus(), producer), 6)
              << " consumer=" << hex(get(machine.bus(), consumer), 6)
              << " flags=" << hex(get(machine.bus(), flags), 6) << " cause=" << error.what()
              << '\n';
    throw;
  }
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 5, "Expected firmware, helper, complete frames and trace directory");
    const auto firmware = Firmware::load(arguments[1]);
    std::ifstream helper_input(arguments[2], std::ios::binary);
    std::ifstream frames(arguments[3], std::ios::binary);
    std::array<unsigned, helper_words> helper{};
    for (auto &word : helper) { word = read_be(helper_input, 3); }
    require(helper_input.peek() == std::char_traits<char>::eof(), "Changed helper size");
    Groups groups;
    constexpr unsigned stride = 0x1610;
    constexpr unsigned marker = 0xddef0e;
    for (unsigned i = 0; i < groups.size(); ++i) {
      auto &group = groups[i];
      group.slot = read_be(frames, 4);
      require(group.slot == first_slot + (i * stride), "Changed original staging slot");
      group.words.resize(group_words);
      for (auto &word : group.words) { word = read_be(frames, 3); }
      require(group.words[data_words + 1] == marker && group.words[data_words] % pages == 0,
              "Changed original bulk metadata");
    }
    const std::filesystem::path output(arguments[4]);
    execute(firmware, helper, groups, false, output / "normal.trace");
    execute(firmware, helper, groups, true, output / "inhibited.trace");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
