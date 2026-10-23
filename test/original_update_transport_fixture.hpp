// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <exception>
#include <fstream>
#include <ios>
#include <iostream>
#include <istream>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace blofeld::test::update_transport {
using namespace blofeld;
using blofeld::test::require;
constexpr unsigned entry = 0x2361, last_instruction = 0x23d7, buffer = 0x778;
constexpr unsigned context = 0x2c25, store_site = 0x23c6, return_site = 0x23ca;
constexpr unsigned cursor_slot = 0x2d, sequence_slot = 0x31, status_slot = 0x76c;
constexpr unsigned memory_mode = 0x600389, mask = 0xffffff, guard = 0x456789;
constexpr unsigned max_steps = 2048, header_size = 13, checksum_shift = 16;
constexpr unsigned template_count = 6, return_template = 4, cursor_template = 5;
constexpr unsigned context_record = 93, command_record = 94, command_words = 126;
constexpr unsigned packet_limit = 5000, septet_mask = 127;
constexpr unsigned staging_rom = 0xff1000;
constexpr unsigned absolute_jump = 0x0af080, completed_status = 19;
constexpr unsigned length_slot = 0x2e, end_slot = 0x2f, start_slot = 0x30;

struct Layout {
  unsigned mode, address, declared, stored, packets;
};

constexpr std::array layouts{
    Layout{.mode = 3, .address = 0xef00, .declared = 256, .stored = 256, .packets = 37},
    Layout{.mode = 0, .address = 0x2e00, .declared = 63, .stored = 63, .packets = 9},
    Layout{.mode = 3, .address = 0x10000, .declared = 2883, .stored = 2884, .packets = 412},
    Layout{.mode = 0, .address = 0x2e00, .declared = 388, .stored = 389, .packets = 56},
    Layout{.mode = 3, .address = 0xd000, .declared = 29881, .stored = 29883, .packets = 4269},
    Layout{.mode = 0, .address = 0x2e00, .declared = 400, .stored = 401, .packets = 58}};
using Templates = std::array<unsigned, template_count>;

struct Block {
  std::vector<unsigned> expected;
  std::vector<std::vector<unsigned>> packets;
};

inline unsigned number(std::istream &stream, const unsigned maximum) {
  unsigned value = 0;
  require(static_cast<bool>(stream >> value) && value <= maximum,
          "Invalid bounded private reference fixture");
  return value;
}

inline std::vector<Block> read_blocks(const char *path) {
  std::ifstream stream(path);
  require(number(stream, layouts.size()) == layouts.size(), "Expected all six original blocks");
  std::vector<Block> blocks;
  for (const auto &layout : layouts) {
    Block block;
    block.expected.reserve(layout.stored);
    block.packets.reserve(layout.packets + 1);
    require(number(stream, packet_limit) == layout.packets + 1 &&
                number(stream, 32768) == layout.stored,
            "Original packet and complete stored-word counts must match");
    for (unsigned n = 0; n < layout.stored; ++n) { block.expected.push_back(number(stream, mask)); }
    for (unsigned n = 0; n <= layout.packets; ++n) {
      const auto count = number(stream, 256);
      require(count >= 7, "Every private packet includes the original command fields");
      std::vector<unsigned> packet;
      packet.reserve(count);
      for (unsigned i = 0; i < count; ++i) { packet.push_back(number(stream, septet_mask)); }
      block.packets.push_back(std::move(packet));
    }
    blocks.push_back(std::move(block));
  }
  stream >> std::ws;
  require(stream.eof(), "Unexpected trailing private packet fixture data");
  return blocks;
}

inline Templates read_templates(const char *path) {
  std::ifstream stream(path);
  stream >> std::hex;
  Templates result{};
  for (auto &value : result) { value = number(stream, mask); }
  stream >> std::ws;
  require(stream.eof(), "Unexpected trailing independent instruction expectations");
  return result;
}

struct Storage {
  DspBus bus;
  DspCore cpu{bus};
};

struct Handoff {
  DspMachine *owner = nullptr;
  bool reuse = false;
  unsigned from = 0;
};

struct Transfer {
  std::unique_ptr<Storage> owned;
  DspBus &bus;
  DspCore &cpu;
  const Handoff handoff;
  const unsigned rom_base;
  const Firmware &firmware;
  const Block &block;
  Layout layout;
  Templates templates;
  std::ostream &trace;
  unsigned caller_begin = 0, caller_end = 0, returned = 0;
  unsigned stored = 0, prefetches = 0, steps = 0;
  unsigned initial_status = guard;
  std::vector<unsigned> caller_words;

  Transfer(const Firmware &image,
           const Block &input,
           const Layout plan,
           const Templates &own,
           std::ostream &output,
           const Handoff link = {})
      : owned(link.owner == nullptr ? std::make_unique<Storage>() : nullptr),
        bus(link.owner != nullptr ? link.owner->bus() : owned->bus),
        cpu(link.owner != nullptr ? link.owner->cpu() : owned->cpu),
        handoff(link),
        rom_base(link.owner != nullptr ? staging_rom : blofeld::test::boot),
        firmware(image),
        block(input),
        layout(plan),
        templates(own),
        trace(output) {
    if (handoff.owner == nullptr) {
      bus.observe_omr(memory_mode);
      bus.advance_instruction_cycles(3);
    }
    if (!handoff.reuse) { install_records(); }
    install_caller();
    if (handoff.owner == nullptr) {
      for (const unsigned address : {layout.address - 1, layout.address + layout.stored}) {
        bus.write({.space = target(), .address = address, .value = guard});
      }
    }
    if (handoff.owner == nullptr) {
      bus.write({.space = DspSpace::x, .address = status_slot, .value = guard});
    } else {
      initial_status = get(status_slot);
    }
  }

  void install_records() {
    for (const unsigned index : {context_record, command_record}) {
      const auto &record = firmware.dsp.at(index);
      require(record.address == (index == context_record ? context : entry) &&
                  record.space == (index == context_record ? 1U : 0U) &&
                  record.words.size() == (index == context_record ? 12U : command_words),
              "Complete original context and command records retain their layout");
      for (unsigned n = 0; n < record.words.size(); ++n) {
        bus.write({.space = static_cast<DspSpace>(record.space),
                   .address = record.address + n,
                   .value = record.words[n]});
      }
    }
  }

  [[nodiscard]] DspSpace target() const { return layout.mode == 0 ? DspSpace::p : DspSpace::y; }

  void install_caller() {
    blofeld::test::DspProgram code;
    code.immediate("omr", memory_mode);
    for (unsigned n = 0; n < 3; ++n) { code.emit("nop"); }
    caller_begin = code.pc() + rom_base - blofeld::test::boot;
    for (unsigned n = 0; n < 8; ++n) {
      code.immediate("m" + std::to_string(n), mask);
      code.immediate("n" + std::to_string(n), 0);
    }
    for (const auto *const reg : {"a0", "a1", "a2", "b0", "b1", "b2"}) { code.immediate(reg, 0); }
    code.immediate("sr", blofeld::test::masked_status);
    code.immediate("r7", entry);
    code.emit("jsr (r7)");
    returned = code.pc() + rom_base - blofeld::test::boot;
    code.word(absolute_jump); // Own absolute JMP back to the declared native caller setup.
    code.word(caller_begin);
    caller_end = code.pc() + rom_base - blofeld::test::boot;
    DspBus scratch;
    code.install(scratch);
    for (unsigned n = 0; n < caller_end - rom_base; ++n) {
      caller_words.push_back(scratch.inspect_program(blofeld::test::boot + n));
    }
    if (handoff.reuse) {
      for (unsigned n = 0; n < caller_words.size(); ++n) {
        require(bus.inspect_program(rom_base + n) == caller_words[n],
                "Reused own caller must remain exact");
      }
      require(cpu.state().pc == returned,
              "Previous native transfer must supply its real return boundary");
    } else {
      bus.install_rom({.start = rom_base, .words = caller_words});
      if (handoff.owner == nullptr) {
        cpu.reset_spi_boot();
      } else {
        require(cpu.state().pc == handoff.from && handoff.from >= blofeld::test::boot &&
                    handoff.from < rom_base,
                "Native stage handoff requires the existing own foreground boundary");
        const std::array<unsigned, 2> jump{absolute_jump, rom_base};
        bus.install_rom({.start = handoff.from, .words = jump});
      }
    }
    unsigned count = 0;
    while (cpu.state().pc != caller_begin) {
      require(++count < 16, "Own native OMR setup must complete before switched-memory guards");
      native_step();
    }
    trace << "SETUP instructions=" << count << " pc=" << cpu.state().pc
          << " omr=" << cpu.state().omr << '\n';
  }

  unsigned get(const unsigned address) {
    return bus.read({.space = DspSpace::x, .address = address});
  }

  void native_step(const DspNativeAccessObserver &observer = {}) {
    cpu.step(observer);
    if (handoff.owner == nullptr) { bus.advance_instruction_cycles(1); }
    ++steps;
  }

  void observe(const DspNativeAccess &access, const unsigned packet, const unsigned checksum) {
    if (access.stage != DspNativeAccessStage::after) { return; }
    if (!access.value.has_value()) { throw Fault("Completed native access has no actual value"); }
    if (access.kind == DspNativeAccessKind::write && access.pc == store_site) {
      require(
          stored < block.expected.size() && access.target.space == target() &&
              access.target.address == layout.address + stored &&
              access.value == block.expected.at(stored),
          "Every actual original destination store matches independent complete packet decoding");
      trace << "STORE index=" << stored << " space=" << static_cast<unsigned>(access.target.space)
            << " address=" << access.target.address << " value=" << access.value.value()
            << " pc=" << access.pc << '\n';
      ++stored;
    }
    const auto &bytes = block.packets.at(packet);
    if (packet != 0 && access.kind == DspNativeAccessKind::read &&
        access.target.space == DspSpace::x && access.target.address == checksum) {
      require(access.value == bytes.back() << checksum_shift,
              "Original checksum prefetch remains exact");
      trace << "PREFETCH packet=" << packet << " address=" << checksum
            << " value=" << access.value.value() << " pc=" << access.pc << '\n';
      ++prefetches;
    }
  }

  void call(const unsigned packet, const bool launch) {
    const auto &bytes = block.packets.at(packet);
    for (unsigned n = 0; n < bytes.size(); ++n) {
      bus.write({.space = DspSpace::x, .address = buffer + n, .value = bytes[n] << checksum_shift});
    }
    const auto checksum = buffer + static_cast<unsigned>(bytes.size()) - 1;
    const DspNativeAccessObserver observer{
        .observe = [this, packet, checksum](const DspNativeAccess &access) {
          observe(access, packet, checksum);
        }};
    unsigned count = 0;
    do {
      require(++count < max_steps,
              "Original packet must return or launch within its instruction bound");
      const auto pc = cpu.state().pc;
      require((pc >= rom_base && pc < caller_end) || (pc >= entry && pc <= last_instruction),
              "No helper, host replacement or unrelated routine executes during download");
      native_step(observer);
      if (launch && cpu.state().pc == layout.address) { break; }
    } while (cpu.state().pc != returned);
    const auto state = cpu.state();
    require(state.pc == (launch ? layout.address : returned) && state.sp == (launch ? 1U : 0U),
            "Original RTS or JMP must preserve the actual own-caller frame");
    trace << "PACKET index=" << packet << " stored=" << stored << " cursor=" << get(cursor_slot)
          << " sequence=" << get(sequence_slot) << " pc=" << state.pc << " sp=" << state.sp
          << " steps=" << count << " prefetches=" << prefetches << '\n';
  }

  void verify_storage() {
    for (unsigned n = 0; n < stored; ++n) {
      require(bus.read({.space = target(), .address = layout.address + n}) == block.expected.at(n),
              "Every earlier original destination word remains exact after later packets");
    }
    for (const unsigned address : {layout.address - 1, layout.address + layout.stored}) {
      if (handoff.owner == nullptr) {
        require(bus.read({.space = target(), .address = address}) == guard,
                "Transfer boundary guards remain intact");
      } else {
        bool unknown = false;
        try {
          static_cast<void>(bus.read({.space = target(), .address = address}));
        } catch (const Fault &error) {
          unknown = std::string_view(error.what()).contains("Unknown DSP memory read");
        }
        require(unknown,
                "Coupled transfer boundary RAM must remain unprovided, not padded by a guard");
      }
    }
    for (unsigned n = 0; n < caller_words.size(); ++n) {
      require(bus.inspect_program(rom_base + n) == caller_words[n], "Own caller remains unchanged");
    }
    for (const unsigned index : {context_record, command_record}) {
      const auto &record = firmware.dsp.at(index);
      for (unsigned n = 0; n < record.words.size(); ++n) {
        const auto address = record.address + n;
        if (index == command_record && (address == store_site || address == return_site)) {
          continue;
        }
        require(bus.read({.space = static_cast<DspSpace>(record.space), .address = address}) ==
                    record.words[n],
                "Original command and context words outside the two live patches remain unchanged");
      }
    }
  }

  void run() {
    require(block.packets.front().size() == header_size,
            "Original configuration has thirteen septets");
    call(0, false);
    require(get(cursor_slot) == layout.address && get(length_slot) == layout.declared * 3 &&
                get(end_slot) == layout.address + layout.declared &&
                get(start_slot) == layout.address && get(sequence_slot) == 0 &&
                bus.inspect_program(store_site) == templates[layout.mode] &&
                bus.inspect_program(return_site) ==
                    templates[layout.mode == 0 ? cursor_template : return_template],
            "Original header arithmetic and independently assembled live templates match");
    for (unsigned packet = 1; packet <= layout.packets; ++packet) {
      if (packet == layout.packets) {
        verify_storage();
        require(get(status_slot) == initial_status,
                "An interrupted prefix has not reported a helper launch");
        trace << "INTERRUPTED stored=" << stored << " cursor=" << get(cursor_slot) << '\n';
      }
      const bool launch = layout.mode == 0 && packet == layout.packets;
      call(packet, launch);
      require(get(cursor_slot) == layout.address + stored &&
                  get(sequence_slot) == (packet % 128) << checksum_shift,
              "Every actual packet publishes its complete cursor and modulo128 sequence");
    }
    require(stored == layout.stored && prefetches == layout.packets &&
                bus.inspect_program(return_site) == templates[return_template] &&
                get(status_slot) == (layout.mode == 0 ? completed_status : initial_status) &&
                (layout.mode != 0 || cpu.state().r[0] == context),
            "All complete padding words and real final launch context remain visible");
    verify_storage();
    trace << "RESULT stored=" << stored << " declared=" << layout.declared
          << " extra=" << stored - layout.declared << " prefetches=" << prefetches
          << " steps=" << steps << " launched=" << (layout.mode == 0) << " guards=1\n";
  }
};
} // namespace blofeld::test::update_transport
