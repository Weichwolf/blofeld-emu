// SPDX-License-Identifier: GPL-3.0-only
#include "io/dsp_reset.hpp"
#include "io/midi_port.hpp"
#include "io/shi.hpp"
#include "isa.hpp"
#include "mcu/execute.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>

namespace {
constexpr std::uint8_t note_on = 0x90, note = 60, velocity = 127;
constexpr std::uint8_t baud = 12, receive_irq = 0x24, transmit_enable = 8;
constexpr cpu12::Address marker = 0x3000, irq_entry = 0x8100;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::unique_ptr<blofeld::vi::FlashImage> image(const std::span<const cpu12::Byte> program) {
  auto result = std::make_unique<blofeld::vi::FlashImage>();
  result->known.fill(1);
  std::ranges::copy(program, result->bytes.begin());
  result->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
  result->bytes.back() = 0;
  return result;
}

void receive_interrupt() {
  namespace isa = cpu12::isa;
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::lds_immediate>(),
                                        std::array<cpu12::Byte, 2>{0x17, 0},
                                        isa::opcode<isa::Opcode::clr_extended>(),
                                        std::array<cpu12::Byte, 2>{0x14, 0},
                                        isa::opcode<isa::Opcode::clr_extended>(),
                                        std::array<cpu12::Byte, 2>{0x14, 1},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{baud},
                                        isa::opcode<isa::Opcode::stab_direct>(),
                                        std::array<cpu12::Byte, 1>{0xc9},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{receive_irq},
                                        isa::opcode<isa::Opcode::stab_direct>(),
                                        std::array<cpu12::Byte, 1>{0xcb},
                                        isa::opcode<isa::Opcode::andcc_immediate>(),
                                        std::array<cpu12::Byte, 1>{0xef},
                                        isa::opcode<isa::Opcode::wai>(),
                                        isa::opcode<isa::Opcode::bra_relative>(),
                                        std::array<cpu12::Byte, 1>{0xfd});
  constexpr auto handler = isa::program(isa::opcode<isa::Opcode::ldaa_direct>(),
                                        std::array<cpu12::Byte, 1>{0xcc},
                                        isa::opcode<isa::Opcode::ldaa_direct>(),
                                        std::array<cpu12::Byte, 1>{0xcf},
                                        isa::opcode<isa::Opcode::asl_extended>(),
                                        std::array<cpu12::Byte, 2>{0x14, 1},
                                        isa::opcode<isa::Opcode::eora_extended>(),
                                        std::array<cpu12::Byte, 2>{0x14, 1},
                                        isa::opcode<isa::Opcode::staa_extended>(),
                                        std::array<cpu12::Byte, 2>{0x14, 1},
                                        isa::opcode<isa::Opcode::inc_extended>(),
                                        std::array<cpu12::Byte, 2>{0x14, 0},
                                        isa::opcode<isa::Opcode::ldab_extended>(),
                                        std::array<cpu12::Byte, 2>{0x14, 0},
                                        isa::opcode<isa::Opcode::cmpb_immediate>(),
                                        std::array<cpu12::Byte, 1>{3},
                                        isa::opcode<isa::Opcode::beq_relative>(),
                                        std::array<cpu12::Byte, 1>{1},
                                        isa::opcode<isa::Opcode::rti>(),
                                        isa::opcode<isa::Opcode::staa_extended>(),
                                        std::array<cpu12::Byte, 2>{0x30, 0});
  auto firmware = image(program);
  std::ranges::copy(handler, firmware->bytes.begin() + (irq_entry - 0x8000));
  constexpr cpu12::Address vector = 0xffd6;
  firmware->bytes[vector - 0x8000] = static_cast<std::uint8_t>(irq_entry >> 8U);
  firmware->bytes[vector - 0x8000 + 1] = static_cast<std::uint8_t>(irq_entry);
  blofeld::vi::MidiPort midi;
  constexpr std::array packet{note_on, note, velocity};
  constexpr auto first = blofeld::vi::MidiPort::byte_ticks;
  require(midi.write_input(packet, first), "Schedule an actual MIDI message in device time");
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::mcu::execute(
      *firmware, shi, reset, {}, nullptr, nullptr, nullptr, nullptr, &midi);
  constexpr std::uint8_t checksum = 0x47; // Shift the previous checksum, then XOR each byte.
  constexpr cpu12::Address completed = irq_entry + 24;
  require(report.failure && report.failure->address == marker &&
              report.failure->value == checksum && report.failure->pc == completed &&
              midi.time() >= first * 4 && midi.time() < first * 5 && !midi.peek_input() &&
              midi.error() == nullptr,
          "Three real SCI interrupts wake WAI, acknowledge data and return before the next byte");
}

void transmit_completion() {
  namespace isa = cpu12::isa;
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::lds_immediate>(),
                                        std::array<cpu12::Byte, 2>{0x17, 0},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{baud},
                                        isa::opcode<isa::Opcode::stab_direct>(),
                                        std::array<cpu12::Byte, 1>{0xc9},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{transmit_enable},
                                        isa::opcode<isa::Opcode::stab_direct>(),
                                        std::array<cpu12::Byte, 1>{0xcb},
                                        isa::opcode<isa::Opcode::ldaa_direct>(),
                                        std::array<cpu12::Byte, 1>{0xcc},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{note_on},
                                        isa::opcode<isa::Opcode::stab_direct>(),
                                        std::array<cpu12::Byte, 1>{0xcf},
                                        isa::opcode<isa::Opcode::wai>());
  auto firmware = image(program);
  blofeld::vi::MidiPort midi;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::mcu::execute(
      *firmware, shi, reset, {}, nullptr, nullptr, nullptr, nullptr, &midi);
  const auto output = midi.read_output();
  // Reset(5), LDS(2), LDAB(1), STAB(2), LDAB(1), then TE starts its preamble.
  constexpr std::uint64_t completed_at = 38510; // 11 * 10 + two 19,200-tick characters.
  require(!report.failure && output && output->byte == note_on && output->ticks == completed_at &&
              output->ticks == midi.time() && !midi.read_output(),
          "Only the real transmitted character is published after the SCI preamble completes");
}
}

int main() try {
  receive_interrupt();
  transmit_completion();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
