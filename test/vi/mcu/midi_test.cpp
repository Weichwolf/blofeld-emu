// SPDX-License-Identifier: GPL-3.0-only
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/shi.hpp"
#include "isa.hpp"
#include "mcu/execute.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <thread>

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

void wait_at(blofeld::vi::MidiPort &midi,
             const std::uint64_t ticks,
             const std::atomic<bool> &done,
             const unsigned tolerance = 0) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (midi.time() + tolerance < ticks) {
    require(!done.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline,
            "Native execution must reach its supplied input frontier");
    std::this_thread::yield();
  }
  const auto held = midi.time();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  require(held <= ticks && ticks - held <= tolerance && midi.time() == held &&
              !done.load(std::memory_order_acquire),
          "Producer delays suspend execution without advancing musical time");
}

blofeld::vi::ExecutionReport
receive(blofeld::vi::FlashImage &firmware, blofeld::vi::MidiPort &midi, const unsigned tolerance) {
  constexpr std::array packet{note_on, note, velocity};
  constexpr auto first = blofeld::vi::MidiPort::byte_ticks;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  if (!midi.input_limit()) {
    require(midi.write_input(packet, first), "Schedule a real message in device time");
    return blofeld::vi::mcu::execute(
        firmware, shi, reset, {}, nullptr, nullptr, nullptr, nullptr, &midi);
  }
  blofeld::vi::ExecutionReport report;
  std::atomic<bool> done{false};
  std::jthread execution([&](const std::stop_token &stop) {
    try {
      report = blofeld::vi::mcu::execute(
          firmware, shi, reset, stop, nullptr, nullptr, nullptr, nullptr, &midi);
    } catch (...) { report.exception = std::current_exception(); }
    done.store(true, std::memory_order_release);
  });
  for (std::size_t index = 0; index < packet.size(); ++index) {
    require(
        midi.write_input(std::span(packet).subspan(index, 1), first, execution.get_stop_token()),
        "Stream each complete byte through bounded admission");
    wait_at(midi, first + ((index + 1) * blofeld::vi::MidiPort::byte_ticks), done, tolerance);
  }
  require(midi.finish_input(), "EOF permits the final genuine SCI handler to complete");
  execution.join();
  if (report.exception) { std::rethrow_exception(report.exception); }
  return report;
}

enum class ReceiveClock : std::uint8_t { reset, divided, divided_unaligned };

template <ReceiveClock clock = ReceiveClock::reset>
void receive_interrupt(const blofeld::vi::MidiPort::InputMode mode) {
  namespace isa = cpu12::isa;
  constexpr cpu12::Byte selected_baud = clock != ReceiveClock::reset ? 2 : baud;
  constexpr auto resting = clock != ReceiveClock::reset ? isa::opcode<isa::Opcode::nop>()
                                                        : isa::opcode<isa::Opcode::wai>();
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::lds_immediate>(),
                                        std::array<cpu12::Byte, 2>{0x17, 0},
                                        isa::opcode<isa::Opcode::clr_extended>(),
                                        std::array<cpu12::Byte, 2>{0x14, 0},
                                        isa::opcode<isa::Opcode::clr_extended>(),
                                        std::array<cpu12::Byte, 2>{0x14, 1},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{selected_baud},
                                        isa::opcode<isa::Opcode::stab_direct>(),
                                        std::array<cpu12::Byte, 1>{0xc9},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{receive_irq},
                                        isa::opcode<isa::Opcode::stab_direct>(),
                                        std::array<cpu12::Byte, 1>{0xcb},
                                        isa::opcode<isa::Opcode::andcc_immediate>(),
                                        std::array<cpu12::Byte, 1>{0xef},
                                        resting,
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
  if constexpr (clock != ReceiveClock::reset) {
    constexpr auto prefix = isa::program(isa::opcode<isa::Opcode::ldab_immediate>(),
                                         std::array<cpu12::Byte, 1>{5},
                                         isa::opcode<isa::Opcode::stab_direct>(),
                                         std::array<cpu12::Byte, 1>{0x35});
    constexpr auto lead = isa::opcode<isa::Opcode::nop>();
    constexpr auto offset = clock == ReceiveClock::divided_unaligned ? lead.size() : 0;
    std::ranges::copy(program, firmware->bytes.begin() + prefix.size() + offset);
    std::ranges::copy(prefix, firmware->bytes.begin() + offset);
    if constexpr (offset != 0) { std::ranges::copy(lead, firmware->bytes.begin()); }
  }
  std::ranges::copy(handler, firmware->bytes.begin() + (irq_entry - 0x8000));
  constexpr cpu12::Address vector = 0xffd6;
  firmware->bytes[vector - 0x8000] = static_cast<std::uint8_t>(irq_entry >> 8U);
  firmware->bytes[vector - 0x8000 + 1] = static_cast<std::uint8_t>(irq_entry);
  blofeld::vi::MidiPort midi(mode);
  constexpr auto first = blofeld::vi::MidiPort::byte_ticks;
  constexpr unsigned tolerance = clock == ReceiveClock::divided_unaligned ? 59 : 0;
  const auto report = receive(*firmware, midi, tolerance);
  constexpr std::uint8_t checksum = 0x47; // Shift the previous checksum, then XOR each byte.
  constexpr cpu12::Address completed = irq_entry + 24;
  require(report.failure && report.failure->address == marker &&
              report.failure->value == checksum && report.failure->pc == completed &&
              midi.time() >= first * 4 && midi.time() < first * 5 && !midi.peek_input() &&
              midi.error() == nullptr,
          "Three real SCI interrupts wake WAI, acknowledge data and return before the next byte");
}

void native_loop_frontier() {
  namespace isa = cpu12::isa;
  constexpr auto program =
      isa::program(isa::opcode<isa::Opcode::bra_relative>(), std::array<cpu12::Byte, 1>{0xfe});
  auto firmware = image(program);
  blofeld::vi::MidiPort midi(blofeld::vi::MidiPort::InputMode::stream);
  constexpr std::uint64_t frontier = 192000;
  require(midi.advance_input(frontier), "Supply an empty known interval without a fake character");
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::ExecutionTime time;
  blofeld::vi::ExecutionReport report;
  std::atomic<bool> done{false};
  std::jthread execution([&](const std::stop_token &stop) {
    try {
      report = blofeld::vi::mcu::execute(
          *firmware, shi, reset, stop, nullptr, nullptr, nullptr, &time, &midi);
    } catch (...) { report.exception = std::current_exception(); }
    done.store(true, std::memory_order_release);
  });
  wait_at(midi, frontier, done);
  std::optional<std::uint64_t> observed;
  std::atomic<bool> published{false};
  std::jthread dsp([&](const std::stop_token &stop) {
    observed = time.dsp_limit(frontier - 1, stop);
    published.store(true, std::memory_order_release);
  });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!published.load(std::memory_order_acquire)) {
    require(std::chrono::steady_clock::now() < deadline,
            "The DSP sees the MCU input frontier before it waits");
    std::this_thread::yield();
  }
  dsp.join();
  require(observed == frontier,
          "Input starvation cannot leave the DSP behind an unpublished MCU frontier");
  execution.request_stop();
  execution.join();
  require(!report.exception && !report.failure && report.instructions > 0 &&
              midi.time() == frontier && !midi.peek_input(),
          "Native code without local I/O stops at the actual input frontier and cancels cleanly");
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
  receive_interrupt(blofeld::vi::MidiPort::InputMode::live);
  receive_interrupt(blofeld::vi::MidiPort::InputMode::stream);
  receive_interrupt<ReceiveClock::divided>(blofeld::vi::MidiPort::InputMode::stream);
  receive_interrupt<ReceiveClock::divided_unaligned>(blofeld::vi::MidiPort::InputMode::stream);
  transmit_completion();
  native_loop_frontier();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
