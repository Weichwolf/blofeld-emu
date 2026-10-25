// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_usb.hpp"
#include "original_gpio_midi_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test::gpio_midi {
namespace {
constexpr unsigned foreground = 0x8000;
constexpr unsigned stack = 0x2200;
constexpr unsigned stack_low = stack - 22;
constexpr unsigned ram_begin = 0x1200;
constexpr unsigned ram_end = 0x2600;
constexpr unsigned activity = 0x1815;
constexpr unsigned midi_activity = 0x1812;
constexpr unsigned transfer_flags = 0x1831;
constexpr unsigned all_bits = 255;
constexpr unsigned bus_activity = 0x80;
constexpr unsigned multibyte = 0x10;
constexpr unsigned usb_sender = 0xb8b0;
constexpr unsigned usb_end = 0xb938;
constexpr unsigned strobe = 0xb6d0;
constexpr unsigned strobe_end = 0xb6dc;
constexpr unsigned out_handler = 0xbe7a;
constexpr unsigned out_end = 0xbe9b;
constexpr unsigned out_vector = 0xffc8;
constexpr unsigned packet_bytes = 64;
constexpr unsigned packets = 4;
constexpr unsigned instruction_limit = 20000;
constexpr unsigned initial_flags = 0x40;
constexpr unsigned initial_d = 0x1234;
constexpr unsigned initial_x = 0x5678;
constexpr unsigned initial_y = 0x9abc;
constexpr unsigned idle_control = 0x1040;
constexpr unsigned unknown_byte = 256;
}

void Peer::prepare_endpoint() {
  usb_host::prepare(machine_.bus(), packet_bytes);
  require(source_.storage.size() == static_cast<std::size_t>(packets) * packet_bytes,
          "Four complete valid OUT packets cover the declared source");
  machine_.cpu().set_state({.pc = foreground,
                            .d = initial_d,
                            .x = initial_x,
                            .y = initial_y,
                            .sp = stack,
                            .ccr = initial_flags});
}

void Peer::submit() {
  if (cursor_ == source_.storage.size()) { return; }
  const auto packet = std::span(source_.storage).subspan(cursor_, packet_bytes);
  const auto initial_pid = machine_.bus().usb_host_endpoint(usb_host::output_address).data1;
  const auto initial_control = usb_host::word(machine_.bus(), usb_host::output_control);
  const auto response = machine_.bus().receive_usb_out(
      {.buffer = UsbBuffer::endpoint3, .data = packet, .data1 = data1_});
  if (response == UsbResponse::nak) {
    require(machine_.bus().usb_host_endpoint(usb_host::output_address).data1 == initial_pid &&
                usb_host::word(machine_.bus(), usb_host::output_control) == initial_control,
            "Backpressure preserves accepted PID and real endpoint ownership");
    ++naks_;
    return;
  }
  require(response == UsbResponse::ack, "Actual valid decoded OUT must ACK or retain backpressure");
  const auto pid = machine_.bus().usb_host_endpoint(usb_host::output_address).data1;
  require(pid.has_value() && *pid != data1_,
          "Only a real accepted PID advances the endpoint phase");
  data1_ = !data1_;
  cursor_ += packet_bytes;
  ++calls_;
}

void Peer::check_usb_frame() {
  const std::array<unsigned, 9> expected{initial_flags,
                                         initial_d & all_bits,
                                         initial_d >> 8U,
                                         initial_x >> 8U,
                                         initial_x & all_bits,
                                         initial_y >> 8U,
                                         initial_y & all_bits,
                                         foreground >> 8U,
                                         0};
  for (unsigned index = 0; index < expected.size(); ++index) {
    require(machine_.bus().read(static_cast<std::uint16_t>(stack - expected.size() + index)) ==
                expected[index],
            "Every literal NXP IRQ frame byte preserves actual foreground registers");
  }
}

void Peer::step_endpoint() {
  require(steps_ < instruction_limit,
          "Unchanged endpoint ISR/forwarder must terminate within its bound");
  submit();
  const auto before = machine_.cpu().state();
  require(before.sp >= stack_low && before.sp <= stack,
          "Native USB IRQ/callee stack remains within 22 bytes");
  const auto result = machine_.cpu().step();
  if (result.instruction_executed) {
    const auto pc = before.pc;
    require(
        (pc >= out_handler && pc < out_end) || (pc >= usb_sender && pc < usb_end) ||
            (pc >= strobe && pc < strobe_end),
        "Every actual retired instruction belongs to the unchanged original ISR/forwarder/strobe");
    ++steps_;
  }
  if (result.interrupt_vector) {
    require(*result.interrupt_vector == out_vector && machine_.cpu().state().pc == out_handler &&
                machine_.cpu().state().sp == stack - 9,
            "Actual OUT completion uses its original vector, handler and hardware frame");
    if (usb_irqs_ == 0) {
      require(!result.instruction_executed && !result.stack_reused && before.pc == foreground &&
                  before.sp == stack,
              "First actual OUT interrupt pushes the new literal foreground frame without retiring "
              "code");
    } else {
      require(result.instruction_executed && result.stack_reused && before.pc == out_end - 1 &&
                  before.sp == stack - 9,
              "Every queued next OUT follows the real original RTI and reuses its actual hardware "
              "frame");
      ++frame_reuses_;
    }
    ++usb_irqs_;
    check_usb_frame();
  }
}

bool Peer::endpoint_done() const {
  return cursor_ == source_.storage.size() && usb_irqs_ == packets &&
         machine_.cpu().state().pc == foreground && machine_.cpu().state().sp == stack &&
         !machine_.cpu().pending_interrupt();
}

void Peer::verify_endpoint() {
  auto &bus = machine_.bus();
  const auto state = machine_.cpu().state();
  // Four original ISR14 + four forwarder entry15 + 48 triple101 + 16 clock51
  // + four post/return8 + two alternate-phase4 = 5820 retired native instructions.
  constexpr unsigned expected_steps =
      (4 * 14) + (4 * 15) + (48 * 101) + (16 * 51) + (4 * 8) + (2 * 4);
  require(
      endpoint_done() && calls_ == packets && usb_irqs_ == packets &&
          frame_reuses_ == packets - 1 && steps_ == expected_steps &&
          transitions_ == 2 * source_.bytes.size() && naks_ > 0 && !data1_ &&
          bus.read(hc12::ddrq) == all_bits && state.d == initial_d && state.x == initial_x &&
          state.y == initial_y && state.ccr == initial_flags,
      "Actual IRQ/RTI restores every foreground register with independently counted native steps");
  require(bus.read(activity) == bus_activity && bus.read(midi_activity) == 4 &&
              bus.read(transfer_flags) == multibyte &&
              usb_host::word(bus, usb_host::output_control) == idle_control &&
              bus.usb_host_endpoint(usb_host::output_address).data1 == false,
          "Only original code releases the actual OUT status and preserves accepted PID/flags");
  const auto now = bus.power_off_flash_image();
  require(now.bytes == flash_.bytes && now.known == flash_.known && now.fresh == flash_.fresh &&
              bus.usb_host_evidence() == evidence_ &&
              bus.usb_host_configuration() == configuration_,
          "All NVM/history and actual USB link/configuration identity remain unchanged");
  for (unsigned address = ram_begin; address < ram_end; ++address) {
    if ((address >= stack_low && address < stack) || address == activity ||
        address == midi_activity || address == transfer_flags) {
      continue;
    }
    require(bus.read(static_cast<std::uint16_t>(address)) == before_.at(address - ram_begin),
            "No host or native code writes received payload into MCU RAM; only real IRQ/callee "
            "stack and flags change");
  }
  const auto observed_buffers = endpoint_buffers();
  for (unsigned buffer = 0; buffer < 4; ++buffer) {
    for (unsigned byte = 0; byte < packet_bytes; ++byte) {
      unsigned expected = before_buffers_.at((buffer * packet_bytes) + byte);
      if (buffer == 3) {
        expected =
            byte % 4 == 0 ? 0U : source_.storage[source_.storage.size() - packet_bytes + byte];
      }
      require(observed_buffers[(buffer * packet_bytes) + byte] == expected,
              "All USB local-buffer values/unknown states match independent untouched or final "
              "native-header-clear expectations");
    }
  }
}

void Peer::endpoint_result(Result &result) {
  result.buffers = endpoint_buffers();
  result.usb = {usb_host::word(machine_.bus(), usb_host::output_control),
                cursor_,
                static_cast<unsigned>(data1_),
                usb_irqs_,
                naks_,
                frame_reuses_};
}

std::vector<unsigned> Peer::endpoint_buffers() {
  std::vector<unsigned> result;
  auto copy = machine_.bus().clone();
  for (unsigned buffer = 0; buffer < 4; ++buffer) {
    copy->write(usb_host::buffer_select, static_cast<std::uint8_t>(buffer));
    for (unsigned byte = 0; byte < packet_bytes; ++byte) {
      unsigned value = unknown_byte;
      try {
        value = copy->read(static_cast<std::uint16_t>(usb_host::local + byte));
      } catch (const Fault &fault) {
        require(std::string_view(fault.what()).contains("Unknown USB endpoint-buffer byte"),
                "Only genuinely unknown buffer contents use the inspection sentinel");
      }
      result.push_back(value);
    }
  }
  return result;
}
}
