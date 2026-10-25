// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "gpio_parameter_caller.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "original_gpio_notification_fixture.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <span>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::gpio_notification;
namespace host = blofeld::test::usb_host;
constexpr unsigned mirror_entry = 0xb4aa, mirror_cursor = 0x18f0;
constexpr unsigned peer_writer = 0x19b4, peer_reader = 0x19b5;
constexpr unsigned usb_ring = 0x18f2, usb_reader = 0x1972, usb_writer = 0x1973;
constexpr unsigned usb_ring_bytes = 128, usb_initial = 124, usb_mask = 127;
constexpr unsigned event_bytes = 4, usb_bytes = 12, endpoint_bytes = 64, idle_control = 0x1000;
constexpr unsigned own_branch_bytes = 2;
constexpr std::array<std::uint8_t, 3> mirror_call{0x16, 0xb4, 0xaa};
constexpr std::array<std::uint8_t, 2> parking_loop{0x20, 0xfe};
constexpr std::uint8_t control_change = 0xb0, control_change_cin = 11;

std::array<unsigned, 6> registers(const CpuState state) {
  return {state.pc, state.d, state.x, state.y, state.sp, state.ccr};
}

std::vector<std::uint8_t> program(const Input input) {
  const auto values = input.values();
  auto code = blofeld::test::gpio_parameter::caller_program(input.sound(), values, result_base);
  code.resize(code.size() - own_branch_bytes);
  for (unsigned n = 0; n < values.size(); ++n) {
    // Own JSRs call the real main-loop forwarding service; no function PC is forced.
    code.insert(code.end(), mirror_call.begin(), mirror_call.end());
  }
  code.insert(code.end(), parking_loop.begin(), parking_loop.end());
  return code;
}

struct Host {
  Run &run;
  std::vector<std::uint8_t> expected;
  unsigned packets = 0, retries = 0, mirror_calls = 0;
  UsbHostEvidence identity;
  UsbConfigurationInfo configuration;

  explicit Host(Run &context) : run(context) {
    for (const auto value : run.input.values()) {
      expected.insert(expected.end(),
                      {control_change_cin,
                       control_change,
                       static_cast<std::uint8_t>(run.input.controller()),
                       static_cast<std::uint8_t>(value)});
    }
    auto &bus = run.pair.mcu.bus();
    host::prepare(bus, endpoint_bytes);
    bus.write(mirror_cursor, cursor);
    bus.write(peer_writer, 0);
    bus.write(peer_reader, 0);
    bus.write(usb_reader, usb_initial);
    bus.write(usb_writer, usb_initial);
    for (unsigned n = 0; n < usb_ring_bytes; ++n) {
      bus.write(static_cast<std::uint16_t>(usb_ring + n), ram_canary);
    }
    bus.write_memory({.address = host::input_control, .size = 2, .value = idle_control});
    identity = bus.usb_host_evidence();
    configuration = bus.usb_host_configuration();
    run.usb_enabled = true;
  }

  void check_packet(const UsbInPacket &packet, const unsigned attempt) {
    const auto offset = packets == 0 ? 0U : event_bytes;
    const auto size = packets == 0 ? event_bytes : usb_bytes - event_bytes;
    require(packet.response == UsbResponse::data && packet.size == size &&
                packet.data1 == (packets != 0) &&
                std::ranges::equal(std::span(packet.data).first(size),
                                   std::span(expected).subspan(offset, size)),
            "Original mirror and endpoint must produce the independent CC event/PID sequence");
    run.pair.trace << "USB_IN index=" << packets << " attempt=" << attempt
                   << " size=" << packet.size << " data1=" << packet.data1 << '\n';
    for (unsigned n = 0; n < packet.size; ++n) {
      run.pair.trace << "USB_BYTE packet=" << packets << " attempt=" << attempt << " index=" << n
                     << " value=" << static_cast<unsigned>(packet.data[n]) << '\n';
    }
  }

  void lost_ack() {
    auto &bus = run.pair.mcu.bus();
    const auto before = registers(run.pair.mcu.cpu().state());
    const auto ticks = run.pair.mcu.run_clock().elapsed_ticks();
    const auto read = bus.read(usb_reader);
    const auto write = bus.read(usb_writer);
    const auto status = host::word(bus, host::input_control);
    bus.finish_usb_in(false);
    const auto pending = run.pair.mcu.cpu().pending_interrupt();
    require(registers(run.pair.mcu.cpu().state()) == before &&
                run.pair.mcu.run_clock().elapsed_ticks() == ticks && bus.read(usb_reader) == read &&
                bus.read(usb_writer) == write && host::word(bus, host::input_control) == status &&
                (!pending || pending->vector != usb_vector),
            "Missing host ACK must preserve endpoint/ring/CPU/time and not invent USB completion");
    check_packet(bus.request_usb_in(UsbBuffer::endpoint2), 1);
    ++retries;
  }

  void poll() {
    if (packets == 2 || !run.pair.done()) { return; }
    auto &bus = run.pair.mcu.bus();
    const auto packet = bus.request_usb_in(UsbBuffer::endpoint2);
    if (packet.response == UsbResponse::nak) { return; }
    check_packet(packet, 0);
    if (packets == 0) { lost_ack(); }
    bus.finish_usb_in(true);
    run.pair.trace << "USB_ACK index=" << packets << '\n';
    ++packets;
  }

  void step() {
    const auto pending = run.pair.mcu.cpu().pending_interrupt();
    if (!pending && run.pair.mcu.cpu().state().pc == mirror_entry) { ++mirror_calls; }
    run.before_step();
    run.pair.step();
    if (pending) {
      require(run.pair.mcu.cpu().state().pc ==
                  (pending->vector == usb_vector ? usb_handler : serial_handler),
              "Actual enabled interrupt vector must enter the original SCI/USB handler");
    }
    run.readback();
    poll();
  }

  void execute() {
    for (unsigned n = 0; n < maximum_instructions; ++n) {
      step();
      if (packets != 2 || run.done != run.input.values().size() || !run.pair.done() ||
          run.pair.dsp.cpu().state().pc != run.pair.foreground ||
          run.pair.dsp.cpu().state().sp != 0 || run.pair.mcu.cpu().pending_interrupt()) {
        continue;
      }
      auto &bus = run.pair.mcu.bus();
      if ((bus.read(hc12::sci_status1) & complete_status) != complete_status ||
          (bus.read(hc12::sci_control2) & irq_enable) != 0) {
        continue;
      }
      require(!run.wire.active() && run.wire.bytes().size() == byte_count,
              "USB forwarding must retain the whole original SCI stream and last stop bit");
      return;
    }
    throw Fault("Original USB notification/mirror/ACK/serial path exceeded instruction budget");
  }

  void verify() {
    auto &bus = run.pair.mcu.bus();
    require(packets == 2 && retries == 1 && mirror_calls == 3 && run.usb_calls == 3 &&
                run.usb_irqs == 2 &&
                bus.read(mirror_cursor) == ((cursor + byte_count) & ring_mask) &&
                bus.read(usb_reader) == ((usb_initial + usb_bytes) & usb_mask) &&
                bus.read(usb_writer) == bus.read(usb_reader) && bus.read(peer_writer) == 0 &&
                bus.read(peer_reader) == 0 && bus.usb_host_evidence() == identity &&
                bus.usb_host_configuration() == configuration,
            "Original mirror cursor, genuine ACK/IRQ drain and retained endpoint identity differ");
    for (unsigned n = 0; n < usb_ring_bytes; ++n) {
      const auto relative = (n + usb_ring_bytes - usb_initial) & usb_mask;
      const auto value = relative < usb_bytes ? expected.at(relative) : ram_canary;
      require(bus.read(static_cast<std::uint16_t>(usb_ring + n)) == value,
              "Actual USB event producer must wrap without touching unrelated ring cells");
    }
    require(bus.request_usb_in(UsbBuffer::endpoint2).response == UsbResponse::nak,
            "A genuinely drained original endpoint must NAK without a fabricated final packet");
    run.pair.trace << "USB_FINAL_NAK value=1\n";
  }
};

int execute(const int argc, char **argv) {
  require(argc == 4, "USB notification test needs original archive/report/trace");
  const auto firmware = Firmware::load(argv[1]);
  std::ofstream report(argv[2]);
  std::ofstream trace(argv[3]);
  require(report.is_open() && trace.is_open(), "USB notification outputs must open");
  report.exceptions(std::ios::badbit | std::ios::failbit);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  for (const bool reject : {false, true}) {
    for (const unsigned order : {0U, 1U}) {
      const Input input{.policy = 3, .reject = reject};
      const auto code = program(input);
      trace << "CASE reject=" << reject << " order=" << order << '\n';
      Run run(firmware, input, trace);
      Host host(run);
      const auto nvm = run.pair.mcu.bus().power_off_flash_image();
      run.pair.verify_retained(firmware, nvm);
      run.install(code, order);
      host.execute();
      host.verify();
      run.verify(code);
      run.pair.verify_retained(firmware, nvm);
      report << "CASE reject=" << reject << " order=" << order << " stores=" << run.stores
             << " readbacks=" << run.done << " notifications=" << run.sender_calls
             << " serial-bytes=" << run.wire.bytes().size() << " sci-irqs=" << run.sci_irqs
             << " usb-irqs=" << run.usb_irqs << " usb-events=" << run.usb_calls
             << " usb-packets=" << host.packets << " retries=" << host.retries
             << " mirror-calls=" << host.mirror_calls << " policy-reads=" << run.policy_reads
             << " queries=" << run.pair.link.pad_queries
             << " unknown-queries=" << run.pair.link.unresolved_queries
             << " unresolved-events=" << run.pair.link.unresolved_events
             << " program-unchanged=1 nvm-unchanged=1 caller-unchanged=1 cache-valid=1 "
                "rings-valid=1 identity-valid=1 final-nak=1 board-rejected=1\n";
    }
  }
  return 0;
}
}

int main(const int argc, char **argv) {
  try {
    return execute(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
