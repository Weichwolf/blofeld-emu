// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "firmware/firmware.hpp"
#include "gpio_notification_wire.hpp"
#include "gpio_parameter_caller.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_usb.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <ostream>
#include <span>
#include <vector>

namespace blofeld::test::gpio_sysex {
namespace pair_fixture = gpio_pair;
namespace host = usb_host;
using pair_fixture::Case;
using pair_fixture::OriginalGpioPair;
constexpr unsigned initial = 42, incoming = 7, sound_base = 0x936, result = 0x2500;
constexpr unsigned policy_address = 0x74c, device_address = 0x745, signature_address = 0x76c;
constexpr unsigned entries = 0x6d2, cache_cursor = 0x19fe, cache_begin = 0x1a12;
constexpr unsigned tx_ring = 0x18ae, tx_reader = 0x18ee, tx_writer = 0x18ef;
constexpr unsigned tx_cursor = 61, tx_bytes = 64, tx_mask = 63;
constexpr unsigned usb_ring = 0x18f2, usb_reader = 0x1972, usb_writer = 0x1973;
constexpr unsigned usb_cursor = 124, usb_ring_bytes = 128, usb_mask = 127;
constexpr unsigned endpoint_bytes = 64, idle_endpoint = 0x1000, cache_bytes = 40;
constexpr unsigned serial_vector = 0xffd6, usb_vector = 0xffca;
constexpr unsigned serial_handler = 0xb5c1, usb_handler = 0xbe6b;
constexpr unsigned sysex_sender = 0xd6ab, midi_sender = 0xb352, mapper = 0xd5e5;
constexpr unsigned uart_queue = 0xb555, usb_queue = 0xb3ed, store_pc = 0x83c;
constexpr unsigned serial_complete = 0xc0, tx_irq_enable = 0x80;
constexpr unsigned instruction_limit = 100000, baud_divisor = 60, selector = 19;
constexpr unsigned packet_bytes = 12, logical_bytes = 10, usb_payload = 16;
constexpr unsigned first_packet = 4, event_bytes = 4, uart_chunk = 3;
constexpr unsigned signature = 127, sentinel = 255, zero_flag = 4;
constexpr unsigned write_read_writes = 13, sysex_write_read_writes = 17;
constexpr unsigned sysex_start = 240, manufacturer = 62, model = 19, sound_change = 32,
                   sysex_end = 247;
constexpr unsigned policy_parameter = 44, device_parameter = 37;

struct Input {
  unsigned policy;
  unsigned parameter;
  unsigned device;

  [[nodiscard]] bool sends() const { return policy >= 2; }

  [[nodiscard]] Case sound(const unsigned value) const {
    return {.kind = 0,
            .location = 0,
            .parameter = parameter,
            .address = sound_base + parameter,
            .value = value};
  }
};

inline std::array<unsigned, logical_bytes> message(const Input input) {
  // Independent SNDP protocol fields; no manufacturer program bytes.
  return {sysex_start,
          manufacturer,
          model,
          input.device,
          sound_change,
          0,
          0,
          input.parameter,
          incoming,
          sysex_end};
}

struct Run {
  Input input;
  OriginalGpioPair pair;
  gpio_notification::Wire wire;
  std::array<unsigned, 2> spare{};
  std::vector<unsigned> raw, events;
  unsigned stores = 0, produced = 0, consumed = 0, source_checks = 0;
  unsigned send_calls = 0, mapper_calls = 0, route_calls = 0, uart_calls = 0, usb_calls = 0;
  unsigned sci_irqs = 0, usb_irqs = 0, packets = 0;
  bool readback = false;
  UsbHostEvidence identity;
  UsbConfigurationInfo configuration;

  Run(const Firmware &firmware, const Input config, std::ostream &trace)
      : input(config),
        pair(firmware, config.sound(initial), trace, pair_fixture::DspSetup::secondary_parser),
        wire(trace) {
    prepare();
    pair.additional_completion.observe = [this](const auto &step) { observe(step); };
  }

  unsigned read(const unsigned address) {
    return pair.dsp.bus().read({.space = DspSpace::x, .address = address});
  }

  [[nodiscard]] std::vector<Case> sources() const {
    std::vector<Case> expected{{.kind = 2,
                                .location = 0,
                                .parameter = policy_parameter,
                                .address = policy_address,
                                .value = input.policy}};
    if (input.sends()) {
      expected.push_back({.kind = 2,
                          .location = 0,
                          .parameter = device_parameter,
                          .address = device_address,
                          .value = input.device});
    }
    expected.push_back(input.sound(incoming));
    return expected;
  }

  void prepare() {
    for (const auto [address, value] :
         std::array<std::array<unsigned, 2>, 5>{{{pair_fixture::part_table, sound_base},
                                                 {policy_address, input.policy},
                                                 {device_address, input.device},
                                                 {entries, 0},
                                                 {entries + 1, 0}}}) {
      pair.dsp.bus().write({.space = DspSpace::x, .address = address, .value = value});
    }
    auto &bus = pair.mcu.bus();
    bus.write(cache_cursor, 0);
    for (unsigned n = 0; n < cache_bytes; ++n) {
      bus.write(static_cast<std::uint16_t>(cache_begin + n), 0);
    }
    bus.write(tx_reader, tx_cursor);
    bus.write(tx_writer, tx_cursor);
    for (unsigned n = 0; n < tx_bytes; ++n) {
      bus.write(static_cast<std::uint16_t>(tx_ring + n), pair_fixture::ram_canary);
    }
    bus.set_pin_drive(McuPort::r, {.mask = 4, .levels = 4});
    bus.write(hc12::sci_baud_high, 0);
    bus.write(hc12::sci_baud_low, baud_divisor);
    bus.write(hc12::sci_control1, 8);
    bus.write(hc12::sci_control2, 12);
    host::prepare(bus, endpoint_bytes);
    bus.write(usb_reader, usb_cursor);
    bus.write(usb_writer, usb_cursor);
    for (unsigned n = 0; n < usb_ring_bytes; ++n) {
      bus.write(static_cast<std::uint16_t>(usb_ring + n), pair_fixture::ram_canary);
    }
    bus.write_memory({.address = host::input_control, .size = 2, .value = idle_endpoint});
    identity = bus.usb_host_evidence();
    configuration = bus.usb_host_configuration();
  }

  void observe(const DspScheduledStep &step) {
    constexpr unsigned request_bytes = 5;
    const auto producer = read(pair_fixture::ring_write);
    if (producer != pair_fixture::input_ring + produced) {
      const std::array<unsigned, request_bytes> request{245, 0, input.parameter, incoming, 247};
      require(produced < request.size() && producer == pair_fixture::input_ring + produced + 1 &&
                  read(pair_fixture::input_ring + produced) == request.at(produced) << 16U,
              "Native GPIO producer must publish the actual original Sound frame");
      ++produced;
    }
    const auto consumer = read(pair_fixture::ring_read);
    if (consumer != pair_fixture::input_ring + consumed) {
      require(consumed < produced && consumer == pair_fixture::input_ring + consumed + 1,
              "Original parser must consume exactly its real published prefix");
      ++consumed;
    }
    if (step.plan.interrupt) { return; }
    const auto &cpu = step.plan.before.cpu;
    if (cpu.pc == store_pc) {
      require(stores == 0 && cpu.r[0] == sound_base && cpu.n[0] == input.parameter &&
                  ((cpu.a >> pair_fixture::word_bits) & pair_fixture::word_mask) == incoming &&
                  read(sound_base + input.parameter) == incoming && (cpu.sr & zero_flag) != 0,
              "Original store must accept the independently valid value");
      ++stores;
    }
    if (cpu.pc != pair_fixture::source_load) { return; }
    const auto expected = sources();
    require(source_checks < expected.size() && cpu.r[6] == expected.at(source_checks).address,
            "Original policy/DeviceID/readback request order differs");
    ++source_checks;
    if (source_checks < expected.size()) { pair.input = expected.at(source_checks); }
  }

  void install(const unsigned order, const std::vector<std::uint8_t> &code) {
    for (unsigned n = 0; n < code.size(); ++n) {
      pair.mcu.bus().write(static_cast<std::uint16_t>(pair_fixture::caller + n), code[n]);
    }
    pair.input = sources().front();
    pair.start(order == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp,
               order == 0 ? 0 : pair_fixture::phase_offset,
               pair_fixture::caller + gpio_parameter::caller_entry_bytes);
    pair.mcu.set_cycle_observer([this](const Hc12BusCycle cycle) {
      pair.on_cycle(cycle);
      wire.sample(pair.mcu.bus(), pair.mcu.run_clock().bus_cycles());
    });
  }

  void packet(const CpuState cpu) {
    require(input.sends() && route_calls == 0 && send_calls == 1 && (cpu.d & sentinel) == selector,
            "Actual original SysEx sender must select both UART and USB chunks");
    const auto pointer = host::word(pair.mcu.bus(), cpu.sp + 2);
    const auto prefix = message(input);
    std::vector<unsigned> bytes(prefix.begin(), prefix.end());
    bytes.insert(bytes.end(), spare.begin(), spare.end());
    for (unsigned n = 0; n < bytes.size(); ++n) {
      require(pair.mcu.bus().read(static_cast<std::uint16_t>(pointer + n)) == bytes[n],
              "Original packet must contain SNDP fields and unchanged caller-stack spare bytes");
      pair.trace << "PACKET_BYTE index=" << n << " value=" << bytes[n] << '\n';
    }
    for (unsigned offset = 0; offset < packet_bytes; offset += uart_chunk) {
      for (unsigned n = offset; n < offset + uart_chunk && bytes[n] != sentinel; ++n) {
        raw.push_back(bytes[n]);
      }
      events.push_back(offset == packet_bytes - uart_chunk ? 5U : 4U);
      events.insert(events.end(), bytes.begin() + offset, bytes.begin() + offset + uart_chunk);
    }
    ++route_calls;
  }

  void before_step() {
    const auto pending = pair.mcu.cpu().pending_interrupt();
    if (pending) {
      if (pending->vector == usb_vector) {
        ++usb_irqs;
      } else {
        require(pending->vector == serial_vector, "Only actual enabled SCI/USB sources may enter");
        ++sci_irqs;
      }
      return;
    }
    const auto cpu = pair.mcu.cpu().state();
    if (cpu.pc == sysex_sender) {
      require(input.sends() && send_calls == 0, "Only enabled original SysEx path may enter");
      for (unsigned n = 0; n < spare.size(); ++n) {
        spare[n] = pair.mcu.bus().read(static_cast<std::uint16_t>(cpu.sp - 2 + n));
        pair.trace << "SPARE index=" << logical_bytes + n << " value=" << spare[n] << '\n';
      }
      ++send_calls;
    }
    if (cpu.pc == midi_sender) { packet(cpu); }
    if (cpu.pc == mapper) { ++mapper_calls; }
    if (cpu.pc == uart_queue) { ++uart_calls; }
    if (cpu.pc == usb_queue) { ++usb_calls; }
  }

  void host_input() {
    if (!input.sends() || packets == 2 || !pair.done()) { return; }
    auto &bus = pair.mcu.bus();
    const auto response = bus.request_usb_in(UsbBuffer::endpoint2);
    if (response.response == UsbResponse::nak) { return; }
    const auto offset = packets == 0 ? 0U : first_packet;
    const auto size = packets == 0 ? first_packet : usb_payload - first_packet;
    require(response.response == UsbResponse::data && response.size == size &&
                response.data1 == (packets != 0) &&
                std::ranges::equal(std::span(response.data).first(size),
                                   std::span(events).subspan(offset, size)),
            "Native original SysEx chunks must reach actual DATA0/DATA1 endpoint packets");
    pair.trace << "USB_IN index=" << packets << " size=" << size << " data1=" << response.data1
               << '\n';
    for (unsigned n = 0; n < size; ++n) {
      pair.trace << "USB_BYTE packet=" << packets << " index=" << n
                 << " value=" << static_cast<unsigned>(response.data[n]) << '\n';
    }
    bus.finish_usb_in(true);
    pair.trace << "USB_ACK index=" << packets << '\n';
    ++packets;
  }

  void step() {
    const auto pending = pair.mcu.cpu().pending_interrupt();
    before_step();
    pair.step();
    if (pending) {
      require(pair.mcu.cpu().state().pc ==
                  (pending->vector == usb_vector ? usb_handler : serial_handler),
              "Native interrupt must enter the unchanged original handler");
    }
    if (!readback && pair.mcu.bus().read(result) != pair_fixture::ram_canary) {
      require(pair.done() && pair.mcu.bus().read(result) == incoming && stores == 1 &&
                  source_checks == sources().size() &&
                  pair.link.data_writes ==
                      (input.sends() ? sysex_write_read_writes : write_read_writes),
              "Original write/notification/readback must finish without host CPU repair");
      pair.trace << "RESULT value=" << incoming << '\n';
      readback = true;
    }
    host_input();
  }

  void execute() {
    for (unsigned n = 0; n < instruction_limit; ++n) {
      step();
      if (!readback || !pair.done() || packets != (input.sends() ? 2U : 0U) ||
          pair.dsp.cpu().state().pc != pair.foreground || pair.dsp.cpu().state().sp != 0 ||
          pair.mcu.cpu().pending_interrupt()) {
        continue;
      }
      auto &bus = pair.mcu.bus();
      if ((bus.read(hc12::sci_status1) & serial_complete) != serial_complete ||
          (bus.read(hc12::sci_control2) & tx_irq_enable) != 0) {
        continue;
      }
      require(wire.bytes() == raw && !wire.active(),
              "Whole native SCI output, including actual trailing stack bytes, must finish");
      return;
    }
    throw Fault("Original SysEx/disabled-notification path exceeded instruction budget");
  }

  void verify(const std::vector<std::uint8_t> &code) {
    auto &bus = pair.mcu.bus();
    const auto chunks = input.sends() ? 4U : 0U;
    require(stores == 1 && produced == 5 && consumed == produced &&
                send_calls == static_cast<unsigned>(input.sends()) && route_calls == send_calls &&
                uart_calls == chunks && usb_calls == chunks &&
                mapper_calls == static_cast<unsigned>((input.policy & 1U) != 0) &&
                usb_irqs == (input.sends() ? 2U : 0U) && (sci_irqs != 0) == input.sends() &&
                pair.link.unresolved_queries == 0 && read(signature_address) == signature &&
                read(device_address) == input.device && read(policy_address) == input.policy &&
                bus.read(cache_cursor) == 1,
            "Actual SysEx/disabled policy branches and retained original sources differ");
    require(bus.read(tx_reader) == ((tx_cursor + raw.size()) & tx_mask) &&
                bus.read(tx_writer) == bus.read(tx_reader) &&
                bus.read(usb_reader) == ((usb_cursor + events.size()) & usb_mask) &&
                bus.read(usb_writer) == bus.read(usb_reader) &&
                bus.usb_host_evidence() == identity &&
                bus.usb_host_configuration() == configuration,
            "Native UART/USB drains must retain ownership and endpoint identity");
    for (unsigned n = 0; n < tx_bytes; ++n) {
      const auto offset = (n + tx_bytes - tx_cursor) & tx_mask;
      const auto expected = offset < raw.size() ? raw.at(offset) : pair_fixture::ram_canary;
      require(bus.read(static_cast<std::uint16_t>(tx_ring + n)) == expected,
              "Actual UART ring bytes and neighboring slots differ");
    }
    for (unsigned n = 0; n < usb_ring_bytes; ++n) {
      const auto offset = (n + usb_ring_bytes - usb_cursor) & usb_mask;
      const auto expected = offset < events.size() ? events.at(offset) : pair_fixture::ram_canary;
      require(bus.read(static_cast<std::uint16_t>(usb_ring + n)) == expected,
              "Actual SysEx event ring must preserve native padding and neighboring slots");
    }
    for (unsigned n = 0; n < code.size(); ++n) {
      require(bus.read(static_cast<std::uint16_t>(pair_fixture::caller + n)) == code[n],
              "Own native caller changed");
    }
    for (unsigned n = 0; n < cache_bytes; ++n) {
      unsigned expected = 0;
      if (n == 5) { expected = 2; }
      if (n == 8) { expected = policy_parameter; }
      if (n == 9) { expected = input.policy; }
      require(bus.read(static_cast<std::uint16_t>(cache_begin + n)) == expected,
              "Actual policy getter must insert exactly its original raw key/value");
    }
    require(bus.request_usb_in(UsbBuffer::endpoint2).response == UsbResponse::nak,
            "Actually empty SysEx endpoint must return NAK");
    bool rejected = false;
    try {
      pair.reject_board_qualification();
    } catch (const Fault &) { rejected = true; }
    require(rejected, "SysEx success cannot qualify unresolved board voltage");
  }
};
} // namespace blofeld::test::gpio_sysex
