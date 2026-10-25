// SPDX-License-Identifier: GPL-3.0-only
#include "original_midi_forward_fixture.hpp"

#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_usb.hpp"
#include "storage/flash_image.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

namespace blofeld::test::midi_forward {
using namespace usb_host;

namespace {
constexpr unsigned idle = 0x1000;
constexpr unsigned occupied = 0x5000;
constexpr unsigned usb_handler = 0xbe6b;
constexpr unsigned sci_handler = 0xb5c1;
constexpr unsigned maximum_instructions = 20000;
constexpr unsigned clear_x = 0xbf;
constexpr unsigned uart_queue_entry = 0xb555;
constexpr unsigned sender_entry = 0xb600;
constexpr unsigned qram_mapping = 0x21;
constexpr unsigned usb_byte_mask = 0x7f;
constexpr CpuState initial{.pc = entry, .d = 0x3526, .x = 0x5678, .y = 0x9abc, .sp = stack};

std::array<unsigned, 6> registers(const CpuState state) {
  return {state.pc, state.d, state.x, state.y, state.sp, state.ccr};
}

bool original_code(const unsigned pc, const bool peer) {
  constexpr std::array<std::pair<unsigned, unsigned>, 5> spans{
      {{entry, 137}, {queue_entry, 189}, {0xb34f, 3}, {0xb600, 104}, {usb_handler, 15}}};
  return (peer && pc >= uart_queue_entry && pc < sender_entry) ||
         std::ranges::any_of(spans, [pc](const auto span) {
           return pc >= span.first && pc - span.first < span.second;
         });
}
}

Fixture::Fixture(const Firmware &firmware, Input input)
    : machine_(McuFlashImage::from_known(firmware.mcu), BLOFELD_DEFAULT_SLA),
      input_(std::move(input)) {
  prepare();
  for (unsigned offset = 0; offset < ram_bytes; ++offset) {
    before_ram_.push_back(machine_.bus().read(static_cast<std::uint16_t>(ram_begin + offset)));
  }
  before_flash_ = machine_.bus().power_off_flash_image();
  evidence_ = machine_.bus().usb_host_evidence();
  configuration_ = machine_.bus().usb_host_configuration();
  observe();
}

void Fixture::prepare() {
  auto &bus = machine_.bus();
  bus.write(hc12::initee, qram_mapping);
  bus.retire_instruction();
  usb_host::prepare(bus, endpoint_bytes);
  for (unsigned offset = 0; offset < ram_bytes; ++offset) {
    bus.write(static_cast<std::uint16_t>(ram_begin + offset), canary);
  }
  bus.write(tx_reader, static_cast<std::uint8_t>(input_.peer ? input_.cursor : independent_reader));
  bus.write(tx_writer, static_cast<std::uint8_t>(input_.cursor));
  bus.write(mirror_reader, static_cast<std::uint8_t>(input_.cursor));
  bus.write(peer_reader, peer_cursor);
  bus.write(peer_writer, peer_cursor);
  for (unsigned index = 0; index < input_.bytes.size(); ++index) {
    const auto address = input_.peer ? peer_ring + ((peer_cursor + index) & ring_mask)
                                     : tx_ring + ((input_.cursor + index) & ring_mask);
    bus.write(static_cast<std::uint16_t>(address), input_.bytes[index]);
  }
  if (input_.raw_alias) {
    require(!input_.bytes.empty() && input_.cursor == ring_mask + 1,
            "Only the explicit raw-reader control aliases the SCI consumer byte");
    bus.write(tx_reader, input_.bytes[0]);
    bus.write(tx_ring, canary); // A normalized first read would see a different status.
  }
  const auto producer = (input_.cursor + input_.bytes.size()) & ring_mask;
  bus.write(input_.peer ? peer_writer : tx_writer,
            static_cast<std::uint8_t>(input_.peer ? (peer_cursor + input_.bytes.size()) & ring_mask
                                                  : producer));
  const auto read = (input_.usb_cursor + (input_.full ? 4 : 0)) & usb_mask;
  bus.write(usb_reader, static_cast<std::uint8_t>(read));
  bus.write(usb_writer, static_cast<std::uint8_t>(input_.usb_cursor));
  if (input_.full) {
    constexpr unsigned old_events = 31;
    for (unsigned index = 0; index < old_events; ++index) {
      const Event event{8, 0x80, static_cast<std::uint8_t>(index + 16), 40};
      for (unsigned byte = 0; byte < event.size(); ++byte) {
        bus.write(
            static_cast<std::uint16_t>(usb_ring + ((read + index * 4 + byte) & usb_byte_mask)),
            event[byte]);
        prior_.push_back(event[byte]);
      }
    }
  }
  bus.write(buffer_select, 2);
  for (unsigned index = 0; index < endpoint_bytes; ++index) {
    bus.write(static_cast<std::uint16_t>(local + index), canary);
  }
  bus.write_memory({.address = input_control, .size = 2, .value = input_.busy ? occupied : idle});
  if (input_.busy) {
    const auto packet = bus.request_usb_in(UsbBuffer::endpoint2);
    require(packet.response == UsbResponse::data && packet.size == 0,
            "An actual own earlier IN remains pending until host ACK");
  }
  bus.write_memory({.address = stack, .size = 2, .value = returned});
  auto state = initial;
  state.ccr = static_cast<std::uint8_t>(input_.ccr);
  machine_.cpu().set_state(state);
}

void Fixture::observe() {
  machine_.bus().set_observer([this](const BusEvent &event) {
    const bool usb = event.address == buffer_select || event.address == input_control ||
                     event.address == input_control + 1 ||
                     (event.address >= local && event.address < local + endpoint_bytes);
    const bool sci =
        input_.peer && (event.address == hc12::sci_control2 || event.address == hc12::sci_data_low);
    require(event.write && (usb || sci),
            "The original forwarding service and actual callees touch only qualified MMIO");
    stats_.writes.emplace_back(event.address, event.value);
  });
}

void Fixture::before_step() {
  const auto state = machine_.cpu().state();
  require(stats_.instructions < maximum_instructions && state.sp >= stack - stack_bytes &&
              state.sp <= stack + 2,
          "Real forwarding, callees and nested IRQs retain bounded execution and stack use");
  if (state.pc == queue_entry && !machine_.cpu().pending_interrupt()) {
    if (!input_.event) {
      throw Fault("Unexpected original USB queue call without a complete message");
    }
    require(state.sp == stack - 8 && (state.d & byte_mask) == 0 &&
                word(machine_.bus(), state.sp + 2) == stack - 3,
            "The real service supplies its actual packet pointer and literal zero queue flags");
    for (unsigned index = 0; index < 3; ++index) {
      require(machine_.bus().read(static_cast<std::uint16_t>(stack - 3 + index)) ==
                  (*input_.event)[index + 1],
              "The caller packet includes independently expected data and untouched C/D padding");
    }
  }
  if (state.pc == publish_pc && !machine_.cpu().pending_interrupt()) {
    require(machine_.bus().read(mirror_reader) == input_.cursor,
            "The service does not publish mirror consumption before its real USB callee returns");
  }
}

void Fixture::step() {
  before_step();
  const auto pc = machine_.cpu().state().pc;
  const auto result = machine_.cpu().step();
  if (result.instruction_executed) {
    require(original_code(pc, input_.peer), "Only pinned original routines execute");
    ++stats_.instructions;
    if (pc == queue_entry) { ++stats_.usb_calls; }
    if (pc == uart_queue_entry) { ++stats_.uart_calls; }
    if (pc == publish_pc) { ++stats_.publications; }
  }
  if (result.interrupt_vector) {
    if (*result.interrupt_vector == usb_vector) {
      ++stats_.usb_irqs;
    } else {
      require(input_.peer && *result.interrupt_vector == sci_vector &&
                  machine_.cpu().state().pc == sci_handler,
              "Only the real SCI TX source may additionally interrupt peer-ring forwarding");
      ++stats_.sci_irqs;
    }
  }
}

void Fixture::run() {
  while (machine_.cpu().state().pc != returned) { step(); }
}

void Fixture::until(const unsigned pc, const bool advance) {
  if (advance) { step(); }
  while (machine_.cpu().state().pc != pc) {
    require(machine_.cpu().state().pc != returned,
            "The native service reaches its declared boundary");
    step();
  }
}

void Fixture::acknowledge() {
  machine_.bus().finish_usb_in(true);
  next_pid_ = !next_pid_;
}

void Fixture::service_usb() {
  const auto before = machine_.cpu().state();
  const auto pending = machine_.cpu().pending_interrupt();
  require(pending && pending->vector == usb_vector,
          "Actual host ACK selects the original MIDI IRQ");
  step();
  require(machine_.cpu().state().pc == usb_handler, "Actual native entry reaches the original ISR");
  until(before.pc);
  auto expected = before;
  expected.ccr = static_cast<std::uint8_t>(expected.ccr & clear_x);
  require(registers(machine_.cpu().state()) == registers(expected),
          "The original USB ISR, sender and native RTI retain the whole interrupted context");
}

void Fixture::drain(const std::span<const std::uint8_t> expected) {
  for (std::size_t offset = 0; offset < expected.size();) {
    const auto count = std::min<std::size_t>(endpoint_bytes, expected.size() - offset);
    const auto packet = machine_.bus().request_usb_in(UsbBuffer::endpoint2);
    require(packet.response == UsbResponse::data && packet.size == count &&
                packet.data1 == next_pid_ &&
                std::ranges::equal(std::span(packet.data).first(count),
                                   expected.subspan(offset, count)),
            "Actual original forwarding and queue produce independent USB batches and DATA PID");
    acknowledge();
    service_usb();
    offset += count;
  }
  require(machine_.bus().request_usb_in(UsbBuffer::endpoint2).response == UsbResponse::nak,
          "The real emptied ring ends in NAK without an invented reply");
}

void Fixture::check_return(const unsigned expected_cursor) const {
  const auto state = machine_.cpu().state();
  auto copy = machine_.bus().clone();
  require(state.pc == returned && state.sp == stack + 2 && word(*copy, stack) == returned &&
              copy->read(mirror_reader) == expected_cursor &&
              stats_.usb_calls == static_cast<unsigned>(input_.event.has_value()),
          "One original service call returns and publishes independently expected mirror progress");
  const auto expected_reader = input_.raw_alias ? input_.bytes[0] : independent_reader;
  if (!input_.peer) {
    require(copy->read(tx_reader) == expected_reader,
            "USB mirroring never consumes the independent SCI TX reader");
  }
  guard();
}

void Fixture::guard() const {
  const auto &bus = machine_.bus();
  const auto flash = bus.power_off_flash_image();
  require(flash.bytes == before_flash_.bytes && flash.known == before_flash_.known &&
              flash.fresh == before_flash_.fresh && bus.usb_host_evidence() == evidence_ &&
              bus.usb_host_configuration() == configuration_,
          "Real MIDI forwarding preserves all NVM bytes/knowledge/history and USB identity");
  auto copy = bus.clone();
  for (unsigned offset = 0; offset < ram_bytes; ++offset) {
    const auto address = ram_begin + offset;
    const bool tx_slot =
        input_.peer && address >= tx_ring && address < tx_ring + 64 &&
        ((address - tx_ring + 64 - input_.cursor) & ring_mask) < input_.bytes.size();
    const bool mutable_peer = input_.peer && (address == tx_reader || address == tx_writer ||
                                              address == peer_reader || tx_slot);
    const bool permitted =
        mutable_peer || address == mirror_reader || address == usb_reader ||
        address == usb_writer || (address >= stack - stack_bytes && address < stack + 2) ||
        (address >= usb_ring + input_.usb_cursor && address < usb_ring + input_.usb_cursor + 4);
    if (!permitted) {
      require(copy->read(static_cast<std::uint16_t>(address)) == before_ram_[offset],
              "Unrelated RAM, peer source, prior USB events and caller bytes remain unchanged");
    }
  }
}

Saved Fixture::save() const {
  return {.machine = machine_.checkpoint(), .stats = stats_, .next_pid = next_pid_};
}

void Fixture::restore(const Saved &saved) {
  machine_.restore(saved.machine);
  stats_ = saved.stats;
  next_pid_ = saved.next_pid;
  observe();
}

Outcome Fixture::outcome() const {
  auto copy = machine_.bus().clone();
  Outcome value{.cpu = registers(machine_.cpu().state()),
                .ram = {},
                .usb_buffer = {},
                .sci = {},
                .control = word(*copy, input_control),
                .host_pid = next_pid_,
                .device_pid = machine_.bus().usb_host_endpoint(input_address).data1,
                .stats = stats_};
  for (unsigned index = 0; index < ram_bytes; ++index) {
    value.ram.push_back(copy->read(static_cast<std::uint16_t>(ram_begin + index)));
  }
  copy->write(buffer_select, 2);
  for (unsigned index = 0; index < endpoint_bytes; ++index) {
    value.usb_buffer.push_back(copy->read(static_cast<std::uint16_t>(local + index)));
  }
  for (unsigned index = 0; index < value.sci.size(); ++index) {
    value.sci[index] = copy->read(static_cast<std::uint16_t>(hc12::sci_baud_high + index));
  }
  return value;
}
}
