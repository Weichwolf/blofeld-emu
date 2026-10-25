// SPDX-License-Identifier: GPL-3.0-only
#include "original_usb_queue_fixture.hpp"

#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "storage/flash_image.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

namespace blofeld::test::usb_queue {
using namespace usb_host;

namespace {
constexpr CpuState initial{.pc = entry, .d = 0x3500, .x = 0x5678, .y = 0x9abc, .sp = stack};
constexpr unsigned qram_mapping = 0x21;
constexpr unsigned occupied = 0x5000;
constexpr unsigned idle = 0x1000;
constexpr unsigned maximum_instructions = 20000;
constexpr unsigned reset_clock_flags = 0xd0;
constexpr unsigned software_clear_x = 0xbf;
constexpr unsigned interrupt_masks = 0x50;

Registers registers(const CpuState state) {
  return {state.pc, state.d, state.x, state.y, state.sp, state.ccr};
}

bool original_code(const unsigned pc) {
  constexpr std::array<std::pair<unsigned, unsigned>, 5> spans{
      {{entry, 189}, {read_ccr, 3}, {0xb600, 104}, {usb_handler, 15}, {timer_handler, 15}}};
  return std::ranges::any_of(
      spans, [pc](const auto span) { return pc >= span.first && pc - span.first < span.second; });
}
}

Fixture::Fixture(const Firmware &firmware, const Input supplied)
    : machine(McuFlashImage::from_known(firmware.mcu), BLOFELD_DEFAULT_SLA), input(supplied) {
  auto &bus = machine.bus();
  bus.write(hc12::initee, qram_mapping);
  bus.retire_instruction();
  usb_host::prepare(bus, endpoint_bytes);
  for (unsigned i = 0; i < ram_bytes; ++i) {
    bus.write(static_cast<std::uint16_t>(ram_begin + i), canary);
  }
  const auto count =
      input.raw_alias ? 0 : ((input.write + ring_bytes - input.read) % ring_bytes) / event_bytes;
  for (unsigned i = 0; i < count; ++i) {
    constexpr unsigned first_note = 16;
    const std::array<std::uint8_t, 4> event{8, 0x80, static_cast<std::uint8_t>(first_note + i), 40};
    prior.insert(prior.end(), event.begin(), event.end());
    for (unsigned byte = 0; byte < event_bytes; ++byte) {
      bus.write(
          static_cast<std::uint16_t>(ring + ((input.read + i * event_bytes + byte) % ring_bytes)),
          event[byte]);
    }
  }
  bus.write(reader, static_cast<std::uint8_t>(input.read));
  bus.write(writer, static_cast<std::uint8_t>(input.write));
  bus.write(tick, initial_tick);
  bus.write(scan_flags, 1); // Declared scan inhibition, not a successful scan stub.
  for (unsigned i = 0; i < input.message.data.size(); ++i) {
    bus.write(static_cast<std::uint16_t>(source + i), input.message.data[i]);
  }
  bus.write_memory({.address = stack, .size = 2, .value = returned});
  bus.write_memory({.address = stack + 2, .size = 2, .value = source});
  bus.write(buffer_select, 2);
  for (unsigned i = 0; i < endpoint_bytes; ++i) {
    bus.write(static_cast<std::uint16_t>(local + i), canary);
  }
  bus.write_memory({.address = input_control, .size = 2, .value = input.busy ? occupied : idle});
  if (input.busy) {
    const auto pending = bus.request_usb_in(UsbBuffer::endpoint2);
    require(pending.response == UsbResponse::data && pending.size == 0,
            "Only a real own previous IN remains awaiting ACK during queue execution");
  }
  if (input.timer) {
    bus.write(hc12::crg_flags, reset_clock_flags);
    bus.write(hc12::rti_control, 0x10);
    bus.write(hc12::crg_interrupts, 0x80);
    require(bus.oscillator_cycles_until_rti() == timer_period,
            "RTICTL0x10 has the independently documented1024-oscillator period");
  }
  auto state = initial;
  state.d = static_cast<std::uint16_t>(state.d | input.flags);
  state.ccr = static_cast<std::uint8_t>(input.ccr);
  machine.cpu().set_state(state);
  for (unsigned i = 0; i < ram_bytes; ++i) {
    before_ram_.push_back(bus.read(static_cast<std::uint16_t>(ram_begin + i)));
  }
  before_flash_ = bus.power_off_flash_image();
  evidence_ = bus.usb_host_evidence();
  configuration_ = bus.usb_host_configuration();
  observe();
}

void Fixture::observe() {
  machine.bus().set_observer([this](const BusEvent &event) {
    require(event.write && (event.address == buffer_select || event.address == input_control ||
                            event.address == input_control + 1 ||
                            (event.address >= local && event.address < local + endpoint_bytes) ||
                            (input.timer && event.address == hc12::crg_flags)),
            "Original queue/callees write only their qualified peripheral registers");
    stats.writes.emplace_back(event.address, event.value);
  });
}

Bytes Fixture::record() const {
  return {
      input.message.header, input.message.data[0], input.message.data[1], input.message.data[2]};
}

void Fixture::stage() {
  const auto bytes = record();
  for (unsigned i = 0; i < bytes.size(); ++i) {
    require(machine.bus().read(static_cast<std::uint16_t>(ring + input.write + i)) == bytes[i],
            "All four original record stores precede capacity checking and publication");
  }
  const auto flags = input.message.header >= 5 && input.message.header <= 7 ? 0 : input.flags;
  require(machine.bus().read(stack - 4) == flags,
          "Only headers5..7 clear every saved software flag before the native wait decision");
  staged_ = true;
}

void Fixture::step() {
  auto &cpu = machine.cpu();
  const auto before = cpu.state();
  require(stats.instructions < maximum_instructions && before.sp >= stack_low &&
              before.sp <= stack + 2,
          "Original queue, nested IRQ and real callees remain within declared instruction/stack "
          "bounds");
  if (before.pc == staged_pc) { stage(); }
  const auto result = cpu.step();
  if (result.instruction_executed) {
    if (before.pc == read_ccr) { ++stats.ccr_calls; }
    require(original_code(before.pc),
            "Only hash-pinned original routines execute; no successful callee stubs");
    ++stats.instructions;
  }
  if (result.interrupt_vector) {
    if (*result.interrupt_vector == usb_vector) {
      ++stats.usb_irqs;
    } else {
      require(input.timer && *result.interrupt_vector == timer_vector,
              "Only an actually clocked original RTI source may interrupt besides MIDI IN");
      ++stats.timer_irqs;
    }
  }
}

void Fixture::run() {
  while (machine.cpu().state().pc != returned) { step(); }
}

void Fixture::until(const unsigned pc, const bool advance) {
  if (advance) { step(); }
  while (machine.cpu().state().pc != pc) {
    require(machine.cpu().state().pc != returned,
            "Original queue cannot return before its observed boundary");
    step();
  }
}

void Fixture::service(const unsigned vector) {
  const auto before = machine.cpu().state();
  const auto pending = machine.cpu().pending_interrupt();
  require(pending && pending->vector == vector,
          "Real peripheral state selects the independently expected vector");
  step();
  require(machine.cpu().state().pc == (vector == usb_vector ? usb_handler : timer_handler),
          "Actual native entry selects the original handler address");
  until(before.pc);
  auto expected = before;
  if (vector == usb_vector) {
    expected.ccr = static_cast<std::uint8_t>(expected.ccr & software_clear_x);
  }
  require(registers(machine.cpu().state()) == registers(expected) &&
              !machine.cpu().pending_interrupt(),
          "Original IRQ/callee/native RTI restore every register and flags except the helper's X "
          "clear");
}

void Fixture::acknowledge() {
  machine.bus().finish_usb_in(true);
  next_pid_ = !next_pid_;
}

void Fixture::drain(const std::span<const std::uint8_t> expected) {
  for (std::size_t offset = 0; offset < expected.size();) {
    const auto count = std::min<std::size_t>(endpoint_bytes, expected.size() - offset);
    const auto packet = machine.bus().request_usb_in(UsbBuffer::endpoint2);
    require(packet.response == UsbResponse::data && packet.size == count &&
                packet.data1 == next_pid_ &&
                std::ranges::equal(std::span(packet.data).first(count),
                                   expected.subspan(offset, count)),
            "Actual original queue/submitter data,64-byte batches and ACK-owned DATA PID match");
    acknowledge();
    service(usb_vector);
    offset += count;
  }
  require(machine.bus().request_usb_in(UsbBuffer::endpoint2).response == UsbResponse::nak,
          "A drained original ring ends in actual NAK without an invented transfer completion");
}

void Fixture::check_return(const ExpectedReturn expected) {
  const auto state = machine.cpu().state();
  require(staged_ && state.pc == returned && state.sp == stack + 2 &&
              (state.d & byte_mask) == expected.result &&
              (state.ccr & interrupt_masks) == expected.masks &&
              machine.bus().read(reader) == expected.read &&
              machine.bus().read(writer) == expected.write,
          "Original raw header/failure, ABI return, interrupt masks and independent ring cursors "
          "match");
  guard();
}

void Fixture::guard() const {
  const auto &bus = machine.bus();
  const auto flash = bus.power_off_flash_image();
  require(
      flash.bytes == before_flash_.bytes && flash.known == before_flash_.known &&
          flash.fresh == before_flash_.fresh,
      "Original queue and all real callees preserve every NVM byte/knowledge/erase-history bit");
  require(bus.usb_host_evidence() == evidence_ && bus.usb_host_configuration() == configuration_,
          "Native queue execution preserves qualified link/configuration identity");
  auto copy = bus.clone();
  for (unsigned i = 0; i < ram_bytes; ++i) {
    const auto address = ram_begin + i;
    const bool permitted =
        (address >= stack_low && address < stack + 2) || address == reader || address == writer ||
        (input.timer && address == tick) ||
        (address >= ring + input.write && address < ring + input.write + event_bytes);
    if (!permitted) {
      require(copy->read(static_cast<std::uint16_t>(address)) == before_ram_[i],
              "Caller packet, pre-existing events and all unrelated mapped RAM remain unchanged");
    }
  }
}

Saved Fixture::save() const {
  return {
      .machine = machine.checkpoint(), .stats = stats, .staged = staged_, .next_pid = next_pid_};
}

void Fixture::restore(const Saved &saved) {
  machine.restore(saved.machine);
  stats = saved.stats;
  staged_ = saved.staged;
  next_pid_ = saved.next_pid;
  observe();
}

Outcome Fixture::outcome() {
  Outcome value{.cpu = registers(machine.cpu().state()),
                .ram = {},
                .buffer = {},
                .control = word(machine.bus(), input_control),
                .host_pid = next_pid_,
                .device_pid = machine.bus().usb_host_endpoint(input_address).data1,
                .stats = stats};
  for (unsigned i = 0; i < ram_bytes; ++i) {
    value.ram.push_back(machine.bus().read(static_cast<std::uint16_t>(ram_begin + i)));
  }
  auto copy = machine.bus().clone();
  copy->write(buffer_select, 2);
  for (unsigned i = 0; i < endpoint_bytes; ++i) {
    value.buffer.push_back(copy->read(static_cast<std::uint16_t>(local + i)));
  }
  return value;
}
}
