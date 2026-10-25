// SPDX-License-Identifier: GPL-3.0-only
#include "original_midi_dispatch_fixture.hpp"

#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "original_midi_forward_fixture.hpp"
#include "storage/flash_image.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <utility>

namespace blofeld::test::midi_dispatch {
namespace mf = midi_forward;
using namespace usb_host;

namespace {
constexpr unsigned uart_entry = 0xb555;
constexpr unsigned sender = 0xb600;
constexpr unsigned ccr_reader = 0xb34f;
constexpr unsigned usb_handler = 0xbe6b;
constexpr unsigned qram = 0x21;
constexpr unsigned idle = 0x1000;
constexpr unsigned maximum_instructions = 20000;
constexpr unsigned stack_bytes = 48;
constexpr unsigned enabled = 0xac;
constexpr unsigned clear_x = 0xbf;
constexpr unsigned usb_byte_mask = 0x7f;
constexpr unsigned interrupt_masks = 0x50;
constexpr unsigned initial_x_mask = 0x40;
constexpr CpuState initial{
    .pc = entry, .d = 0x3500, .x = 0x5678, .y = 0x9abc, .sp = mf::stack, .ccr = 0x40};

std::array<unsigned, 6> registers(const CpuState value) {
  return {value.pc, value.d, value.x, value.y, value.sp, value.ccr};
}

bool original(const unsigned pc) {
  constexpr std::array<std::pair<unsigned, unsigned>, 6> spans{{{entry, 155},
                                                                {mf::entry, 137},
                                                                {mf::queue_entry, 189},
                                                                {uart_entry, 171},
                                                                {sender, 104},
                                                                {usb_handler, 15}}};
  return (pc >= ccr_reader && pc < ccr_reader + 3) ||
         std::ranges::any_of(spans, [pc](const auto pair) {
           return pc >= pair.first && pc - pair.first < pair.second;
         });
}
}

Rig::Rig(const Firmware &firmware, Case supplied)
    : machine_(McuFlashImage::from_known(firmware.mcu), BLOFELD_DEFAULT_SLA),
      input_(std::move(supplied)) {
  prepare();
  for (unsigned index = 0; index < mf::ram_bytes; ++index) {
    before_.push_back(machine_.bus().read(static_cast<std::uint16_t>(mf::ram_begin + index)));
  }
  flash_ = machine_.bus().power_off_flash_image();
  evidence_ = machine_.bus().usb_host_evidence();
  configuration_ = machine_.bus().usb_host_configuration();
  observe();
}

void Rig::prepare() {
  auto &bus = machine_.bus();
  bus.write(hc12::initee, qram);
  bus.retire_instruction();
  usb_host::prepare(bus, mf::endpoint_bytes);
  for (unsigned index = 0; index < mf::ram_bytes; ++index) {
    bus.write(static_cast<std::uint16_t>(mf::ram_begin + index), mf::canary);
  }
  for (unsigned index = 0; index < input_.source.size(); ++index) {
    bus.write(static_cast<std::uint16_t>(source + index), input_.source[index]);
  }
  // Own PULX; JSR B4AA caller. It removes the original argument and invokes the
  // genuine forwarding service. Stop before fetching the following RAM byte.
  constexpr std::array<std::uint8_t, 4> harness{0x30, 0x16, 0xb4, 0xaa};
  for (unsigned index = 0; index < harness.size(); ++index) {
    bus.write(static_cast<std::uint16_t>(caller + index), harness[index]);
  }
  for (const auto address : {mf::tx_reader, mf::tx_writer, mf::mirror_reader}) {
    bus.write(static_cast<std::uint16_t>(address), static_cast<std::uint8_t>(input_.cursor));
  }
  bus.write(mf::peer_reader, 0);
  bus.write(mf::peer_writer, 0);
  const auto cursor = input_.cursor == mf::ring_mask ? mf::usb_mask : 0;
  bus.write(mf::usb_reader, static_cast<std::uint8_t>(cursor));
  bus.write(mf::usb_writer, static_cast<std::uint8_t>(cursor));
  bus.write(buffer_select, 2);
  for (unsigned index = 0; index < mf::endpoint_bytes; ++index) {
    bus.write(static_cast<std::uint16_t>(local + index), mf::canary);
  }
  bus.write_memory({.address = input_control, .size = 2, .value = idle});
  bus.write_memory({.address = mf::stack, .size = 2, .value = caller});
  bus.write_memory({.address = mf::stack + 2, .size = 2, .value = source});
  auto state = initial;
  state.d = static_cast<std::uint16_t>(state.d | input_.selector);
  machine_.cpu().set_state(state);
}

void Rig::observe() {
  machine_.bus().set_observer([this](const BusEvent &event) {
    const bool usb = event.address == buffer_select || event.address == input_control ||
                     event.address == input_control + 1 ||
                     (event.address >= local && event.address < local + mf::endpoint_bytes);
    const bool sci = event.address == hc12::sci_control2 || event.address == hc12::sci_data_low;
    require(event.write && (usb || sci), "Original dispatcher/callees retain qualified MMIO only");
    stats_.writes.emplace_back(event.address, event.value);
  });
}

void Rig::record_call() {
  const auto state = machine_.cpu().state();
  const bool call = state.pc == uart_entry || state.pc == mf::queue_entry;
  if (call && !machine_.cpu().pending_interrupt()) {
    Call record{.pointer = word(machine_.bus(), state.sp + 2),
                .argument = state.d & byte_mask,
                .bytes = {},
                .ordinal = static_cast<unsigned>(uart_.size() + usb_.size())};
    const auto count = state.pc == uart_entry ? record.argument : 3;
    require(count == 2 || count == 3, "The actual dispatcher supplies its original byte count");
    for (unsigned index = 0; index < count; ++index) {
      record.bytes.push_back(
          machine_.bus().read(static_cast<std::uint16_t>(record.pointer + index)));
    }
    if (state.pc == uart_entry) {
      uart_.push_back(record);
    } else {
      usb_.push_back(record);
    }
  }
}

void Rig::step() {
  const auto state = machine_.cpu().state();
  require(stats_.instructions + own_ < maximum_instructions &&
              state.sp >= mf::stack - stack_bytes && state.sp <= mf::stack + 4,
          "Actual dispatcher/caller/callees retain bounded instruction and stack use");
  record_call();
  const auto result = machine_.cpu().step();
  if (result.instruction_executed) {
    if (state.pc == caller || state.pc == caller + 1) {
      require(input_.mirror, "Only the declared own near caller may execute");
      ++own_;
    } else {
      require(original(state.pc), "No substituted callee or unpinned manufacturer code executes");
      ++stats_.instructions;
      if (state.pc == uart_entry) { ++stats_.uart_calls; }
      if (state.pc == mf::queue_entry) { ++stats_.usb_calls; }
    }
  }
  if (result.interrupt_vector) {
    if (*result.interrupt_vector == mf::usb_vector) {
      ++stats_.usb_irqs;
    } else {
      require(*result.interrupt_vector == mf::sci_vector,
              "Only real SCI/USB interrupts are admitted");
      ++stats_.sci_irqs;
    }
  }
}

void Rig::until(const unsigned pc) {
  while (machine_.cpu().state().pc != pc) { step(); }
}

void Rig::capture_boundary() {
  if (input_.after_dispatch) {
    until(caller);
  } else if (!input_.uart_calls.empty()) {
    until(uart_entry);
  } else if (!input_.usb_calls.empty()) {
    until(mf::queue_entry);
  }
}

void Rig::check_dispatch() const {
  auto copy = machine_.bus().clone();
  require(machine_.cpu().state().pc == caller && machine_.cpu().state().sp == mf::stack + 2 &&
              word(*copy, mf::stack + 2) == source + input_.pointer_advance && own_ == 0,
          "Native return and raw terminator-position pointer publication match independent "
          "expectations");
  for (unsigned index = 0; index < input_.mutated.size(); ++index) {
    require(copy->read(static_cast<std::uint16_t>(source + index)) == input_.mutated[index],
            "Original C/D tail clearing mutates writable storage even with no output route");
  }
}

void Rig::complete() {
  until(caller);
  check_dispatch();
  if (input_.mirror) {
    step(); // Actual own PULX consumes the dispatcher argument.
    require(machine_.cpu().state().pc == caller + 1 && machine_.cpu().state().sp == mf::stack + 4 &&
                machine_.cpu().state().x == source + input_.pointer_advance,
            "Own native caller removes the actual returned pointer without host CPU/RAM rewrites");
    step();
    require(machine_.cpu().state().pc == mf::entry && machine_.cpu().state().sp == mf::stack + 2,
            "Own JSR enters the unchanged main-loop forwarding service");
    until(finished);
  }
  drain();
  check_final();
  guard();
}

void Rig::drain() {
  Bytes stream;
  for (const auto &event : input_.events) {
    stream.insert(stream.end(), event.begin(), event.end());
  }
  unsigned offset = 0;
  for (const auto length : input_.packets) {
    const auto packet = machine_.bus().request_usb_in(UsbBuffer::endpoint2);
    require(packet.response == UsbResponse::data && packet.size == length && packet.data1 == pid_ &&
                std::ranges::equal(std::span(packet.data).first(length),
                                   std::span(stream).subspan(offset, length)),
            "Real original queues/sender produce independent packets, batching and ACK-owned PID");
    const auto before = machine_.cpu().state();
    machine_.bus().finish_usb_in(true);
    pid_ = !pid_;
    const auto pending = machine_.cpu().pending_interrupt();
    require(pending && pending->vector == mf::usb_vector, "Only actual ACK raises the MIDI IN ISR");
    step();
    require(machine_.cpu().state().pc == usb_handler,
            "Actual native entry uses the original handler");
    until(before.pc);
    auto expected = before;
    expected.ccr = static_cast<std::uint8_t>(expected.ccr & clear_x);
    require(registers(machine_.cpu().state()) == registers(expected),
            "The genuine ISR/callee/RTI preserves the independently checked interrupted context");
    offset += length;
  }
  require(machine_.bus().request_usb_in(UsbBuffer::endpoint2).response == UsbResponse::nak,
          "Deferred/unselected/drained original output remains actual NAK");
}

void Rig::check_final() const {
  auto copy = machine_.bus().clone();
  require(uart_ == input_.uart_calls && usb_ == input_.usb_calls &&
              stats_.uart_calls == uart_.size() && stats_.usb_calls == usb_.size() &&
              own_ == (input_.mirror ? 2U : 0U) && stats_.usb_irqs == input_.packets.size() &&
              stats_.sci_irqs == static_cast<unsigned>(!input_.tx.empty()),
          "Actual raw callee arguments, call order/counts and own/native execution are distinct");
  std::array<std::uint8_t, 64> tx{};
  tx.fill(mf::canary);
  for (unsigned index = 0; index < input_.tx.size(); ++index) {
    tx[(input_.cursor + index) & mf::ring_mask] = input_.tx[index];
  }
  for (unsigned index = 0; index < tx.size(); ++index) {
    require(copy->read(static_cast<std::uint16_t>(mf::tx_ring + index)) == tx[index],
            "Independent UART bytes and untouched TX ring slots match");
  }
  unsigned delivered = 0;
  for (const auto length : input_.packets) { delivered += length; }
  const auto state = machine_.cpu().state();
  require(
      state.pc == (input_.mirror ? finished : caller) &&
          state.sp == mf::stack + (input_.mirror ? 4 : 2) &&
          (state.ccr & interrupt_masks) == (delivered != 0 ? 0 : initial_x_mask),
      "Caller ABI and one-way IRQ-mask effects match independent original execution expectations");
  for (unsigned index = 0; index < input_.mutated.size(); ++index) {
    require(copy->read(static_cast<std::uint16_t>(source + index)) == input_.mutated[index],
            "The original writable source remains correct after native caller and IRQ execution");
  }
  const auto usb_cursor = input_.cursor == mf::ring_mask ? mf::usb_mask : 0;
  require(copy->read(mf::tx_writer) == ((input_.cursor + input_.tx.size()) & mf::ring_mask) &&
              copy->read(mf::tx_reader) ==
                  ((input_.cursor + (input_.tx.empty() ? 0 : 1)) & mf::ring_mask) &&
              copy->read(mf::mirror_reader) ==
                  ((input_.cursor + (input_.mirror ? input_.tx.size() : 0)) & mf::ring_mask) &&
              copy->read(mf::usb_writer) ==
                  ((usb_cursor + 4 * input_.events.size()) & mf::usb_mask) &&
              copy->read(mf::usb_reader) == ((usb_cursor + delivered) & mf::usb_mask),
          "Raw UART/mirror/USB publication and deferred ownership match independent expectations");
  for (unsigned index = 0; index < input_.events.size(); ++index) {
    for (unsigned byte = 0; byte < 4; ++byte) {
      require(copy->read(static_cast<std::uint16_t>(
                  mf::usb_ring + ((usb_cursor + index * 4 + byte) & usb_byte_mask))) ==
                  input_.events[index][byte],
              "Every actual four-byte USB ring record retains the original packet payload");
    }
  }
  Bytes sci;
  unsigned enables = 0;
  for (const auto &[address, value] : stats_.writes) {
    if (address == hc12::sci_data_low) { sci.push_back(static_cast<std::uint8_t>(value)); }
    if (address == hc12::sci_control2) {
      require(value == enabled, "Whole original SCI preset is retained");
      ++enables;
    }
  }
  const Bytes first = input_.tx.empty() ? Bytes{} : Bytes{input_.tx.front()};
  require(sci == first && enables == input_.tx.size(),
          "Actual SCI TX data/presets match expected bytes");
}

void Rig::guard() const {
  const auto flash = machine_.bus().power_off_flash_image();
  require(flash.bytes == flash_.bytes && flash.known == flash_.known &&
              flash.fresh == flash_.fresh && machine_.bus().usb_host_evidence() == evidence_ &&
              machine_.bus().usb_host_configuration() == configuration_,
          "All NVM bytes/knowledge/history and USB identity are retained");
  auto copy = machine_.bus().clone();
  const auto usb_cursor = input_.cursor == mf::ring_mask ? mf::usb_mask : 0;
  for (unsigned index = 0; index < mf::ram_bytes; ++index) {
    const auto address = mf::ram_begin + index;
    const bool tx =
        address >= mf::tx_ring && address < mf::tx_ring + 64 &&
        ((address - mf::tx_ring + 64 - input_.cursor) & mf::ring_mask) < input_.tx.size();
    const bool usb =
        address >= mf::usb_ring && address < mf::usb_ring + 128 &&
        ((address - mf::usb_ring + 128 - usb_cursor) & usb_byte_mask) < input_.events.size() * 4;
    const bool mutable_ram = tx || usb || address == source + 2 || address == mf::tx_reader ||
                             address == mf::tx_writer || address == mf::usb_reader ||
                             address == mf::usb_writer ||
                             (input_.mirror && address == mf::mirror_reader) ||
                             (address >= mf::stack - stack_bytes && address < mf::stack + 4);
    if (!mutable_ram) {
      require(copy->read(static_cast<std::uint16_t>(address)) == before_[index],
              "All unrelated RAM, caller code and source extents are retained");
    }
  }
}

Saved Rig::save() const {
  return {.board = machine_.checkpoint(),
          .stats = stats_,
          .uart = uart_,
          .usb = usb_,
          .own = own_,
          .pid = pid_};
}

void Rig::restore(const Saved &saved) {
  machine_.restore(saved.board);
  stats_ = saved.stats;
  uart_ = saved.uart;
  usb_ = saved.usb;
  own_ = saved.own;
  pid_ = saved.pid;
  observe();
}

Outcome Rig::outcome() const {
  auto copy = machine_.bus().clone();
  midi_forward::Outcome board{.cpu = registers(machine_.cpu().state()),
                              .ram = {},
                              .usb_buffer = {},
                              .sci = {},
                              .control = word(*copy, input_control),
                              .host_pid = pid_,
                              .device_pid = machine_.bus().usb_host_endpoint(input_address).data1,
                              .stats = stats_};
  for (unsigned index = 0; index < mf::ram_bytes; ++index) {
    board.ram.push_back(copy->read(static_cast<std::uint16_t>(mf::ram_begin + index)));
  }
  copy->write(buffer_select, 2);
  for (unsigned index = 0; index < mf::endpoint_bytes; ++index) {
    board.usb_buffer.push_back(copy->read(static_cast<std::uint16_t>(local + index)));
  }
  for (unsigned index = 0; index < board.sci.size(); ++index) {
    board.sci[index] = copy->read(static_cast<std::uint16_t>(hc12::sci_baud_high + index));
  }
  return {.board = std::move(board), .uart = uart_, .usb = usb_, .own = own_};
}
}
