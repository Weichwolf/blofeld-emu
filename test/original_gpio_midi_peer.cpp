// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "original_gpio_midi_fixture.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld::test::gpio_midi {
namespace {
constexpr unsigned caller = 0x1400;
constexpr unsigned call_bytes = 5;
constexpr unsigned data = 0x1600;
constexpr unsigned stack = 0x2200;
constexpr unsigned stack_low = stack - 11;
constexpr unsigned ram_begin = 0x1200;
constexpr unsigned ram_end = 0x2600;
constexpr unsigned ram_guard = 0xa5;
constexpr unsigned activity = 0x1815;
constexpr unsigned midi_activity = 0x1812;
constexpr unsigned transfer_flags = 0x1831;
constexpr unsigned qram_mapping = 0x21;
constexpr unsigned all_bits = 255;
constexpr unsigned direction_out = 1;
constexpr unsigned j_outputs = 7;
constexpr unsigned bus_activity = 0x80;
constexpr unsigned multibyte = 0x10;
constexpr unsigned alternate_phase = 2;
constexpr unsigned frame_sender = 0xb870;
constexpr unsigned frame_end = 0xb8b0;
constexpr unsigned usb_sender = 0xb8b0;
constexpr unsigned usb_end = 0xb938;
constexpr unsigned strobe = 0xb6d0;
constexpr unsigned strobe_end = 0xb6dc;
constexpr unsigned instruction_limit = 20000;

unsigned sender(const Transport transport) {
  return transport == Transport::frame ? frame_sender : usb_sender;
}
}

Peer::Peer(const Firmware &firmware, const Scenario scenario, const Source &source)
    : scenario_(scenario),
      source_(source),
      machine_(McuFlashImage::from_known(firmware.mcu), BLOFELD_DEFAULT_SLA) {
  prepare();
  flash_ = machine_.bus().power_off_flash_image();
  for (unsigned address = ram_begin; address < ram_end; ++address) {
    before_.push_back(machine_.bus().read(static_cast<std::uint16_t>(address)));
  }
}

void Peer::prepare() {
  auto &bus = machine_.bus();
  bus.write(hc12::initee, qram_mapping);
  bus.retire_instruction();
  for (unsigned address = ram_begin; address < ram_end; ++address) {
    bus.write(static_cast<std::uint16_t>(address), ram_guard);
  }
  bus.write(hc12::ptj, direction_out);
  bus.write(hc12::ddrj, j_outputs);
  bus.write(hc12::ptt, direction_out);
  bus.write(hc12::ddrt, all_bits);
  bus.write(hc12::ptq, 0);
  bus.set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  bus.set_pin_drive(McuPort::q, {.floating = all_bits});
  for (const auto address : {activity, midi_activity, transfer_flags}) {
    bus.write(static_cast<std::uint16_t>(address), 0);
  }
  require(data + source_.storage.size() < midi_activity, "Own source stays below native flags");
  for (unsigned index = 0; index < source_.storage.size(); ++index) {
    bus.write(static_cast<std::uint16_t>(data + index), source_.storage[index]);
  }
  unsigned pc = caller;
  for (const auto count : source_.counts) {
    // Own LDAB #count; JSR original routine. These are not manufacturer program bytes.
    const auto target = sender(scenario_.transport);
    const std::array<std::uint8_t, call_bytes> code{0xc6,
                                                    count,
                                                    0x16,
                                                    static_cast<std::uint8_t>(target >> 8U),
                                                    static_cast<std::uint8_t>(target)};
    for (const auto byte : code) { bus.write(static_cast<std::uint16_t>(pc++), byte); }
  }
  finished_ = pc;
  bus.write_memory({.address = stack, .size = 2, .value = data});
  machine_.cpu().set_state({.pc = caller, .sp = stack, .ccr = hc12::reset_ccr});
}

bool Peer::irq_level() const {
  const auto pin = machine_.bus().observe_pads(McuPort::j, 1);
  require(pin.known == 1, "Candidate J0 strobe must be a known native MCU output");
  return (pin.levels != 0) != scenario_.inverted;
}

void Peer::synchronize(DspBus &dsp) {
  auto &bus = machine_.bus();
  const auto direction = bus.read(hc12::ddrq);
  require(direction == 0 || direction == all_bits, "Native sender selects a whole-byte direction");
  auto &port = dsp.gpio(DspPort::c);
  require((port.outputs().mask & all_bits) == 0,
          "Ordinary original receiver releases all data pads");
  bus.set_pin_drive(McuPort::q, {.floating = all_bits});
  if (direction == all_bits) {
    const auto output = bus.observe_pads(McuPort::q, all_bits);
    const auto control = bus.observe_pads(McuPort::t, 1);
    require(output.known == all_bits && control.known == 1 && control.levels == direction_out,
            "Actual native outgoing data and direction control must be known");
    port.set_drive({.mask = output.known, .levels = output.levels});
  } else {
    port.set_drive({.floating = all_bits});
    const auto weak = port.observe_pads(all_bits);
    require(weak.known == all_bits, "Explicit isolated data link resolves the real DSP pad bias");
    bus.set_pin_drive(McuPort::q,
                      {.mask = all_bits, .levels = static_cast<std::uint8_t>(weak.levels)});
  }
  const auto level = irq_level();
  if (last_irq_.has_value() && level != *last_irq_) { ++transitions_; }
  last_irq_ = level;
  dsp.observe_irqa(level);
}

void Peer::step() {
  require(steps_ < instruction_limit, "Unchanged native sender must terminate within its bound");
  const auto before = machine_.cpu().state();
  const auto pc = before.pc;
  const auto last = scenario_.transport == Transport::frame ? frame_end : usb_end;
  const auto entry = sender(scenario_.transport);
  require((pc >= caller && pc < finished_) || (pc >= entry && pc < last) ||
              (pc >= strobe && pc < strobe_end),
          "Only own LDAB/JSR and complete original sender/strobe programs execute");
  require(before.sp >= stack_low && before.sp <= stack,
          "Actual native stack remains within 11 bytes");
  const auto step = machine_.cpu().step();
  require(step.instruction_executed && !step.interrupt_vector,
          "No host interrupt or return injection");
  ++steps_;
  if (pc >= caller && pc < finished_ && (pc - caller) % call_bytes == 2) {
    const unsigned returned = static_cast<unsigned>(pc) + 3;
    const auto &state = machine_.cpu().state();
    require(state.pc == entry && state.sp == stack - 2 &&
                machine_.bus().read_memory({.address = stack - 2, .size = 2}) == returned,
            "Own native JSR pushes the independent actual return address");
    ++calls_;
  }
}

bool Peer::done() const {
  return machine_.cpu().state().pc == finished_;
}

unsigned Peer::byte(const unsigned index) const {
  return source_.bytes.at(index);
}

Peer::Saved Peer::save() const {
  return {.machine = machine_.checkpoint(),
          .steps = steps_,
          .calls = calls_,
          .transitions = transitions_,
          .irq = last_irq_};
}

void Peer::restore(const Saved &saved) {
  machine_.restore(saved.machine);
  steps_ = saved.steps;
  calls_ = saved.calls;
  transitions_ = saved.transitions;
  last_irq_ = saved.irq;
}

void Peer::verify() {
  auto &bus = machine_.bus();
  const bool usb = scenario_.transport == Transport::usb;
  const auto expected_flags = usb ? multibyte | alternate_phase : 0U;
  const auto expected_mask = usb ? hc12::interrupt_mask : 0U;
  const auto expected_direction = usb ? all_bits : 0U;
  // Frame: own LDAB/JSR2 + setup16 + 128*18 + tail4.
  // USB: three entry/caller17 + 48 triple-event101 + 16 single-event51 + three tail8 + two phase4.
  const auto expected_steps = usb ? (3U * 17) + (48U * 101) + (16U * 51) + (3U * 8) + (2U * 4)
                                  : 2U + 16U + (128U * 18) + 4U;
  require(
      done() && machine_.cpu().state().sp == stack && calls_ == source_.counts.size() &&
          steps_ == expected_steps && transitions_ == 2 * source_.bytes.size() &&
          bus.read(hc12::ddrq) == expected_direction &&
          (machine_.cpu().state().ccr & hc12::interrupt_mask) == expected_mask,
      "Independent native instruction/transition totals, actual RTS, direction and IRQ mask match");
  require(bus.read(activity) == bus_activity && bus.read(midi_activity) == (usb ? 4U : 0U) &&
              bus.read(transfer_flags) == expected_flags &&
              bus.read_memory({.address = stack, .size = 2}) ==
                  data + (usb ? source_.storage.size() : 0U),
          "Original flags and actual caller pointer match the selected transport");
  const auto now = bus.power_off_flash_image();
  require(now.bytes == flash_.bytes && now.known == flash_.known && now.fresh == flash_.fresh,
          "Every native call preserves MCU NVM bytes, knownness and erase/program history");
  for (unsigned address = ram_begin; address < ram_end; ++address) {
    if ((address >= stack_low && address < stack + 2) || address == activity ||
        address == midi_activity || address == transfer_flags) {
      continue;
    }
    auto expected = before_.at(address - ram_begin);
    if (usb && address >= data && address - data < source_.storage.size() &&
        (address - data) % 4 == 0) {
      expected = 0;
    }
    require(bus.read(static_cast<std::uint16_t>(address)) == expected,
            "Native forwarding clears only USB headers and preserves all other guarded RAM");
  }
}

Peer::Result Peer::result() {
  auto &bus = machine_.bus();
  const auto state = machine_.cpu().state();
  Result result{.cpu = {state.pc, state.d, state.x, state.y, state.sp, state.ccr},
                .ram = {},
                .ports = {},
                .steps = steps_,
                .calls = calls_,
                .transitions = transitions_};
  for (unsigned address = ram_begin; address < ram_end; ++address) {
    result.ram.push_back(bus.read(static_cast<std::uint16_t>(address)));
  }
  const auto q = bus.observe_pads(McuPort::q, all_bits);
  result.ports = {bus.read(hc12::ddrq),
                  bus.read(hc12::perq),
                  bus.read(hc12::ppsq),
                  bus.observe_pads(McuPort::j, 1).levels,
                  bus.observe_pads(McuPort::t, 3).levels,
                  q.known,
                  q.levels};
  return result;
}
}
