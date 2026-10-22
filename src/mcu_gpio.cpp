// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_gpio.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace blofeld {
namespace {
constexpr unsigned byte_mask = 0xff;
constexpr unsigned port_e_direction_mask = 0xfc;
constexpr unsigned port_e_pull_mask = 0x9f;
constexpr unsigned port_e_clock_disable = 0x10;
constexpr unsigned port_m_mask = 0x3f;
constexpr unsigned port_u_mask = 0x3f;
constexpr unsigned port_t_mask = 0x0f;
constexpr unsigned routing_mask = 0x1f;
constexpr unsigned port_m_tied_mask = 0x27;
constexpr unsigned port_u_tied_mask = 0x3e;
constexpr unsigned port_t_tied_mask = 0x0c;
constexpr unsigned core_pull_mask = 0x93;
constexpr unsigned core_pull_reset = 0x90;
constexpr std::uint16_t pim_end = 0x0280;

struct PortSpec {
  std::uint16_t base = 0;
  unsigned valid = byte_mask;
  unsigned initial_latch_known = byte_mask;
  unsigned initial_pull = 0;
};

constexpr std::array<PortSpec, 11> specs{{{.base = hc12::porta, .initial_latch_known = 0},
                                          {.base = hc12::portb, .initial_latch_known = 0},
                                          {.base = hc12::porte, .initial_latch_known = 0},
                                          {.base = hc12::ptt, .valid = port_t_mask},
                                          {.base = hc12::ptm, .valid = port_m_mask},
                                          {.base = hc12::ptj, .valid = 7},
                                          {.base = hc12::ptp},
                                          {.base = hc12::ptq},
                                          {.base = hc12::ptr},
                                          {.base = hc12::pts},
                                          {.base = hc12::ptu, .valid = port_u_mask}}};

std::size_t port_index(const McuPort port) {
  const auto index = static_cast<std::size_t>(port);
  if (index >= specs.size()) { throw Fault("Unknown MCU GPIO port"); }
  return index;
}

struct Register {
  McuPort port = McuPort::a;
  unsigned kind = 0;
};

std::optional<Register> lookup(const std::uint16_t offset) {
  switch (offset) {
    case hc12::porta: return Register{.port = McuPort::a, .kind = 0};
    case hc12::portb: return Register{.port = McuPort::b, .kind = 0};
    case hc12::ddra: return Register{.port = McuPort::a, .kind = 2};
    case hc12::ddrb: return Register{.port = McuPort::b, .kind = 2};
    case hc12::porte: return Register{.port = McuPort::e, .kind = 0};
    case hc12::ddre: return Register{.port = McuPort::e, .kind = 2};
    default: break;
  }
  if (offset < hc12::ptt || offset >= pim_end) { return {}; }
  const auto relative = static_cast<unsigned>(offset) - hc12::ptt;
  if (relative % 8 > 5) { return {}; }
  return Register{.port = static_cast<McuPort>(static_cast<unsigned>(McuPort::t) + (relative / 8)),
                  .kind = relative % 8};
}
}

McuGpio::McuGpio() {
  reset();
}

void McuGpio::reset() {
  for (std::size_t index = 0; index < ports_.size(); ++index) {
    const auto external = ports_[index].external;
    ports_[index] = {.latch_known = static_cast<std::uint8_t>(specs[index].initial_latch_known),
                     .pull_enable = static_cast<std::uint8_t>(specs[index].initial_pull),
                     .external = external};
  }
  routing_ = 0;
  assignment_e_ = 0;
  pull_control_ = core_pull_reset;
}

void McuGpio::set_drive(const McuPort port, const PinDrive drive) {
  const auto index = port_index(port);
  if ((drive.mask & drive.floating) != 0 || (drive.levels & ~drive.mask) != 0 ||
      (drive.unknown & (drive.mask | drive.floating)) != 0 ||
      ((drive.mask | drive.floating | drive.unknown) & ~specs[index].valid) != 0) {
    throw Fault("Invalid MCU GPIO external drive masks");
  }
  ports_[index].external = drive;
}

void McuGpio::update_drive(const McuPort port, const std::uint8_t affected, const PinDrive drive) {
  const auto index = port_index(port);
  if ((affected & ~specs[index].valid) != 0 ||
      ((drive.mask | drive.floating | drive.unknown) & ~affected) != 0 ||
      (drive.unknown & (drive.mask | drive.floating)) != 0 || (drive.mask & drive.floating) != 0 ||
      (drive.levels & ~drive.mask) != 0) {
    throw Fault("Invalid MCU GPIO partial external drive mask");
  }
  const auto previous = ports_[index].external;
  set_drive(
      port,
      {.mask = static_cast<std::uint8_t>((previous.mask & ~affected) | drive.mask),
       .levels = static_cast<std::uint8_t>((previous.levels & ~affected) | drive.levels),
       .floating = static_cast<std::uint8_t>((previous.floating & ~affected) | drive.floating),
       .unknown = static_cast<std::uint8_t>((previous.unknown & ~affected) | drive.unknown)});
}

void McuGpio::set_function(const McuPort port, const PinFunction function) {
  const auto index = port_index(port);
  if ((function.inputs & function.outputs) != 0 || (function.levels & ~function.outputs) != 0 ||
      (function.unknown & ~function.outputs) != 0 || (function.levels & function.unknown) != 0 ||
      ((function.inputs | function.outputs) & ~specs[index].valid) != 0) {
    throw Fault("Invalid MCU GPIO peripheral-function masks");
  }
  ports_[index].function = function;
}

std::uint8_t McuGpio::read_pad(const McuPort port, const std::uint8_t mask) const {
  if ((mask & ~specs[port_index(port)].valid) != 0) { throw Fault("Invalid MCU GPIO sample mask"); }
  return sample(port, mask);
}

PinSample McuGpio::observe_pads(const McuPort port, const std::uint8_t mask) const {
  if ((mask & ~specs[port_index(port)].valid) != 0) {
    throw Fault("Invalid MCU GPIO observation mask");
  }
  return resolve(port, mask);
}

PinOutputs McuGpio::outputs(const McuPort port) const {
  const auto index = port_index(port);
  if (tied_low(port) != 0) {
    throw Fault("Unqualified MCU package GPIO output observation at " + hex(specs[index].base));
  }
  if (port == McuPort::e && (assignment_e_ & port_e_clock_disable) == 0) {
    throw Fault("Unimplemented MCU ECLK output observation at " + hex(hc12::porte));
  }
  const auto &state = ports_[index];
  const auto &function = state.function;
  const auto functions = static_cast<unsigned>(function.inputs) | function.outputs;
  const auto direction = (state.direction & ~functions) | function.outputs;
  const auto latch = (state.latch & ~static_cast<unsigned>(function.outputs)) | function.levels;
  const auto latch_known = (static_cast<unsigned>(state.latch_known) | function.outputs) &
                           ~static_cast<unsigned>(function.unknown);
  const auto mask = direction & specs[index].valid;
  const auto known = mask & latch_known;
  return {.mask = static_cast<std::uint8_t>(mask),
          .known = static_cast<std::uint8_t>(known),
          .levels = static_cast<std::uint8_t>(latch & known)};
}

std::uint8_t McuGpio::tied_low(const McuPort port) const {
  if ((routing_ & 3U) != 3) { return 0; }
  switch (port) {
    case McuPort::p:
    case McuPort::q: return byte_mask;
    case McuPort::m: return port_m_tied_mask;
    case McuPort::t: return port_t_tied_mask;
    case McuPort::u: return port_u_tied_mask;
    case McuPort::s: return port_t_mask;
    case McuPort::a:
    case McuPort::b:
    case McuPort::e:
    case McuPort::j:
    case McuPort::r: return 0;
    default: throw Fault("Unknown MCU GPIO port");
  }
}

PinSample McuGpio::resolve(const McuPort port, const std::uint8_t requested) const {
  const auto index = port_index(port);
  const auto &state = ports_[index];
  const auto tied = static_cast<unsigned>(tied_low(port));
  const auto &function = state.function;
  const auto functions = static_cast<unsigned>(function.inputs) | function.outputs;
  const auto direction = (state.direction & ~functions) | function.outputs;
  const auto latch = (state.latch & ~static_cast<unsigned>(function.outputs)) | function.levels;
  const auto latch_known = (static_cast<unsigned>(state.latch_known) | function.outputs) &
                           ~static_cast<unsigned>(function.unknown);
  const auto output = direction & latch_known & ~tied;
  const auto unknown_output = direction & ~latch_known & ~tied;
  unsigned known = tied | state.external.mask | output;
  unsigned value = (state.external.levels | (latch & output)) & ~tied;
  const auto conflict =
      state.external.mask & (tied | output) & (state.external.levels ^ (latch & ~tied));
  if ((conflict & requested) != 0) {
    throw Fault("MCU GPIO output contention at " + hex(specs[index].base));
  }
  auto pull_enable = static_cast<unsigned>(state.pull_enable);
  if (port == McuPort::a) { pull_enable = (pull_control_ & 1U) != 0 ? byte_mask : 0; }
  if (port == McuPort::b) { pull_enable = (pull_control_ & 2U) != 0 ? byte_mask : 0; }
  if (port == McuPort::e) {
    pull_enable = (pull_control_ & port_e_clock_disable) != 0 ? port_e_pull_mask : 0;
  }
  const auto weak =
      state.external.floating & pull_enable & ~static_cast<unsigned>(direction) & ~tied;
  known |= weak;
  value |= weak & ~static_cast<unsigned>(state.pull_down);
  known = known & ~unknown_output & ~static_cast<unsigned>(state.external.unknown) & requested &
          specs[index].valid;
  return {.known = static_cast<std::uint8_t>(known),
          .levels = static_cast<std::uint8_t>(value & known)};
}

std::uint8_t McuGpio::sample(const McuPort port, const std::uint8_t requested) const {
  const auto index = port_index(port);
  const auto unknown_output =
      ports_[index].function.unknown & requested & ~static_cast<unsigned>(tied_low(port));
  if (unknown_output != 0) {
    throw Fault("Unknown MCU peripheral output latch at " + hex(specs[index].base));
  }
  const auto observation = resolve(port, requested);
  const auto unknown = requested & ~static_cast<unsigned>(observation.known) & specs[index].valid;
  if (unknown != 0) {
    throw Fault("Unknown MCU GPIO input at " + hex(specs[index].base) + " mask=" + hex(unknown, 2));
  }
  return observation.levels;
}

std::uint8_t McuGpio::data(const McuPort port) const {
  const auto index = port_index(port);
  const auto &state = ports_[index];
  if (port == McuPort::e && (assignment_e_ & port_e_clock_disable) == 0) {
    throw Fault("Unimplemented MCU ECLK pin function at " + hex(hc12::porte));
  }
  if ((state.direction & ~state.latch_known) != 0) {
    throw Fault("Uninitialized MCU GPIO output latch at " + hex(specs[index].base));
  }
  const auto inputs =
      static_cast<std::uint8_t>(~static_cast<unsigned>(state.direction) & specs[index].valid);
  return static_cast<std::uint8_t>((state.latch & state.direction) | sample(port, inputs));
}

std::string_view McuGpio::register_name(const std::uint16_t offset) {
  switch (offset) {
    case hc12::pear: return "PEAR";
    case hc12::pucr: return "PUCR";
    case hc12::modrr: return "MODRR";
    default: break;
  }
  const auto reg = lookup(offset);
  if (!reg) { return {}; }
  switch (reg->kind) {
    case 0: return "GPIO data";
    case 1: return "GPIO pin input";
    case 2: return "GPIO direction";
    case 3: return "GPIO reduced drive";
    case 4: return "GPIO pull enable";
    case 5: return "GPIO pull select";
    default: return {};
  }
}

std::uint8_t McuGpio::read(const std::uint16_t offset) const {
  switch (offset) {
    case hc12::pear: return assignment_e_;
    case hc12::pucr: return pull_control_;
    case hc12::modrr: return routing_;
    default: break;
  }
  const auto reg = lookup(offset);
  if (!reg) { throw Fault("Unknown MCU GPIO read at " + hex(offset)); }
  const auto &state = ports_[port_index(reg->port)];
  switch (reg->kind) {
    case 0: return data(reg->port);
    case 1: return sample(reg->port, static_cast<std::uint8_t>(specs[port_index(reg->port)].valid));
    case 2: return state.direction;
    case 3: return state.reduced_drive;
    case 4: return state.pull_enable;
    case 5: return state.pull_down;
    default: throw Fault("Unknown MCU GPIO register kind");
  }
}

void McuGpio::write(const ByteWrite write) {
  const auto offset = write.address;
  const auto value = write.value;
  if (offset == hc12::pear) {
    if ((value & ~port_e_clock_disable) != 0) {
      throw Fault("Unimplemented MCU port E alternate function");
    }
    assignment_e_ = value;
    return;
  }
  if (offset == hc12::pucr) {
    pull_control_ = static_cast<std::uint8_t>(value & core_pull_mask);
    return;
  }
  if (offset == hc12::modrr) {
    routing_ = static_cast<std::uint8_t>(value & routing_mask);
    return;
  }
  const auto reg = lookup(offset);
  if (!reg) { throw Fault("Unknown MCU GPIO write at " + hex(offset)); }
  const auto index = port_index(reg->port);
  auto &state = ports_[index];
  const auto masked = static_cast<std::uint8_t>(value & specs[index].valid);
  switch (reg->kind) {
    case 0:
      state.latch = masked;
      state.latch_known = static_cast<std::uint8_t>(specs[index].valid);
      return;
    case 1: return; // Read-only input register; documented writes have no effect.
    case 2:
      state.direction = reg->port == McuPort::e
                            ? static_cast<std::uint8_t>(value & port_e_direction_mask)
                            : masked;
      return;
    case 3: state.reduced_drive = masked; return;
    case 4: state.pull_enable = masked; return;
    case 5: state.pull_down = masked; return;
    default: throw Fault("Unknown MCU GPIO register kind");
  }
}

}
