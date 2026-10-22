// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_bus.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_clock.hpp"
#include "blofeld/mcu_flash.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_interrupts.hpp"
#include "blofeld/mcu_memory.hpp"
#include "blofeld/mcu_sci.hpp"
#include "blofeld/mcu_timer.hpp"
#include "blofeld/mcu_usb.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>

namespace blofeld {
namespace {
constexpr std::uint16_t reserved_sci_end = 0x00d0;
constexpr std::uint16_t reserved_flash_begin = 0x0100;
constexpr unsigned port_count = 11;
constexpr unsigned small_tx = 0x20;

struct Pad {
  McuPort port = McuPort::t;
  std::uint8_t mask = 0;
};

struct SignalRoute {
  unsigned source = 0;
  Pad pad;
};

constexpr std::array<Pad, 8> large_timer_pads{{{.mask = 1},
                                               {.mask = 2},
                                               {.mask = 4},
                                               {.mask = 8},
                                               {.port = McuPort::r, .mask = 16},
                                               {.port = McuPort::r, .mask = 32},
                                               {.port = McuPort::r, .mask = 64},
                                               {.port = McuPort::r, .mask = 128}}};
constexpr std::array<Pad, 8> small_timer_pads{{{.mask = 1},
                                               {.mask = 2},
                                               {.port = McuPort::m, .mask = 8},
                                               {.port = McuPort::m, .mask = 16},
                                               {.port = McuPort::q, .mask = 16},
                                               {.port = McuPort::q, .mask = 32},
                                               {.port = McuPort::q, .mask = 64},
                                               {.port = McuPort::q, .mask = 128}}};

struct TimerRoute {
  unsigned channel = 0;
  std::uint8_t routing = 0;
};

class ObserverGuard {
public:
  explicit ObserverGuard(bool &active) : active_(active), previous_(active) { active_ = true; }

  ~ObserverGuard() { active_ = previous_; }

  ObserverGuard(const ObserverGuard &) = delete;
  ObserverGuard &operator=(const ObserverGuard &) = delete;

private:
  bool &active_;
  bool previous_;
};

Pad timer_pad(const TimerRoute route) {
  const auto channel = route.channel;
  const auto routing = route.routing;
  if ((routing & 1U) == 0) { return large_timer_pads.at(channel); }
  if ((routing & 4U) != 0) {
    return {.port = McuPort::b, .mask = static_cast<std::uint8_t>(1U << channel)};
  }
  return small_timer_pads.at(channel);
}

void route_function(std::array<PinFunction, port_count> &ports,
                    const SignalRoute route,
                    const PinFunction signals) {
  if (((signals.inputs | signals.outputs) & route.source) == 0) { return; }
  auto &function = ports.at(static_cast<std::size_t>(route.pad.port));
  const auto bit = route.pad.mask;
  if (((function.inputs | function.outputs) & bit) != 0) {
    throw Fault("Unimplemented overlapping peripheral pin ownership");
  }
  if ((signals.inputs & route.source) != 0) { function.inputs |= bit; }
  if ((signals.outputs & route.source) != 0) { function.outputs |= bit; }
  if ((signals.levels & route.source) != 0) { function.levels |= bit; }
  if ((signals.unknown & route.source) != 0) { function.unknown |= bit; }
}
}

McuBus::McuBus(const std::span<const std::uint8_t> flash) : memory_(flash), flash_(memory_) {}

void McuBus::reset() {
  memory_.reset();
  decoded_bytes_.reset();
  ++code_revision_;
  ++revision_;
  flash_.reset();
  gpio_.reset();
  peripheral_pins_.reset();
  clock_.reset();
  sci_.reset();
  timer_.reset();
  usb_.reset();
  interrupts_.reset();
  context_ = {};
  interrupts_.observe_pins(gpio_.observe_pads(McuPort::e, 3));
}

bool McuBus::reserved_register(const std::uint16_t offset) {
  // Explicitly documented read-zero/write-ignored range, not an unknown-IO fallback.
  return offset >= reserved_sci_end && offset < reserved_flash_begin;
}

std::uint8_t McuBus::read_register(const std::uint16_t offset) {
  if (!McuMemory::register_name(offset).empty()) { return memory_.read_register(offset); }
  if (!McuFlash::register_name(offset).empty()) { return flash_.read(offset); }
  if (!McuGpio::register_name(offset).empty()) { return gpio_.read(offset); }
  if (!McuClock::register_name(offset).empty()) { return clock_.read(offset); }
  if (!McuSci::register_name(offset).empty()) { return sci_.read(offset); }
  if (!McuTimer::register_name(offset).empty()) { return timer_.read(offset); }
  if (!McuInterrupts::register_name(offset).empty()) { return interrupts_.read(offset); }
  if (!McuUsb::register_name(offset).empty()) { return usb_.read(offset); }
  if (offset == hc12::smramcfg || offset == hc12::smramstat) { return 0; }
  if (reserved_register(offset)) { return 0; }
  throw Fault("Unimplemented MCU peripheral read at " + hex(offset));
}

std::string_view McuBus::write_register(const ByteWrite write) {
  const auto offset = write.address;
  const auto value = write.value;
  const auto memory_name = McuMemory::register_name(offset);
  if (!memory_name.empty()) {
    memory_.write_register(write);
    return memory_name;
  }
  const auto flash_name = McuFlash::register_name(offset);
  if (!flash_name.empty()) {
    try {
      flash_.write(write);
    } catch (const Fault &fault) {
      throw Fault("MCU flash register pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr, 2) +
                  ": " + fault.what());
    }
    return flash_name;
  }
  const auto gpio_name = McuGpio::register_name(offset);
  if (!gpio_name.empty()) {
    gpio_.write(write);
    return gpio_name;
  }
  const auto clock_name = McuClock::register_name(offset);
  if (!clock_name.empty()) {
    const auto command = clock_.write(write);
    usb_.set_control(
        static_cast<std::uint16_t>((static_cast<unsigned>(clock_.read(hc12::usb_control)) << 8U) |
                                   clock_.read(hc12::usb_control + 1)));
    if (command == UsbControlCommand::endpoint_reset) { usb_.reset_endpoints(); }
    return clock_name;
  }
  const auto sci_name = McuSci::register_name(offset);
  if (!sci_name.empty()) {
    sci_.write(write);
    return sci_name;
  }
  const auto timer_name = McuTimer::register_name(offset);
  if (!timer_name.empty()) {
    timer_.write(write);
    return timer_name;
  }
  const auto interrupt_name = McuInterrupts::register_name(offset);
  if (!interrupt_name.empty()) {
    interrupts_.write(write, context_.ccr);
    return interrupt_name;
  }
  const auto usb_name = McuUsb::register_name(offset);
  if (!usb_name.empty()) {
    constexpr unsigned module_clock_enable = 0x40;
    if ((clock_.read(hc12::usb_control) & module_clock_enable) == 0) {
      throw Fault("Unverified USB configuration write with module clock disabled");
    }
    usb_.write(write);
    return usb_name;
  }
  if (reserved_register(offset)) { return "reserved/write ignored"; }
  if (offset == hc12::smramcfg && (value & 1U) == 0) { return "SMRAMCFG normal mode"; }
  throw Fault("Unimplemented MCU peripheral write at " + hex(offset));
}

std::uint8_t McuBus::read(const std::uint16_t address) {
  const auto location = memory_.locate(address);
  if (location.space == McuSpace::registers) {
    try {
      return read_register(location.offset);
    } catch (const Fault &fault) {
      throw Fault("MCU read at " + hex(address) + " (register offset " + hex(location.offset) +
                  ") pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr, 2) + ": " +
                  fault.what());
    }
  }
  if (location.space == McuSpace::flash) {
    try {
      flash_.check_array_read();
      return memory_.read(address);
    } catch (const Fault &fault) {
      throw Fault("MCU flash read at " + hex(address) + " pc=" + hex(context_.pc) +
                  " ccr=" + hex(context_.ccr, 2) + ": " + fault.what());
    }
  }
  return memory_.read(address);
}

void McuBus::write(const std::uint16_t address, const std::uint8_t value) {
  const auto location = memory_.locate(address);
  if (location.space == McuSpace::registers) {
    try {
      const auto name = write_register({.address = location.offset, .value = value});
      sync_peripheral_pins();
      interrupts_.observe_pins(gpio_.observe_pads(McuPort::e, 3));
      notify({.write = true, .address = address, .value = value, .name = name});
    } catch (const Fault &fault) {
      throw Fault("MCU write at " + hex(address) + " (register offset " + hex(location.offset) +
                  ") pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr, 2) + ": " +
                  fault.what());
    }
  } else if (location.space == McuSpace::flash) {
    stage_flash({.address = address, .size = 1, .value = value});
    return;
  } else {
    memory_.write({.address = address, .value = value});
  }
  ++revision_;
  invalidate_code({.address = address});
}

std::uint8_t McuBus::peek(const std::uint16_t address) const {
  return memory_.peek(address);
}

void McuBus::write_memory(const MemoryWrite access) {
  const auto location = memory_.locate(access.address);
  if (location.space == McuSpace::flash) {
    stage_flash(access);
    return;
  }
  if (location.space != McuSpace::registers || !McuUsb::configuration_data(location.offset)) {
    Bus::write_memory(access);
    return;
  }
  try {
    constexpr unsigned module_clock_enable = 0x40;
    if ((clock_.read(hc12::usb_control) & module_clock_enable) == 0) {
      throw Fault("Unverified USB configuration write with module clock disabled");
    }
    usb_.write_memory({.address = location.offset, .size = access.size, .value = access.value});
    invalidate_code({.address = access.address, .size = access.size});
    ++revision_;
    if (observer_) {
      constexpr unsigned byte_mask = 0xff;
      for (unsigned index = 0; index < 2; ++index) {
        const auto value =
            static_cast<std::uint8_t>((access.value >> ((1U - index) * 8)) & byte_mask);
        notify({.write = true,
                .address = static_cast<std::uint16_t>(access.address + index),
                .value = value,
                .name = McuUsb::register_name(location.offset)});
      }
    }
  } catch (const Fault &fault) {
    throw Fault("MCU operand write at " + hex(access.address) + " (register offset " +
                hex(location.offset) + ") pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr, 2) +
                ": " + fault.what());
  }
}

void McuBus::check_execute(const InstructionFetch fetch) const {
  constexpr unsigned address_space_size = 65536;
  if (fetch.length == 0 || fetch.length > address_space_size - fetch.address) {
    throw Fault("Invalid MCU instruction fetch range at " + hex(fetch.address) +
                " pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr, 2));
  }
  for (unsigned index = 0; index < fetch.length; ++index) {
    const auto target = static_cast<std::uint32_t>(fetch.address) + index;
    if (target > hc12::address_mask) { throw Fault("MCU instruction fetch exceeds address space"); }
    if (memory_.locate(static_cast<std::uint16_t>(target)).space == McuSpace::flash) {
      try {
        flash_.check_array_read();
        memory_.check_execute(static_cast<std::uint16_t>(target));
      } catch (const Fault &fault) {
        throw Fault("MCU flash instruction fetch at " + hex(target) + " pc=" + hex(context_.pc) +
                    " ccr=" + hex(context_.ccr, 2) + ": " + fault.what());
      }
    } else {
      try {
        memory_.check_execute(static_cast<std::uint16_t>(target));
      } catch (const Fault &fault) {
        throw Fault("MCU instruction fetch at " + hex(target) + " pc=" + hex(context_.pc) +
                    " ccr=" + hex(context_.ccr, 2) + ": " + fault.what());
      }
    }
  }
}

void McuBus::retire_instruction() {
  if (memory_.retire_instruction()) {
    ++revision_;
    ++code_revision_;
  }
}

void McuBus::set_pin_drive(const McuPort port, const PinDrive drive) {
  gpio_.set_drive(port, drive);
  if (port == McuPort::e) { interrupts_.observe_pins(gpio_.observe_pads(port, 3)); }
}

void McuBus::update_pin_drive(const McuPort port,
                              const std::uint8_t affected,
                              const PinDrive drive) {
  gpio_.update_drive(port, affected, drive);
  if (port == McuPort::e) { interrupts_.observe_pins(gpio_.observe_pads(port, 3)); }
}

PinSample McuBus::observe_pads(const McuPort port, const std::uint8_t mask) const {
  return gpio_.observe_pads(port, mask);
}

void McuBus::begin_instruction(const ExecutionContext context) {
  context_ = context;
}

std::optional<InterruptRequest> McuBus::interrupt_request(const std::uint8_t ccr) const {
  const auto urgent = interrupts_.select({}, static_cast<std::uint8_t>(ccr | hc12::interrupt_mask));
  if (urgent) { return urgent; }
  if ((ccr & hc12::interrupt_mask) != 0) { return {}; }
  constexpr unsigned maximum_sources = 21;
  std::array<std::uint16_t, maximum_sources> pending{};
  std::size_t count = 0;
  const auto append = [&pending, &count](const std::uint16_t vector) {
    pending.at(count) = vector;
    ++count;
  };
  constexpr unsigned timer0_vector = 0xffee;
  constexpr unsigned overflow_vector = 0xffde;
  constexpr unsigned sci_vector = 0xffd6;
  constexpr unsigned pll_vector = 0xffc6;
  constexpr unsigned realtime_vector = 0xfff0;
  const auto timer_channels = timer_.interrupt_channels();
  for (unsigned channel = 0; channel < 8; ++channel) {
    if ((timer_channels & (1U << channel)) != 0) {
      append(static_cast<std::uint16_t>(timer0_vector - (channel * 2)));
    }
  }
  if (timer_.overflow_interrupt()) { append(overflow_vector); }
  if (sci_.interrupt_pending()) { append(sci_vector); }
  if (clock_.lock_interrupt_pending()) { append(pll_vector); }
  if (clock_.realtime_interrupt_pending()) { append(realtime_vector); }
  constexpr unsigned flash_vector = 0xffb8;
  if (flash_.interrupt_pending()) { append(flash_vector); }
  constexpr std::array<std::uint16_t, 8> usb_vectors{
      0xffd4, 0xffd2, 0xffd0, 0xffbe, 0xffce, 0xffcc, 0xffca, 0xffc8};
  const auto usb_sources = usb_.interrupt_sources();
  for (unsigned source = 0; source < usb_vectors.size(); ++source) {
    if ((usb_sources & (1U << source)) != 0) { append(usb_vectors[source]); }
  }
  return interrupts_.select(std::span{pending}.first(count),
                            static_cast<std::uint8_t>(ccr | hc12::xirq_mask));
}

void McuBus::acknowledge_interrupt(const std::uint16_t vector) {
  interrupts_.acknowledge(vector);
}

void McuBus::observe_phy(const PhyClockWindow window) {
  clock_.observe_phy(window);
}

McuRunClockRoute McuBus::run_clock_route() const {
  try {
    return clock_.run_clock_route();
  } catch (const Fault &fault) {
    const auto base = static_cast<unsigned>(memory_.read_register(hc12::initrg)) << 8U;
    throw Fault("MCU run clock source at " + hex(base + hc12::clock_select) +
                " pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr, 2) + ": " + fault.what());
  }
}

McuWaitClockGates McuBus::wait_clock_gates() const {
  constexpr unsigned unsupported_crg_wait = 0x37;
  const auto selected = clock_.read(hc12::clock_select);
  if ((selected & unsupported_crg_wait) != 0) {
    const auto base = static_cast<unsigned>(memory_.read_register(hc12::initrg)) << 8U;
    throw Fault("Unqualified MCU wait clock at " + hex(base + hc12::clock_select) + " value=" +
                hex(selected, 2) + " pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr, 2));
  }
  return {.sci = sci_.runs_in_wait(), .timer = timer_.runs_in_wait()};
}

void McuBus::advance_oscillator(const unsigned cycles) {
  clock_.advance_oscillator(cycles);
}

unsigned McuBus::oscillator_cycles_until_rti() const {
  return clock_.oscillator_cycles_until_rti();
}

void McuBus::sync_peripheral_pins() {
  const PeripheralPinState state{.routing = gpio_.read(hc12::modrr),
                                 .timer = timer_.pins(),
                                 .sci_receive = sci_.receiver_enabled(),
                                 .sci_transmit = sci_.transmit_level()};
  if (peripheral_pins_ == state) { return; }
  // A failed rebuild must never leave a reusable key for partially applied pins.
  peripheral_pins_.reset();
  const auto routing = state.routing;
  const bool small_package = (routing & 1U) != 0;
  std::array<PinFunction, port_count> ports{};
  const auto timer_signals = state.timer;
  for (unsigned channel = 0; channel < 8; ++channel) {
    const auto bit = 1U << channel;
    route_function(ports,
                   {.source = bit, .pad = timer_pad({.channel = channel, .routing = routing})},
                   timer_signals);
    if (small_package && (routing & 4U) == 0) {
      const auto outputs = PinFunction{.outputs = timer_signals.outputs,
                                       .levels = timer_signals.levels,
                                       .unknown = timer_signals.unknown};
      route_function(
          ports,
          {.source = bit, .pad = {.port = McuPort::b, .mask = static_cast<std::uint8_t>(bit)}},
          outputs);
    }
  }
  const auto transmit = state.sci_transmit;
  const auto tx_mask = static_cast<std::uint8_t>(small_package ? small_tx : 8);
  const auto rx_mask = static_cast<std::uint8_t>(small_package ? 16 : 4);
  const auto port = small_package ? McuPort::s : McuPort::r;
  const auto sci_signals =
      PinFunction{.inputs = static_cast<std::uint8_t>(state.sci_receive ? 1 : 0),
                  .outputs = static_cast<std::uint8_t>(transmit ? 2 : 0),
                  .levels = static_cast<std::uint8_t>(transmit && *transmit ? 2 : 0)};
  route_function(ports, {.source = 1, .pad = {.port = port, .mask = rx_mask}}, sci_signals);
  route_function(ports, {.source = 2, .pad = {.port = port, .mask = tx_mask}}, sci_signals);
  for (unsigned index = 0; index < ports.size(); ++index) {
    gpio_.set_function(static_cast<McuPort>(index), ports[index]);
  }
  peripheral_pins_ = state;
}

void McuBus::advance_sci(const unsigned bus_cycles) {
  if (bus_cycles == 0) { return; }
  constexpr unsigned maximum_cycles = 1048576;
  if (bus_cycles > maximum_cycles) { throw Fault("SCI board clock slice exceeds cycle limit"); }
  const bool small_package = (gpio_.read(hc12::modrr) & 1U) != 0;
  const auto rx_mask = static_cast<std::uint8_t>(small_package ? 16 : 4);
  const auto tx_mask = static_cast<std::uint8_t>(small_package ? small_tx : 8);
  const auto port = small_package ? McuPort::s : McuPort::r;
  unsigned remaining = bus_cycles;
  while (remaining != 0) {
    const auto until_sample = sci_.bus_cycles_until_sample();
    const auto count = until_sample == 0 ? remaining : std::min(remaining, until_sample);
    std::optional<bool> receive;
    if (until_sample != 0 && count == until_sample && sci_.receiver_enabled()) {
      receive = (gpio_.read_pad(port, rx_mask) & rx_mask) != 0;
    }
    const auto previous = sci_.transmit_level();
    sci_.advance({.bus_cycles = count, .receive_level = receive});
    remaining -= count;
    if (sci_.transmit_level() != previous) { sync_peripheral_pins(); }
    if (sci_.transmit_level()) { static_cast<void>(gpio_.read_pad(port, tx_mask)); }
  }
}

void McuBus::sample_timer_inputs(const std::uint8_t mask) {
  const auto routing = gpio_.read(hc12::modrr);
  unsigned levels = 0;
  for (unsigned channel = 0; channel < 8; ++channel) {
    const auto bit = 1U << channel;
    if ((mask & bit) == 0) { continue; }
    const auto pad = timer_pad({.channel = channel, .routing = routing});
    if ((gpio_.read_pad(pad.port, pad.mask) & pad.mask) != 0) { levels |= bit; }
  }
  timer_.observe_inputs({.known = mask, .levels = static_cast<std::uint8_t>(levels)});
}

void McuBus::check_timer_outputs() const {
  const auto routing = gpio_.read(hc12::modrr);
  const auto function = timer_.pins();
  for (unsigned channel = 0; channel < 8; ++channel) {
    const auto bit = 1U << channel;
    if ((function.outputs & ~static_cast<unsigned>(function.unknown) & bit) == 0) { continue; }
    const auto pad = timer_pad({.channel = channel, .routing = routing});
    static_cast<void>(gpio_.read_pad(pad.port, pad.mask));
    if ((routing & 5U) == 1) {
      static_cast<void>(gpio_.read_pad(McuPort::b, static_cast<std::uint8_t>(bit)));
    }
  }
}

void McuBus::advance_timer(const unsigned bus_cycles) {
  try {
    constexpr unsigned maximum_cycles = 1048576;
    if (bus_cycles > maximum_cycles) { throw Fault("Timer board clock slice exceeds cycle limit"); }
    unsigned remaining = bus_cycles;
    while (remaining != 0) {
      const auto until_tick = timer_.cycles_until_tick();
      const auto count = until_tick == 0 ? remaining : std::min(remaining, until_tick);
      timer_.advance(count);
      remaining -= count;
      sync_peripheral_pins();
      check_timer_outputs();
    }
  } catch (const Fault &fault) {
    throw Fault("MCU timer advance at " + hex(hc12::timer_base) + " pc=" + hex(context_.pc) +
                " ccr=" + hex(context_.ccr, 2) + " cycles=" + std::to_string(bus_cycles) + ": " +
                fault.what());
  }
}

void McuBus::set_observer(std::function<void(const BusEvent &)> observer) {
  if (notifying_) { throw Fault("Cannot replace an active MCU bus observer"); }
  observer_ = std::move(observer);
}

void McuBus::notify(const BusEvent &event) {
  if (!observer_) { return; }
  const ObserverGuard guard(notifying_);
  observer_(event);
}

}
