// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_interrupts.hpp"
#include "blofeld/dsp_timers.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace blofeld {
namespace {
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned program_rom_base = 0xff0000;
constexpr unsigned data_rom_base = 0x4000;
constexpr unsigned data_rom_end = 0xc000;
constexpr unsigned program_ram_end = 0x1000;
constexpr unsigned x_low_end = 0x3000;
constexpr unsigned y_low_end = 0x2000;
constexpr unsigned x_high_end = 0x12000;
constexpr unsigned y_high_end = 0x16000;
constexpr unsigned x_switched_end = 0x10000;
constexpr unsigned io_base = 0xffff80;
constexpr unsigned esai_offset_mask = 0x1f;
constexpr unsigned ms = 0x80;
constexpr unsigned msw = 0x600000;
constexpr unsigned switch_cycles = 3;
constexpr unsigned bcr_reset = 0x1fffff;
constexpr unsigned port_f_bus_control = 0x01ffff;

std::uint32_t
change_bit(const std::uint32_t value, const std::uint32_t mask, const DspBitOperation operation) {
  switch (operation) {
    case DspBitOperation::test: return value;
    case DspBitOperation::clear: return value & ~mask;
    case DspBitOperation::set: return value | mask;
    case DspBitOperation::change: return value ^ mask;
    default: throw Fault("Invalid DSP peripheral bit operation");
  }
}

struct GpioRegister {
  DspPort port = DspPort::c;
  DspGpioRegister reg = DspGpioRegister::data;
};

std::optional<GpioRegister> gpio_register(const DspAddress access) {
  constexpr unsigned port_c_base = 0xffffbd;
  constexpr unsigned port_e_base = 0xffff9d;
  constexpr unsigned port_f_base = 0xfffff5;

  struct Mapping {
    DspPort port;
    DspSpace space;
    unsigned base;
  };

  constexpr std::array<Mapping, 3> ports{
      {{.port = DspPort::c, .space = DspSpace::x, .base = port_c_base},
       {.port = DspPort::e, .space = DspSpace::y, .base = port_e_base},
       {.port = DspPort::f, .space = DspSpace::y, .base = port_f_base}}};
  for (const auto &mapping : ports) {
    if (access.space == mapping.space && access.address >= mapping.base &&
        access.address - mapping.base < 3) {
      return GpioRegister{.port = mapping.port,
                          .reg = static_cast<DspGpioRegister>(access.address - mapping.base)};
    }
  }
  return std::nullopt;
}

std::size_t bank(const DspSpace space) {
  switch (space) {
    case DspSpace::p: return 0;
    case DspSpace::x: return 1;
    case DspSpace::y: return 2;
    default: throw Fault("Invalid DSP memory space");
  }
}

std::string address_text(const DspAddress access) {
  constexpr std::array<const char *, 3> names{"P:", "X:", "Y:"};
  return std::string(names.at(bank(access.space))) + hex(access.address, 6);
}
}

DspBus::DspBus() {
  constexpr std::array<std::size_t, 3> ram_words{4096, 36864, 49152};
  constexpr std::array<std::size_t, 3> rom_words{65536, 32768, 32768};
  for (std::size_t index = 0; index < ram_.size(); ++index) {
    ram_[index].words.resize(ram_words[index]);
    ram_[index].known.resize(ram_words[index]);
    rom_[index].words.resize(rom_words[index]);
    rom_[index].known.resize(rom_words[index]);
  }
  reset();
}

void DspBus::reset() {
  switched_ = false;
  requested_switch_ = false;
  switch_delay_ = 0;
  changes_ = {.all = true, .addresses = {}};
  reset_peripherals();
  pll_.reset();
  bus_control_ = bcr_reset;
  once_latch_.reset();
}

void DspBus::reset_peripherals() {
  interrupts_.reset();
  shi_.reset();
  timers_.reset();
  for (auto &port : gpio_) { port.reset(); }
  for (auto &serial : esai_) { serial.reset(); }
}

DspGpio &DspBus::gpio(const DspPort port) {
  const auto index = static_cast<std::size_t>(port);
  if (index >= gpio_.size()) { throw Fault("Invalid DSP GPIO port"); }
  return gpio_.at(index);
}

std::optional<bool> DspBus::access_peripheral_bit(const DspBitAccess access) {
  constexpr unsigned word_bits = 24;
  const DspAddress target{.space = access.space, .address = access.address};
  static_cast<void>(bank(access.space));
  if (access.address > word_mask || access.bit >= word_bits ||
      static_cast<unsigned>(access.operation) > static_cast<unsigned>(DspBitOperation::change)) {
    throw Fault("Invalid DSP bit access at " + address_text(target));
  }
  const auto mask = 1U << access.bit;
  if (access.space == DspSpace::x && DspTimers::is_control(access.address)) {
    constexpr unsigned timer_input = 0x1000;
    const bool old = (timers_.read(access.address, mask) & mask) != 0;
    if (access.operation != DspBitOperation::test) {
      // DI is read-only. All other TCSR fields participate in word writeback,
      // including unselected W1C flags, exactly as in a hardware RMW access.
      const auto current = timers_.read(access.address, word_mask & ~timer_input);
      timers_.write(
          {.address = access.address, .value = change_bit(current, mask, access.operation)});
    }
    return old;
  }
  const auto mapping = gpio_register(target);
  if (!mapping.has_value()) { return std::nullopt; }
  if (mapping->port == DspPort::f && bus_control_ != port_f_bus_control) {
    throw Fault("Unqualified DSP GPIO bit access at " + address_text(target) +
                ": Port F requires BCR=0x01ffff");
  }
  auto &port = gpio(mapping->port);
  const bool old = (port.read(mapping->reg, mask) & mask) != 0;
  if (access.operation == DspBitOperation::test) { return old; }
  const auto value = change_bit(old ? mask : 0U, mask, access.operation);
  port.write({.reg = mapping->reg, .value = value, .mask = mask});
  sync_esai_port(mapping->port);
  return old;
}

DspBus::Location DspBus::locate(const DspAddress access, const Operation operation) const {
  static_cast<void>(bank(access.space));
  const auto address = access.address;
  if (address > word_mask) { throw Fault("DSP address exceeds 24 bits: " + address_text(access)); }
  if (switch_delay_ != 0 &&
      ((access.space == DspSpace::p && address >= program_ram_end && address < x_low_end) ||
       (access.space == DspSpace::x && address >= x_switched_end && address < x_high_end))) {
    throw Fault("DSP memory access requires elapsed switch cycles: " + address_text(access));
  }
  return access.space == DspSpace::p ? locate_program(address, operation)
                                     : locate_data(access, operation);
}

DspBus::Location DspBus::locate_program(const std::uint32_t address,
                                        const Operation operation) const {
  if (address < program_ram_end) { return {.offset = address}; }
  if (switched_ && address < x_low_end) {
    return {.bank = 1,
            .offset = x_low_end + x_switched_end - data_rom_end + address - program_ram_end};
  }
  if (address >= program_rom_base) { return {.offset = address - program_rom_base, .rom = true}; }
  throw Fault(std::string("Unmapped DSP memory ") +
              (operation == Operation::read ? "read at " : "write at ") +
              address_text({.address = address}));
}

DspBus::Location DspBus::locate_data(const DspAddress access, const Operation operation) const {
  const auto address = access.address;
  const auto index = bank(access.space);
  const auto low_end = access.space == DspSpace::x ? x_low_end : y_low_end;
  const auto x_end = switched_ ? x_switched_end : x_high_end;
  const auto high_end = access.space == DspSpace::x ? x_end : y_high_end;
  if (address < low_end) { return {.bank = index, .offset = address}; }
  if (address >= data_rom_base && address < data_rom_end) {
    return {.bank = index, .offset = address - data_rom_base, .rom = true};
  }
  if (address >= data_rom_end && address < high_end) {
    return {.bank = index, .offset = low_end + address - data_rom_end};
  }
  throw Fault(std::string("Unmapped DSP memory ") +
              (operation == Operation::read ? "read at " : "write at ") + address_text(access));
}

std::uint32_t DspBus::read(const DspAddress access) {
  if (access.space != DspSpace::p && access.address >= io_base && access.address <= word_mask) {
    return read_peripheral(access);
  }
  const auto location = locate(access);
  const auto &memory = (location.rom ? rom_ : ram_).at(location.bank);
  if (memory.known.at(location.offset) == 0) {
    throw Fault("Unknown DSP memory read at " + address_text(access));
  }
  return memory.words.at(location.offset);
}

void DspBus::write(const DspStore access) {
  const DspAddress target{.space = access.space, .address = access.address};
  if (access.value > word_mask) {
    throw Fault("DSP write exceeds 24 bits at " + address_text(target));
  }
  if (access.space != DspSpace::p && access.address >= io_base && access.address <= word_mask) {
    write_peripheral(access);
    return;
  }
  const auto location = locate(target, Operation::write);
  if (location.rom) { throw Fault("Protected DSP ROM write at " + address_text(target)); }
  auto &memory = ram_.at(location.bank);
  memory.words.at(location.offset) = access.value;
  memory.known.at(location.offset) = 1;
  if (access.space == DspSpace::p) { changes_.addresses.push_back(access.address); }
}

std::uint32_t DspBus::peek_program(const std::uint32_t address) const {
  try {
    const auto location = locate({.address = address});
    const auto &memory = (location.rom ? rom_ : ram_).at(location.bank);
    return memory.known.at(location.offset) != 0 ? memory.words.at(location.offset) : 0;
  } catch (const Fault &) {
    return 0; // Decoder lookahead only; actual extension/fetch always uses strict read().
  }
}

void DspBus::install_rom(const DspRom image) {
  const auto index = bank(image.space);
  const auto base = image.space == DspSpace::p ? program_rom_base : data_rom_base;
  auto &memory = rom_.at(index);
  if (image.start < base || image.start - base > memory.words.size() ||
      image.words.size() > memory.words.size() - (image.start - base)) {
    throw Fault("DSP ROM installation exceeds its documented window");
  }
  for (const auto value : image.words) {
    if (value > word_mask) { throw Fault("DSP ROM input word exceeds 24 bits"); }
  }
  for (std::size_t index_word = 0; index_word < image.words.size(); ++index_word) {
    const auto offset = image.start - base + index_word;
    memory.words.at(offset) = image.words[index_word];
    memory.known.at(offset) = 1;
  }
  changes_.all = true;
}

void DspBus::observe_omr(const std::uint32_t value) {
  if (value > word_mask) { throw Fault("DSP OMR exceeds 24 bits"); }
  const bool requested = (value & ms) != 0;
  if (requested && (value & msw) != msw) {
    throw Fault("Unimplemented DSP56371 memory-switch configuration OMR=" + hex(value, 6));
  }
  if (requested == requested_switch_) { return; }
  if (switch_delay_ != 0) { throw Fault("Unverified overlapping DSP memory-switch requests"); }
  requested_switch_ = requested;
  switch_delay_ = switch_cycles;
}

void DspBus::advance_instruction_cycles(const unsigned cycles) {
  shi_.advance_instruction_cycles(cycles);
  if (switch_delay_ == 0) { return; }
  if (cycles < switch_delay_) {
    switch_delay_ -= cycles;
  } else {
    switch_delay_ = 0;
    switched_ = requested_switch_;
    changes_.all = true;
  }
}

DspCodeChanges DspBus::take_code_changes() {
  return std::exchange(changes_, DspCodeChanges{});
}

std::optional<DspInterruptRequest> DspBus::interrupt_request(const unsigned mask) const {
  constexpr unsigned shi_receive = 0x44;
  constexpr unsigned timer_compare = 0x54;
  std::vector<std::uint32_t> pending;
  const auto shi_level = interrupts_.priority(shi_receive);
  if (shi_level && *shi_level >= mask) { pending = shi_.interrupt_vectors(); }
  const auto timer_level = interrupts_.priority(timer_compare);
  if (timer_level && *timer_level >= mask) {
    const auto timer = timers_.interrupt_vectors();
    pending.insert(pending.end(), timer.begin(), timer.end());
  }
  return interrupts_.select(pending, mask);
}

void DspBus::acknowledge_interrupt(const DspInterruptRequest request) {
  const auto current = interrupt_request(request.level);
  if (!current || current->vector != request.vector || current->level != request.level) {
    throw Fault("DSP interrupt fetch has no matching pending source");
  }
  constexpr unsigned first_timer = 0x54;
  if (request.vector >= first_timer) { timers_.acknowledge_interrupt(request.vector); }
}

std::uint32_t DspBus::read_peripheral(const DspAddress access) {
  if (auto *const serial = esai_register(access)) {
    return serial->read(access.address & esai_offset_mask);
  }
  constexpr unsigned first_shi = 0xffff90;
  constexpr unsigned last_shi = 0xffff94;
  constexpr unsigned pll_control = 0xfffffd;
  constexpr unsigned bus_control = 0xfffffb;
  if (access.space == DspSpace::x && DspTimers::handles(access.address)) {
    return timers_.read(access.address);
  }
  if (const auto target = gpio_register(access)) {
    if (target->port == DspPort::f && bus_control_ != port_f_bus_control) {
      throw Fault("Unqualified DSP GPIO read at " + address_text(access) +
                  ": Port F requires BCR=0x01ffff");
    }
    return gpio(target->port).read(target->reg);
  }
  if (access.space == DspSpace::x && access.address == bus_control) {
    if (bus_control_ != bcr_reset) {
      throw Fault("Unknown DSP BCR read at X:0xfffffb: BIU bus-master state is unobserved");
    }
    return bus_control_;
  }
  if (access.space == DspSpace::x && access.address == pll_control) { return pll_.read(); }
  if (access.space == DspSpace::x && access.address >= first_shi && access.address <= last_shi) {
    return shi_.read(access.address);
  }
  if (access.space == DspSpace::x && DspInterrupts::handles(access.address)) {
    return interrupts_.read(access.address);
  }
  throw Fault("Unimplemented DSP peripheral read at " + address_text(access));
}

void DspBus::write_peripheral(const DspStore access) {
  if (auto *const serial = esai_register({.space = access.space, .address = access.address})) {
    serial->write(access.address & esai_offset_mask, access.value);
    return;
  }
  constexpr unsigned first_shi = 0xffff90;
  constexpr unsigned last_shi = 0xffff94;
  constexpr unsigned once_gdb = 0xfffffc;
  constexpr unsigned pll_control = 0xfffffd;
  constexpr unsigned bus_control = 0xfffffb;
  if (access.space == DspSpace::x && DspTimers::handles(access.address)) {
    timers_.write({.address = access.address, .value = access.value});
    return;
  }
  if (const auto target = gpio_register({.space = access.space, .address = access.address})) {
    if (target->port == DspPort::f && bus_control_ != port_f_bus_control) {
      throw Fault("Unqualified DSP GPIO write at " +
                  address_text({.space = access.space, .address = access.address}) +
                  ": Port F requires BCR=0x01ffff");
    }
    gpio(target->port).write({.reg = target->reg, .value = access.value});
    sync_esai_port(target->port);
    return;
  }
  if (access.space == DspSpace::x && access.address == bus_control) {
    if (access.value != port_f_bus_control) {
      throw Fault("Unsupported DSP BCR write at X:0xfffffb value=" + hex(access.value, 6));
    }
    bus_control_ = access.value;
    return;
  }
  if (access.space == DspSpace::x && access.address == pll_control) {
    pll_.write(access.value);
    return;
  }
  if (access.space == DspSpace::x && access.address == once_gdb) {
    once_latch_ = access.value;
    return;
  }
  if (access.space == DspSpace::x && access.address >= first_shi && access.address <= last_shi) {
    shi_.write({.address = access.address, .value = access.value});
    return;
  }
  if (access.space == DspSpace::x && DspInterrupts::handles(access.address)) {
    interrupts_.write(access.address, access.value);
    return;
  }
  throw Fault("Unimplemented DSP peripheral write at " +
              address_text({.space = access.space, .address = access.address}) +
              " value=" + hex(access.value, 6));
}

}
