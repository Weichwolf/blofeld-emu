// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"

#include "blofeld/bus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

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
constexpr unsigned ms = 0x80;
constexpr unsigned msw = 0x600000;
constexpr unsigned switch_cycles = 3;

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
  shi_.reset();
  once_latch_.reset();
}

DspBus::Location DspBus::locate(const DspAddress access) const {
  static_cast<void>(bank(access.space));
  const auto address = access.address;
  if (address > word_mask) { throw Fault("DSP address exceeds 24 bits: " + address_text(access)); }
  if (switch_delay_ != 0 &&
      ((access.space == DspSpace::p && address >= program_ram_end && address < x_low_end) ||
       (access.space == DspSpace::x && address >= x_switched_end && address < x_high_end))) {
    throw Fault("DSP memory access requires elapsed switch cycles: " + address_text(access));
  }
  return access.space == DspSpace::p ? locate_program(address) : locate_data(access);
}

DspBus::Location DspBus::locate_program(const std::uint32_t address) const {
  if (address < program_ram_end) { return {.offset = address}; }
  if (switched_ && address < x_low_end) {
    return {.bank = 1,
            .offset = x_low_end + x_switched_end - data_rom_end + address - program_ram_end};
  }
  if (address >= program_rom_base) { return {.offset = address - program_rom_base, .rom = true}; }
  throw Fault("Unmapped DSP memory at " + address_text({.address = address}));
}

DspBus::Location DspBus::locate_data(const DspAddress access) const {
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
  throw Fault("Unmapped DSP memory at " + address_text(access));
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
  const auto location = locate(target);
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

std::uint32_t DspBus::read_peripheral(const DspAddress access) {
  constexpr unsigned first_shi = 0xffff90;
  constexpr unsigned last_shi = 0xffff94;
  constexpr unsigned first_priority = 0xfffffe;
  if (access.space == DspSpace::x && access.address >= first_shi && access.address <= last_shi) {
    return shi_.read(access.address);
  }
  if (access.space == DspSpace::x && access.address >= first_priority) { return 0; }
  throw Fault("Unimplemented DSP peripheral read at " + address_text(access));
}

void DspBus::write_peripheral(const DspStore access) {
  constexpr unsigned first_shi = 0xffff90;
  constexpr unsigned last_shi = 0xffff94;
  constexpr unsigned first_priority = 0xfffffe;
  constexpr unsigned once_gdb = 0xfffffc;
  if (access.space == DspSpace::x && access.address == once_gdb) {
    once_latch_ = access.value;
    return;
  }
  if (access.space == DspSpace::x && access.address >= first_shi && access.address <= last_shi) {
    shi_.write({.address = access.address, .value = access.value});
    return;
  }
  // Only the disabled interrupt-controller state is implemented. Enables cannot
  // appear successful until CPU delivery and derivative priorities are modeled.
  if (access.space == DspSpace::x && access.address >= first_priority && access.value == 0) {
    return;
  }
  throw Fault("Unimplemented DSP peripheral write at " +
              address_text({.space = access.space, .address = access.address}) +
              " value=" + hex(access.value, 6));
}

}
