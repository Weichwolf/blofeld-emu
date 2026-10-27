// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "hcs08_fixture.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_vector_memory.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace blofeld::test::swi {
using hcs08::require;
constexpr unsigned entry = 0xe000, handler = 0xe100, distractor = 0xe200;
constexpr unsigned vector = 0xfffc, ram_first = 0x80, ram_end = 0x480;
constexpr unsigned byte_mask = 0xff, high_known = 0xff00, fixed_flags = 0x60, mask_irq = 8;

struct Configuration {
  unsigned protection, options, physical, target;
};

// Independent GT8A examples include relocated SWI bytes and direct-read exclusions.
constexpr std::array<Configuration, 6> cases{
    {{.protection = 0xfc, .options = 2, .physical = 0xfdfc, .target = handler},
     {.protection = 0xfa, .options = 2, .physical = 0xfbfc, .target = handler},
     {.protection = 0xe0, .options = 2, .physical = 0xe1fc, .target = handler},
     {.protection = 0xfc, .options = 0x42, .physical = vector, .target = distractor},
     {.protection = 0xff, .options = 2, .physical = vector, .target = distractor},
     {.protection = 0xde, .options = 2, .physical = vector, .target = distractor}}};

inline void put(Hcs08FlashImage &image, const unsigned address, const std::uint8_t value) {
  image.data.at(address - hcs08_flash::first) = value;
  image.known.set(address - hcs08_flash::first);
  image.fresh.reset(address - hcs08_flash::first);
}

inline Hcs08FlashImage own_image(const Configuration config,
                                 const std::span<const std::uint8_t> code) {
  Hcs08FlashImage image;
  image.data.fill(byte_mask);
  image.known.set();
  image.fresh.set();
  for (unsigned i = 0; i < code.size(); ++i) { put(image, entry + i, code[i]); }
  constexpr unsigned rti = 0x80;
  constexpr unsigned nop = 0x9d;
  constexpr unsigned shift = 8;
  for (const unsigned target : {handler, distractor}) {
    put(image, target, rti);
    put(image, target + 1, nop);
    put(image, target + 2, nop);
  }
  put(image, hcs08_flash::nv_options, static_cast<std::uint8_t>(config.options));
  put(image, hcs08_flash::nv_protection, static_cast<std::uint8_t>(config.protection));
  put(image, vector, static_cast<std::uint8_t>(distractor >> shift));
  put(image, vector + 1, 0);
  put(image, config.physical, static_cast<std::uint8_t>(config.target >> shift));
  put(image, config.physical + 1, 0);
  put(image, vector + 2, static_cast<std::uint8_t>(entry >> shift));
  put(image, vector + 3, 0);
  return image;
}

// Untimed CPU bus with actual FLASH ownership and explicit raw SRAM masks.
// It supplies no clock, peripheral success, IRQ acknowledgement or target PC.
class Memory final : public hcs08::MaskedMemory, public Hcs08VectorMemory {
public:
  explicit Memory(const Hcs08FlashImage &image) : flash(image, Hcs08FlashContract::candidate_gt8a) {
    for (unsigned i = ram_first; i < ram_end; ++i) { writable.set(i); }
  }

  std::uint8_t read(const std::uint16_t address) override {
    if (resetting) { return flash.read_reset_vector_byte(address); }
    if (address >= hcs08_flash::first) { return flash.read_array(address); }
    return hcs08::Memory::read(address);
  }

  std::uint8_t peek(const std::uint16_t address) const override {
    if (address >= hcs08_flash::first) { return flash.read_array(address); }
    fail(address, "peek outside own FLASH");
  }

  void check_execute(const InstructionFetch fetch) const override {
    require(fetch.length > 0 && fetch.address >= hcs08_flash::first &&
                fetch.length <= hcs08::space_bytes - fetch.address,
            "SWI fixture fetch requires own bounded FLASH");
    for (unsigned i = 0; i < fetch.length; ++i) {
      static_cast<void>(flash.read_array(static_cast<std::uint16_t>(fetch.address + i)));
    }
  }

  void begin_instruction(const ExecutionContext value) override {
    context = value;
    flash.context(value);
  }

  void begin_reset(const ExecutionContext value) override {
    begin_instruction(value);
    resetting = true;
  }

  void end_reset() override { resetting = false; }

  std::uint8_t read_interrupt_vector_byte(const std::uint16_t address) override {
    vector_reads.push_back(address);
    const auto value = flash.read_interrupt_vector_byte(address);
    accesses.push_back({.address = address, .value = value, .write = false});
    return value;
  }

  Hcs08Flash flash;
  std::vector<std::uint16_t> vector_reads;
  bool resetting = false;
};

struct Saved {
  Memory memory;
  Hcs08Checkpoint cpu;
  unsigned phase;
};

struct Unit {
  Memory memory;
  Hcs08 cpu;

  explicit Unit(const Hcs08FlashImage &image, const std::string &spec = BLOFELD_HCS08_SLA)
      : memory(image), cpu(memory, spec, Hcs08RtiIrqContract::reference_immediate) {
    cpu.reset(Hcs08ResetEntry::normal, Hcs08ResetCause::power_on);
  }

  explicit Unit(const Saved &saved)
      : memory(saved.memory),
        cpu(memory, BLOFELD_HCS08_SLA, Hcs08RtiIrqContract::reference_immediate) {
    cpu.restore_checkpoint(saved.cpu);
  }
};
} // namespace blofeld::test::swi
