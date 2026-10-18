// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/mcu_memory.hpp"

#include <cstddef>
#include <cstdint>

namespace blofeld {
namespace {
constexpr unsigned array_bytes = 32768;
constexpr unsigned erased = 0xff;

void validate(const FlashRange range) {
  if (range.size == 0 || range.offset >= array_bytes || range.size > array_bytes - range.offset) {
    throw Fault("Invalid physical MCU flash range at " + hex(range.offset));
  }
}
}

McuMemory::McuMemory(const McuFlashImage &image) {
  image.validate();
  flash_ = image.bytes;
  flash_known_ = image.known;
  flash_fresh_ = image.fresh;
  reset();
}

McuFlashImage McuMemory::flash_image() const {
  McuFlashImage image{.bytes = flash_, .known = flash_known_, .fresh = flash_fresh_};
  for (std::size_t offset = 0; offset < image.bytes.size(); ++offset) {
    if (!image.known.test(offset)) { image.bytes[offset] = 0; }
  }
  return image;
}

std::uint8_t McuMemory::read_flash(const std::uint16_t offset) const {
  validate({.offset = offset, .size = 1});
  if (!flash_known_.test(offset)) {
    throw Fault("Unknown physical MCU flash byte at " + hex(offset));
  }
  return flash_[offset];
}

void McuMemory::check_flash_program(const std::uint16_t offset) const {
  validate({.offset = offset, .size = 2});
  if ((offset & 1U) != 0 || !flash_fresh_.test(offset / 2) || read_flash(offset) != erased ||
      read_flash(static_cast<std::uint16_t>(offset + 1)) != erased) {
    throw Fault("MCU flash word requires a known preceding emulated erase at " + hex(offset));
  }
}

void McuMemory::program_flash(const MemoryWrite physical_word) {
  if (physical_word.size != 2) { throw Fault("Physical MCU flash program requires a word"); }
  check_flash_program(physical_word.address);
  const auto offset = physical_word.address;
  flash_[offset] = static_cast<std::uint8_t>((physical_word.value >> 8U) & erased);
  flash_[static_cast<unsigned>(offset) + 1] =
      static_cast<std::uint8_t>(physical_word.value & erased);
  flash_fresh_.reset(offset / 2);
}

void McuMemory::erase_flash(const FlashRange range) {
  validate(range);
  constexpr unsigned sector_bytes = 512;
  if (range.offset % sector_bytes != 0 || range.size % sector_bytes != 0) {
    throw Fault("MCU flash erase requires complete aligned sectors");
  }
  for (unsigned offset = range.offset; offset < range.offset + range.size; ++offset) {
    flash_[offset] = erased;
    flash_known_.set(offset);
    flash_fresh_.set(offset / 2);
  }
}

void McuMemory::invalidate_flash(const FlashRange range) {
  validate(range);
  for (unsigned offset = range.offset; offset < range.offset + range.size; ++offset) {
    flash_known_.reset(offset);
    flash_fresh_.reset(offset / 2);
  }
}

}
