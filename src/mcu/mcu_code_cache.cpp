// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_memory.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace blofeld {

void McuBus::note_code_byte(const std::uint16_t address) const {
  decoded_bytes_.set(address);
}

void McuBus::invalidate_code(const MemoryRead access) {
  if (ram_program_active_ && ram_program_ &&
      access.address < unsigned{ram_program_->first} + ram_program_->size &&
      ram_program_->first < unsigned{access.address} + access.size) {
    // Retire on the first write, not after every byte of a replacement image.
    // The read-only native grant routes this write through its owner and yields.
    retire_ram_program();
    ++code_revision_;
    ++ram_program_revisions_;
    return;
  }
  for (unsigned index = 0; index < access.size; ++index) {
    if (decoded_bytes_.test(static_cast<unsigned>(access.address) + index)) {
      ++code_revision_;
      return;
    }
  }
}

void McuBus::configure_ram_program(const std::optional<McuProgramWindow> program) {
  constexpr unsigned address_space = 65536;
  if (program && (program->size == 0 || program->size > address_space - program->first)) {
    throw Fault("Invalid MCU RAM program window");
  }
  if (program == ram_program_) { return; }
  retire_ram_program();
  ram_program_ = program;
  ++code_revision_;
}

void McuBus::qualify_ram_program(const std::uint16_t pc) {
  if (ram_program_active_ || !ram_program_ || pc < ram_program_->first ||
      unsigned{pc} - ram_program_->first >= ram_program_->size) {
    return;
  }
  const auto first = memory_.locate(ram_program_->first);
  if (first.space != McuSpace::smram && first.space != McuSpace::qram) { return; }
  std::vector<std::uint8_t> bytes;
  bytes.reserve(ram_program_->size);
  for (unsigned offset = 0; offset < ram_program_->size; ++offset) {
    const auto byte = memory_.inspect(static_cast<std::uint16_t>(ram_program_->first + offset));
    if (!byte.value || byte.space != first.space || byte.offset != first.offset + offset) {
      return;
    }
    bytes.push_back(*byte.value);
  }
  const auto image = std::ranges::find_if(
      ram_program_images_, [&bytes](const auto &candidate) { return *candidate == bytes; });
  ram_program_image_ = static_cast<std::size_t>(image - ram_program_images_.begin());
  if (image == ram_program_images_.end()) {
    ram_program_images_.push_back(std::make_shared<std::vector<std::uint8_t>>(std::move(bytes)));
  }
  // A complete image stays immutable even after RAM is overwritten. Reloading
  // identical bytes reuses its binding and already compiled native entries.
  ram_program_active_ = true;
  ++code_revision_;
  ++ram_program_revisions_;
}

void McuBus::retire_ram_program() {
  if (!ram_program_active_ || !ram_program_) { return; }
  ram_program_active_ = false;
  for (unsigned offset = 0; offset < ram_program_->size; ++offset) {
    decoded_bytes_.reset(ram_program_->first + offset);
  }
}

namespace {
McuMemoryWindow
slice(const McuMemoryWindow window, const std::size_t offset, const std::size_t size) {
  return {.first = static_cast<std::uint16_t>(window.first + offset),
          .bytes = window.bytes.subspan(offset, size),
          .known = window.known.empty() ? window.known : window.known.subspan(offset, size),
          .writable = window.writable};
}
}

std::vector<McuMemoryWindow> McuBus::program_windows(std::vector<McuMemoryWindow> windows) const {
  if (!ram_program_active_ || !ram_program_) { return windows; }
  std::vector<McuMemoryWindow> result;
  result.reserve(windows.size() + 2);
  for (const auto &window : windows) {
    const auto begin = std::max(std::size_t{window.first}, std::size_t{ram_program_->first});
    const auto end = std::min(window.first + window.bytes.size(),
                              std::size_t{ram_program_->first} + ram_program_->size);
    if (begin >= end) {
      result.push_back(window);
      continue;
    }
    const auto offset = begin - window.first;
    if (offset != 0) { result.push_back(slice(window, 0, offset)); }
    auto program = slice(window, offset, end - begin);
    program.bytes = std::span{*ram_program_images_.at(ram_program_image_)}.subspan(
        begin - ram_program_->first, end - begin);
    program.known = {};
    program.writable = false;
    result.push_back(program);
    const auto after = end - window.first;
    if (after < window.bytes.size()) {
      result.push_back(slice(window, after, window.bytes.size() - after));
    }
  }
  return result;
}

}
