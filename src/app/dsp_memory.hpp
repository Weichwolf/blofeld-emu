// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp/boot_rom.hpp"
#include "dsp/memory.hpp"
#include "memory/address_space.hpp"

#include <array>
#include <cstddef>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <memory>

namespace blofeld::app {
[[nodiscard]] dsp56300::CompileOptions::MemoryMode dsp_memory_mode();

// App-owned host mappings; the VM borrows only physical words and typed views.
class DspMemory {
public:
  explicit DspMemory(vi::dsp::BootRom boot);
  [[nodiscard]] vi::dsp::Memory::Storage storage() const noexcept;

private:
  void mapping(std::size_t index, bool has_eeprom);
  std::size_t page_;
  host::memory::Backing backing_;
  std::array<std::unique_ptr<host::memory::AddressSpace>, vi::dsp::Memory::mapping_count * 6>
      spaces_;
  std::array<std::array<dsp56300::MemoryWindow, 2>, 5> program_;
  std::array<vi::dsp::Memory::Mapping, 5> mappings_;
};
}
