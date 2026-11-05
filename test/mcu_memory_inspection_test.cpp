// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_memory.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned nop = 0xa7;
constexpr unsigned first_ram = 0x1200;
constexpr unsigned next_ram = first_ram + 1;
constexpr unsigned old_qram = 0x0400;
constexpr unsigned moved_qram = 0x2400;
constexpr unsigned relocated_qram = 0x21;
constexpr unsigned first_value = 0x5a;
constexpr unsigned next_value = 0xa5;
constexpr unsigned qram_value = 0x6b;
constexpr unsigned qram_offset = 0x0400;
constexpr unsigned flash_base = 0x8000;
constexpr unsigned flash_alias = 0x4000;
constexpr unsigned flash_hole = 0x20;
constexpr unsigned reserved_ram = 0x0800;
constexpr unsigned unmapped = 0x3000;
constexpr unsigned system_ram_size = 0x0e00;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

void rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &) { return; }
  throw Fault("Expected ordinary read refusal remains mandatory after inspection");
}

std::array<std::uint8_t, McuFlashImage::byte_count> flash() {
  std::array<std::uint8_t, McuFlashImage::byte_count> result{};
  result.fill(nop);
  return result;
}

void unknown_and_storage_classes() {
  const auto image = flash();
  McuMemory memory(image);
  require(memory.inspect(first_ram) == McuMemoryByte{.space = McuSpace::smram},
          "Cold backing zeros are unknown storage, not successful reads");
  require(memory.inspect(hc12::ptq) ==
              McuMemoryByte{.space = McuSpace::registers, .offset = hc12::ptq},
          "Inspection never samples a peripheral register or pin");
  require(memory.inspect(reserved_ram).space == McuSpace::reserved &&
              !memory.inspect(reserved_ram).value &&
              memory.inspect(unmapped).space == McuSpace::unmapped &&
              !memory.inspect(unmapped).value,
          "Reserved and absent storage retain their decoded selection and unknownness");
  require(memory.inspect(flash_base) == McuMemoryByte{.space = McuSpace::flash, .value = nop} &&
              memory.inspect(flash_alias) == memory.inspect(flash_base),
          "Known flash and its enabled alias expose the same actual storage identity");
  rejects([&] { static_cast<void>(memory.read(first_ram)); });
  memory.invalidate_flash({.offset = flash_hole, .size = 2});
  require(memory.inspect(flash_base + flash_hole) ==
                  McuMemoryByte{.space = McuSpace::flash, .offset = flash_hole} &&
              memory.inspect(flash_alias + flash_hole) == memory.inspect(flash_base + flash_hole),
          "Unknown flash is not silently converted to the ordinary peek placeholder");
  rejects([&] { static_cast<void>(memory.read(flash_base + flash_hole)); });
}

void mapping_and_lifetime() {
  const auto image = flash();
  McuMemory memory(image);
  memory.write({.address = first_ram, .value = first_value});
  memory.write({.address = old_qram, .value = qram_value});
  const auto retained = memory.inspect(first_ram);
  require(retained == McuMemoryByte{.space = McuSpace::smram, .value = first_value} &&
              !memory.inspect(next_ram).value,
          "Only written bytes become known and inspection retains independent values");
  memory.write_register({.address = hc12::initee, .value = relocated_qram});
  require(memory.inspect(old_qram) ==
              McuMemoryByte{.space = McuSpace::qram, .offset = qram_offset, .value = qram_value},
          "Observation cannot commit pending native mapping changes");
  require(memory.retire_instruction(), "The authored mapping change really commits at retirement");
  require(
      memory.inspect(moved_qram) ==
              McuMemoryByte{.space = McuSpace::qram, .offset = qram_offset, .value = qram_value} &&
          memory.inspect(old_qram).space == McuSpace::unmapped && !memory.inspect(old_qram).value,
      "Remapped QRAM follows its actual bank and offset, not a logical-address replay");
  memory.write({.address = first_ram, .value = next_value});
  require(retained.value == first_value && memory.inspect(first_ram).value == next_value,
          "Returned storage observations never alias subsequent guest writes");
  auto copy = memory;
  copy.write({.address = first_ram, .value = first_value});
  require(copy.inspect(first_ram).value == first_value &&
              memory.inspect(first_ram).value == next_value,
          "Inspection honors independent copied memory and its knownness");
  memory.reset();
  require(memory.inspect(first_ram).value == next_value &&
              memory.inspect(old_qram).value == qram_value,
          "Ordinary reset preserves physical RAM contents while restoring address mapping");
  memory.power_cycle();
  require(!memory.inspect(first_ram).value && !memory.inspect(old_qram).value,
          "Power cycling withdraws knowledge even when the backing array still contains old bytes");
  rejects([&] { static_cast<void>(memory.read(first_ram)); });
}

void no_storage_or_mapping_side_effects() {
  const auto image = flash();
  McuMemory memory(image);
  memory.write({.address = first_ram, .value = first_value});
  memory.invalidate_flash({.offset = flash_hole, .size = 2});
  const auto before = encode_flash_image(memory.flash_image());
  const auto mapping = memory.read_register(hc12::initee);
  for (const auto address :
       {first_ram, old_qram, unmapped, reserved_ram, flash_base, flash_base + flash_hole}) {
    static_cast<void>(memory.inspect(static_cast<std::uint16_t>(address)));
  }
  require(encode_flash_image(memory.flash_image()) == before &&
              memory.read_register(hc12::initee) == mapping &&
              memory.inspect(first_ram).value == first_value,
          "Inspection changes no cold cells, knowledge, flash programming history or mapping");
  require(!memory.retire_instruction(), "Inspection creates no pending mapping work");
}

void native_ram_knowledge() {
  const auto image = flash();
  McuMemory memory(image);
  auto windows = memory.ram_windows();
  constexpr unsigned visible_qram_size = 0x0200;
  require(windows[0].first == old_qram && windows[0].bytes.size() == visible_qram_size &&
              windows[1].first == first_ram && windows[1].bytes.size() == system_ram_size &&
              windows[2].bytes.empty() && windows[3].bytes.empty() && windows[4].bytes.empty(),
          "Reset grants expose only QRAM0400..05ff and SMRAM1200..1fff");
  require(windows[0].known.size() == visible_qram_size &&
              windows[1].known.size() == system_ram_size && windows[1].known.front() == 0,
          "Cold native RAM has explicit unknown-byte knowledge");
  windows[1].bytes.front() = first_value;
  rejects([&] { static_cast<void>(memory.read(first_ram)); });
  windows[1].known.front() = 1;
  require(memory.read(first_ram) == first_value,
          "A native store publishes the actual physical byte and its knowledge");
  memory.write({.address = next_ram, .value = next_value});
  require(windows[1].bytes[1] == next_value && windows[1].known[1] == 1,
          "Ordinary stores update the same borrowed byte and knowledge arrays");
  memory.reset();
  require(windows[1].known.front() == 1 && memory.read(first_ram) == first_value,
          "Reset retains physical RAM and live knowledge");
  auto copy = memory;
  auto copied = copy.ram_windows();
  copied[1].bytes.front() = next_value;
  require(copied[1].bytes.data() != windows[1].bytes.data() &&
              copied[1].known.data() != windows[1].known.data() &&
              memory.read(first_ram) == first_value && copy.read(first_ram) == next_value,
          "Copied owners expose independent storage and knowledge");
  memory.power_cycle();
  require(windows[1].known.front() == 0 && windows[1].bytes.front() == first_value,
          "Power cycling withdraws live knowledge without inventing initialized contents");
  rejects([&] { static_cast<void>(memory.read(first_ram)); });
}

void native_ram_decoder() {
  const auto image = flash();
  McuMemory memory(image);
  constexpr unsigned moved_registers = 0x18;
  constexpr unsigned split_first = 0x1800;
  constexpr unsigned split_end = 0x1c00;
  constexpr unsigned first_split_size = 0x0600;
  constexpr unsigned last_split_size = 0x0400;
  memory.write_register({.address = hc12::initrg, .value = moved_registers});
  require(memory.ram_windows()[1].bytes.size() == system_ram_size,
          "Pending mapping writes cannot change native grants before retirement");
  require(memory.retire_instruction(), "The register relocation commits");
  const auto split = memory.ram_windows();
  require(split[0].first == 0 && split[0].bytes.size() == first_split_size &&
              split[1].first == first_ram && split[1].bytes.size() == split_first - first_ram &&
              split[2].first == split_end && split[2].bytes.size() == last_split_size &&
              split[3].bytes.empty(),
          "Register1800..1bff splits SMRAM and reveals all QRAM0000..05ff");
  constexpr unsigned hidden_qram = 0x11;
  memory.write_register({.address = hc12::initee, .value = hidden_qram});
  require(memory.retire_instruction(), "The overlapping QRAM relocation commits");
  const auto hidden = memory.ram_windows();
  require(hidden[0].first == first_ram && hidden[0].bytes.size() == first_split_size &&
              hidden[1].first == split_end && hidden[1].bytes.size() == last_split_size &&
              hidden[2].bytes.empty(),
          "The entire SMRAM decode window hides QRAM, including its reserved prefix");
  McuMemory high(image);
  constexpr unsigned high_mapping = 0xe1;
  constexpr unsigned high_first = 0xf200;
  high.write_register({.address = hc12::initrm, .value = high_mapping});
  require(high.retire_instruction(), "The high SMRAM mapping commits");
  auto upper = high.ram_windows();
  require(upper[1].first == high_first && upper[1].bytes.size() == system_ram_size,
          "A native RAM grant may end exactly at the16-bit address-space boundary");
  upper[1].bytes.back() = first_value;
  upper[1].known.back() = 1;
  constexpr unsigned last_address = 0xffff;
  require(high.read(last_address) == first_value,
          "RAM decoder priority over high flash is retained for the final byte");
}
}

int main() {
  try {
    unknown_and_storage_classes();
    mapping_and_lifetime();
    no_storage_or_mapping_side_effects();
    native_ram_knowledge();
    native_ram_decoder();
    std::cout << "MCU storage inspection and native RAM grant cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
