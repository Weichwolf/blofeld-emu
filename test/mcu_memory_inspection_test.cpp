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
}

int main() {
  try {
    unknown_and_storage_classes();
    mapping_and_lifetime();
    no_storage_or_mapping_side_effects();
    std::cout << "Storage inspection: literal knowledge/alias/reset/copy and purity cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
