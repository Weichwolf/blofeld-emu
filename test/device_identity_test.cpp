// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/device_identity.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned canary = 0xa5, erased = 0xff;
constexpr unsigned offset = 0x1e0, field_bytes = 32;

void require(const bool condition, const std::string_view text) {
  if (!condition) { throw Fault(std::string(text)); }
}

void rejected(const auto &operation, const std::string_view reason) {
  try {
    operation();
  } catch (const Fault &error) {
    require(std::string_view(error.what()).contains(reason),
            "Expected precise provisioning refusal");
    return;
  }
  throw Fault("Expected factory provisioning refusal");
}

McuFlashImage own_image() {
  McuFlashImage image;
  image.bytes.fill(canary);
  image.known.set();
  std::fill_n(image.bytes.begin() + offset, field_bytes, erased);
  return image;
}

void valid_serials() {
  const auto original = own_image();
  for (unsigned length = 1; length < field_bytes; ++length) {
    const std::string serial(length, 'E');
    const auto image = provision_virtual_identity(original, serial);
    require(image.known == original.known && image.fresh == original.fresh,
            "Factory data does not invent knowledge or erase history");
    for (unsigned n = 0; n < McuFlashImage::byte_count; ++n) {
      unsigned expected = original.bytes[n];
      if (n >= offset && n < offset + length) {
        expected = 'E';
      } else if (n == offset + length) {
        expected = 0;
      }
      require(image.bytes[n] == expected,
              "Only the bounded serial and actual terminator may differ");
    }
    require(original.bytes[offset] == erased && original.bytes[offset + length] == erased,
            "Reference/update bytes remain unchanged after factory image construction");
    const auto restored = decode_flash_image(encode_flash_image(image));
    require(restored.bytes == image.bytes && restored.known == image.known &&
                restored.fresh == image.fresh,
            "Cold NVM serialization retains the same complete virtual unit state");
    rejected([&] { (void)provision_virtual_identity(image, serial); }, "existing unit data");
  }
}

void invalid_and_unknown() {
  const auto original = own_image();
  const std::array<std::string, 6> invalid{
      "", std::string(field_bytes, 'E'), std::string("A\0B", 3), "A\nB", "\x80", "\x7f"};
  for (const auto &serial : invalid) {
    rejected([&] { (void)provision_virtual_identity(original, serial); }, "serial");
  }
  for (unsigned missing = 0; missing < field_bytes; ++missing) {
    auto image = original;
    image.invalidate({.offset = static_cast<std::uint16_t>(offset + missing), .size = 1});
    const auto before = image;
    rejected([&] { (void)provision_virtual_identity(image, "EMU-TEST-01"); },
             "unknown serial storage");
    require(image.bytes == before.bytes && image.known == before.known &&
                image.fresh == before.fresh,
            "Unknown factory contents are never filled or mutated after a refusal");
  }
}

void retained_history() {
  auto initial = own_image();
  for (unsigned n = 0; n < field_bytes / 2; ++n) { initial.fresh.set((offset / 2) + n); }
  constexpr std::string_view serial = "EMU-TEST-01";
  const auto result = provision_virtual_identity(initial, serial);
  for (unsigned word = 0; word < McuFlashImage::word_count; ++word) {
    const bool written = word >= offset / 2 && word <= (offset + serial.size()) / 2;
    require(result.fresh[word] == (initial.fresh[word] && !written),
            "Factory-programmed words consume their erase eligibility; untouched words retain it");
  }
  require(result.bytes[offset + serial.size()] == 0 && result.bytes[offset - 1] == canary &&
              result.bytes[offset + field_bytes] == canary,
          "The original C-string terminator stays within the protected resident data extent");
}
}

int main() {
  try {
    valid_serials();
    invalid_and_unknown();
    retained_history();
    std::cout << "Virtual factory identity preserves program bytes, existing units and cold NVM.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
