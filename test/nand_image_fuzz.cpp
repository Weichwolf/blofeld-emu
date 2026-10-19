// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/nand.hpp"
#include "blofeld/nand_image.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <zconf.h>
#include <zlib.h>

extern "C" std::size_t LLVMFuzzerMutate(std::uint8_t *data, std::size_t size, std::size_t maximum);

namespace {
void address(blofeld::Nand &device, const unsigned page) {
  constexpr unsigned mask = 0xff;
  device.address(0);
  device.address(static_cast<std::uint8_t>(page & mask));
  device.address(static_cast<std::uint8_t>((page >> 8U) & mask));
  device.address(static_cast<std::uint8_t>(page >> 16U));
}

bool exercise_read(blofeld::Nand device, const unsigned page) {
  try {
    device.observe_power_ready();
    device.command(0);
    address(device, page);
    device.observe_completion();
    static_cast<void>(device.read());
    return true;
  } catch (const blofeld::Fault &) {
    return false; // Unobserved contents reject this read branch.
  }
}

bool exercise_write(blofeld::Nand device, const unsigned page, const bool pass) {
  try {
    device.observe_power_ready();
    device.observe_wp(true);
    constexpr unsigned program = 0x80;
    constexpr unsigned confirm = 0x10;
    device.command(program);
    address(device, page);
    device.write(0);
    device.command(confirm);
    device.observe_completion(pass ? blofeld::NandOutcome::pass : blofeld::NandOutcome::fail);
    device.power_loss();
    static_cast<void>(blofeld::encode_nand_image(device.persistent_image()));
    return true;
  } catch (const blofeld::Fault &) {
    return false; // Unknown permission/history must not become writable.
  }
}

struct Mutation {
  std::uint8_t *data;
  std::size_t size;
  std::size_t capacity;
  unsigned seed;
};

std::size_t mutate(const Mutation &request) {
  auto *data = request.data;
  const auto mutated = LLVMFuzzerMutate(data, request.size, request.capacity);
  constexpr unsigned mask = 0xff;
  if (request.seed % 2 == 0 &&
      mutated >= blofeld::nand_image_header_bytes + sizeof(std::uint32_t) &&
      mutated <= blofeld::nand_image_max_bytes) {
    const auto end = mutated - sizeof(std::uint32_t);
    const auto crc = static_cast<std::uint32_t>(crc32(0, data, static_cast<uInt>(end)));
    for (unsigned index = 0; index < 4; ++index) {
      data[end + index] = static_cast<std::uint8_t>((crc >> (index * 8U)) & mask);
    }
  }
  return mutated;
}
}

extern "C" std::size_t LLVMFuzzerCustomMutator(std::uint8_t *data,
                                               const std::size_t size,
                                               const std::size_t maximum,
                                               const unsigned seed) {
  return mutate({.data = data, .size = size, .capacity = maximum, .seed = seed});
}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, const std::size_t size) {
  try {
    const auto image = blofeld::decode_nand_image(std::span(data, size));
    const blofeld::Nand device(image);
    static_cast<void>(blofeld::encode_nand_image(device.persistent_image()));
    if (image.pages.empty()) { return 0; }
    const auto page = image.pages.begin()->first;
    static_cast<void>(exercise_read(device, page));
    static_cast<void>(exercise_write(device, page, (data[size - 1] & 1U) != 0));
  } catch (const blofeld::Fault &) {
    // Header, geometry, CRC and physical-state invariant rejections are expected.
    return 0;
  }
  return 0;
}
