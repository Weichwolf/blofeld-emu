// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <zconf.h>
#include <zlib.h>

extern "C" std::size_t
LLVMFuzzerMutate(std::uint8_t *data, std::size_t size, std::size_t maximum_size);

namespace {
struct Mutation {
  std::uint8_t *data;
  std::size_t size;
  std::size_t capacity;
  unsigned seed;
};

std::size_t mutate(const Mutation &request) {
  auto *data = request.data;
  const auto mutated = LLVMFuzzerMutate(data, request.size, request.capacity);
  // Keep both raw corruptions and checksum-correct semantic mutations reachable.
  if (mutated == blofeld::flash_image_file_bytes && request.seed % 2 == 0) {
    constexpr auto checksum_offset = blofeld::flash_image_file_bytes - sizeof(std::uint32_t);
    const auto crc = static_cast<std::uint32_t>(crc32(0, data, static_cast<uInt>(checksum_offset)));
    constexpr unsigned byte_mask = 0xff;
    for (unsigned index = 0; index < 4; ++index) {
      data[checksum_offset + index] = static_cast<std::uint8_t>((crc >> (index * 8U)) & byte_mask);
    }
  }
  return mutated;
}
}

extern "C" std::size_t LLVMFuzzerCustomMutator(std::uint8_t *data,
                                               const std::size_t size,
                                               const std::size_t maximum_size,
                                               const unsigned seed) {
  return mutate({.data = data, .size = size, .capacity = maximum_size, .seed = seed});
}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, const std::size_t size) {
  try {
    const auto image = blofeld::decode_flash_image(std::span(data, size));
    static_cast<void>(blofeld::encode_flash_image(image));
  } catch (const blofeld::Fault &) {
    // Fixed length, CRC, header and invariant rejections are expected.
    return 0;
  }
  return 0;
}
