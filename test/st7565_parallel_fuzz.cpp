// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/st7565_parallel.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

extern "C" std::size_t
LLVMFuzzerMutate(std::uint8_t *data, std::size_t size, std::size_t maximum_size);

namespace {
constexpr std::size_t stride = 5;
constexpr std::size_t operation_limit = 4096;

struct Mutation {
  std::uint8_t *data;
  std::size_t size;
  std::size_t capacity;
  unsigned seed;
};

std::size_t mutate(const Mutation &request) {
  const auto mutated = LLVMFuzzerMutate(request.data, request.size, request.capacity);
  if (request.seed % 2 == 0) {
    for (std::size_t index = 0; index < mutated / stride; ++index) {
      auto *record = request.data + (index * stride);
      // Retain raw invalid inputs as well as canonical unknown/drive observations.
      record[1] &= blofeld::st7565_pin::all;
      record[0] &= record[1];
      record[4] &= record[2];
      record[3] &= record[4];
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
  blofeld::St7565Parallel bus;
  const auto count = std::min(size / stride, operation_limit);
  for (std::size_t index = 0; index < count; ++index) {
    const auto *record = data + (index * stride);
    try {
      bus.sample({.control = record[0],
                  .known = record[1],
                  .drive = record[2],
                  .data = {.value = record[3], .known = record[4]}});
    } catch (const blofeld::Fault &) {
      // Deliberate qualification failures retain state for later observations/reset.
      continue;
    }
  }
  return 0;
}
