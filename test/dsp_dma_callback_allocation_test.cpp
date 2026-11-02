// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <sanitizer/allocator_interface.h>

namespace {
using namespace blofeld;
constexpr unsigned periods = 4096;
thread_local bool tracking = false;
thread_local std::uint64_t allocations = 0;
thread_local std::uint64_t releases = 0;
thread_local std::uint64_t bytes = 0;

void allocated(const volatile void *pointer, const std::size_t size) {
  static_cast<void>(pointer);
  if (tracking) {
    ++allocations;
    bytes += size;
  }
}

void released(const volatile void *pointer) {
  static_cast<void>(pointer);
  if (tracking) { ++releases; }
}

class CountAllocations {
public:
  CountAllocations() {
    allocations = 0;
    releases = 0;
    bytes = 0;
    tracking = true;
  }

  ~CountAllocations() { tracking = false; }

  CountAllocations(const CountAllocations &) = delete;
  CountAllocations &operator=(const CountAllocations &) = delete;
};

void check_hooks() {
  constexpr std::size_t control_bytes = 64;
  // Volatile function pointers keep this positive control observable under optimization.
  auto *volatile allocate = &std::malloc;
  auto *volatile release = &std::free;
  {
    const CountAllocations measure;
    void *const block = allocate(control_bytes);
    if (block == nullptr) { throw Fault("Allocator positive control failed"); }
    release(block);
  }
  if (allocations != 1 || releases != 1 || bytes != control_bytes) {
    throw Fault("Sanitizer hooks did not observe the explicit allocation and release");
  }
}

void idle(DspBus &bus, const unsigned cycle) {
  const auto result = bus.advance_dma(
      {.cycle = cycle,
       .known_requests = 15, // IRQA..IRQD are known inactive; internal requests retain their model.
       .timing = DspDmaTiming::reference_sampled_cycles,
       .grant = DspDmaGrant::available},
      {.accesses = {}, .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
  if (result.word || result.phase || result.core_stall) {
    throw Fault("Inactive DMA invented a word, admission phase or core stall");
  }
}
}

int main() {
  try {
    if (__sanitizer_install_malloc_and_free_hooks(allocated, released) == 0) {
      throw Fault("Sanitizer allocator hook installation failed");
    }
    check_hooks();
    DspBus bus;
    idle(bus, 0);
    {
      const CountAllocations measure;
      for (unsigned cycle = 1; cycle <= periods; ++cycle) { idle(bus, cycle); }
    }
    if (allocations != 0 || releases != 0 || bytes != 0) {
      throw Fault("Inactive DMA callback storage allocated or released heap memory");
    }
    std::cout << "Allocator positive control passed; " << periods
              << " inactive DMA periods: zero allocations, releases and requested bytes\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
