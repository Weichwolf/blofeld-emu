// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned control = 0xffffec, count = 0xffffed, target = 0xffffee, source = 0xffffef;
constexpr unsigned payload = 0x123456, canary = 0xa55a33, destination = 0x900;
constexpr unsigned software_copy = 0x9802d4, x_address = 0x660;
constexpr unsigned port_data = 0xfffff5, port_direction = 0xfffff6, port_control = 0xfffff7;
unsigned guards = 0, assertions = 0;

void require(const bool condition, const std::string_view message) {
  ++assertions;
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

void put(DspBus &bus, const unsigned address, const unsigned value) {
  bus.write({.space = DspSpace::x, .address = address, .value = value});
}

DspDmaInputs inputs(const unsigned cycle) {
  return {.cycle = cycle,
          .known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

void setup(DspBus &bus, const bool program_source = false) {
  require(!bus.advance_dma(
                  inputs(0),
                  {.accesses = {}, .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy})
               .phase,
          "Real controller baseline");
  bus.write({.space = program_source ? DspSpace::p : DspSpace::x,
             .address = program_source ? 0U : x_address,
             .value = payload});
  bus.write({.space = DspSpace::y, .address = destination, .value = canary});
  put(bus, source, program_source ? 0U : x_address);
  put(bus, target, destination);
  put(bus, count, 0);
  // FMTable10-5 DSS=2 selects program memory; every other field is retained.
  put(bus, control, software_copy | (program_source ? 2U : 0U));
}

void refusal(const DspMemoryAccess access, const bool unknown_owner = false) {
  DspBus bus;
  setup(bus);
  const DspCoreBusUse empty{.accesses = {},
                            .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy};
  const auto read = bus.advance_dma(inputs(1), empty);
  require(read.phase && read.phase->kind == DspDmaPhaseKind::source_read && !read.word,
          "A genuine source phase precedes every rejected footprint");
  const std::array footprint{access};
  std::string message;
  try {
    static_cast<void>(bus.advance_dma(
        inputs(2),
        {.accesses = footprint,
         .knowledge = unknown_owner ? DspCoreBusKnowledge::unqualified
                                    : DspCoreBusKnowledge::reference_cycle_occupancy}));
  } catch (const Fault &error) { message = error.what(); }
  require((message.contains("Unqualified") || message.contains("requires explicitly known")) &&
              bus.last_dma_cycle() == 1 &&
              bus.read({.space = DspSpace::y, .address = destination}) == canary,
          "Unknown ownership, neighbor words and unsupported phases refuse before DMA progress");
  const auto write = bus.advance_dma(inputs(2), empty);
  require(write.word && write.word->value == payload &&
              bus.read({.space = DspSpace::y, .address = destination}) == payload,
          "A rejected footprint preserves the original captured DMA destination phase");
  ++guards;
}

void program_contention() {
  DspBus bus;
  setup(bus, true);
  const std::array held{DspMemoryAccess{.target = {.space = DspSpace::p, .address = 1}},
                        DspMemoryAccess{.target = {.space = DspSpace::y, .address = port_control},
                                        .operation = DspMemoryOperation::write}};
  const auto waiting = bus.advance_dma(
      inputs(1), {.accesses = held, .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
  require(waiting.phase && waiting.phase->kind == DspDmaPhaseKind::source_read && !waiting.word &&
              bus.read({.space = DspSpace::y, .address = destination}) == canary,
          "A Port-F footprint cannot erase actual same-partition program-fetch contention");
  const std::array free{DspMemoryAccess{.target = {.space = DspSpace::p, .address = 0x800}},
                        held[1]};
  const auto read = bus.advance_dma(
      inputs(2), {.accesses = free, .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
  require(read.phase && read.phase->kind == DspDmaPhaseKind::source_read && !read.word,
          "Moving the core fetch to a distinct physical partition releases the original source");
  const auto write = bus.advance_dma(
      inputs(3), {.accesses = free, .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
  require(write.word && write.word->value == payload && !write.core_stall,
          "Only the real independent destination phase completes the program-memory transfer");
}
}

int main() {
  try {
    for (const unsigned address : {port_direction, port_control}) {
      for (const auto phase : {DspMemoryPhase::peripheral_bit, DspMemoryPhase::peripheral_sample}) {
        for (const auto operation : {DspMemoryOperation::read, DspMemoryOperation::write}) {
          refusal({.target = {.space = DspSpace::y, .address = address},
                   .operation = operation,
                   .phase = phase});
        }
      }
      refusal({.target = {.space = DspSpace::y, .address = address}});
      refusal({.target = {.space = DspSpace::x, .address = address},
               .operation = DspMemoryOperation::write});
      refusal({.target = {.space = DspSpace::y, .address = address},
               .operation = DspMemoryOperation::write},
              true);
    }
    for (const unsigned address : {port_data, port_control + 1, 0xffff9eU, 0xffff9fU}) {
      refusal({.target = {.space = DspSpace::y, .address = address},
               .operation = DspMemoryOperation::write});
    }
    program_contention();
    std::cout << "Port-F DMA guards=" << guards << " assertions=" << assertions
              << ";real captured DMA and program-fetch contention retained\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
