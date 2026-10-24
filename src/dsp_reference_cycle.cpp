// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_reference_cycle.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "blofeld/dsp_reference_cycle.hpp"
#include "blofeld/dsp_reference_execution.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <variant>

namespace blofeld {
namespace {
std::string describe(const DspMemoryAccess access) {
  constexpr std::array spaces{"P:", "X:", "Y:"};
  return std::string(access.operation == DspMemoryOperation::read ? "read " : "write ") +
         spaces.at(static_cast<unsigned>(access.target.space)) + hex(access.target.address, 6);
}

bool controller_read(const DspMemoryAccess access) {
  return access.target.space == DspSpace::x && access.operation == DspMemoryOperation::read &&
         DspDma::handles(access.target.address);
}

std::optional<DspMemoryAccess> native_access(const DspReferencePending &pending) {
  const auto *access = std::get_if<DspNativeAccess>(&pending.effect);
  if (access == nullptr) {
    if (const auto *ack = std::get_if<DspInterruptAcknowledge>(&pending.effect)) {
      throw Fault("Unqualified native bus-cycle interrupt acknowledge vector=" +
                  hex(ack->request.vector, 6));
    }
    throw Fault("Unqualified native bus-cycle peripheral reset");
  }
  if (access->kind == DspNativeAccessKind::opcode ||
      access->kind == DspNativeAccessKind::extension) {
    if (pending.latched_program_word.has_value()) { return std::nullopt; }
    return DspMemoryAccess{.target = access->target};
  }
  if (access->kind == DspNativeAccessKind::read) {
    return DspMemoryAccess{.target = access->target};
  }
  if (access->kind == DspNativeAccessKind::write) {
    return DspMemoryAccess{.target = access->target, .operation = DspMemoryOperation::write};
  }
  const DspMemoryAccess test{.target = access->target};
  if (access->kind == DspNativeAccessKind::bit && access->bit &&
      access->bit->operation == DspBitOperation::test && controller_read(test)) {
    return test;
  }
  constexpr std::array spaces{"P:", "X:", "Y:"};
  constexpr std::array kinds{"opcode", "extension", "read", "write", "bit", "vector"};
  throw Fault("Unqualified native bus-cycle " +
              std::string(kinds.at(static_cast<unsigned>(access->kind))) + " at " +
              std::string(spaces.at(static_cast<unsigned>(access->target.space))) +
              hex(access->target.address, 6));
}

void append(DspReferenceCycleResult &result, const DspMemoryAccess access) {
  result.core_accesses.at(result.core_access_count++) = access;
}

void check_resources(const DspBus &bus, const DspReferenceCycleResult &result) {
  std::array<std::optional<DspMemoryResource>, dsp_reference_cycle_accesses> resources;
  for (std::size_t index = 0; index < result.core_access_count; ++index) {
    const auto access = result.core_accesses.at(index);
    if (!controller_read(access)) { resources.at(index) = bus.memory_resource(access); }
  }
  if (result.core_access_count != 2) { return; }
  const auto first = resources.front();
  const auto second = resources.back();
  if (result.core_accesses.front().target.space == result.core_accesses.back().target.space ||
      (first && second && first->bank == second->bank && first->rom == second->rom)) {
    throw Fault("Unqualified dual core bus-cycle use: " + describe(result.core_accesses.front()) +
                " and " + describe(result.core_accesses.back()));
  }
}
}

DspReferenceCycleResult reference_cycle_footprint(const DspBus &bus,
                                                  const DspReferenceCycleInputs &inputs,
                                                  const std::optional<DspReferencePending> &pending,
                                                  const DspProgramFetchView &fetch) {
  DspReferenceCycleResult result;
  if (inputs.fetch == DspProgramFetchStage::read) {
    if (!fetch.pending_address) { throw Fault("Bus-cycle F2 lost its actual issued address"); }
    append(result, {.target = {.address = *fetch.pending_address}});
  }
  if (inputs.access) {
    if (!pending) { throw Fault("Bus-cycle admission lost its actual native pending effect"); }
    if (const auto access = native_access(*pending)) { append(result, *access); }
  }
  check_resources(bus, result);
  return result;
}
}
