// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/detail/dsp_reference_cycle.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <cstddef>
#include <exception>
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
  if (std::holds_alternative<DspInstructionRetire>(pending.effect) ||
      std::holds_alternative<DspAddressUpdate>(pending.effect) ||
      std::holds_alternative<DspMoveWriteback>(pending.effect)) {
    return std::nullopt;
  }
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
  for (std::size_t index = 0; index < result.core_access_count; ++index) {
    for (std::size_t other = 0; other < index; ++other) {
      const auto first = resources.at(index);
      const auto second = resources.at(other);
      const auto a = result.core_accesses.at(index);
      const auto b = result.core_accesses.at(other);
      if (a.target.space == b.target.space ||
          (first && second && first->bank == second->bank && first->rom == second->rom)) {
        throw Fault("Unqualified dual core bus-cycle use: " + describe(a) + " and " + describe(b));
      }
    }
  }
}
}

DspReferenceCycleResult reference_cycle_footprint(const DspBus &bus,
                                                  const DspReferenceCycleInputs &inputs,
                                                  const std::optional<DspReferencePending> &pending,
                                                  const DspProgramFetchView &fetch,
                                                  const std::optional<DspAddressStageView> memory) {
  DspReferenceCycleResult result;
  if (inputs.fetch == DspProgramFetchStage::read) {
    if (!fetch.pending_address) { throw Fault("Bus-cycle F2 lost its actual issued address"); }
    append(result, {.target = {.address = *fetch.pending_address}});
  }
  if (inputs.access) {
    if (!pending) { throw Fault("Bus-cycle admission lost its actual native pending effect"); }
    if (const auto access = native_access(*pending)) { append(result, *access); }
  }
  if (memory) {
    for (std::size_t index = 0; index < memory->data_access_count; ++index) {
      if (result.core_access_count == result.core_accesses.size()) {
        throw Fault("Unqualified extra native memory-stage footprint");
      }
      append(result, memory->data_accesses.at(index));
    }
  }
  try {
    check_resources(bus, result);
  } catch (const std::exception &error) {
    if (!memory) { throw; }
    std::string group;
    for (std::size_t index = 0; index < memory->data_access_count; ++index) {
      if (index != 0) { group += " and "; }
      group += describe(memory->data_accesses.at(index));
    }
    throw Fault("Native memory E1 " + group + " pc=" + hex(memory->word.address, 6) +
                " omr=" + hex(memory->update.omr, 6) + ": " + error.what());
  }
  return result;
}
}
