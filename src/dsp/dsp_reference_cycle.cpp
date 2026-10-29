// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/detail/dsp_reference_cycle.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_dma_registers.hpp"
#include "dsp/dsp_memory_form.hpp"
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
constexpr unsigned short_io_begin = 0xffff80;
constexpr unsigned address_mask = 0xffffff;
constexpr unsigned shi_receive = 0xffff94;
constexpr unsigned shi_status = 0xffff91;

std::string describe(const DspMemoryAccess access) {
  constexpr std::array spaces{"P:", "X:", "Y:"};
  return std::string(access.operation == DspMemoryOperation::read ? "read " : "write ") +
         spaces.at(static_cast<unsigned>(access.target.space)) + hex(access.target.address, 6);
}

bool controller_access(const DspMemoryAccess access) {
  return access.target.space == DspSpace::x && DspDma::handles(access.target.address) &&
         (access.operation == DspMemoryOperation::read ||
          (access.operation == DspMemoryOperation::write && access.target.address != dma::status));
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
      access->bit->operation == DspBitOperation::test && controller_access(test)) {
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

std::optional<DspMemoryAccess> peripheral_data(const std::optional<DspAddressStageView> &memory) {
  if (!memory) { return std::nullopt; }
  if (memory->kind == DspAddressStageKind::absolute_short ||
      memory->kind == DspAddressStageKind::absolute_long) {
    return std::nullopt;
  }
  if (memory->kind == DspAddressStageKind::register_agu) {
    const auto form = decode_memory_stage(memory->word.value);
    if (!form || !is_program_transfer(*form)) { return std::nullopt; }
    const auto peripheral = form->peripheral;
    if (!peripheral) { throw Fault("Native P-transfer footprint lost its peripheral operand"); }
    const auto target = *peripheral;
    const auto port = memory->data_accesses[0];
    const auto program = memory->data_accesses[1];
    if (memory->data_access_count != 2 || memory->extension || memory->secondary ||
        port.target.space != target.space || port.target.address != target.address ||
        port.operation != DspMemoryOperation::read || program.target.space != DspSpace::p ||
        program.operation != DspMemoryOperation::write ||
        program.target.address != memory->update.address || memory->update.space != DspSpace::p ||
        memory->update.operation != DspMemoryOperation::write) {
      throw Fault("Unqualified native peripheral-to-P footprint");
    }
    return port;
  }
  const auto access = memory->data_accesses[0];
  if (memory->kind == DspAddressStageKind::peripheral_bit) {
    const auto form = decode_memory_stage(memory->word.value);
    if (!form || !form->peripheral_bit || memory->data_access_count != 1 || memory->extension ||
        memory->pointer_update || memory->secondary ||
        access.phase != DspMemoryPhase::peripheral_bit ||
        access.target.space != form->peripheral_bit->space ||
        access.target.address != form->peripheral_bit->address) {
      throw Fault("Unqualified native indivisible peripheral bit footprint");
    }
    return access;
  }
  const bool immediate = memory->kind == DspAddressStageKind::peripheral_immediate;
  if (memory->extension.has_value() != immediate || memory->pointer_update || memory->secondary ||
      memory->data_access_count != 1 || access.target.space == DspSpace::p ||
      access.target.address < short_io_begin || access.target.address > address_mask ||
      access.operation != DspMemoryOperation::write) {
    throw Fault("Unqualified native peripheral store footprint");
  }
  return access;
}

bool same_access(const DspMemoryAccess first, const DspMemoryAccess second) {
  return first.target.space == second.target.space &&
         first.target.address == second.target.address && first.operation == second.operation &&
         first.phase == second.phase;
}

void check_port_overlap(const std::optional<DspMemoryAccess> peripheral,
                        const std::optional<DspMemoryAccess> early) {
  if (early && peripheral && peripheral->operation == DspMemoryOperation::read &&
      (peripheral->target.space != DspSpace::x || peripheral->target.address != shi_receive ||
       early->target.space != DspSpace::x || early->target.address != shi_status)) {
    throw Fault("Unqualified peripheral read/status overlap: " + describe(*peripheral) + " and " +
                describe(*early));
  }
}

void check_resources(const DspBus &bus,
                     const DspReferenceCycleResult &result,
                     const std::optional<DspMemoryAccess> peripheral,
                     const std::optional<DspMemoryAccess> early) {
  check_port_overlap(peripheral, early);
  std::array<std::optional<DspMemoryResource>, dsp_reference_cycle_accesses> resources;
  unsigned data_accesses = 0;
  for (std::size_t index = 0; index < result.core_access_count; ++index) {
    const auto access = result.core_accesses.at(index);
    if (access.target.space != DspSpace::p) { ++data_accesses; }
    const bool port = peripheral && same_access(access, *peripheral);
    const bool sample = early && same_access(access, *early);
    if (!controller_access(access) && !port && !sample) {
      resources.at(index) = bus.memory_resource(access);
    }
  }
  if ((peripheral || early) && data_accesses != static_cast<unsigned>(peripheral.has_value()) +
                                                    static_cast<unsigned>(early.has_value())) {
    throw Fault("Unqualified concurrent native peripheral/data bus effects");
  }
  for (std::size_t index = 0; index < result.core_access_count; ++index) {
    for (std::size_t other = 0; other < index; ++other) {
      const auto a = result.core_accesses.at(index);
      const auto b = result.core_accesses.at(other);
      // The selected port pipeline admits an early read with one older E1 store,
      // or SHI status after an older HRX read. No extra data phase is admitted.
      const bool phased_ports = early && peripheral &&
                                ((same_access(a, *early) && same_access(b, *peripheral)) ||
                                 (same_access(b, *early) && same_access(a, *peripheral)));
      const auto first = resources.at(index);
      const auto second = resources.at(other);
      if (!phased_ports &&
          (a.target.space == b.target.space ||
           (first && second && first->bank == second->bank && first->rom == second->rom))) {
        throw Fault("Unqualified dual core bus-cycle use: " + describe(a) + " and " + describe(b));
      }
    }
  }
}

}

namespace {
void reject_sampled_program_write(const std::optional<DspAddressStageView> &memory,
                                  const DspProgramFetchView &fetch) {
  if (!memory) { return; }
  const auto form = decode_memory_stage(memory->word.value);
  if (!form || !is_program_memory(*form) || memory->update.operation != DspMemoryOperation::write) {
    return;
  }
  const auto address = memory->update.address;
  bool sampled = fetch.pending_address == address;
  for (std::size_t index = 0; index < fetch.used; ++index) {
    sampled = sampled || fetch.words.at(index).address == address;
  }
  if (sampled) {
    throw Fault(
        "Unqualified automatic P-write to captured/issued instruction write P:" + hex(address, 6) +
        " pc=" + hex(memory->word.address, 6) + " omr=" + hex(memory->update.omr, 6));
  }
}
}

DspReferenceCycleResult reference_cycle_footprint(const DspBus &bus,
                                                  const DspReferenceCycleInputs &inputs,
                                                  const std::optional<DspReferencePending> &pending,
                                                  const DspProgramFetchView &fetch,
                                                  const std::optional<DspAddressStageView> memory,
                                                  const std::optional<DspMemoryAccess> early) {
  reject_sampled_program_write(memory, fetch);
  DspReferenceCycleResult result;
  if (inputs.fetch == DspProgramFetchStage::read) {
    if (!fetch.pending_address) { throw Fault("Bus-cycle F2 lost its actual issued address"); }
    append(result, {.target = {.address = *fetch.pending_address}});
  }
  if (inputs.access) {
    if (!pending) { throw Fault("Bus-cycle admission lost its actual native pending effect"); }
    if (const auto access = native_access(*pending)) { append(result, *access); }
  }
  if (early) { append(result, *early); }
  if (memory) {
    for (std::size_t index = 0; index < memory->data_access_count; ++index) {
      if (result.core_access_count == result.core_accesses.size()) {
        throw Fault("Unqualified extra native memory-stage footprint");
      }
      append(result, memory->data_accesses.at(index));
    }
  }
  try {
    if (memory && memory->kind == DspAddressStageKind::peripheral_bit && early) {
      throw Fault("Unqualified atomic peripheral bit/early-read overlap");
    }
    check_resources(bus, result, peripheral_data(memory), early);
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
