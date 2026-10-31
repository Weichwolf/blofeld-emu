// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_dma_registers.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_timers.hpp"

#include <algorithm>
#include <array>
#include <optional>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned io_base = 0xffff80;
constexpr unsigned shi_status = 0xffff91;
constexpr unsigned shi_receive = 0xffff94;
constexpr std::array gpio_data_registers{DspAddress{.space = DspSpace::x, .address = 0xffffbd},
                                         DspAddress{.space = DspSpace::y, .address = 0xffff9d},
                                         DspAddress{.space = DspSpace::y, .address = 0xfffff5},
                                         DspAddress{.space = DspSpace::x, .address = 0xffffd5}};

bool controller_access(const DspMemoryAccess access) {
  return access.target.space == DspSpace::x && DspDma::handles(access.target.address) &&
         (access.operation == DspMemoryOperation::read ||
          (access.operation == DspMemoryOperation::write && access.target.address != dma::status));
}

// UM tables3-7/3-8 identify these GPIO data words; their direction/control
// neighbors are separate registers. APR23 section5.2 locates peripheral
// contention at the accessed data register, not every active DMA channel.
bool gpio_data_access(const DspMemoryAccess access) {
  const bool bit = access.phase == DspMemoryPhase::peripheral_bit;
  const bool write =
      access.phase == DspMemoryPhase::execute && access.operation == DspMemoryOperation::write;
  const bool read = access.phase == DspMemoryPhase::peripheral_sample &&
                    access.operation == DspMemoryOperation::read;
  return (bit || write || read) && std::ranges::any_of(gpio_data_registers, [&](const auto port) {
           return access.target.space == port.space && access.target.address == port.address;
         });
}

// UM table3-7: timer control/counter words are distinct from every admitted
// DMA ESAI/DAX data endpoint. Native E1/E2 retain all timer validation and
// clock ordering; this predicate grants resources, never performs a transaction.
bool timer_access(const DspMemoryAccess access) {
  const bool bit = access.phase == DspMemoryPhase::peripheral_bit;
  const bool read = access.phase == DspMemoryPhase::peripheral_sample &&
                    access.operation == DspMemoryOperation::read;
  const bool write =
      access.phase == DspMemoryPhase::execute && access.operation == DspMemoryOperation::write;
  return access.target.space == DspSpace::x && DspTimers::handles(access.target.address) &&
         (bit || read || write);
}

// UM GPIO maps place direction/control beside each data word; these are
// distinct from admitted DMA data endpoints (APR23sections5.1..5.3). Port F
// uses the BIU (UM6.2.4); native BCR, mux and clock validation still apply.
// This predicate supplies neither a GPIO transaction nor physical pad timing.
bool gpio_configuration_write(const DspMemoryAccess access) {
  constexpr auto direction = static_cast<unsigned>(DspGpioRegister::direction);
  constexpr auto control = static_cast<unsigned>(DspGpioRegister::control);
  return access.phase == DspMemoryPhase::execute && access.operation == DspMemoryOperation::write &&
         std::ranges::any_of(gpio_data_registers, [&](const auto port) {
           return access.target.space == port.space &&
                  (access.target.address == port.address + direction ||
                   access.target.address == port.address + control);
         });
}

bool independent_peripheral(const DspMemoryAccess access) {
  return gpio_data_access(access) || timer_access(access) || gpio_configuration_write(access);
}

DspSpace space(const DspDmaSpace value) {
  switch (value) {
    case DspDmaSpace::x: return DspSpace::x;
    case DspDmaSpace::y: return DspSpace::y;
    case DspDmaSpace::p: return DspSpace::p;
    default: throw Fault("Invalid DSP DMA memory space");
  }
}

std::string describe(const DspAddress address) {
  constexpr std::array names{"P:", "X:", "Y:"};
  return std::string(names.at(static_cast<unsigned>(address.space))) + hex(address.address, 6);
}

bool program_store_pair(const DspCoreBusUse core) {
  const bool status_overlap = core.accesses.size() == 3;
  if (core.accesses.size() != 2 && !status_overlap) { return false; }
  const auto read = core.accesses[status_overlap ? 1U : 0U];
  if (status_overlap) {
    const auto status = core.accesses.front();
    if (status.phase != DspMemoryPhase::peripheral_sample ||
        status.operation != DspMemoryOperation::read || status.target.space != DspSpace::x ||
        status.target.address != shi_status || read.target.space != DspSpace::x ||
        read.target.address != shi_receive) {
      return false;
    }
  }
  const auto write = core.accesses.back();
  const auto port = read.target;
  return read.phase == DspMemoryPhase::execute && write.phase == DspMemoryPhase::execute &&
         read.operation == DspMemoryOperation::read &&
         (port.space == DspSpace::x || port.space == DspSpace::y) && port.address >= io_base &&
         port.address <= DspDma::full_word && write.target.space == DspSpace::p &&
         write.operation == DspMemoryOperation::write;
}

std::optional<DspMemoryAccess> bit_peripheral(const DspCoreBusUse core) {
  std::optional<DspMemoryAccess> selected;
  for (const auto access : core.accesses) {
    const auto target = access.target;
    if (target.space == DspSpace::p && access.phase == DspMemoryPhase::execute &&
        access.operation == DspMemoryOperation::read) {
      continue;
    }
    const bool operation = access.operation == DspMemoryOperation::read ||
                           access.operation == DspMemoryOperation::write;
    if (selected || target.space == DspSpace::p || target.address < io_base ||
        target.address > DspDma::full_word || access.phase != DspMemoryPhase::peripheral_bit ||
        !operation) {
      throw Fault("Unqualified indivisible peripheral bit footprint at " + describe(target));
    }
    selected = access;
  }
  if (!selected) { throw Fault("Indivisible peripheral bit footprint lost its selected operand"); }
  return controller_access(*selected) ? std::nullopt : selected;
}

std::optional<DspMemoryAccess> core_peripheral(const DspBus &bus, const DspCoreBusUse core) {
  // Short-I/O occupancy is separate from RAM. This never dispatches a core transaction;
  // native E1 still owns register validation and the actual read/write. Writable DMA
  // controller words use their register port after this cycle's actual DMA phase.
  std::optional<DspMemoryAccess> peripheral;
  unsigned data_accesses = 0;
  bool early = false;
  bool store = false;
  bool program_read = false;
  if (std::ranges::any_of(core.accesses, [](const auto access) {
        return access.phase == DspMemoryPhase::peripheral_bit;
      })) {
    return bit_peripheral(core);
  }
  const bool program_pair = program_store_pair(core);
  for (const auto access : core.accesses) {
    const auto target = access.target;
    if (target.space != DspSpace::p) { ++data_accesses; }
    const bool io = (target.space == DspSpace::x || target.space == DspSpace::y) &&
                    target.address >= io_base && target.address <= DspDma::full_word;
    if (access.phase == DspMemoryPhase::peripheral_sample) {
      if (!io || access.operation != DspMemoryOperation::read || early) {
        throw Fault("Unqualified early peripheral read footprint at " + describe(target));
      }
      early = true;
      peripheral = access;
    } else if (access.phase != DspMemoryPhase::execute) {
      throw Fault("Unknown DSP core bus subphase at " + describe(target));
    } else if (io && access.operation == DspMemoryOperation::write &&
               (target.space != DspSpace::x || !DspDma::handles(target.address))) {
      if (store) { throw Fault("Multiple DSP peripheral stores in one execute phase"); }
      store = true;
      peripheral = access;
    } else if (io && access.operation == DspMemoryOperation::read && program_pair) {
      program_read = true;
      peripheral = access;
    } else if (!controller_access(access)) {
      static_cast<void>(bus.memory_resource(access));
    }
  }
  const unsigned admitted_data = static_cast<unsigned>(early) + static_cast<unsigned>(store) +
                                 static_cast<unsigned>(program_read);
  if (peripheral && data_accesses != admitted_data) {
    throw Fault("Unqualified concurrent DSP peripheral/data bus effects at " +
                describe(peripheral->target));
  }
  return peripheral;
}

bool core_conflict(const DspBus &bus, const DspCoreBusUse core, const DspMemoryAccess target) {
  bool conflict = false;
  for (const auto access : core.accesses) {
    // Controller inspection and qualified independent peripheral accesses do not
    // consume the DMA memory path. Other core accesses still arbitrate normally.
    if (controller_access(access) || independent_peripheral(access)) { continue; }
    const auto relation = bus.memory_conflict({.core = access, .dma = target});
    if (relation == DspMemoryConflict::unqualified) {
      const bool write = target.operation == DspMemoryOperation::write;
      throw Fault("Unqualified DSP core/DMA bank contention at " +
                  std::string(write ? "destination write " : "source read ") +
                  describe(target.target) + " core=" + describe(access.target));
    }
    conflict = conflict || relation == DspMemoryConflict::protected_partition;
  }
  return conflict;
}

void check_access(const DspDmaAddress address, const bool write) {
  if (address.space == DspDmaSpace::p || address.address < io_base) { return; }
  constexpr unsigned primary_tx = 0xffffa0;
  constexpr unsigned secondary_tx = 0xffff80;
  constexpr unsigned dax_non_audio = 0xffffd1;
  constexpr unsigned dax_audio_a = 0xffffd2;
  constexpr unsigned dax_audio_b = 0xffffd3;
  const bool x = address.space == DspDmaSpace::x;
  const auto first = x ? primary_tx : secondary_tx;
  if (write && ((address.address >= first && address.address - first < 6) ||
                (x && (address.address == dax_non_audio || address.address == dax_audio_a ||
                       address.address == dax_audio_b)))) {
    return;
  }
  throw Fault(std::string("Unsupported DSP DMA peripheral ") + (write ? "write at " : "read at ") +
              (x ? "X:" : "Y:") + hex(address.address, 6));
}
}

std::optional<DspDmaWord> DspBus::advance_dma(DspDmaInputs inputs) {
  return advance_dma_with_admission(inputs, {});
}

std::optional<DspDmaWord> DspBus::advance_dma_with_admission(DspDmaInputs inputs,
                                                             const DspDmaAdmission &admission) {
  constexpr unsigned external = 15;
  constexpr unsigned primary = 1U << 12U;
  constexpr unsigned secondary = 1U << 22U;
  constexpr unsigned dax_request = 1U << 10U;
  if (((inputs.known_requests | inputs.requests) & ~external) != 0) {
    throw Fault("External DSP DMA requests may drive only IRQA..IRQD, not internal status");
  }
  const auto required = dma_.requested_sources();
  if ((required & ~(external | primary | secondary | dax_request)) != 0) {
    throw Fault("Unsupported/unobserved DSP DMA request source at X:0xfffff4");
  }
  if ((required & dax_request) != 0 || gpio(DspPort::d).outputs().peripheral == 0) {
    sync_dax();
    inputs.known_requests |= dax_request;
    if (dax_.request()) { inputs.requests |= dax_request; }
  }
  for (const auto instance : {DspEsaiInstance::primary, DspEsaiInstance::secondary}) {
    const auto bit = instance == DspEsaiInstance::primary ? primary : secondary;
    // Preserve a real low baseline before DE, without reading SAISR or arming TUE clear.
    const auto &serial = esai_.at(instance == DspEsaiInstance::primary ? 0U : 1U);
    if ((required & bit) != 0 || !serial.needs_transmit_clock()) {
      inputs.known_requests |= bit;
      if (serial.transmit_request()) { inputs.requests |= bit; }
    }
  }
  return dma_.advance(
      inputs,
      {.read =
           [this](const DspDmaAddress address) {
             check_access(address, false);
             return read({.space = space(address.space), .address = address.address});
           },
       .write =
           [this](const DspDmaAddress address, const auto value) {
             check_access(address, true);
             write({.space = space(address.space), .address = address.address, .value = value});
           },
       .admit = admission});
}

DspDmaBusResult DspBus::advance_dma(const DspDmaInputs inputs, const DspCoreBusUse core) {
  if (core.knowledge != DspCoreBusKnowledge::reference_cycle_occupancy) {
    throw Fault("DSP DMA bank arbitration requires explicitly known core-cycle occupancy");
  }
  const auto peripheral = core_peripheral(*this, core);
  // Qualified GPIO/timer/Port-F configuration resources are distinct from admitted DMA paths.
  // Native handlers still own the actual transactions and every strict fault.
  // Other peripheral collisions require separate qualification.
  if (peripheral && dma_.active() && !independent_peripheral(*peripheral)) {
    const auto *const operation =
        peripheral->operation == DspMemoryOperation::write ? "write" : "read";
    throw Fault("Unqualified DSP peripheral memory resource " + std::string(operation) + " at " +
                describe(peripheral->target) + " with active DMA");
  }
  DspDmaBusResult result;
  const DspDmaAdmission admission = [&](const DspDmaPhase &phase) {
    result.phase = phase;
    const bool write = phase.kind == DspDmaPhaseKind::destination_write;
    const auto selected = write ? phase.destination : phase.source;
    const DspMemoryAccess target{
        .target = {.space = space(selected.space), .address = selected.address},
        .operation = write ? DspMemoryOperation::write : DspMemoryOperation::read};
    if (selected.space != DspDmaSpace::p && selected.address >= io_base) {
      // Active DMA permits only physical memory or DMA-controller reads in the core footprint.
      // FM10.2/APR23 section5: these resources do not contend with a data register.
      check_access(selected, write);
      return DspDmaGrant::available;
    }
    const bool conflict = core_conflict(*this, core, target);
    result.core_stall = write && conflict;
    return !write && conflict ? DspDmaGrant::wait : DspDmaGrant::available;
  };
  result.word = advance_dma_with_admission(inputs, admission);
  return result;
}
}
