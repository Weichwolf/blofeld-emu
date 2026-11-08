// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_memory.hpp"
#include "mcu/mcu_reset.hpp"
#include "runtime/mcu_dsp_runtime.hpp"
#include "usb/usb_midi_host.hpp"
#include "usb/usb_transfer.hpp"
#include "usb/usbip_runtime.hpp"
#include "usb/usbip_tcp.hpp"
#include "vi/io/memory_controller.hpp"
#include "vi/vi.hpp"

#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <ostream>
#include <vector>

namespace blofeld::vi {
class VirtualInstrument::Busy {
public:
  explicit Busy(VirtualInstrument &board) : board_(board) {
    board.require_idle();
    board_.vm_.active = true;
  }

  ~Busy() { board_.vm_.active = false; }

  Busy(const Busy &) = delete;
  Busy &operator=(const Busy &) = delete;

private:
  VirtualInstrument &board_;
};

void VirtualInstrument::require_idle() const {
  if (vm_.active) { throw Fault("Instrument control requires an inactive operation boundary"); }
}

void VirtualInstrument::require_healthy() const {
  require_idle();
  if (timing_faulted()) { throw Fault("Instrument failure requires a fresh device instance"); }
}

void VirtualInstrument::initialize() {
  require_healthy();
  if (vm_.started) { return; }
  const Busy guard(*this);
  try {
    check_pcm_output();
    boundary();
    vm_.started = true;
  } catch (...) {
    mark_fault();
    throw;
  }
}

void VirtualInstrument::advance() {
  require_healthy();
  initialize();
  const Busy guard(*this);
  try {
    check_pcm_output();
    execute_until_event();
    if (execution_.trace != nullptr) {
      *execution_.trace << "MCU end=" << ticks();
      const auto reset = vm_.mcu.system_reset_state();
      if (reset && reset->phase != McuResetPhase::complete) {
        *execution_.trace << " held-reset=1 reset-edges=" << reset->edges;
      } else {
        *execution_.trace << " pc=" << hex(vm_.mcu.cpu().state().pc);
      }
      *execution_.trace << " instructions=" << vm_.instructions
                        << " idle=" << vm_.mcu.idle_bus_cycles() << '\n';
    }
    boundary();
  } catch (...) {
    mark_fault();
    throw;
  }
}

void VirtualInstrument::poll_usb(UsbipRuntime &runtime) {
  require_healthy();
  if (!vm_.started) { throw Fault("USB/IP input requires an initialized instrument"); }
  if (vm_.mcu.system_reset_active()) { return; }
  const Busy guard(*this);
  const auto cpu = vm_.mcu.cpu().state();
  const auto before_started = runtime.started();
  const auto before_published = runtime.published();
  try {
    if (!prepare_usb(&runtime)) { return; }
    const auto events =
        runtime.poll(vm_.mcu.bus(),
                     {.pc = cpu.pc, .ccr = cpu.ccr},
                     {.owner = vm_.identity, .restore_revision = 0, .ticks = ticks()});
    if (execution_.trace != nullptr) {
      if (!before_started && runtime.started()) {
        *execution_.trace << "USBIP-START actual=" << ticks() << '\n';
      }
      if (!before_published && runtime.published()) {
        *execution_.trace << "USBIP-PUBLISH actual=" << ticks() << '\n';
      }
      for (const auto &event : events) {
        *execution_.trace << "USBIP-PEER actual=" << ticks() << " connection=" << event.connection
                          << " error=" << event.error << " reason=" << event.reason << '\n';
      }
    }
    if (!before_published && runtime.published()) {
      std::cout << "USBIP descriptors=ready tick=" << ticks() << '\n' << std::flush;
    }
    for (const auto &event : events) {
      if (event.error) { std::cerr << "USB/IP peer: " << event.reason << '\n'; }
    }
  } catch (...) {
    mark_fault();
    runtime.retire();
    throw;
  }
}

bool VirtualInstrument::prepare_usb(const void *owner) {
  if (vm_.usb_owner != nullptr && vm_.usb_owner != owner) {
    throw Fault("An instrument USB link requires one exclusive host session");
  }
  if (!vm_.usb_link) {
    constexpr unsigned frames_per_second = 1000;
    if (common_hz() % frames_per_second != 0) {
      throw Fault("USB frame time is not representable");
    }
    vm_.usb_link.emplace(common_hz() / frames_per_second);
    vm_.usb_owner = owner;
  }
  return vm_.usb_link->poll(vm_.mcu.bus(), ticks());
}

std::vector<UsbTransferCompletion> VirtualInstrument::poll_usb(UsbMidiHost &host) {
  require_healthy();
  if (!vm_.started) { throw Fault("USB MIDI host requires an initialized instrument"); }
  if (vm_.mcu.system_reset_active()) { return {}; }
  const Busy guard(*this);
  try {
    if (!prepare_usb(&host)) { return {}; }
    const auto cpu = vm_.mcu.cpu().state();
    return host.poll(vm_.mcu.bus(), {.pc = cpu.pc, .ccr = cpu.ccr}, ticks());
  } catch (...) {
    mark_fault();
    throw;
  }
}

Status VirtualInstrument::status() const {
  require_idle();
  const bool known = !vm_.mcu.system_reset_active();
  return {.mcu = known ? vm_.mcu.cpu().state() : CpuState{},
          .dsp = vm_.dsp.cpu().registers(),
          .mcu_state_known = known,
          .common_ticks = ticks(),
          .mcu_cycles = vm_.mcu.run_clock().bus_cycles(),
          .dsp_cycles = vm_.runtime->completed_cycles(),
          .dsp_steps = dsp_steps(),
          .instructions = vm_.instructions,
          .interrupts = vm_.interrupts,
          .waiting = known && vm_.mcu.cpu().waiting(),
          .held_reset = dsp_reset_active(),
          .faulted = timing_faulted()};
}

McuMemoryByte VirtualInstrument::inspect_mcu(const std::uint16_t address) const {
  require_idle();
  return vm_.mcu.bus().inspect_memory(address);
}

std::optional<std::uint32_t> VirtualInstrument::inspect_dsp(const DspAddress address) const {
  require_idle();
  return vm_.dsp.bus().inspect_memory(address);
}

}
